//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"
#include "app_sys.h"
#include "sys_task.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "app_ins.h"
#include "app_motor.h"
#include "alg_filter.h"
#include "app_msg.h"
#include "app_sys.h"
#include "bsp_rc.h"
#include "sys_task.h"
#include "app_gimbal.h"
#include "dev_motor_dji.h"
#include "ctrl_motor_base_pid.h"
#include "dev_cap.h"
#include "bsp_time.h"
#include "power_ctrl.h"
#include "app_total_cmd.h"
#ifdef COMPILE_CHASSIS
using namespace Motor;
using namespace Algorithm;
#define yaw_zero_pos 3652
/*
 *  适用于麦克纳姆轮（民航英雄）
 *  实现了基础的旋转、平移
 */

/*
 *  麦克纳姆轮
 *  ^ vy
 *  |       LU              RU
 *  |           O ------ O
 *  |           |        |
 *  |           |        |
 *  |           O ------ O
 *  |       LD              RD
 *  O------------------------------> vx
 *
 *  定义每个轮子的正速度为 vy 方向的速度，故 RU、RD 需要 reverse 一下
 *
 *  v_LU =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_LD =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_RU = -rotate * sqrt(2) + vy - vx * sqrt(2)
 *  v_RD = -rotate * sqrt(2) + vy - vx * sqrt(2)
 */

MotorController LU(std::make_unique <Motor::DJIMotor> (
    "chassis_left_up",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x01, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController LD(std::make_unique <Motor::DJIMotor> (
    "chassis_left_down",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x04, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController RU(std::make_unique <Motor::DJIMotor> (
    "chassis_right_up",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x02, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController RD(std::make_unique <Motor::DJIMotor> (
    "chassis_right_down",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x03, .port = E_CAN2, .mode = Motor::DJIMotor::CURRENT }
    ));

motor_power_init_t motor_3508_power_data(0.65213,(-0.15659),0.00041660,0.00235415,0.20022,1.08e-7,1000);
MotorPower m3508_1_power(motor_3508_power_data);
MotorPower m3508_2_power(motor_3508_power_data);
MotorPower m3508_3_power(motor_3508_power_data);
MotorPower m3508_4_power(motor_3508_power_data);
ChassisPowerManager chassis_(&m3508_1_power, &m3508_2_power, &m3508_3_power, &m3508_4_power);

// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0, v_basic =0,vmax = 0;
// 旋转速度
double rotate = 0;

double motor_target[4] = {0};
uint8_t cap_count = 0,ui_count = 0;
const auto ins = app_ins_data();

LowPassFilter vx_filter(20), vy_filter(20), rotate_filter(20);
static Chassis_cmd_t chassis;
static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}
void chassis_update_handle(){
    vx = chassis.cmd_vx,vy = chassis.cmd_vy;
    rotate = chassis.cmd_rotate_1 + chassis.cmd_rotate_2;
    auto theta = std::atan2(vy, vx), r = std::sqrt((vx * vx) + (vy * vy));
    theta -= ((yaw_zero_pos - static_cast <int16_t> (app_gimbal_data()->yaw_angle) + 8192) % 8192) * M_PI / 4096;
    //        theta -= (4096 - static_cast <int16_t>(read_yaw_angle()))
    vx = r * std::cos(theta);
    vy = r * std::sin(theta);

    motor_target[0] = rotate + vy * M_SQRT2 + vx * M_SQRT2;
    motor_target[1] = rotate - vy * M_SQRT2 + vx * M_SQRT2;
    motor_target[2] = rotate - vy * M_SQRT2 - vx * M_SQRT2;
    motor_target[3] = rotate + vy * M_SQRT2 - vx * M_SQRT2;
    LU.update(motor_target[0]) ;
    RU.update(motor_target[1]) ;
    RD.update(motor_target[2]) ;
    LD.update(motor_target[3]) ;
}

void chassis_powerctrl_handle(){
    chassis_.updateMotorError(0, motor_target[0] - static_cast<float>(LU.device()->speed));
    chassis_.updateMotorError(1, motor_target[1] - static_cast<float>(RU.device()->speed));
    chassis_.updateMotorError(2, motor_target[2] - static_cast<float>(RD.device()->speed));
    chassis_.updateMotorError(3, motor_target[3] - static_cast<float>(LD.device()->speed));
    chassis_.allocatePower(120);

    m3508_1_power.limiter(&LU.output,LU.device()->speed,m3508_1_power.power_limit);
    m3508_2_power.limiter(&RU.output,RU.device()->speed,m3508_2_power.power_limit);
    m3508_3_power.limiter(&RD.output,RD.device()->speed,m3508_3_power.power_limit);
    m3508_4_power.limiter(&LD.output,LD.device()->speed,m3508_4_power.power_limit);
    LU.send_output(LU.output) ;
    RU.send_output(RU.output) ;
    RD.send_output(RD.output) ;
    LD.send_output(LD.output) ;
}
Chassis_cmd_t *app_chassis_data(){
    return &chassis;
}
// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    while(true) {
        chassis_update_handle();
        chassis_powerctrl_handle();

//        //超级电容
//        if(++cap_count == 50) {
//            if(bsp_time_get_ms() - ->timestamp < 500)
//                CAP::send(referee->robot_status.chassis_power_limit);
//            else
//                CAP::send(70);
//            cap_count = 0;
//        }



//        //UI界面
//        if(++ui_count >= 50) {
//            ui_count = 50;
//            // referee_ui.ui_pit = 0.01f*bsp_rng_random(0,100)*bsp_rng_random(-40,40);
//            referee_ui.ui_pit = ins->roll;
////            referee_ui.s_sum  = read_trigger_angle()/60;
//            referee_ui.En     = CAP::data()->cap_percent;
//            referee_ui.cis = ((yaw_zero_pos - static_cast<int16_t>(app_gimbal_data()->yaw_angle) + 8192) % 8192) * 360 / 8192;
//
//            referee_ui_.rotate = chassis.cmd_rotate;
////            if(key_c.key.b)referee_ui_.ui_rst ^=1;
//            ui_reset(referee_ui_.ui_rst);
////            if(referee->robot_status.current_HP == 0 )status = 1 ;
////            referee_ui_.ui_die = status;
////            referee_ui_.ui_shoot =
//            app_ui_dot_update(&referee_ui, &referee_ui_);
//            app_ui_task(&referee_ui);
//        }
        OS::Task::SleepMilliseconds(1);
    }
}

void app_chassis_init() {
    CAP::init();
LU.add_controller(std::make_unique <Controller::MotorBasePID> (
    Controller::MotorBasePID::PID_SPEED,
    std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 16384, 1000),
    nullptr
    ));
LD.add_controller(std::make_unique <Controller::MotorBasePID> (
    Controller::MotorBasePID::PID_SPEED,
    std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 16384, 1000),
    nullptr
    ));
RU.add_controller(std::make_unique <Controller::MotorBasePID> (
    Controller::MotorBasePID::PID_SPEED,
    std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 16384, 1000),
    nullptr
    ));

RD.add_controller(std::make_unique <Controller::MotorBasePID> (
    Controller::MotorBasePID::PID_SPEED,
    std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 16384, 1000),
    nullptr
    ));

LU.init(); LD.init(); RU.init(); RD.init();
//     LU.relax(); LD.relax();  RD.relax();RU.relax();
}

#endif