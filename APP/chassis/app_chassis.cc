//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"
#include "app_sys.h"
#include "sys_task.h"


#include "app_chassis.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "app_ins.h"
#include "app_motor.h"
#include "app_msg.h"
#include "app_sys.h"
#include "bsp_rc.h"
#include "sys_task.h"
#include "app_gimbal.h"
#include "dev_motor_dji.h"
#include "ctrl_motor_base_pid.h"
#include "app_referee.h"
#include "dev_cap.h"
#include "bsp_time.h"
#include "app_referee_ui.h"
#include "bsp_rng.h"
#ifdef COMPILE_CHASSIS
//@Todo：底盘跟随云台 完善UI

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
// MotorController RD(std::make_unique <Motor::DJIMotor> (
//     "RD", Motor::DJIMotor::GM6020, (Motor::DJIMotor::Param) { 0x03, E_CAN2, Motor::DJIMotor::CURRENT }
//     ));
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

// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0;
// 旋转速度
double rotate = 0,rotate_1 = 0,rotate_2 = 0;
bool status = 1,follow_state = 0;
uint8_t cap_count = 0,ui_count = 0;
const auto rc = bsp_rc_data();
const auto ins = app_ins_data();
const auto referee = app_referee_data();

constexpr int16_t yaw_zero_position = 9124;

bsp_rc_keyboard_u lst_keyboard,now_keyboard,press_key,key_c;
app_ui_data_t referee_ui;app_ui_dot_t referee_ui_;

static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    app_ui_add_init();
    while(true) {

        //按键状态检测
        lst_keyboard = now_keyboard, now_keyboard = key_c, press_key.raw = (now_keyboard.raw ^ lst_keyboard.raw) & now_keyboard.raw;

        if(rc->s_r==RC_CONTROL) {
            vx = 1.0 * rc->rc_l[0] * 3, vy = 1.0 * rc->rc_l[1] * 3;

        }
            if(rc->s_r==KEYBOARD_CONTROL) {
                memcpy(&key_c.key,&rc->keyboard.key, sizeof(key_c.key));
            }

             if(rc->s_r==PICTRANS_CONTROL) {
                 memcpy(&key_c.key,&referee->remote_control.keyboard, sizeof(key_c.key));
             }

            vx = 1.0 * key_c.key.d * 900 - 1.0 * key_c.key.a * 900;
            vy = 1.0 * key_c.key.w * 900 - 1.0 * key_c.key.s * 900;

            if(press_key.key.ctrl){follow_state^=1;}
            //小陀螺模式
            if(press_key.key.v){
                rotate_1 = 1.0 * key_c.key.q * 1000 - 1.0 * key_c.key.e * 1000;
                rotate_2 = rotate_2 >= 3000 ? 0 : rotate_2 + 1000;
                follow_state = 0;
            }
            //底盘跟随云台
            else if (follow_state) {
                rotate_1 = 0;
                rotate_2 = -map_angle() * 30;
            }
                //            else {
//                rotate_2 = 0;  // 防止残留值影响
//            }
            rotate = rotate_1+rotate_2;
        auto theta = std::atan2(vy, vx), r = std::sqrt((vx * vx) + (vy * vy));
        theta -= ((yaw_zero_position - static_cast <int16_t> (read_yaw_angle()) + 8192) % 8192) * M_PI / 4096;
        vx = r * std::cos(theta);
        vy = r * std::sin(theta);

         app_msg_vofa_send(E_UART_DEBUG, {
             rotate_1,
             rotate_2,
             1.0*map_angle(),
             follow_state*1.0

                                         });
        LU.update(rotate + vy * M_SQRT2 + vx * M_SQRT2) ;
        RD.update(rotate - vy * M_SQRT2 - vx * M_SQRT2) ;
        LD.update(rotate + vy * M_SQRT2 - vx * M_SQRT2) ;
        RU.update(rotate - vy * M_SQRT2 + vx * M_SQRT2) ;

        //超级电容
        if(++cap_count == 50) {
            if(bsp_time_get_ms() - referee->timestamp < 500)
                CAP::send(referee->robot_status.chassis_power_limit);
            else
                CAP::send(40);
            cap_count = 0;
        }
        //UI界面
        if(++ui_count >= 50) {
            ui_count = 50;
            // referee_ui.ui_pit = 0.01f*bsp_rng_random(0,100)*bsp_rng_random(-40,40);
            referee_ui.ui_pit = ins->roll;
            referee_ui.s_sum  = (int16_t)bsp_rng_random(0, 200);
            referee_ui.En     = CAP::data()->cap_percent;
            referee_ui.cis = ((yaw_zero_position - static_cast<int16_t>(read_yaw_angle()) + 8192) % 8192) * 360 / 8192;

            referee_ui_.rotate = rotate;
            if(key_c.key.b)referee_ui_.ui_rst ^=1;
            ui_reset(referee_ui_.ui_rst);
            if(referee->robot_status.current_HP == 0 )status = 1 ;
            referee_ui_.ui_die = status;
            app_ui_dot_update(&referee_ui, &referee_ui_);
            app_ui_task(&referee_ui);
        }
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