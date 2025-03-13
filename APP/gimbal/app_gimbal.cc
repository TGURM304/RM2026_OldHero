//
// Created by fish on 2024/11/17.
//

#include <cmath>
#include "app_gimbal.h"
#include "app_sys.h"
#include "sys_task.h"
#include "bsp_can.h"
#include "app_motor.h"
#include "dev_motor_dji.h"
#include "ctrl_motor_base_pid.h"
#include "bsp_rc.h"
#include "app_ins.h"
#include "bsp_uart.h"
#include "app_msg.h"
#include "ctrl_low_pass_filter.h"
#include "bsp_time.h"
#include "bsp_keyboard.h"
#ifdef COMPILE_GIMBAL

//@Todo:单发射击限位 键盘检测控制 图传模块 PID调参
using namespace Motor;
using namespace Controller;

/* Yaw & Pitch */
MotorController yaw(std::make_unique <DJIMotor> (
    "yaw", DJIMotor::GM6020, (DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::VOLTAGE }
));
MotorController pit(std::make_unique <DJIMotor> (
    "pit", DJIMotor::GM6020, (DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::CURRENT }
));
MotorController shoot_left(std::make_unique <Motor::DJIMotor> (
    "shoot_left",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x02, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController shoot_right(std::make_unique <Motor::DJIMotor> (
    "shoot_right",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x02, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController Bullet_supply(std::make_unique <Motor::DJIMotor> (
    "Bullet_supply",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x01, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));
uint32_t read_yaw_angle(){
    return yaw.angle;
}
static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}

const auto rc = bsp_rc_data();
const auto ins = app_ins_data();
bsp_keyboard_t gimbal_keyboard;
float target = 0, yaw_lst_angle = 0, yaw_sum_angle = 0, yaw_target = 0,pit_target = 0;
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(500);
    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    while(true) {
        yaw_sum_angle = calc_delta(360, yaw_lst_angle, ins->yaw);
        yaw_lst_angle = ins->yaw;

        //遥控器离线检测
        if(bsp_time_get_ms() - rc->timestamp > 100) {
            pit.clear();
            pit.update(0);
            yaw.clear();
            yaw.update(yaw_target);
            OS::Task::SleepMilliseconds(1);
            continue;
        }
        if(rc->s_r==RC_CONTROL){
        yaw_target = (rc->rc_r[0] * 0.1f);
        pit_target += (rc->rc_r[1] * 0.2f);
        if(rc->s_l == SHOOT){
            shoot_left.update(6000);shoot_right.update(-6000);
            Bullet_supply.update(60);
            continue;
        }
        }
        if(rc->s_r == KEYBOARD_CONTROL){
            yaw_target -= static_cast <float> (rc->mouse_x) * 0.0020f;
            pit_target -= static_cast <float> (rc->mouse_y) * 0.0012f;

            switch(update_key_state(gimbal_keyboard.f)%2){
            case 0:shoot_right.update(0);shoot_left.update(0);
                break;
            case 1:shoot_left.update(6000);shoot_right.update(-6000);
                break;
            default:shoot_left.clear();shoot_right.clear();
                break;
            }
        }
        std::clamp(pit_target,-15.f,20.f);//限幅

        yaw.update(yaw_target);
        pit.update(pit_target);


        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    yaw.init(); pit.init();
    shoot_left.init();shoot_right.init();
    Bullet_supply.init();
    /*Yaw PID 先为位置环后为速度环*/
    yaw.add_controller(
        [](const auto x) -> double { return yaw_sum_angle; },
        std::make_unique <PID> (10, 0, 0, 360, 0)
    );
    yaw.add_controller(
        [](const auto x) -> double { return ins->raw.gyro[2] / M_PI * 180; },
        std::make_unique <PID> (60, 0.8, 0.0, 25000, 20000)
    );


    /*Pit PID 先为位置环后为速度环*/
    pit.add_controller(
        [](const auto x) -> double { return ins->roll; },
        std::make_unique <PID> (10, 0, 0.0, 45, 0)
    );

    pit.add_controller(
    [](const auto x) -> double { return ins->raw.gyro[0]/ M_PI * 180; },
    std::make_unique <PID> (100, 0.0, 0.05, 25000, 5000));

    shoot_left.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 25000, 5000),
        nullptr
        ));
    shoot_right.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 25000, 5000),
        nullptr
        ));
    Bullet_supply.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED|Controller::MotorBasePID::PID_ANGLE,
        std::make_unique <Controller::PID> (14.5, 0.08, 0.03, 25000, 5000),
        std::make_unique <Controller::PID> (10.0, 0.08, 0.03, 16384, 1000)
        ));
    /*低通滤波*/
    yaw.add_controller(
        std::make_unique <LowPassFilter> (50, 0.001)
    );
    pit.add_controller(
        std::make_unique <LowPassFilter> (50, 0.001)
    );
}
//    pit.relax();
//        yaw.relax();
#endif