//
// Created by fish on 2024/11/17.
//

#include <cmath>
#include <cstring>
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
#include "app_referee.h"
#ifdef COMPILE_GIMBAL

//@Todo:单发射击限位 键盘检测控制 图传模块z PID调参
using namespace Motor;
using namespace Controller;

/* Yaw & Pitch */
MotorController yaw(std::make_unique <DJIMotor> (
    "yaw", DJIMotor::GM6020, (DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::VOLTAGE }
));
MotorController pit(std::make_unique <DJIMotor> (
    "pit", DJIMotor::GM6020, (DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::VOLTAGE }
));
MotorController shoot_left(std::make_unique <Motor::DJIMotor> (
    "shoot_left",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x02, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController shoot_right(std::make_unique <Motor::DJIMotor> (
    "shoot_right",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x01, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));
MotorController Bullet_supply(std::make_unique <Motor::DJIMotor> (
    "Bullet_supply",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x03, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
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
bsp_rc_keyboard_u lst_keyboard_,now_keyboard_,press_key_,key_g;;
const auto rc = bsp_rc_data();
const auto ins = app_ins_data();
const auto referee = app_referee_data();

float target = 0, yaw_lst_angle = 0, yaw_sum_angle = 0, yaw_target = 0,pit_target = 0,shoot_speed = 0;
float yaw_control=0,pit_control=0;
bool shoot_flag = 0;
bool lst_=0,now_=0,pres_=0;
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())

    OS::Task::SleepMilliseconds(500);
    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    Bullet_supply.use_extend_angle = 1;Bullet_supply.use_degree_angle = 1;
    while(true) {
        yaw_sum_angle += calc_delta(360, yaw_lst_angle, ins->yaw);
        yaw_lst_angle = ins->yaw;
        //按键状态检测
        lst_keyboard_ = now_keyboard_, now_keyboard_ = key_g, press_key_.raw = (now_keyboard_.raw ^ lst_keyboard_.raw) & now_keyboard_.raw;
        //遥控器/图传 离线检测
        if(bsp_time_get_ms() - rc->time_stamp > 100 ) {
            pit.clear();
            pit.update(0);
            yaw.clear();
            yaw.update(yaw_target);
            shoot_flag = shoot_speed = 0;
            OS::Task::SleepMilliseconds(1);
            continue;
        }

        if(rc->s_r==RC_CONTROL){
        yaw_target -= (rc->rc_r[0] * 0.001f);
        pit_target += (rc->rc_r[1] * 0.002f);

    }
        if(rc->s_r == KEYBOARD_CONTROL){
            memcpy(&key_g.key,&rc->keyboard.key, sizeof(key_g.key));
            lst_ = now_;now_ = rc->mouse_l;pres_ = (now_ ^lst_)&now_;
            yaw_control = rc->mouse_x;
            yaw_control = rc->mouse_y;
            if(pres_) shoot_speed += 60;
            Bullet_supply.update(1000*rc->mouse_r);

        }
        if(rc->s_r == PICTRANS_CONTROL) {
            memcpy(&key_g.key, &referee->remote_control.keyboard, sizeof(key_g.key));
            lst_ = now_;
            now_ = referee->remote_control.mouse_l;
            pres_ = (now_ ^ lst_) & now_;if(pres_) shoot_speed += 60;
            yaw_control = referee->remote_control.mouse_x;
            yaw_control = referee->remote_control.mouse_y;
            Bullet_supply.update(1000*referee->remote_control.mouse_r);
        }

        yaw_target -= -static_cast <float> (1.0*yaw_control) * 0.0020f;
        pit_target -= -static_cast <float> (1.0*pit_control) * 0.022f;
        pit_target = std::clamp(pit_target,-15.f,25.f);//限幅
        yaw.update(yaw_target);
        pit.update(pit_target);

        app_msg_vofa_send(E_UART_DEBUG,{
            1.0*yaw.angle,
            1.0*yaw.speed,

        });

        //开启摩擦轮
        if(press_key_.key.f ) {
            shoot_flag ^= 1;
        }
        shoot_left.update(6000*shoot_flag);
        shoot_right.update(-6000*shoot_flag);
        Bullet_supply.update(-19*shoot_speed);
        OS::Task::SleepMilliseconds(10);
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
    std::make_unique <PID> (150, 0.1, 0.05, 25000, 5000));

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
        std::make_unique <Controller::PID>(5, 0.0, 0.0, 360, 5000)
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