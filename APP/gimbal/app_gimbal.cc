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
#include "ctrl_forward_feed.h"
#ifdef COMPILE_GIMBAL

//@Todo：将云台传出的所有数据都整合为一个结构体 并使用结构体指针访问
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
float read_yaw_angle(){
    return yaw.angle;
}
float read_trigger_angle(){
    float trigger_sum_angle = 0;
       trigger_sum_angle +=  Bullet_supply.angle;
    return trigger_sum_angle;
}
bool return_shoot_status(){
    bool status =0;
    if(shoot_left.speed!=0)status=1;
    return status;
};
float map_angle(){
    const int ANGLE_ZERO =858;
    const int ANGLE_MAX = 9050;
    uint16_t angle= 0;float angle_ = 0 ;
    angle = ((uint16_t)yaw.angle - ANGLE_ZERO +8192) % 8192;
    angle_ = angle*360.0f/8192.0f;
    if(angle_>180){angle_-=360.0f;}

    if(abs((int)angle_)<15||abs((int)angle_)>165||(abs((int)angle_)<95&&abs((int)angle_)>85))angle_=0;//四个死区 位于车身四周
    if(abs((int)angle_)>90)angle_ = angle_>0?angle_-180.0f:angle_+180.0f;//优劣弧

    return angle_;
}
static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}
bsp_rc_keyboard_u lst_keyboard_,now_keyboard_,press_key_,key_g;
const auto rc = bsp_rc_data();
const auto ins = app_ins_data();
const auto referee = app_referee_data();

float target = 0, yaw_lst_angle = 0, yaw_sum_angle = 0, yaw_target = 0,pit_target = 0,shoot_speed = 0,return_speed=0;
float yaw_control=0,pit_control=0;
bool shoot_flag = 0;
bool lst_=0,now_=0,pres_l=0;
bool lst =0,now=0,pres_r=0;

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())

    OS::Task::SleepMilliseconds(500);
//    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    Bullet_supply.use_extend_angle = 1;Bullet_supply.use_degree_angle = 1;
    while(true) {
        yaw_sum_angle += calc_delta(360, yaw_lst_angle, ins->yaw);
        yaw_lst_angle = ins->yaw;
        //按键状态检测
        lst_keyboard_ = now_keyboard_, now_keyboard_ = key_g, press_key_.raw = (now_keyboard_.raw ^ lst_keyboard_.raw) & now_keyboard_.raw;
        //遥控器/图传 离线检测
//        if(bsp_time_get_ms() - rc->time_stamp > 100 and bsp_time_get_ms() - referee->timestamp>100 ) {
//            pit.clear();
//            pit.update(0);
//            yaw.clear();
//            yaw.update(yaw_target);
//            shoot_flag = shoot_speed = 0;
//            OS::Task::SleepMilliseconds(1);
//            continue;
//        }

        if(rc->s_r==RC_CONTROL){
            yaw_target -= static_cast<float>(1.0*rc->rc_r[0] * 0.001f);
            pit_target -= static_cast<float>(1.0*rc->rc_r[1] * 0.0012f);
            pit_target = std::clamp(pit_target, -10.f, 25.f); //限幅
            if(rc->s_l==-1){
            shoot_left.update(6000);shoot_right.update(-6000);
            Bullet_supply.update(-19*60*10);
            }
            if(rc->s_l==0){
                shoot_left.update(0);shoot_right.update(0);
                Bullet_supply.update(0);
            }
            yaw.update(yaw_target);pit.update(pit_target);

        }
        //使用键盘控制
        else {
            if(rc->s_r == KEYBOARD_CONTROL) {
                memcpy(&key_g.key, &rc->keyboard.key, sizeof(key_g.key));
                lst_        = now_;
                now_        = rc->mouse_l;
                pres_l      = (now_ ^ lst_) & now_;
                lst         = now;
                now         = rc->mouse_r;
                pres_r      = (now ^ lst) & now;
                yaw_control = rc->mouse_x;
                pit_control = rc->mouse_y;
                if(pres_l) shoot_speed += 60;
                if(pres_r) return_speed += 1000;
                //图传电路
            } else if(rc->s_r == PICTRANS_CONTROL) {
                memcpy(&key_g.key, &referee->remote_control.keyboard, sizeof(key_g.key));
                lst_        = now_;
                now_        = referee->remote_control.mouse_l;
                pres_l      = (now_ ^ lst_) & now_;
                lst         = now;
                now         = referee->remote_control.mouse_r;
                pres_r      = (now ^ lst) & now;
                yaw_control = referee->remote_control.mouse_x;
                pit_control = referee->remote_control.mouse_y;
                if(pres_l) shoot_speed += 60;
                if(pres_r) return_speed += 1000;
            }

            yaw_target -= static_cast<float>(1.0 * yaw_control) * 0.020f;
            pit_target -= static_cast<float>(1.0 * pit_control) * 0.012f;
            pit_target = std::clamp(pit_target, -10.f, 25.f); //限幅
            yaw.update(yaw_target);
            pit.update(pit_target);

            //        app_msg_vofa_send(E_UART_DEBUG, {
            //                                            Bullet_supply.current,
            //                                            Bullet_supply.angle,
            //                                            referee->remote_control.mouse_x*1.0
            //                                        });
            //开启摩擦轮
            if(press_key_.key.f) {
                shoot_flag ^= 1;
            }
            shoot_left.update(6000 * shoot_flag);
            shoot_right.update(-6000 * shoot_flag);
            Bullet_supply.update(-19*shoot_speed+return_speed);
        }
        //堵转保护
        if(bsp_time_get_ms() - Bullet_supply.device()->last_online_time > 100 && Bullet_supply.current>10000){
            Bullet_supply.relax();
        }
        if(bsp_time_get_ms() - shoot_left.device()->last_online_time > 100 && shoot_left.current>10000){
            shoot_left.relax();shoot_right.relax();
            Bullet_supply.relax();
        }
//        Bullet_supply.relax();shoot_left.relax();shoot_right.relax();
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
        std::make_unique <PID> (16, 0, 0, 720, 0)
    );
    yaw.add_controller(
        [](const auto x) -> double { return ins->raw.gyro[2] / M_PI * 180; },
        std::make_unique <PID> (95, 2.5, 0.0, 25000, 20000)
    );

    /*Pit PID 先为位置环后为速度环*/
    pit.add_controller(
        [](const auto x) -> double { return ins->roll; },
        std::make_unique <PID> (14, 0, 0.0, 120, 0)
    );

    pit.add_controller(
    [](const auto x) -> double { return ins->raw.gyro[0]/ M_PI * 180; },
    std::make_unique <PID> (120, 1.5, 0.0, 25000, 5000));

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
        std::make_unique <LowPassFilter> (150, 0.001)
    );
    pit.add_controller(
        std::make_unique <LowPassFilter> (150, 0.001)
    );
}
//    pit.relax();
//        yaw.relax();

#endif