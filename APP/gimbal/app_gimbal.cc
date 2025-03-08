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


#ifdef COMPILE_GIMBAL
using namespace Motor;
using namespace Controller;

MotorController yaw(std::make_unique <DJIMotor> (
    "yaw",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x05, E_CAN1, DJIMotor::VOLTAGE }
    ));

MotorController pit(std::make_unique <DJIMotor> (
    "pit",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x06, E_CAN1, DJIMotor::VOLTAGE }
    ));
MotorController shoot_left(std::make_unique <DJIMotor> (
    "shoot_left",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::VOLTAGE }
    ));
MotorController shoot_right(std::make_unique <DJIMotor> (
    "shoot_right",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x03, E_CAN1, DJIMotor::VOLTAGE }
    ));
MotorController Bullet_supply(std::make_unique <DJIMotor> (
    "Bullet_supply",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::VOLTAGE }
    ));
static double degree_delta(double current, double target) {
    double dt = target - current;
    // 0 - 359
    if(dt >  180) dt -= 360;
    if(dt < -180) dt += 360;
    return dt;
}

const auto rc = bsp_rc_data();
const auto ins = app_ins_data();
double yaw_target = 0, pit_target = 0;
double yaw_output = 0, pit_output = 0;
double yaw_lst_angle = 0;
double yaw_sum_angle = 0;
uint8_t state = 0;
int32_t last_angle=0;
int32_t now_angle=0;

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%lf", &yaw_target);
}

//读取yaw轴角度接口函数
uint32_t read_yaw_angle(){
    uint32_t yaw_motor_angle = yaw.angle;
    return yaw_motor_angle;
}
// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    yaw_target = ins->yaw;
    pit_target = 0;

    OS::Task::SleepMilliseconds(500);
    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    while(true) {
        OS::Task::SleepMilliseconds(1);
        yaw_target = (rc->rc_r[0] * 0.1);
        pit_target += (rc->rc_r[1] * 0.001);

        //        yaw_sum_angle += degree_delta(yaw_lst_angle, ins->yaw);
        //        yaw_lst_angle = ins->yaw;

        /* Yaw Control */
        yaw_output = yaw_target;
        yaw.update(degree_delta(ins->raw.gyro[2] * 180.0 / M_PI,yaw_output));
        yaw.update(static_cast <float> (yaw_target) * 1);
        Bullet_supply.use_extend_angle = 1;
        //Bullet_supply.use_degree_angle=1;
        /* Pitch Control */
        if (pit_target >= 35)pit_target = 35;
        if (pit_target <= -25)pit_target = -25;//软限位

        pit.update(degree_delta(ins->roll, pit_output));
        pit.update(ins->raw.gyro[0] * 180.0 / M_PI);
        pit.update(static_cast <float> (pit_target) * 1);

        shoot_left.update(6000),
        shoot_right.update(-6000);
        if(rc->s_r==1){
            Bullet_supply.update(-600);
        }
        else  {
            Bullet_supply.update(0);}
        if(rc->s_r==-1){
            Bullet_supply.update(-600);
        }//调试射击时退弹，以防危险
        app_msg_vofa_send(E_UART_DEBUG, {
                                            yaw.angle,
                                            pit.angle,
                                        });
    }
}

void app_gimbal_init() {
//    yaw.add_controller(std::make_unique <Controller::MotorBasePID> (
//        Controller::MotorBasePID::PID_SPEED|Controller::MotorBasePID::PID_ANGLE,
//        std::make_unique <Controller::PID> (60, 1, 0.03, 16384, 1000),
//        std::make_unique <Controller::PID> (7, 0, 0, 16384, 1000)
//        ));
//    pit.add_controller(std::make_unique <Controller::MotorBasePID> (
//        Controller::MotorBasePID::PID_SPEED|Controller::MotorBasePID::PID_ANGLE,
//        std::make_unique <Controller::PID> (45, 0.08, 0.03, 16384, 1000),
//        std::make_unique <Controller::PID> (10, 0, 0, 16384, 1000)
//        ));
    shoot_left.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (10, 0.08, 0.03, 16384, 1000),
        nullptr
            ));
    shoot_right.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (10, 0.08, 0.03, 16384, 1000),
        nullptr
            ));
    Bullet_supply.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED|Controller::MotorBasePID::PID_ANGLE,
        std::make_unique <Controller::PID> (15, 0.08, 0.03, 16384, 1000),
        std::make_unique <Controller::PID> (10, 0, 0, 16384, 1000)
            ));
}

#endif