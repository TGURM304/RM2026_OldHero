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
#include "dev_cap.h"
#ifdef COMPILE_CHASSIS
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
// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0;
// 旋转速度
double rotate = 0;

const auto rc = bsp_rc_data();
const auto ins = app_ins_data();

constexpr int16_t yaw_zero_position = 7076;
// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    while(true) {
        OS::Task::SleepMilliseconds(1);
        vx = 1.0 * rc->rc_l[0] * 3, vy = 1.0 * rc->rc_l[1] * 3;
        if(rc->s_l == 1) rotate = 2000;
        else if(rc->s_l == -1) rotate = -2000;
        else rotate = 0;

        auto theta = std::atan2(vy, vx), r = std::sqrt((vx * vx) + (vy * vy));
        // theta -= (ins->yaw - gimbal.yaw_ins) / 180.0 * M_PI;
        theta -= ((yaw_zero_position - static_cast <int16_t> (read_yaw_angle()) + 8192) % 8192) * M_PI / 4096;
        vx = r * std::cos(theta), vy = r * std::sin(theta);

        LU.update(rotate + vy * M_SQRT2 + vx * M_SQRT2) ;
        RD.update(rotate - vy * M_SQRT2 - vx * M_SQRT2) ;
        LD.update(rotate + vy * M_SQRT2 - vx * M_SQRT2) ;
        RU.update(rotate - vy * M_SQRT2 + vx * M_SQRT2) ;
    }
}

void app_chassis_init() {
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
//    LU.relax(); LD.relax(); RU.relax(); RD.relax();
//CAP::init();//真的有吗？
}

#endif