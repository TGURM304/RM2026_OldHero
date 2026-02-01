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
#include "app_total_cmd.h"
#ifdef COMPILE_GIMBAL
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
MotorController trigger(std::make_unique <Motor::DJIMotor> (
    "trigger",
    Motor::DJIMotor::M3508,
    (Motor::DJIMotor::Param) { .id = 0x03, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT }
    ));

static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}
const auto ins = app_ins_data();
const auto referee = app_referee_data();

static Gimbal_cmd_t gimbal;
Gimbal_cmd_t *app_gimbal_data(){
    return &gimbal;
}
float target = 0, yaw_lst_angle = 0, yaw_sum_angle = 0;
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

void shoot_on_handle(double speed){
    shoot_left.update(speed );
    shoot_right.update(-speed);
}
void shoot_off_handle(){
    shoot_left.update(0 );
    shoot_right.update(0);
    shoot_left.clear();
    shoot_right.clear();
}
void gimbal_safe_handle(){
    yaw.update(ins->yaw);
    pit.update(ins->roll);
    shoot_off_handle();
}
void motor_status_update_handle(){
    yaw_sum_angle += calc_delta(360, yaw_lst_angle, ins->yaw);
    yaw_lst_angle = ins->yaw;

    gimbal.yaw_angle = yaw.angle;
    gimbal.pit_angle = pit.angle;
    gimbal.sl_current = shoot_left.current,gimbal.sr_current = shoot_right.current;
    gimbal.trigger_current = trigger.current;
}
void gimbal_update_handle(){
    yaw.update(gimbal.yaw_target);
    pit.update(gimbal.pit_target);
    trigger.update(gimbal.trigger_target);
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())

    OS::Task::SleepMilliseconds(500);

    while(true) {
        ctrl.update();

        if(ctrl.mainState == main_state::SAFE)gimbal_safe_handle();

        if(ctrl.shootState == shoot_state::SHOOT_ON) shoot_on_handle(6000);
        else if(ctrl.shootState == shoot_state::SHOOT_OFF)shoot_off_handle();
        motor_status_update_handle();
        gimbal_update_handle();


            OS::Task::SleepMilliseconds(1);
        }
    }


void app_gimbal_init() {
    yaw.init(); pit.init();
    shoot_left.init();shoot_right.init();
    trigger.init();
    trigger.use_extend_angle = true;trigger.use_degree_angle = true;
    trigger.use_stall_detect = true;
//    shoot_left.relax();shoot_right.relax();
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
    trigger.add_controller(std::make_unique <Controller::MotorBasePID> (
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