//
// Created by fish on 2024/11/16.
//

#pragma once

#include <cstdint>
#include "app_conf.h"

#ifdef __cplusplus
extern "C" {
#endif
 struct Gimbal_cmd_t{
    float yaw_target=0,pit_target=0;
    float yaw_target_raw=0,pit_target_raw=0;
    bool shoot_flag = false;
    float trigger_target = 0;
    float yaw_angle,pit_angle;
    float trigger_current,sl_current,sr_current;//拨弹盘电流，左右摩擦轮电流
     typedef struct {
         bool shoot = false;
     }ui;
};
/*!
 * 云台初始化 (随系统初始化调用)
 */

void app_gimbal_init();
float read_yaw_angle();
float read_trigger_angle();
bool return_shoot_status();
Gimbal_cmd_t *app_gimbal_data();
/*!
 * 云台任务
 * @param args RTOS 任务参数
 */
void app_gimbal_task(void *args);

#ifdef __cplusplus
}
#endif