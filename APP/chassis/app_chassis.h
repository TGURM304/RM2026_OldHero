//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Chassis_cmd_t{
    // 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
    double cmd_vx = 0, cmd_vy = 0, v_basic =0,vmax=0;
    double dx = 0,dy = 0;
    double cmd_acc = 2;
    // 旋转速度
    double cmd_rotate_1 = 0,cmd_rotate_2 = 0;
    double cmd_rotate = cmd_rotate_1 + cmd_rotate_2;
    bool status = true,follow_state = false;
};
/*!
 * 底盘初始化 (随系统初始化调用)
 */
void app_chassis_init();
Chassis_cmd_t *app_chassis_data();
/*!
 * 底盘任务
 * @param args RTOS 任务参数
 */
void app_chassis_task(void *args);
double read_rotate();
#ifdef __cplusplus
}
#endif