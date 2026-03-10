//
// Created by 郭巴 on 2026/1/30.
//

#include <cstring>
#include <algorithm>
#include <cmath>
#include "app_total_cmd.h"
#include "bsp_rc.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "bsp_time.h"
#include "robomaster.h"
#include "app_second_order.h"
const auto rc = bsp_rc_data();
auto referee = robomaster::image::rc::data();
auto gimbal = app_gimbal_data();
auto chassis = app_chassis_data();
bsp_rc_keyboard_u lst_keyboard_,now_keyboard_,key_g;
uint8_t lst = 0,now = 0 ,lst_ = 0,now_ = 0;
uint8_t lst_s = 0,now_s = 0;
Mode_control ctrl;
void Mode_control::update() {
    main_judge();
    source_judge();
    keyboard_update();  // 先更新键盘状态
    chassis_judge();    // 再判断底盘状态（使用最新的按键状态）
    chassis_update();
    gimbal_update();
    shoot_judge();
    shoot_update();
}
void Mode_control:: main_judge(){
    bool rc_oline = (bsp_time_get_ms() - rc->time_stamp < 100);
    if(!rc_oline)mainState = main_state::SAFE;
    else mainState = main_state::ACTIVE;
}
void Mode_control::source_judge(){
    switch(rc->s_r) {
    case 1:
        inputSource = input_source::RC_KEYBOARD;
        break;
    case 0:
        inputSource = input_source::RC_REMOTE;
        break;
    case -1:
        inputSource = input_source::REFEREE_KEYBOARD;
        break;
    }
}
void Mode_control::shoot_judge() {
    if(inputSource == input_source::RC_REMOTE){
        if(rc->s_l == -1 || mainState == main_state::SAFE)//左拨杆拨到中间
            shootState = shoot_state::SHOOT_OFF;
        else shootState =   shoot_state::SHOOT_ON;
    }
    else {
        if(pres.k.key.f)shoot_launched = !shoot_launched;
        if(shoot_launched)shootState = shoot_state::SHOOT_OFF;
        else shootState = shoot_state::SHOOT_ON;
    }
}

void Mode_control::chassis_judge() {
    // 只有在键鼠控制才涉及到底盘的状态切换
    // 使用 pres.k.key.ctrl 的上升沿检测（按下瞬间触发）
    if(pres.k.key.ctrl) {
        chassis_mode_count++;
        if(chassis_mode_count > 2) chassis_mode_count = 0;
    }
    
    // shift 键切换跟随状态（上升沿检测）
    if(pres.k.key.shift) {
        follow_state = !follow_state;
    }
    
    if(follow_state) {
        chassisState = chassis_state::FOLLOW;
    }
    else {
        switch(chassis_mode_count) {
        case 0:
            chassisState = chassis_state::NORMAL;
            break;
        case 1:
            chassisState = chassis_state::TOP_SPEED1;
            break;
        case 2:
            chassisState = chassis_state::TOP_SPEED2;
            break;
        default:
            chassis_mode_count = 0;
            chassisState = chassis_state::NORMAL;
            break;
        }
    }
}
void Mode_control::keyboard_update() {
    lst_keyboard_ = now_keyboard_, now_keyboard_ = key_g,
    pres.k.raw = (now_keyboard_.raw ^ lst_keyboard_.raw) & now_keyboard_.raw;
    switch(inputSource) {
    case input_source::RC_REMOTE:
        break;
    case input_source::RC_KEYBOARD:
        memcpy(&key_g.key, &rc->keyboard.key, sizeof(key_g.key));
        lst        = now;
        now        = rc->mouse_l;
        pres.mouse_l      = (now ^ lst) & now;
        lst_         = now_;
        now_         = rc->mouse_r;
        pres.mouse_r      = (now_ ^ lst_) & now_;
        pres.mouse_x = rc->mouse_x;
        pres.mouse_y = rc->mouse_y;
        break;
    case input_source::REFEREE_KEYBOARD:
        memcpy(&key_g.key, &referee->keyboard, sizeof(key_g.key));
        lst        = now;
        now        = referee->mouse_l;
        pres.mouse_l      = (now ^ lst) & now;
        lst_         = now_;
        now_         = referee->mouse_r;
        pres.mouse_r      = (now_ ^ lst_) & now_;
        pres.mouse_x = referee->mouse_x;
        pres.mouse_y = referee->mouse_y;
        break;
    }
}
void Mode_control::chassis_update() {
    if(inputSource == input_source::RC_REMOTE){
        chassis->cmd_vx = 1.0 * rc->rc_l[0] * 5, chassis->cmd_vy = 1.0 * rc->rc_l[1] * 5;
        chassis->cmd_rotate = 5*rc->reserved;
    }
    else{
        //底盘速度的控制模型为匀加速直线运动 v = v0+at
        double input_x = 1.0*(key_g.key.d  - key_g.key.a),input_y = 1.0*(key_g.key.w  - key_g.key.s);

        chassis->v_basic = 400 ;//v_basic应该根据等级和功率实测
        chassis->dx += input_x * chassis->cmd_acc;
        chassis->cmd_vx = chassis->v_basic * input_x + chassis->dx;
        if(input_x == 0){
            chassis->cmd_vx *= 0.3;
            chassis->dx = 0;
            if(abs((int)chassis->cmd_vx) < chassis->v_basic)chassis->cmd_vx = 0;
        }
        chassis->cmd_vx = std::clamp((float)chassis->cmd_vx,-1000.f,1000.f);

        chassis->dy += input_y * chassis->cmd_acc;
        chassis->cmd_vy = chassis->v_basic * input_y + chassis->dy;
        if(input_y == 0){
            chassis->cmd_vy *= 0.3;
            chassis->dy = 0;
            if(abs((int)chassis->cmd_vy) < chassis->v_basic)chassis->cmd_vy = 0;
        }
        chassis->cmd_vy = std::clamp((float)chassis->cmd_vy,-1000.f,1000.f);
        chassis->cmd_rotate_1 = 1.0* key_g.key.q * 1000 - 1.0* key_g.key.e * 1000;
//        if(!key_g.key.q and !key_g.key.e){
//            chassis->cmd_rotate_1 *= 0.5;//指数衰减
//            if(abs((int)chassis->cmd_rotate_1) < 100)chassis->cmd_rotate_1 = 0;
//        }
        switch(chassisState) {
        case chassis_state::NORMAL:
            chassis->cmd_rotate_2 = 0;
            break;
        case chassis_state::TOP_SPEED1:
            chassis->cmd_rotate_2 = 1000;
            break;
        case chassis_state::TOP_SPEED2:
            chassis->cmd_rotate_2 = 2000;
            break;
        case chassis_state::FOLLOW:
//            chassis->cmd_rotate_1 =  0;
//            const int ANGLE_ZERO =858;
//            const int ANGLE_MAX = 9050;
//            uint16_t angle= 0;float angle_ = 0 ;
//            angle = ((uint16_t)gimbal->yaw_angle - ANGLE_ZERO +8192) % 8192;
//            angle_ = angle*360.0f/8192.0f;
//            if(angle_>180){angle_-=360.0f;}
//            if(abs((int)angle_)<15||abs((int)angle_)>165||(abs((int)angle_)<95&&abs((int)angle_)>85))angle_=0;//四个死区 位于车身四周
//            if(abs((int)angle_)>90)angle_ = angle_>0?angle_-180.0f:angle_+180.0f;//优劣弧
//            chassis->cmd_rotate_2 = angle_ * angle_; //取相差角度的平方作为旋转速度输出
            break;
        }
    }
}
void Mode_control::gimbal_update() {

    if(inputSource == input_source::RC_REMOTE){
        gimbal->yaw_target_raw -= static_cast<float>(1.0*rc->rc_r[0] * 0.001f);
        gimbal->pit_target_raw += static_cast<float>(1.0*rc->rc_r[1] * 0.001f);
        gimbal->pit_target_raw = std::clamp(gimbal->pit_target_raw, -15.f, 20.f); //限幅
    }
    else{
        gimbal->yaw_target_raw -= static_cast<float>(1.0 * pres.mouse_x) * 0.020f;
        gimbal->pit_target_raw += static_cast<float>(1.0 * pres.mouse_y) * 0.05f;
        gimbal->pit_target_raw = std::clamp(gimbal->pit_target_raw, -15.f, 20.f); //限幅
    }

}

void Mode_control::shoot_update() {
    if(inputSource == input_source::RC_REMOTE and rc->s_l != -1){
        lst_s        = now_s;
        now_s        = rc->s_l;
        if((now_s ^ lst_s) & now_s)gimbal->trigger_target -= 60*19;
        else gimbal->trigger_target -= 0;
    }
        else if(inputSource == input_source::RC_KEYBOARD or inputSource == input_source::REFEREE_KEYBOARD)
    {
            if(pres.mouse_l)gimbal->trigger_target -= (60*19);
            else gimbal->trigger_target -= 0;
        }

}