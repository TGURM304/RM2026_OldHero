//
// Created by 郭巴 on 2026/1/30.
//

#include <cstring>
#include <algorithm>
#include "app_mode_control.h"
#include "bsp_rc.h"
#include "app_referee.h"
#include "app_gimbal.h"
#include "app_chassis.h"
#include "bsp_time.h"
const auto rc = bsp_rc_data();
const auto referee = app_referee_data();
bsp_rc_keyboard_u lst_keyboard_,now_keyboard_,key_g;

void Mode_control::update() {

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
        if(rc->s_l == -1)//左拨杆拨到最下侧
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
    //只有在键鼠控制才涉及到底盘的状态切换
    uint8_t count = 0;
    if(pres.k.key.ctrl)count++;
    if(count>=2)count = 0;
    if(pres.k.key.shift)follow_state = !follow_state;
    if(follow_state){
        chassisState = chassis_state::FOLLOW;
    }
    else {
        switch(count) {
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
            break;
        }
    }
}
void Mode_control::keyboard_update() {
    uint8_t lst = 0,now = 0 ,lst_ = 0,now_ = 0;
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
        memcpy(&key_g.key, &referee->remote_control.keyboard, sizeof(key_g.key));
        lst        = now;
        now        = referee->remote_control.mouse_l;
        pres.mouse_l      = (now ^ lst) & now;
        lst_         = now_;
        now_         = referee->remote_control.mouse_r;
        pres.mouse_r      = (now_ ^ lst_) & now_;
        pres.mouse_x = referee->remote_control.mouse_x;
        pres.mouse_y = referee->remote_control.mouse_y;
        break;
    }
}
void Mode_control::chassis_update() {
    if(inputSource == input_source::RC_REMOTE){
        chassis.vx = 1.0 * rc->rc_l[0] * 3, chassis.vy = 1.0 * rc->rc_l[1] * 3;
        chassis.rotate = 5*rc->reserved;
    }
    else{
        chassis.vx += 1.0 * pres.k.key.d * 10 - 1.0 * pres.k.key.a *10;
        if(!pres.k.key.d && !pres.k.key.a)chassis.vx = 0;
        chassis.vx = std::clamp((float)chassis.vx,-4000.f,4000.f);
        chassis.vy += 1.0 * pres.k.key.w * 10 - 1.0 * pres.k.key.s *10;
        if(!pres.k.key.w && !pres.k.key.s)chassis.vy = 0;
        chassis.vy = std::clamp((float)chassis.vy,-4000.f,4000.f);

        chassis.rotate_1 += 1.0* pres.k.key.q * 10 - 1.0* pres.k.key.e * 10;
        if(!pres.k.key.q and !pres.k.key.e){
            chassis.rotate_1 *= 0.85;//指数衰减
            if(abs((int)chassis.rotate_1)<100)chassis.rotate_1 = 0;
        }
        switch(chassisState) {
        case chassis_state::NORMAL:
            chassis.rotate_2 = 0;
            break;
        case chassis_state::TOP_SPEED1:
            chassis.rotate_2 = 1000;
            break;
        case chassis_state::TOP_SPEED2:
            chassis.rotate_2 = 2000;
            break;
        case chassis_state::FOLLOW:
            chassis.rotate_1 = chassis.rotate_2 = chassis.rotate = 0;
            break;
        }
    }
}
void Mode_control::gimbal_update() {
    if(inputSource == input_source::RC_REMOTE){
        gimbal.yaw_target -= static_cast<float>(1.0*rc->rc_r[0] * 0.001f);
        gimbal.pit_target += static_cast<float>(1.0*rc->rc_r[1] * 0.0012f);
    }
    else{
        gimbal.yaw_target -= static_cast<float>(1.0 * pres.mouse_x) * 0.020f;
        gimbal.pit_target -= static_cast<float>(1.0 * pres.mouse_y) * 0.012f;
    }
    gimbal.pit_target = std::clamp(gimbal.pit_target, -10.f, 25.f); //限幅
}

void Mode_control::shoot_update() {

}