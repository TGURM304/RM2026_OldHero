//
// Created by 郭巴 on 2026/1/30.
//

#include <cstring>
#include <algorithm>
#include <cmath>
#include "app_total_cmd.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "bsp_time.h"
#include "dev_cap.h"
#include "robomaster.h"
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

    chassis_judge();
    chassis_update();
    gimbal_update();
    shoot_judge();
    shoot_update();
    data_update();

    //三组合键bsp_sys_reset

    if(++ui_count >=100) {
        ui_update();
        ui_count = 0;
    }


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
        chassis_mode_cnt++;
        if(chassis_mode_cnt > 2) chassis_mode_cnt = 0;
    }
    
    // shift 键切换跟随状态（上升沿检测）
    if(pres.k.key.shift) {
        follow_state = !follow_state;
    }
    
    if(follow_state) {
        chassisState = chassis_state::FOLLOW;
    }
    else {
        switch(chassis_mode_cnt) {
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
            chassis_mode_cnt = 0;
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
        chassis->cmd_rotate_1 = 5*rc->reserved;
    }
    else{
        //底盘速度的控制模型为匀加速直线运动 v = v0+at
        double input_x = 1.0*(key_g.key.d  - key_g.key.a),input_y = 1.0*(key_g.key.w  - key_g.key.s);

        chassis->v_basic = 600 ;//v_basic应该根据等级和功率实测
        chassis->dx += input_x * chassis->cmd_acc;
        chassis->cmd_vx = chassis->v_basic * input_x + chassis->dx;
        if(input_x == 0){
            chassis->cmd_vx *= 0.3;
            chassis->dx = 0;
            if(abs((int)chassis->cmd_vx) < chassis->v_basic)chassis->cmd_vx = 0;
        }
        chassis->cmd_vx = std::clamp((float)chassis->cmd_vx,-4000.f,4000.f);

        chassis->dy += input_y * chassis->cmd_acc;
        chassis->cmd_vy = chassis->v_basic * input_y + chassis->dy;
        if(input_y == 0){
            chassis->cmd_vy *= 0.3;
            chassis->dy = 0;
            if(abs((int)chassis->cmd_vy) < chassis->v_basic)chassis->cmd_vy = 0;
        }
        chassis->cmd_vy = std::clamp((float)chassis->cmd_vy,-4000.f,4000.f);
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
            chassis->cmd_rotate_2 = 3000;
            break;
        case chassis_state::TOP_SPEED2:
            chassis->cmd_rotate_2 = 5000;
            break;
        case chassis_state::FOLLOW:
            chassis->cmd_rotate_1 = 0;
            const int ANGLE_ZERO = 252 ;
            const int ANGLE_MAX = 9050;
            uint16_t angle= 0;float angle_ = 0 ;
            angle = ((uint16_t)gimbal->yaw_angle - ANGLE_ZERO +8192) % 8192;
            angle_ = angle * 360.0f/8192.0f;
            if(angle_>180){
                angle_-=360.0f;
            }
            if(fabs(angle_) < 5
               || fabs(angle_) > 175
               || (fabs(angle_) > 85 && fabs(angle_) < 95))
            {
                angle_ = 0;
            }
            if(abs((int)angle_)>90)angle_ = angle_ > 0 ? angle_ - 180.0f : angle_ + 180.0f;//优劣弧
            chassis->cmd_rotate_2 =  -0.012 * angle_ * angle_ * angle_ ;
            break;
        }
    }
}
void Mode_control::gimbal_update() {

    if(inputSource == input_source::RC_REMOTE){
        gimbal->yaw_target_raw -= static_cast<float>(1.0*rc->rc_r[0] * 0.001f);
        gimbal->pit_target_raw += static_cast<float>(1.0*rc->rc_r[1] * 0.001f);
        gimbal->pit_target_raw = std::clamp(gimbal->pit_target_raw, -18.f, 20.f); //限幅
    }
    else{
        int16_t mouse_x_lim = std::clamp(pres.mouse_x,(int16_t )-1000,(int16_t)1000);
        gimbal->yaw_target_raw -= static_cast<float>(1.0 * mouse_x_lim) * 0.0025f;
        gimbal->pit_target_raw += static_cast<float>(1.0 * pres.mouse_y) * 0.001f;
        gimbal->pit_target_raw = std::clamp(gimbal->pit_target_raw, -18.f, 20.f); //限幅
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
void Mode_control::data_update() {
    chassis->ui.shoot_flag = shoot_launched;
}
void Mode_control::ui_init() {
    // basic::ui::add_float("a", 0, 0, 5, 700, 200, 30, static_cast<float>(rotate));
    // basic::ui::add_int("b", 0, 1, 5, 700, 400, 30, CAP::data()->cap_percent);
    // basic::ui::add_int("c", 0, 2, 5, 700, 600, 30, CAP::data()->limit_power );
    // basic::ui::add_int("d", 0, 3, 5, 700, 800, 30, 114514);
    // basic::ui::add_int("e", 0, 4, 5, 1000, 200, 30, 114514);
    // basic::ui::add_int("f", 0, 5, 5, 1000, 400, 30, 114514);
    // basic::ui::add_int("g", 0, 6, 5, 1000, 600, 30, 114514);

    robomaster::basic::ui::add_line("L1", 0, 2, 3, 1180, 493, 730, 493);
    robomaster::basic::ui::add_line("L2", 0, 2, 3, 1180, 393, 730, 393);
    robomaster::basic::ui::add_line("L3", 0, 2, 3, 1180, 293, 730, 293);
    robomaster::basic::ui::add_line("L4", 0, 2, 3, 1180, 193, 730, 193);

    robomaster::basic::ui::add_string("a2", 0, 6, 2, 44, 866, 20, "rotate");            //Rotate
    robomaster::basic::ui::add_string("b2", 0, 6, 2, 44, 826, 20, "trigger");   //ShootState
    robomaster::basic::ui::add_string("c2", 0, 6, 2, 44, 786, 20, "cap");//CapPercent
    robomaster::basic::ui::add_string("d2", 0, 6, 2, 44, 746, 20, "pitch");//pit_target

    robomaster::basic::ui::add_float("a1", 0, 6, 2, 244, 866, 20, static_cast<float>(chassis->cmd_rotate_1) + static_cast<float>(chassis->cmd_rotate_2));            //Rotate
    robomaster::basic::ui::add_float("b1", 0, 6, 2, 244, 826, 20, gimbal->trigger_target);   //ShootState
    robomaster::basic::ui::add_float("c1", 0, 6, 2, 244, 786, 20, CAP::data()->cap_percent);//CapPercent
    robomaster::basic::ui::add_float("d1", 0, 6, 2, 244, 746, 20, gimbal->pit_target);//pit_target



    // basic::ui::add_float("d4", 0, 6, 2, 14, 746, 20, "");//Towards

    // basic::ui::add_string("SUM", 0, 2, 3, 1490, 866, 30, "SUM");
    // basic::ui::add_arc("E_l", 0 ,8, 4, 958, 540, 229, 311, 390, 400);
    // basic::ui::add_arc("E_r", 0 ,8, 4, 973, 540, 231, 309, 390, 400);
    // basic::ui::add_circle("Gim", 0, 5, 8, 440, 800, 35);
    //
    // basic::ui::add_arc("En", 1 ,2, 12, 967, 540, 230, 310, 391, 400);
    // basic::ui::add_arc("Cis", 1 ,6, 10, 440, 800, 35, 325, 50, 50);
    // basic::ui::add_arc("ht0", 2 ,8, 1, 60, 40, 1, 10, 10, 10);
    // basic::ui::add_arc("ht1", 2 ,8, 1, 60, 40, 1, 10, 10, 10);
    // basic::ui::add_arc("ht2", 2 ,8, 1, 60, 40, 1, 10, 10, 10);
    // basic::ui::add_arc("ht3", 2 ,8, 1, 60, 40, 1, 10, 10, 10);
    //
    // basic::ui::add_circle("seS", 1, 8, 1, 165, 852, 9);
    // basic::ui::add_circle("sin", 1, 8, 1, 165, 812, 9);
    // basic::ui::add_circle("vs", 1, 8, 1, 165, 772, 9);
    //
    // basic::ui::add_circle("Vis", 1, 8, 1, 810, 130, 9);
    // basic::ui::add_circle("Sho", 1, 8, 1, 910, 130, 9);
    // basic::ui::add_circle("Spi", 1, 8, 1, 1010, 130, 9);
    // basic::ui::add_circle("Die", 1, 8, 1, 1110, 130, 9);
    // basic::ui::add_float("pit", 1, 2, 3, 1350, 670, 30, 0.1);

}
void Mode_control::ui_update() {

    if(key_g.key.r == 1) {
        ui_init();
    }

    robomaster::basic::ui::update_float("a1", 0, 6, 2, 244, 866, 20, static_cast<float>(chassis->cmd_rotate_1) + static_cast<float>(chassis->cmd_rotate_2));            //Rotate
    robomaster::basic::ui::update_float("b1", 0, 6, 2, 244, 826, 20, gimbal->trigger_target);   //ShootState
    robomaster::basic::ui::update_float("c1", 0, 6, 2, 244, 786, 20, CAP::data()->cap_percent);//CapPercent
    robomaster::basic::ui::update_float("d1", 0, 6, 2, 244, 746, 20, gimbal->pit_target);//pit_target
}