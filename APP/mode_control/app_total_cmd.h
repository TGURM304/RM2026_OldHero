//
// Created by 郭巴 on 2026/1/30.
//

#ifndef RM_FRAMEWORK_F407_APP_TOTAL_CMD_H
#define RM_FRAMEWORK_F407_APP_TOTAL_CMD_H
#include "bsp_rc.h"

enum class chassis_state{
    NORMAL,
    FOLLOW,
    TOP_SPEED1,   //tips：小陀螺模式
    TOP_SPEED2
};
//开火状态机 现在测得拨弹盘正常拨一发弹丸的电流约5800-6200 堵转电流基本稳定10000左右
enum class shoot_state {
    SHOOT_ON,
    SHOOT_OFF,
    SHOOT_ERROR
};
enum class main_state {
    SAFE,
    ACTIVE
};
enum class input_source{
    RC_REMOTE,
    RC_KEYBOARD,
    REFEREE_KEYBOARD
};

class Mode_control{
public:
    void update();
    void ui_init();
    main_state mainState ;
    chassis_state chassisState;
    shoot_state shootState;
    input_source inputSource;
private:
    struct {
        bsp_rc_keyboard_u k;
        int16_t mouse_x, mouse_y;
        uint8_t mouse_l, mouse_r;
    }pres;
    bool shoot_launched = true;
    bool follow_state = false;

    bool referee_online = false;
    uint8_t chassis_mode_cnt = 0;  // 底盘模式切换计数器
    uint8_t ui_count = 0;
    //状态判断层
    void source_judge();
    void main_judge();
    void shoot_judge();
    void chassis_judge();

    //数据更新层
    void shoot_update();
    void gimbal_update();
    void chassis_update();
    void keyboard_update();
    void data_update();
    void ui_update();
};

extern Mode_control ctrl;

#endif //RM_FRAMEWORK_F407_APP_TOTAL_CMD_H
