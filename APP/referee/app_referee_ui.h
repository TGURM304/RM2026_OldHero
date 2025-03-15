//
// Created by 郭巴 on 2025/3/15.
//

#pragma once

#include <bitset>
#include <cstdint>
#include "bsp_rc.h"
#include "app_chassis.h"

//ser_state,vison_state,shoot,spin,d
struct app_ui_data_t {
    std::bitset<5> dot;
    std::bitset<5> lst_dot;
    std::bitset<5> diff_dot;

    uint8_t En = 0;
    uint16_t cis = 0;
    int16_t s_sum = 0;
    float ui_pit = 0;
    uint32_t timesetp;
};

struct app_ui_dot_t{
    bool ui_rst = bsp_rc_data()->keyboard.key.b;
    bool ui_shoot = 1;
    bool ui_die = 1;
    double rotate = read_rotate();

};
void app_ui_add_init();
void app_ui_dot_update(app_ui_data_t *ui, const app_ui_dot_t* ui_);
void ui_reset(bool reset);

void app_ui_task(app_ui_data_t *ui);
