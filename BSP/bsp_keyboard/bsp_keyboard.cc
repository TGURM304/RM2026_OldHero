//
// Created by 郭巴 on 2025/3/12.
//

#include <cstring>
#include "bsp_keyboard.h"
#include "bsp_rc.h"
bsp_keyboard_t keyboard;


void read_key_state(bsp_keyboard_t *key, uint16_t data) {
    key->w = (uint8_t)(data>>0)&1; key->s = (uint8_t)(data>>1)&1;
    key->a = (uint8_t)(data>>2)&1; key->d = (uint8_t)(data>>3)&1;
    key->shift = (data>>4)&1; key->ctrl = (data>>5)&1;
    key->q = (data>>6)&1; key->e = (data>>7)&1;
    key->r = (data>>8)&1; key->f = (uint8_t)(data>>9)&1;
    key->g = (data>>10)&1; key->z = (data>>11)&1;
    key->x = (data>>12)&1; key->c = (data>>13)&1;
    key->v = (data>>14)&1; key->b = (data>>15)&1;
}

uint8_t update_key_state(uint8_t key) {
    static uint8_t last_state = 0;  // 记录上次按键状态
    uint8_t count = 0;
    if (key && !last_state) {
        count++;
    }
    last_state = key;
    return count;
}
