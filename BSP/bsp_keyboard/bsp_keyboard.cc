//
// Created by 郭巴 on 2025/3/12.
//

#include <cstring>
#include "bsp_keyboard.h"
#include "bsp_rc.h"
const auto rc = bsp_rc_data();
void read_key_state(bsp_keyboard_t key,bsp_rc_data_t data){
    key.w = (data.keyboard>>0)&1; key.s = (data.keyboard>>1)&1;
    key.a = (data.keyboard>>2)&1; key.d = (data.keyboard>>3)&1;
    key.shift = (data.keyboard>>4)&1; key.ctrl = (data.keyboard>>5)&1;
    key.q = (data.keyboard>>6)&1; key.e = (data.keyboard>>7)&1;
    key.r = (data.keyboard>>8)&1; key.f = (data.keyboard>>9)&1;
    key.g = (data.keyboard>>10)&1; key.z = (data.keyboard>>11)&1;
    key.x = (data.keyboard>>12)&1; key.c = (data.keyboard>>13)&1;
    key.v = (data.keyboard>>14)&1; key.b = (data.keyboard>>15)&1;
}
