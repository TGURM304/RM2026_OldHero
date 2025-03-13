//
// Created by 郭巴 on 2025/3/12.
//

#ifndef RM_FRAMEWORK_F407_BSP_KEYBOARD_H
#define RM_FRAMEWORK_F407_BSP_KEYBOARD_H
#include <cstdint>
#include "bsp_rc.h"

typedef struct
{
    uint8_t w,a,s,d;
    uint8_t q,e,r,f,g,z,x,c,v,b;
    uint8_t ctrl,shift;
} bsp_keyboard_t;
void read_key_state(bsp_keyboard_t key,bsp_rc_data_t data);
#endif
