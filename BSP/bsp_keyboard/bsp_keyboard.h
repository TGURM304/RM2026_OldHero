//
// Created by 郭巴 on 2025/3/12.
//

#ifndef RM_FRAMEWORK_F407_BSP_KEYBOARD_H
#define RM_FRAMEWORK_F407_BSP_KEYBOARD_H
#include <cstdint>
#include "bsp_rc.h"

typedef struct bsp_keyboard_t
{
    uint8_t w = 0;
    uint8_t s = 0;
    uint8_t a = 0;
    uint8_t d = 0;
    uint8_t shift = 0;
    uint8_t ctrl = 0;
    uint8_t q = 0;
    uint8_t e = 0;
    uint8_t r = 0;
    uint8_t f = 0;
    uint8_t g = 0;
    uint8_t z = 0;
    uint8_t x = 0;
    uint8_t c = 0;
    uint8_t v = 0;
    uint8_t b = 0;
    struct {
        bool lst_state;
        bool key_state;
    }k_state;
} bsp_keyboard_t;

void read_key_state(bsp_keyboard_t *key,uint16_t data);
uint8_t update_key_state(uint8_t key);
#endif
