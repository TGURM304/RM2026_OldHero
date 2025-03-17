//
// Created by 郭巴 on 2025/3/15.
//

#include "app_referee_ui.h"

#include "app_msg.h"
#include "app_referee.h"
#include "bsp_time.h"

uint8_t count = 0;
uint8_t count1 = 0;

void ui_draw_cap(uint8_t cap) {
    uint8_t c = cap>=70? 2: (cap>=25? 1:3);
    float E_c = 79*cap*0.01f + 231;
    app_referee_ui_upd_arc("En", 1 ,c, 12, 967, 540, 230, static_cast<uint32_t>(E_c), 391, 400);
}

void ui_draw_dot(const char *name, uint8_t tar_color, uint32_t x, uint32_t y, bool state) {
    uint32_t wi = state? 20:1;
    uint32_t co = state? tar_color:8;
    app_referee_ui_upd_circle(name, 1, co, wi, x, y, 9);
}

void app_ui_add_init() {
    app_referee_ui_add_string("str", 0, 0, 3, 1920 / 2 - 250, 1080 / 2, 30, "MAN WHAT CAN I SAY");

    app_referee_ui_add_line("L1", 0, 8, 1, 1160, 540, 760, 540);
    app_referee_ui_add_line("L2", 0, 8, 1, 960, 500, 960, 250);
    app_referee_ui_add_string("ser", 0, 6, 2, 14, 866, 25, "SERVO");
    app_referee_ui_add_string("sum", 0, 2, 3, 1490, 866, 30, "SUM");
    app_referee_ui_add_arc("E_l", 0 ,8, 4, 958, 540, 229, 311, 390, 400);
    app_referee_ui_add_arc("E_r", 0 ,8, 4, 973, 540, 231, 309, 390, 400);
    app_referee_ui_add_circle("Gim", 0, 5, 8, 440, 800, 35);

    app_referee_ui_add_arc("En", 1 ,2, 12, 967, 540, 230, 310, 391, 400);
    app_referee_ui_add_arc("Cis", 1 ,6, 10, 440, 800, 35, 325, 50, 50);
    app_referee_ui_add_circle("seS", 1, 8, 1, 165, 852, 9);
    app_referee_ui_add_circle("Vis", 1, 8, 1, 810, 130, 9);
    app_referee_ui_add_circle("Sho", 1, 8, 1, 910, 130, 9);
    app_referee_ui_add_circle("Spi", 1, 8, 1, 1010, 130, 9);
    app_referee_ui_add_circle("die", 1, 8, 1, 1110, 130, 9);
    app_referee_ui_add_float("pit", 1, 2, 3, 1350, 670, 30, 0.1);
    app_referee_ui_add_int("s", 1, 8, 4, 1590, 877, 40, 500);
}

void ui_reset(bool reset) {
    if(reset)
        app_ui_add_init();
    else
        return;
}
void app_ui_dot_update(app_ui_data_t *ui, const app_ui_dot_t* ui_) {
    ui->dot.set(2,ui_->ui_shoot);
    ui->dot.set(3, (ui_->rotate != 0));
    ui->dot.set(4, ui_->ui_die);
}

void app_ui_task(app_ui_data_t *ui) {
    if(++count == 50) {
        count = 0;

        ui_draw_cap(ui->En);

        if(ui->diff_dot.test(1)) ui_draw_dot("Vis",2, 810,  130, ui->dot.test(1));
        if(ui->diff_dot.test(2)) ui_draw_dot("Sho",2, 910,  130, ui->dot.test(2));
        if(ui->diff_dot.test(3)) ui_draw_dot("Spi",2, 1010, 130, ui->dot.test(3));
        app_referee_ui_upd_arc("Cis", 1 ,6, 10, 440, 800, (35 + ui->cis) % 360, (325 + ui->cis) % 360, 50, 50);
        app_referee_ui_upd_float("pit", 1, 2, 3, 1350, 670, 30, ui->ui_pit);

        // if(++count1 == 5) {
        if(ui->diff_dot.test(0)) ui_draw_dot("seS",2, 165,  852, ui->dot.test(0));
        if(ui->diff_dot.test(4)) ui_draw_dot("die",2, 1110, 130, ui->dot.test(4));
        app_referee_ui_upd_int("s", 1, 8, 4, 1590, 877, 40, ui->s_sum);
        // count1 = 0;
        // }
        for(uint8_t i = 0; i < 5; i++) {
            if(ui->dot.test(i) ^ ui->lst_dot.test(i)) {
                ui->diff_dot.flip(i);
            }
        }
        ui->lst_dot = ui->dot;
        ui->timesetp = bsp_time_get_ms();
    }
}