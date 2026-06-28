#include "led_ui.hpp"
#include "topic_pool.h"
#include "ws2812.hpp"
#include <cstdint>
#include <stdint.h>

struct MF_Tale {
Ws2812::Color color;
uint8_t data;
};

const static MF_Tale mf_map[3][4] = {
{
    {Ws2812::Color::DarkGreen, 0x00},
    {Ws2812::Color::Green, 0x00},
    {Ws2812::Color::Yellow, 0x00},
    {Ws2812::Color::Green, 0x00}
},
{
    {Ws2812::Color::Green, 0x00},
    {Ws2812::Color::Yellow, 0x00},
    {Ws2812::Color::Green, 0x00},
    {Ws2812::Color::DarkGreen, 0x00}
},
{
    {Ws2812::Color::DarkGreen, 0x00},
    {Ws2812::Color::Green, 0x00},
    {Ws2812::Color::DarkGreen, 0x00},
    {Ws2812::Color::Green, 0x00}
}
};

void LedUi::drawMotorStatus(pub_motor_status &status) {

    auto showMotorSign = [this](uint8_t x, uint8_t y, bool isOffline) -> void {
        led_matrix_.SetPixel(x, y, isOffline ? Ws2812::Color::Red : Ws2812::Color::Green);
    };

    showMotorSign(0, 0, status.chassis_motor1);
    showMotorSign(15, 0, status.chassis_motor2);
    showMotorSign(15, 15, status.chassis_motor3);
    showMotorSign(0, 15, status.chassis_motor4);

    showMotorSign(2, 0, status.picker_yaw_motor);
    showMotorSign(0, 2, status.picker_lift_motor);
    showMotorSign(2, 2, status.picker_extend_motor);

    showMotorSign(15, 2, status.weapon_lift_motor);
    showMotorSign(13, 2, status.weapon_extend_motor);
    showMotorSign(13, 0, false);//达妙目前没接入

    showMotorSign(0, 12, status.lift_left_motor);
    showMotorSign(15, 12, status.lift_right_motor);

}

void LedUi::drawOmniIrStatus(pub_omni_ir_status &status) {
    if(status.isSending) {
        if(ir_show_time_flag_ == 0) {
            ir_show_time_flag_ = HAL_GetTick();
        }
        if(HAL_GetTick() - ir_show_time_flag_ < 200) {
            led_matrix_.SetPixel(7,0, Ws2812::Color::Red);
            led_matrix_.SetPixel(8,0, Ws2812::Color::Black);

            led_matrix_.SetPixel(0, 7, Ws2812::Color::Black);
            led_matrix_.SetPixel(0, 8, Ws2812::Color::Red);

            led_matrix_.SetPixel(15, 7, Ws2812::Color::Red);
            led_matrix_.SetPixel(15, 8, Ws2812::Color::Black);

            led_matrix_.SetPixel(7, 15, Ws2812::Color::Black);
            led_matrix_.SetPixel(8, 15, Ws2812::Color::Red);
        } 
        else if(HAL_GetTick() - ir_show_time_flag_ < 400) {
            led_matrix_.SetPixel(7,0, Ws2812::Color::Black);
            led_matrix_.SetPixel(8,0, Ws2812::Color::Red);

            led_matrix_.SetPixel(0, 7, Ws2812::Color::Red);
            led_matrix_.SetPixel(0, 8, Ws2812::Color::Black);

            led_matrix_.SetPixel(15, 7, Ws2812::Color::Black);
            led_matrix_.SetPixel(15, 8, Ws2812::Color::Red);

            led_matrix_.SetPixel(7, 15, Ws2812::Color::Red);
            led_matrix_.SetPixel(8, 15, Ws2812::Color::Black);
        }
        else if(HAL_GetTick() - ir_show_time_flag_ >= 400) {
            ir_show_time_flag_ = 0;
        }
    }
    else {

        ir_show_time_flag_ = 0;

        led_matrix_.SetPixel(7,0, Ws2812::Color::Red);
        led_matrix_.SetPixel(8,0, Ws2812::Color::Red);

        led_matrix_.SetPixel(0, 7, Ws2812::Color::Red);
        led_matrix_.SetPixel(0, 8, Ws2812::Color::Red);

        led_matrix_.SetPixel(15, 7, Ws2812::Color::Red);
        led_matrix_.SetPixel(15, 8, Ws2812::Color::Red);

        led_matrix_.SetPixel(7, 15, Ws2812::Color::Red);
        led_matrix_.SetPixel(8, 15, Ws2812::Color::Red);
    }
}

void LedUi::drawMF() {

    auto drawMFTale = [this](uint8_t x, uint8_t y, const MF_Tale &mf) -> void {
        led_matrix_.SetPixel(x, y, mf.color);
        led_matrix_.SetPixel(x+1, y, mf.color);
        led_matrix_.SetPixel(x+2, y, mf.color);
        led_matrix_.SetPixel(x, y+1, mf.color);
        led_matrix_.SetPixel(x, y+2, mf.color);
        led_matrix_.SetPixel(x+1, y+2, mf.color);
        led_matrix_.SetPixel(x+2, y+1, mf.color);
        led_matrix_.SetPixel(x+2, y+2, mf.color);
    };

    if(field_side_ == FieldSide_t::Left) {
        uint8_t draw_cursor_y = 4;
        for(uint8_t i = 0; i < 3; i++) {
            uint8_t draw_cursor_x = 2;
            for(auto &mf: mf_map[i]) {
                drawMFTale(draw_cursor_x, draw_cursor_y, mf);
                draw_cursor_x += 3;
            }
            draw_cursor_y += 3;
        }
    }

    if(field_side_ == FieldSide_t::right) {
        uint8_t draw_cursor_y = 4;
        for(uint8_t i = 0; i < 3; i++) {
            uint8_t  draw_cursor_x = 13;
            for(auto &mf: mf_map[i]) {
                drawMFTale(draw_cursor_x, draw_cursor_y, mf);
                draw_cursor_x -= 3;
            }
            draw_cursor_y += 3;
        }
    }
}

void LedUi::clearMFandCursor() {
    for(uint8_t i = 1; i<15; i++) {
        for(uint8_t j = 3; j<14; j++) {
            led_matrix_.SetPixel(i,j,Ws2812::Color::Black);
        }
    }
}

void LedUi::drawCursor(uint8_t x, uint8_t y) {
    switch (field_side_) {
        case FieldSide_t::Left:{
            // if(x == 0 && y == 0) {led_matrix_.SetPixel(14,13,Ws2812::Color::Blue);break;}
            // if(x == 0 && y == 4) {led_matrix_.SetPixel(14,3,Ws2812::Color::Blue);break;}
            // if(x == 5 && y == 0) {led_matrix_.SetPixel(1,13,Ws2812::Color::Blue);break;}
            // if(x == 5 && y == 4) {led_matrix_.SetPixel(1,3,Ws2812::Color::Blue);break;}
            uint8_t draw_x = 15 - 3*x;
            uint8_t draw_y = 14 - 3*y;
            if(x == 0) draw_x = 14;
            if(x == 5) draw_x = 1;
            if(y == 0) draw_y = 13;
            if(y == 4) draw_y = 3;
            led_matrix_.SetPixel(draw_x, draw_y, Ws2812::Color::Red);
            break;
        }
        case FieldSide_t::right: {
            uint8_t draw_x = 15 - 3 * ( 5 - x);
            uint8_t draw_y = 15 - 3 * (4 - y);

            if(x == 0) draw_x = 1;
            if(x == 5) draw_x = 14;
            if(y == 0) draw_y = 3;
            if(y == 4) draw_y = 13;
            led_matrix_.SetPixel(draw_x, draw_y, Ws2812::Color::Red);
        
            break;
        }
    }
}