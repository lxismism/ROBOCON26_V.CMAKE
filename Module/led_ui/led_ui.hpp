#pragma once
#include "ws2812.hpp"
#include "topic_pool.h"
#include <stdint.h>
#include <sys/types.h>

class LedUi {
public:
  LedUi(Ws2812 &led_matrix) : led_matrix_(led_matrix) {};
  void setFieldSide(FieldSide_t side) { field_side_ = side;}

  void refresh() {
    led_matrix_.Show();
  }
  
  void clear() {
    led_matrix_.Fill(Ws2812::Color::Black);
  }

  void drawMotorStatus(pub_motor_status &status);
  void drawOmniIrStatus(pub_omni_ir_status &status);
  void drawMF();
  void setBright(uint8_t bright) {
    led_matrix_.SetBrightness(bright);
  }
  void drawCursor(uint8_t x, uint8_t y);
  void clearMFandCursor();
  
private:

  Ws2812 &led_matrix_;
  FieldSide_t field_side_{FieldSide_t::Left};
  uint32_t ir_show_time_flag_{0};

};