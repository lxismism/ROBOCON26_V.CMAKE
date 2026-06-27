#include "display_task.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "ws2812.hpp"
#include "led_ui.hpp"
#include "topics.hpp"
#include "topic_pool.h"

osThreadId_t DisplayTaskHandle;

TypedTopicSubscriber<pub_motor_status> motor_status_sub("motor_status", 2);
pub_motor_status motor_status_pop{};

TypedTopicSubscriber<pub_omni_ir_status> omni_ir_status_sub("omni_ir_status", 2);
pub_omni_ir_status omni_ir_status_pop{};

Ws2812 led_matrix(&htim4);

LedUi led_ui(led_matrix);

constexpr Ws2812::Color kTestColors[] = {
    Ws2812::Color::Yellow,    Ws2812::Color::Green,
    Ws2812::Color::DarkGreen, Ws2812::Color::Blue,
    Ws2812::Color::Red,       Ws2812::Color::White,
};

void displayTask(void *argument) {
  (void)argument;

  if(!motor_status_sub.IsValid() || !omni_ir_status_sub.IsValid()) {
    return;
  }

  led_ui.setBright(10);

  for (;;) {
    
    motor_status_sub.TryGet(&motor_status_pop);
    led_ui.drawMotorStatus(motor_status_pop);
    omni_ir_status_sub.TryGet(&omni_ir_status_pop);
    led_ui.drawOmniIrStatus(omni_ir_status_pop);
    led_ui.drawMF();

    led_ui.refresh();

    osDelay(100);
  }
}
