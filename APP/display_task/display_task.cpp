#include "display_task.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "ws2812.hpp"

osThreadId_t DisplayTaskHandle;

namespace {

Ws2812 led_matrix(&htim4);

constexpr Ws2812::Color kTestColors[] = {
    Ws2812::Color::Yellow,    Ws2812::Color::Green,
    Ws2812::Color::DarkGreen, Ws2812::Color::Blue,
    Ws2812::Color::Red,       Ws2812::Color::White,
};

void DrawTestFrame(uint8_t frame) {
  led_matrix.Fill(Ws2812::Color::Black);

  // 外框常亮白色，用来确认 16x16 矩阵边界和安装方向。
  for (uint16_t i = 0; i < Ws2812::kWidth; ++i) {
    led_matrix.SetPixel(i, 0, Ws2812::Color::White);
    led_matrix.SetPixel(i, Ws2812::kHeight - 1, Ws2812::Color::White);
  }
  for (uint16_t i = 0; i < Ws2812::kHeight; ++i) {
    led_matrix.SetPixel(0, i, Ws2812::Color::White);
    led_matrix.SetPixel(Ws2812::kWidth - 1, i, Ws2812::Color::White);
  }

  // 移动色带用于测试行优先映射：每行从左到右，下一行继续。
  for (uint16_t y = 1; y < Ws2812::kHeight - 1; ++y) {
    const uint16_t x =
        static_cast<uint16_t>((frame + y) % (Ws2812::kWidth - 2) + 1);
    const auto color = kTestColors[(frame / 2 + y) %
                                   (sizeof(kTestColors) / sizeof(kTestColors[0]))];
    led_matrix.SetPixel(x, y, color);
  }

  // 中心十字依次切换预设颜色，便于观察颜色顺序是否正确。
  const auto center_color =
      kTestColors[frame % (sizeof(kTestColors) / sizeof(kTestColors[0]))];
  for (uint16_t i = 5; i <= 10; ++i) {
    led_matrix.SetPixel(8, i, center_color);
    led_matrix.SetPixel(i, 8, center_color);
  }
}

} // namespace

void displayTask(void *argument) {
  (void)argument;

  uint8_t frame = 0;
  led_matrix.SetBrightness(10);
  led_matrix.Clear();
  vTaskDelay(pdMS_TO_TICKS(50));

  auto fillsquare = [](uint16_t x, uint16_t y, uint16_t size, Ws2812::Color color) {
    for (uint16_t i = 0; i < size; ++i) {
      for (uint16_t j = 0; j < size; ++j) {
        led_matrix.SetPixel(x + i, y + j, color);
      }
    }
  };

  for (;;) {
    fillsquare(0, 0, 3, Ws2812::Color::DarkGreen);
    fillsquare(3, 0, 3, Ws2812::Color::Green);
    fillsquare(6, 0, 3, Ws2812::Color::DarkGreen);
    fillsquare(0, 3, 3, Ws2812::Color::Green);
    fillsquare(3, 3, 3, Ws2812::Color::Yellow);
    fillsquare(6, 3, 3, Ws2812::Color::Green);
    (void)led_matrix.Show();
    osDelay(1000);
    //vTaskDelay(pdMS_TO_TICKS(150));
  }
}
