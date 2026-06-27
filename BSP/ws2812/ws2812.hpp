#pragma once

#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class Ws2812 {
public:
  static constexpr uint16_t kWidth = 16;
  static constexpr uint16_t kHeight = 16;
  static constexpr uint16_t kLedCount = kWidth * kHeight;

  struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  enum class Color {
    Yellow,
    Green,
    DarkGreen,
    Blue,
    Red,
    White,
    Black
  };

  explicit Ws2812(TIM_HandleTypeDef *htim, uint32_t channel = TIM_CHANNEL_3);

  HAL_StatusTypeDef Show();
  HAL_StatusTypeDef Clear();

  void SetBrightness(uint8_t brightness);
  uint8_t Brightness() const { return brightness_; }

  bool SetPixel(uint16_t x, uint16_t y, Color color);
  bool SetPixel(uint16_t x, uint16_t y, Rgb color);
  bool SetLed(uint16_t index, Color color);
  bool SetLed(uint16_t index, Rgb color);

  void Fill(Color color);
  void Fill(Rgb color);

  bool Busy() const { return busy_; }
  TIM_HandleTypeDef *Handle() const { return htim_; }
  uint32_t Channel() const { return channel_; }

  void OnPwmDmaFinished();

  static Ws2812 *FromHandle(TIM_HandleTypeDef *htim);
  static Rgb Preset(Color color);

private:
  struct Grb {
    uint8_t g;
    uint8_t r;
    uint8_t b;
  };

  static constexpr uint16_t kBitsPerLed = 24;
  static constexpr uint16_t kResetSlots = 64;
  static constexpr uint16_t kDmaBufferLength =
      kLedCount * kBitsPerLed + kResetSlots;
  static constexpr size_t kMaxInstances = 2;

  static uint16_t LedIndex(uint16_t x, uint16_t y);
  static uint16_t DutyForBit(bool bit_is_one);

  void Encode();
  Grb ApplyBrightness(Rgb color) const;

  TIM_HandleTypeDef *htim_{nullptr};
  uint32_t channel_{TIM_CHANNEL_3};
  uint8_t brightness_{64};
  volatile bool busy_{false};
  Rgb pixels_[kLedCount]{};

  static Ws2812 *instances_[kMaxInstances];
  static uint16_t dma_buffer_[kDmaBufferLength];
};
