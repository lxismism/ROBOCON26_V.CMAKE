#include "ws2812.hpp"

#include "memory_map.h"
#include "tim.h"

#include <cstring>

Ws2812 *Ws2812::instances_[Ws2812::kMaxInstances] = {nullptr, nullptr};
DMA_BUFFER_ATTR uint16_t Ws2812::dma_buffer_[Ws2812::kDmaBufferLength] = {};

Ws2812::Ws2812(TIM_HandleTypeDef *htim, uint32_t channel)
    : htim_(htim), channel_(channel) {
  for (size_t i = 0; i < kMaxInstances; ++i) {
    if (instances_[i] == nullptr) {
      instances_[i] = this;
      break;
    }
  }
}

HAL_StatusTypeDef Ws2812::Show() {
  if (htim_ == nullptr) {
    return HAL_ERROR;
  }
  if (busy_) {
    return HAL_BUSY;
  }

  Encode();
  busy_ = true;

  HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_DMA(
      htim_, channel_, reinterpret_cast<const uint32_t *>(dma_buffer_),
      kDmaBufferLength);
  if (ret != HAL_OK) {
    busy_ = false;
  }
  return ret;
}

HAL_StatusTypeDef Ws2812::Clear() {
  Fill(Color::Black);
  return Show();
}

void Ws2812::SetBrightness(uint8_t brightness) {
  // 统一亮度缩放只影响下一次 Show() 编码，不改变像素原始颜色。
  brightness_ = brightness;
}

bool Ws2812::SetPixel(uint16_t x, uint16_t y, Color color) {
  return SetPixel(x, y, Preset(color));
}

bool Ws2812::SetPixel(uint16_t x, uint16_t y, Rgb color) {
  if (x >= kWidth || y >= kHeight) {
    return false;
  }
  pixels_[LedIndex(x, y)] = color;
  return true;
}

bool Ws2812::SetLed(uint16_t index, Color color) {
  return SetLed(index, Preset(color));
}

bool Ws2812::SetLed(uint16_t index, Rgb color) {
  if (index >= kLedCount) {
    return false;
  }
  pixels_[index] = color;
  return true;
}

void Ws2812::Fill(Color color) {
  Fill(Preset(color));
}

void Ws2812::Fill(Rgb color) {
  for (auto &pixel : pixels_) {
    pixel = color;
  }
}

void Ws2812::OnPwmDmaFinished() {
  if (htim_ == nullptr) {
    busy_ = false;
    return;
  }

  (void)HAL_TIM_PWM_Stop_DMA(htim_, channel_);
  __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
  busy_ = false;
}

Ws2812 *Ws2812::FromHandle(TIM_HandleTypeDef *htim) {
  if (htim == nullptr) {
    return nullptr;
  }

  for (auto *instance : instances_) {
    if (instance != nullptr && instance->Handle() == htim) {
      return instance;
    }
  }
  return nullptr;
}

Ws2812::Rgb Ws2812::Preset(Color color) {
  switch (color) {
  case Color::Yellow:
    return {255, 180, 0};
  case Color::Green:
    return {0, 255, 0};
  case Color::DarkGreen:
    return {0, 40, 0};
  case Color::Blue:
    return {0, 0, 255};
  case Color::Red:
    return {255, 0, 0};
  case Color::White:
    return {255, 255, 255};
  case Color::Black:
  default:
    return {0, 0, 0};
  }
}

uint16_t Ws2812::LedIndex(uint16_t x, uint16_t y) {
  // 灯珠按行优先连接：第一行从左到右，第二行继续从左到右。
  return static_cast<uint16_t>(y * kWidth + x);
}

uint16_t Ws2812::DutyForBit(bool bit_is_one) {
  const uint32_t period_ticks = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  // WS2812 通过高电平宽度区分 0/1。这里按当前 TIM4 周期折算占空比。
  const uint32_t duty = bit_is_one ? (period_ticks * 64U + 50U) / 100U
                                  : (period_ticks * 32U + 50U) / 100U;
  return static_cast<uint16_t>(duty);
}

void Ws2812::Encode() {
  size_t pos = 0;

  for (const auto &pixel : pixels_) {
    const Grb grb = ApplyBrightness(pixel);
    const uint8_t data[3] = {grb.g, grb.r, grb.b};

    for (uint8_t byte : data) {
      // WS2812 每个颜色字节高位先发，颜色顺序为 GRB。
      for (int8_t bit = 7; bit >= 0; --bit) {
        dma_buffer_[pos++] = DutyForBit((byte & (1U << bit)) != 0U);
      }
    }
  }

  // 末尾保持低电平超过 50us，锁存整帧数据。
  while (pos < kDmaBufferLength) {
    dma_buffer_[pos++] = 0;
  }
}

Ws2812::Grb Ws2812::ApplyBrightness(Rgb color) const {
  const uint16_t scale = brightness_;
  return {static_cast<uint8_t>((static_cast<uint16_t>(color.g) * scale) / 255U),
          static_cast<uint8_t>((static_cast<uint16_t>(color.r) * scale) / 255U),
          static_cast<uint8_t>((static_cast<uint16_t>(color.b) * scale) / 255U)};
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  Ws2812 *ws2812 = Ws2812::FromHandle(htim);
  if (ws2812 != nullptr) {
    ws2812->OnPwmDmaFinished();
  }
}
