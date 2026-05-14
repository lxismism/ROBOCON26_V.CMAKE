#pragma once

#include <cstdint>

class Position {
public:
  static constexpr uint8_t kFrameHead0 = 0xFC;
  static constexpr uint8_t kFrameHead1 = 0xFB;
  static constexpr uint8_t kFrameIdPosition = 0x01;
  static constexpr uint8_t kFrameEnd0 = 0xFD;
  static constexpr uint8_t kFrameEnd1 = 0xFE;
  static constexpr uint8_t kMinPayloadLength = 16;
  static constexpr uint8_t kMaxPayloadLength = 64;

  struct Data {
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
    float yaw_speed{0.0f};
    uint8_t frame_id{0};
    uint8_t payload_length{0};
    uint32_t frame_count{0};
  };

  uint8_t init();

  // Feed one UART byte. Returns the decoded frame id, or 0 when no frame is ready.
  uint8_t processByte(uint8_t byte);

  const Data &getData() const { return data_; }
  bool frameReady() const { return frame_ready_; }
  void clearFrameReady() { frame_ready_ = false; }
  void resetState();

private:
  enum class RxState {
    WaitingHead0,
    WaitingHead1,
    WaitingId,
    WaitingLength,
    ReceivingPayload,
    WaitingCrc0,
    WaitingCrc1,
    WaitingEnd0,
    WaitingEnd1,
  };

  static float readFloatLE(const uint8_t *data);
  static bool isSupportedPayloadLength(uint8_t payload_length);

  void parseFrame();

  RxState rx_state_{RxState::WaitingHead0};
  uint8_t frame_id_{0};
  uint8_t payload_length_{0};
  uint8_t payload_[kMaxPayloadLength]{};
  uint8_t payload_index_{0};
  uint8_t crc_[2]{};
  Data data_{};
  bool frame_ready_{false};
};
