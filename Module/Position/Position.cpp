#include "Position.hpp"

#include <cstring>

uint8_t Position::init() {
  resetState();
  data_ = Data{};
  return 1;
}

void Position::resetState() {
  rx_state_ = RxState::WaitingHead0;
  frame_id_ = 0;
  payload_length_ = 0;
  payload_index_ = 0;
  std::memset(payload_, 0, sizeof(payload_));
  std::memset(crc_, 0, sizeof(crc_));
}

uint8_t Position::processByte(uint8_t byte) {
  switch (rx_state_) {
  case RxState::WaitingHead0:
    if (byte == kFrameHead0) {
      rx_state_ = RxState::WaitingHead1;
    }
    break;

  case RxState::WaitingHead1:
    if (byte == kFrameHead1) {
      rx_state_ = RxState::WaitingId;
    } else if (byte != kFrameHead0) {
      resetState();
    }
    break;

  case RxState::WaitingId:
    frame_id_ = byte;
    rx_state_ = RxState::WaitingLength;
    break;

  case RxState::WaitingLength:
    payload_length_ = byte;
    if (!isSupportedPayloadLength(payload_length_)) {
      resetState();
      break;
    }
    payload_index_ = 0;
    rx_state_ = RxState::ReceivingPayload;
    break;

  case RxState::ReceivingPayload:
    payload_[payload_index_++] = byte;
    if (payload_index_ >= payload_length_) {
      rx_state_ = RxState::WaitingCrc0;
    }
    break;

  case RxState::WaitingCrc0:
    crc_[0] = byte;
    rx_state_ = RxState::WaitingCrc1;
    break;

  case RxState::WaitingCrc1:
    crc_[1] = byte;
    rx_state_ = RxState::WaitingEnd0;
    break;

  case RxState::WaitingEnd0:
    if (byte == kFrameEnd0) {
      rx_state_ = RxState::WaitingEnd1;
    } else {
      resetState();
    }
    break;

  case RxState::WaitingEnd1:
    if (byte == kFrameEnd1) {
      uint8_t decoded_id = 0;
      if (frame_id_ == kFrameIdPosition &&
          isSupportedPayloadLength(payload_length_)) {
        parseFrame();
        decoded_id = frame_id_;

      }
      resetState();
      return decoded_id;
    }
    resetState();
    break;

  default:
    resetState();
    break;
  }

  return 0;
}

float Position::readFloatLE(const uint8_t *data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

bool Position::isSupportedPayloadLength(uint8_t payload_length) {
  return payload_length >= kMinPayloadLength &&
         payload_length <= kMaxPayloadLength && (payload_length % 4U) == 0U;
}

void Position::parseFrame() {
  data_.frame_id = frame_id_;
  data_.payload_length = payload_length_;
  ++data_.frame_count;
  data_.x = readFloatLE(&payload_[0]) / 1000.0f;
  data_.y = readFloatLE(&payload_[4]) / 1000.0f;
  data_.yaw = readFloatLE(&payload_[8]);
  data_.yaw_speed = readFloatLE(&payload_[12]);
}
