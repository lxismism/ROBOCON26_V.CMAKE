/**
 * @file WitMotionImu.cpp
 * @brief Wit-Motion姿态传感器协议解析 —— 状态机实现
 */
#include "WitMotionImu.hpp"

// π 的float精度常量，用于角度转弧度
static constexpr float kPi = 3.14159265358979323846f;
// 传感器的满量程参数 —— 一个16位有符号整数对应±180°
static constexpr float kAngleScale = 180.0f / 32768.0f;
// 只用角度帧(0x53)，所以数据部分的长度固定为8字节
static constexpr uint8_t kAngleFrameDataLen = 8;


void WitMotionImu::init() { resetState(); }

void WitMotionImu::resetState() {
  state_ = RxState::WAIT_HEADER;
  rx_idx_ = 0;
}


uint8_t WitMotionImu::processByte(uint8_t byte) {
  switch (state_) {

  // ── 状态1：等帧头 ──
  case RxState::WAIT_HEADER:
    if (byte == 0x55) {
      // 找到了 —— 把它存入buf[0]，准备收帧类型
      rx_buf_[0] = byte;
      rx_idx_ = 1;
      state_ = RxState::WAIT_TYPE;
    }
    // 如果不是0x55：忽略，继续等（丢弃杂散字节）
    break;

  // ── 状态2：等帧类型 ──
  case RxState::WAIT_TYPE:
    rx_buf_[1] = byte;
    rx_idx_ = 2;
    // 不同帧类型的数据长度不同，但我们目前只关心0x53（角度帧）
    if (byte == 0x53) {
      state_ = RxState::WAIT_DATA;
    } else {
      // 不认识 —— 重置，回WAIT_HEADER等下一帧
      resetState();
    }
    break;

  // ── 状态3：收数据 ──
  case RxState::WAIT_DATA:
    rx_buf_[rx_idx_] = byte;
    rx_idx_++;
    // 收满kAngleFrameDataLen(8)个数据字节后，进入等校验和
    if (rx_idx_ >= 2 + kAngleFrameDataLen) {  // 2(header+type) + 8(data) = 10
      state_ = RxState::WAIT_SUM;
    }
    break;

  // ── 状态4：校验和 ──
  case RxState::WAIT_SUM: {
    rx_buf_[rx_idx_] = byte; // rx_idx_此时为10（最后一字节，SUM）
    resetState();             // 无论校验通过与否，状态机复位准备下一帧

    // 计算校验和：帧前10字节累加，取低8位，与收到的第11字节比较
    uint8_t expected = calcChecksum(rx_buf_);
    if (expected == byte) {
      // ── 校验通过，解析数据 ──
      //
      // WIT协议数据字节序：低字节在前（Little-Endian）
      // YawL = rx_buf_[6], YawH = rx_buf_[7]
      // 拼接：(int16_t)(YawH << 8 | YawL)
      //
      // 为什么强制转int16_t：
      //   如果不转，YawH << 8 会被提升为int（32位），负数的高位会被
      //   填充0而不是1，导致角度值错误。先转int16_t保证符号位正确。

      int16_t roll_raw =
          static_cast<int16_t>((static_cast<uint16_t>(rx_buf_[3]) << 8) |
                               static_cast<uint16_t>(rx_buf_[2]));
      int16_t pitch_raw =
          static_cast<int16_t>((static_cast<uint16_t>(rx_buf_[5]) << 8) |
                               static_cast<uint16_t>(rx_buf_[4]));
      int16_t yaw_raw =
          static_cast<int16_t>((static_cast<uint16_t>(rx_buf_[7]) << 8) |
                               static_cast<uint16_t>(rx_buf_[6]));

      // 转换为弧度
      imu_data_.roll_rad = static_cast<float>(roll_raw) * kAngleScale *
                           (kPi / 180.0f);
      imu_data_.pitch_rad = static_cast<float>(pitch_raw) * kAngleScale *
                            (kPi / 180.0f);
      imu_data_.yaw_rad = static_cast<float>(yaw_raw) * kAngleScale *
                          (kPi / 180.0f);

      return 0x53; // 返回帧类型，告诉调用方"角度帧解析完毕"
    }
    // 校验失败：静默丢弃，imu_data_保持上一次的有效值
    break;
  }

  default:
    resetState();
    break;
  }

  return 0; // 还没收到完整的一帧
}

uint8_t WitMotionImu::calcChecksum(const uint8_t *frame) {
  // 校验和不包括SUM字节本身 —— 即前10字节累加
  uint16_t sum = 0;
  for (uint8_t i = 0; i < kFrameLen - 1; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF); // 取低8位
}
