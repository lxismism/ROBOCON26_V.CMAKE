/**
 * @file WitMotionImu.hpp
 * @brief Wit-Motion姿态传感器协议解析器
 *
 * 适用传感器型号：WT901 / WT61 / JY901 等基于WIT标准协议的姿态传感器
 * 通信接口：UART，默认波特率9600，8N1
 * 数据帧：
 *   [0x55] [TYPE] [Data0..Data7] [SUM]
 *   角度帧TYPE=0x53，数据为 RollL RollH PitchL PitchH YawL YawH VL VH
 *   Yaw角度 = ((int16_t)(YawH<<8|YawL)) / 32768.0 × 180.0  （度数）
 */
#pragma once

#include <cstdint>

class WitMotionImu {
public:
  // 解析结果 —— 三个姿态角，单位：弧度
  struct ImuData {
    float yaw_rad;   // 偏航角，绕Z轴旋转，范围 -π ~ +π
    float pitch_rad; // 俯仰角
    float roll_rad;  // 滚转角
  };

  WitMotionImu() = default;

  /**
   * @brief 初始化状态机，使其处于等待帧头的干净状态
   */
  void init();

  /**
   * @brief 送入一个字节，由状态机处理
   * @param byte 当前收到的字节
   * @return 0 —— 尚未完成一帧解析
   *         非0 —— 刚完成一帧解析，返回值为帧类型（如0x53表示角度帧）
   */
  uint8_t processByte(uint8_t byte);

  /**
   * @brief 获取最近一次成功解析的姿态数据
   */
  const ImuData &getImuData() const { return imu_data_; }

private:
  // 状态机 —— 把"收一帧数据"这件事拆成几个明确的步骤
  enum class RxState : uint8_t {
    WAIT_HEADER, // 正在等0x55帧头
    WAIT_TYPE,   // 已收到0x55，在等帧类型字节
    WAIT_DATA,   // 正在收数据载荷
    WAIT_SUM,    // 正在等校验和字节
  };

  void resetState();

  // 校验和计算 —— 帧内所有字节累加，取低8位
  static uint8_t calcChecksum(const uint8_t *frame);

  RxState state_{RxState::WAIT_HEADER};

  // 接收一帧的缓冲区 —— WIT标准帧最长为11字节（0x53角度帧）
  static constexpr uint8_t kFrameLen = 11;
  uint8_t rx_buf_[kFrameLen]{};
  uint8_t rx_idx_{0}; // 当前正在填充rx_buf_的第几个字节

  ImuData imu_data_{}; // 最近一次成功解析的结果
};
