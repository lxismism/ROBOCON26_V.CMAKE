/**
 * @file lift.cpp
 * @author lxlx
 * @brief 电梯抬升机构位置环控制实现
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 双电机同步是硬需求，位置差超阈值时需立即停机检查
 * @note 两侧PID参数默认相同，若左右负载不均则需分开微调
 * @versioninfo v0.1: 双电机独立位置环 + 同步误差检测
 */

#include "lift.hpp"
#include "pid_controller.h"
#include <cmath>

void Lift::init() {
  // ---- 左侧电机速度PID (3508) ----
  PID_Init(&left_v_pid_);
  left_v_pid_ = {
    .Kp = 3000.0f,
    .Ki = 1.0f,
    .Kd = 0.0f,
    .MaxOut = 15000.0f,
    .IntegralLimit = 5000.0f,
    .DeadBand = 0.5f,
    .Improve = Integral_Limit | Derivative_On_Measurement | IMCREATEMENT_OF_OUT
  };

  // ---- 右侧电机速度PID (3508) ----
  PID_Init(&right_v_pid_);
  right_v_pid_ = {
    .Kp = 3000.0f,
    .Ki = 1.0f,
    .Kd = 0.0f,
    .MaxOut = 15000.0f,
    .IntegralLimit = 5000.0f,
    .DeadBand = 0.5f,
    .Improve = Integral_Limit | Derivative_On_Measurement | IMCREATEMENT_OF_OUT
  };

  //平台位置环
  PID_Init(&platfrom_pos_pid_);
  platfrom_pos_pid_ = {
    .Kp = 0.1f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 5.0f, //位置环输出速度目标，单位为弧度/秒
    .IntegralLimit = 0.0f,
    .DeadBand = 0.0f,
    .Improve = Integral_Limit | Derivative_On_Measurement
  };
}
/**
 * @brief 升降平台控制更新
 * @note  100Hz位置环+1kHz速度环，内部分频，调用频率应为1kHz
 */
void Lift::update() {
  if (left_motor_ == nullptr || right_motor_ == nullptr) {
    return;
  }

  static uint8_t prescaler_cnt = 0;

  float cur_left_h_deg_ = left_motor_->getCurrentSumPos();     //左电机角度（度），反映左夹板高度
  float cur_right_h_deg_ = -right_motor_->getCurrentSumPos();  //-1*右电机角度（度），反映右夹板高度

  float cur_left_v_ = left_motor_->getCurrentSpeed();     //左电机速度（弧度/s），反映左夹板速度
  float cur_right_v_ = -right_motor_->getCurrentSpeed();  //-1*右电机速度（弧度/s），反映右夹板速度

  float cur_platform_h_deg_ = (cur_left_h_deg_ + cur_right_h_deg_) * 0.5f;   //左右电机角度均值（度），反映平台高度
  float cur_platform_v_ = (cur_left_v_ + cur_right_v_) * 0.5f;   //左右速度均值（弧度/s），反映平台升降速度

  sync_error = cur_left_h_deg_ - cur_right_h_deg_;  //同步误差，左高为正

  // ---- 首次初始化：两侧各以自己的当前位置为目标，避免跳变 ----
  if (!inited_) {
    // 取两侧当前位置的平均值作为初始目标，这样两侧都不会跳
    target_deg_ = cur_platform_h_deg_;
    inited_ = true;
  }

  // ---- 角度限幅（共用同一限位） ----
  {
    float target_abs_mm = target_deg_ * kMmPerDeg + ground_clearance_mm_;
    float cur_abs_mm = cur_platform_h_deg_ * kMmPerDeg + ground_clearance_mm_;
    target_abs_mm = clampTarget(target_abs_mm, cur_abs_mm,
                                ground_clearance_mm_,
                                ground_clearance_mm_ + travel_max_mm_);
    target_deg_ = (target_abs_mm - ground_clearance_mm_) / kMmPerDeg;
  }



  // ---- 同步误差检测 ----
  if (sync_error > sync_error_threshold_) {
    // TODO: 触发报警或紧急停机
    // 目前仅占位，不改变控制逻辑
  }

  static float target_v_left_;
  static float target_v_right_;
  //平台位置环
  if(prescaler_cnt >= 10)
  {
    platform_pos_out_ = PID_Calculate(&platfrom_pos_pid_, cur_platform_h_deg_, target_deg_);
    //同步控制
    target_v_left_ = platform_pos_out_ - sync_error * 0.05f;  //左侧速度目标 = 平台位置输出 - 同步误差补偿
    target_v_right_ = platform_pos_out_ + sync_error * 0.05f;

    prescaler_cnt = 0;
  }

  // ---- 左侧速度环 ----
  float left_v_out = PID_Calculate(&left_v_pid_, cur_left_v_, target_v_left_);
  left_motor_->setMotorCmd(left_v_out); //电流(0.001A)

  // ---- 右侧速度环 ----
  float right_v_out = PID_Calculate(&right_v_pid_, cur_right_v_, target_v_right_);
  right_motor_->setMotorCmd(-right_v_out);  //电流(0.001A)

  prescaler_cnt++;
}

float Lift::clampTarget(float target_mm, float current_mm,
                        float min_mm, float max_mm) {
  if (min_mm >= max_mm) return target_mm;
  if (target_mm < min_mm) return min_mm;
  if (target_mm > max_mm) return max_mm;
  return target_mm;
}


void Lift::addTargetDelta(float delta_mm) {
  target_deg_ += delta_mm / kMmPerDeg;
}

float Lift::getCurrentHeightMm() const {
  if (left_motor_ == nullptr || right_motor_ == nullptr) return 0.0f;
  float cur_left = left_motor_->getCurrentSumPos();
  float cur_right = -right_motor_->getCurrentSumPos();
  return (cur_left + cur_right) * 0.5f * kMmPerDeg + ground_clearance_mm_;

}

float Lift::getSyncErrorMm() const {
  if (left_motor_ == nullptr || right_motor_ == nullptr) return 0.0f;
  float cur_left = left_motor_->getCurrentSumPos();
  float cur_right = -right_motor_->getCurrentSumPos();
  return std::fabs(cur_left - cur_right) * kMmPerDeg;
}

float Lift::getSyncError() const {
  return sync_error;
}