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
#include <cmath>

void Lift::init() {
  // ---- 左侧电机 PID (3508) ----
  PID_Init(&left_pid_);
  left_pid_.Kp = 40.0f;
  left_pid_.Ki = 40.0f;
  left_pid_.Kd = 9.0f;
  left_pid_.MaxOut = 8000.0f;
  left_pid_.IntegralLimit = 5000.0f;
  left_pid_.DeadBand = 0.5f;
  left_pid_.Improve = Integral_Limit | Derivative_On_Measurement;

  // ---- 右侧电机 PID (3508) ----
  PID_Init(&right_pid_);
  right_pid_.Kp = 40.0f;
  right_pid_.Ki = 45.0f;
  right_pid_.Kd = 9.0f;
  right_pid_.MaxOut = 8000.0f;
  right_pid_.IntegralLimit = 5000.0f;
  right_pid_.DeadBand = 0.5f;
  right_pid_.Improve = Integral_Limit | Derivative_On_Measurement;
}

void Lift::update() {
  if (left_motor_ == nullptr || right_motor_ == nullptr) {
    return;
  }

  float cur_left = left_motor_->getCurrentSumPos();
  float cur_right = right_motor_->getCurrentSumPos();

  // ---- 首次初始化：两侧各以自己的当前位置为目标，避免跳变 ----
  if (!inited_) {
    // 取两侧当前位置的平均值作为初始目标，这样两侧都不会跳
    target_deg_ = (cur_left + cur_right) * 0.5f;
    inited_ = true;
  }

  // ---- 角度限幅（共用同一限位） ----
  target_deg_ = clampAngle(target_deg_, (cur_left + cur_right) * 0.5f,
                           min_deg_, max_deg_);

  // ---- 同步误差检测 ----
  float sync_error = std::fabs(cur_left - cur_right);
  if (sync_error > sync_error_threshold_) {
    // TODO: 触发报警或紧急停机
    // 目前仅占位，不改变控制逻辑
  }

  // ---- 左侧位置环 ----
  float left_out = PID_Calculate(&left_pid_, cur_left, target_deg_);
  left_motor_->setMotorCmd(left_out);

  // ---- 右侧位置环 ----
  float right_out = PID_Calculate(&right_pid_, cur_right, target_deg_);
  right_motor_->setMotorCmd(right_out);
}

float Lift::getSyncError() const {
  if (left_motor_ == nullptr || right_motor_ == nullptr) {
    return 0.0f;
  }
  return std::fabs(left_motor_->getCurrentSumPos() -
                   right_motor_->getCurrentSumPos());
}

float Lift::clampAngle(float target, float current,
                       float min_deg, float max_deg) {
  if (min_deg >= max_deg) {
    return target;
  }
  if (target < min_deg) return min_deg;
  if (target > max_deg) return max_deg;
  return target;
}
