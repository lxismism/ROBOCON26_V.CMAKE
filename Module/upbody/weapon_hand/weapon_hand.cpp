/**
 * @file weapon_hand.cpp
 * @author lxlx
 * @brief 武器手机构位置环控制实现
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 上电首次调用 update() 会以当前角度初始化目标值，避免电机跳变
 * @note PID参数为默认值，需在机构装配完成后实测微调
 * @versioninfo v0.1: 抬升/伸缩两路位置环，带角度限位保护
 */

#include "weapon_hand.hpp"

void WeaponHand::init() {
  // ---- 抬升电机 PID (3508) ----
  PID_Init(&lift_pid_);
  lift_pid_.Kp = 11.5f;
  lift_pid_.Ki = 4.0f;
  lift_pid_.Kd = 0.61f;
  lift_pid_.MaxOut = 8000.0f;
  lift_pid_.IntegralLimit = 5000.0f;
  lift_pid_.DeadBand = 0.5f;
  lift_pid_.Improve = Integral_Limit | Derivative_On_Measurement;

  // ---- 伸缩电机 PID (2006) ----
  PID_Init(&extend_pid_);
  extend_pid_.Kp = 40.0f;
  extend_pid_.Ki = 40.0f;
  extend_pid_.Kd = 6.0f;
  extend_pid_.MaxOut = 5000.0f;
  extend_pid_.IntegralLimit = 3000.0f;
  extend_pid_.DeadBand = 0.5f;
  extend_pid_.Improve = Integral_Limit | Derivative_On_Measurement;
}

void WeaponHand::update() {
  if (lift_motor_ == nullptr || extend_motor_ == nullptr) {
    return;
  }

  // ===== 1. 抬升电机位置环 =====
  float cur_lift = lift_motor_->getCurrentSumPos();

  if (!lift_inited_) {
    lift_target_deg_ = cur_lift;
    lift_inited_ = true;
  }

  lift_target_deg_ = clampAngle(lift_target_deg_, cur_lift,
                                lift_min_deg_, lift_max_deg_);

  float lift_out = PID_Calculate(&lift_pid_, cur_lift, lift_target_deg_);
  lift_motor_->setMotorCmd(lift_out);

  // ===== 2. 伸缩电机位置环 =====
  float cur_extend = extend_motor_->getCurrentSumPos();

  if (!extend_inited_) {
    extend_target_deg_ = cur_extend;
    extend_inited_ = true;
  }

  extend_target_deg_ = clampAngle(extend_target_deg_, cur_extend,
                                  extend_min_deg_, extend_max_deg_);

  float extend_out = PID_Calculate(&extend_pid_, cur_extend, extend_target_deg_);
  extend_motor_->setMotorCmd(extend_out);
}

float WeaponHand::clampAngle(float target, float current,
                             float min_deg, float max_deg) {
  if (min_deg >= max_deg) {
    return target;
  }
  if (target < min_deg) return min_deg;
  if (target > max_deg) return max_deg;
  return target;
}
