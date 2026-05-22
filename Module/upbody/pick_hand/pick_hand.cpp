/**
 * @file pick_hand.cpp
 * @author  lxlx
 * @brief 吸取手机构位置环控制实现
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 上电首次调用 update() 会以当前角度初始化目标值，避免电机跳变
 * @note PID参数为默认值，需在机构装配完成后实测微调
 * @versioninfo v0.1: 抬升/云台/伸缩三路位置环，带角度限位保护
 */


#include "pick_hand.hpp"
#include "main.h"  

void PickHand::init() {
  // ---- 抬升电机 PID (3508) ----
  PID_Init(&lift_pid_);
  lift_pid_.Kp = 40.0f;
  lift_pid_.Ki = 40.0f;
  lift_pid_.Kd = 9.0f;
  lift_pid_.MaxOut = 8000.0f;
  lift_pid_.IntegralLimit = 5000.0f;
  lift_pid_.DeadBand = 0.5f;
  lift_pid_.Improve = Integral_Limit | Derivative_On_Measurement;

  // ---- 云台旋转电机 PID (2006) ----
  PID_Init(&yaw_pid_);
  yaw_pid_.Kp = 40.0f;
  yaw_pid_.Ki = 40.0f;
  yaw_pid_.Kd = 6.0f;
  yaw_pid_.MaxOut = 5000.0f;
  yaw_pid_.IntegralLimit = 3000.0f;
  yaw_pid_.DeadBand = 0.5f;
  yaw_pid_.Improve = Integral_Limit | Derivative_On_Measurement;

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

void PickHand::update() {
  if (lift_motor_ == nullptr || yaw_motor_ == nullptr ||
      extend_motor_ == nullptr) {
    return; // 安全保护：电机未绑定则跳过
  }

  // ===== 1. 抬升电机位置环 =====
  float cur_lift = lift_motor_->getCurrentSumPos();

  if (!lift_inited_) {
    lift_target_deg_ = cur_lift;
    lift_inited_ = true;
  }

  // 角度限幅
  lift_target_deg_ = clampAngle(lift_target_deg_, cur_lift,
                                lift_min_deg_, lift_max_deg_);

  float lift_out = PID_Calculate(&lift_pid_, cur_lift, lift_target_deg_);
  lift_motor_->setMotorCmd(lift_out);

  // ===== 2. 云台旋转电机位置环 =====
  float cur_yaw = yaw_motor_->getCurrentSumPos();

  if (!yaw_inited_) {
    yaw_target_deg_ = cur_yaw;
    yaw_inited_ = true;
  }

  yaw_target_deg_ = clampAngle(yaw_target_deg_, cur_yaw,
                                yaw_min_deg_, yaw_max_deg_);

  float yaw_out = PID_Calculate(&yaw_pid_, cur_yaw, yaw_target_deg_);

  yaw_motor_->setMotorCmd(yaw_out);

  // ===== 3. 伸缩电机位置环 =====
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

float PickHand::clampAngle(float target, float current,
                           float min_deg, float max_deg) {
  // 如果限位尚未设置（均为0），跳过限幅
  if (min_deg >= max_deg) {
    return target;
  }
  if (target < min_deg) return min_deg;
  if (target > max_deg) return max_deg;
  return target;
}

// ======== 新增：真空泵通断切换 ========
void PickHand::pumpToggle() {
  GPIO_PinState cur = HAL_GPIO_ReadPin(PUMP_PICK_GPIO_Port, PUMP_PICK_Pin);
  HAL_GPIO_WritePin(PUMP_PICK_GPIO_Port, PUMP_PICK_Pin,
                    (cur == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// ======== 新增：电磁阀通断切换 ========
void PickHand::valveToggle() {
  GPIO_PinState cur = HAL_GPIO_ReadPin(VALVE_PICK_GPIO_Port, VALVE_PICK_Pin);
  HAL_GPIO_WritePin(VALVE_PICK_GPIO_Port, VALVE_PICK_Pin,
                    (cur == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
