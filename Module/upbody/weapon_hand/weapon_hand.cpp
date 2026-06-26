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

#include "main.h"
#include "weapon_hand.hpp"


void WeaponHand::init() {
  // ---- 抬升电机 PID (3508) ----
  PID_Init(&lift_pid_);
  lift_pid_.Kp = 600.0f;  //原来是40
  lift_pid_.Ki = 30.0f;  //原来40  //还没试这个
  lift_pid_.Kd = 30.0f;  //9
  lift_pid_.MaxOut = 8000.0f;  //8000
  lift_pid_.IntegralLimit = 1000.0f;  //5000
  lift_pid_.DeadBand = 0.5f;  //0.5
  lift_pid_.Improve = Integral_Limit | Derivative_On_Measurement;

  // ---- 伸缩电机 PID (2006) ----
  PID_Init(&extend_pid_);
  extend_pid_.Kp = 1200.0f;  //原来40
  extend_pid_.Ki = 20.0f;  //40
  extend_pid_.Kd = 60.0f;   //6
  extend_pid_.MaxOut = 5000.0f;  //5000
  extend_pid_.IntegralLimit = 800.0f;  //3000
  extend_pid_.DeadBand = 0.1f;
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

  lift_target_deg_ = clampTarget(lift_target_deg_, cur_lift,
                                 lift_ground_clearance_mm_ / kLiftMmPerDeg,
                                 (lift_ground_clearance_mm_ + lift_travel_max_mm_) / kLiftMmPerDeg);


  float lift_out = PID_Calculate(&lift_pid_, cur_lift, lift_target_deg_);
  lift_out += lift_gravity_comp_;  //加上重力补偿
  lift_motor_->setMotorCmd(lift_out);

  // ===== 2. 伸缩电机位置环 =====
  float cur_extend = extend_motor_->getCurrentSumPos();

  if (!extend_inited_) {
    extend_target_deg_ = cur_extend;
    extend_inited_ = true;
  }

  extend_target_deg_ = clampTarget(extend_target_deg_, cur_extend,
                                   extend_min_mm_ / kExtendMmPerDeg,
                                   extend_max_mm_ / kExtendMmPerDeg);


  float extend_out = PID_Calculate(&extend_pid_, cur_extend, extend_target_deg_);
  extend_motor_->setMotorCmd(extend_out);

    // ===== 3. 腕部达妙电机维护 =====
  if (wrist_motor_ != nullptr) {
    if (wrist_motor_->isOffline()) {
      // 电机离线（未上电或掉线），每200ms重试一次使能
      static uint32_t last_retry_tick = 0;
      if (HAL_GetTick() - last_retry_tick > 200) {
        wrist_motor_->dmMotorEnable();
        last_retry_tick = HAL_GetTick();
      }
    } else {
        // 电机在线，正常发位置速度指令
        // 绝对角度模式优先（由动作系统写入），否则用手动翻转开关
        float target = wrist_target_rad_;
        wrist_motor_->posWithSpeedControl(target, kWristFlipSpeed_radps);

    }
  }


}

// ======== 新增：夹爪开合切换 ========
void WeaponHand::clawToggle() {
  // 读取PG5当前状态：高电平=夹爪合，低电平=夹爪开
  GPIO_PinState cur = HAL_GPIO_ReadPin(VALVE_CLAW_GPIO_Port, VALVE_CLAW_Pin);
  // 翻转电平
  HAL_GPIO_WritePin(VALVE_CLAW_GPIO_Port, VALVE_CLAW_Pin,
                    (cur == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// ======== 腕部达妙翻转切换 ========
void WeaponHand::wristFlip() {
  if (wrist_motor_ == nullptr) return;
  wrist_flipped_ = !wrist_flipped_;
  wrist_target_rad_ = wrist_flipped_ ? kWristDownAngle_rad : kWristUpAngle_rad;
}



float WeaponHand::clampTarget(float target, float current,
                              float min_val, float max_val) {
  if (min_val >= max_val) return target;
  if (target < min_val) return min_val;
  if (target > max_val) return max_val;
  return target;
}

void WeaponHand::addLiftDelta(float delta_mm) {
  lift_target_deg_ += delta_mm / kLiftMmPerDeg;
}

void WeaponHand::addExtendDelta(float delta_mm) {
  extend_target_deg_ += delta_mm / kExtendMmPerDeg;
}

float WeaponHand::getLiftHeightMm() const {
  if (lift_motor_ == nullptr) return 0.0f;
  return lift_motor_->getCurrentSumPos() * kLiftMmPerDeg + lift_ground_clearance_mm_;
}

float WeaponHand::getExtendLengthMm() const {
  if (extend_motor_ == nullptr) return 0.0f;
  return -extend_motor_->getCurrentSumPos() * kExtendMmPerDeg;
}
