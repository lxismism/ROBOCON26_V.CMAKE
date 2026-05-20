/**
 * @file weapon_hand.hpp
 * @author lxlx
 * @brief 武器手机构模块 —— 抬升、伸缩、夹爪夹取、腕部翻转
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 抬升和伸缩均有机械硬限位，目标角度必须走在 [min, max] 范围内
 * @note 后续接入夹爪气缸（电磁阀+真空泵）和腕部双轴舵机时在本模块内扩展
 * @versioninfo v0.1: 搭框架，两个电机位置环 + 角度限位
 */

#pragma once

#include "Motor.hpp"
#include "pid_controller.h"
#include "pm20s.hpp"

class WeaponHand {
public:
  // ---------- 电机引用（由外部 com_config.cpp 传入） ----------
  C620Motor *lift_motor_{nullptr};   // 3508 抬升电机
  C610Motor *extend_motor_{nullptr}; // 2006 伸缩电机

  // ======== 新增：舵机指针（由外部 com_config.cpp 传入） ========
  PM20sServo *wrist_servo_{nullptr}; // 腕部双轴舵机

  // ---------- 各电机 PID ----------
  PID_t lift_pid_;
  PID_t extend_pid_;

  // ---------- 目标角度 ----------
  float lift_target_deg_{0.0f};
  float extend_target_deg_{0.0f};

  // ---------- 角度限位（单位：度，上电后实测填入） ----------
  float lift_min_deg_{0.0f};
  float lift_max_deg_{360.0f};
  float extend_min_deg_{0.0f};
  float extend_max_deg_{360.0f};

  // ---------- 首次初始化标志 ----------
  bool lift_inited_{false};
  bool extend_inited_{false};

  // ======== 新增：腕部舵机状态 ========
  bool wrist_flipped_{false};   // false=朝上(初始位), true=朝前(下翻90°)
  static constexpr float kWristUpAngle = 0.0f;     // 朝上的舵机角度（需实测校准）
  static constexpr float kWristDownAngle = 90.0f;   // 朝前的舵机角度（需实测校准）

  // ---------- 方法 ----------
  void init();
  void update();

  // ======== 新增：GPIO控制方法 ========
  void clawToggle();    // 夹爪开合切换（气缸电磁阀PG5）
  void wristFlip();     // 腕部舵机翻转切换（朝上↔朝前）

private:
  float clampAngle(float target, float current, float min_deg, float max_deg);
};
