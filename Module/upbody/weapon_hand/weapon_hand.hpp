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
class WeaponHand {
public:
  // ---------- 电机引用（由外部 com_config.cpp 传入） ----------
  C620Motor *lift_motor_{nullptr};   // 3508 抬升电机
  C610Motor *extend_motor_{nullptr}; // 2006 伸缩电机

  // ======== 达妙指针（由外部 com_config.cpp 传入） ========
  DM4310Motor *wrist_motor_{nullptr}; // 腕部达妙4310电机（位置速度模式）


  // ---------- 各电机 PID ----------
  PID_t lift_pid_;
  PID_t extend_pid_;

  // ---------- 目标角度 ----------
  float lift_target_deg_{0.0f};
  float extend_target_deg_{0.0f};

  // ---------- 重力补偿 ----------
   //float lift_gravity_comp_{68.6f};  //武器手抬升重力补偿值
   float lift_gravity_comp_{0.0f};

  // ---------- 换算系数 ----------
  static constexpr float kLiftMmPerDeg = 90.0f / 360.0f;
  static constexpr float kExtendMmPerDeg = 44.0f * 3.14159f / 360.0f;

  // ---------- 限位（mm） ----------
  float lift_ground_clearance_mm_{0.0f};  // 武器手底座离地高度（=电梯平台高度）
  float lift_travel_max_mm_{347.25f};       // 1389° × 90/360
  float extend_min_mm_{0.0f};
  float extend_max_mm_{422.75f};            // 1101° × 44π/360


  // ---------- 首次初始化标志 ----------
  bool lift_inited_{false};
  bool extend_inited_{false};

  // ======== 腕部翻转状态（达妙4310电机，位置速度模式） ========
  bool wrist_flipped_{false};   // false=朝上(初始位), true=朝前(下翻90°)
  static constexpr float kWristUpAngle_rad = 0.0f;        // 朝上（零位），单位 rad
  static constexpr float kWristDownAngle_rad = 1.5708f;   // 朝前（下翻90° = π/2 rad）
  static constexpr float kWristFlipSpeed_radps = 3.0f;    // 翻转速度 rad/s


  // ---------- 方法 ----------
  void init();
  void update();

  // ======== 新增：GPIO控制方法 ========
  void clawToggle();    // 夹爪开合切换（气缸电磁阀PG4）
  void wristFlip();     // 腕部舵机翻转切换（朝上↔朝前）

  // ======== mm接口 ========
  void addLiftDelta(float delta_mm);
  void addExtendDelta(float delta_mm);
  float getLiftHeightMm() const;
  float getExtendLengthMm() const;


private:
  float clampTarget(float target, float current, float min_val, float max_val);
};
