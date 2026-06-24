/**
 * @file pick_hand.hpp
 * @author  lxlx
 * @brief 吸取手机构模块 —— 抬升、云台旋转、伸缩、吸盘控制
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 抬升和伸缩均有机械硬限位，目标角度必须走在 [min, max] 范围内，否则可能损坏机构
 * @note 后续接入吸盘（电磁阀+真空泵）时在本模块内扩展，不要散落到其他文件
 * @versioninfo v0.1: 搭框架，三个电机位置环 + 角度限位
 */

#pragma once

#include "Motor.hpp"
#include "pid_controller.h"

class PickHand {
public:
  // ---------- 电机引用（由外部 com_config.cpp 传入） ----------
  C620Motor *lift_motor_{nullptr};   // 3508 抬升电机
  C610Motor *yaw_motor_{nullptr};    // 2006 云台旋转电机
  C610Motor *extend_motor_{nullptr}; // 2006 伸缩电机

  // ---------- 各电机 PID ----------
  PID_t lift_pid_;
  PID_t yaw_pid_;
  PID_t extend_pid_;

    // ---------- 重力补偿 ----------
  float lift_gravity_comp_{900.0f};      // 没吸取kfs实抬升重力补偿
  float lift_gravity_comp_kfs_{1500.0f};  // 吸取KFS后抬升重力补偿
  bool is_pump_on_{false};
  bool is_valve_on_{false};

  // ---------- 目标角度 ----------
  float lift_target_deg_{0.0f};
  float yaw_target_deg_{0.0f};
  float extend_target_deg_{0.0f};

  // ---------- 换算系数 ----------
  static constexpr float kLiftMmPerDeg = 105.0f / 360.0f;
  static constexpr float kExtendMmPerDeg = 68.0f * 3.14159f / 360.0f;
  static constexpr float kYawDegPerMotorDeg = 3.0f / 13.0f;

  // ---------- 限位（升降和伸缩用mm，云台用真实°） ----------
  float lift_ground_clearance_mm_{0.0f};  // 吸取手底座离地高度（=电梯平台高度）
  float lift_travel_max_mm_{442.6f};        // 吸取手升降行程（1346° × 105/360）
  // lift min=225, max=225+392.6=617.6
  float yaw_min_deg_{-786.0f};
  float yaw_max_deg_{786.0f};
  float extend_min_mm_{0.0f};
  float extend_max_mm_{191.6f};             // 伸缩行程，不涉及离地高度




  // ---------- 首次初始化标志 ----------
  bool lift_inited_{false};
  bool yaw_inited_{false};
  bool extend_inited_{false};

  // ---------- 方法 ----------
  void init();    // PID 参数初始化
  void update();  // 每控制周期调用一次：读取当前位置 → PID计算 → 写入电机指令

  // ======== 新增：吸盘控制方法 ========
  void pumpToggle();
  void valveToggle();
  void setPump(bool on);
  void setValve(bool on);
  // ======== mm接口 ========
  void addLiftDelta(float delta_mm);
  void addExtendDelta(float delta_mm);
  float getLiftHeightMm() const;
  float getExtendLengthMm() const;


private:
  float clampTarget(float target, float current, float min_val, float max_val);

};
