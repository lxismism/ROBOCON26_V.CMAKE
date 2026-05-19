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

  // ---------- 目标角度 ----------
  float lift_target_deg_{0.0f};
  float yaw_target_deg_{0.0f};
  float extend_target_deg_{0.0f};

  // ---------- 角度限位（单位：度，上电后实测填入） ----------
  float lift_min_deg_{0.0f};
  float lift_max_deg_{360.0f};
  float yaw_min_deg_{0.0f};
  float yaw_max_deg_{360.0f};
  float extend_min_deg_{0.0f};
  float extend_max_deg_{360.0f};


  // ---------- 首次初始化标志 ----------
  bool lift_inited_{false};
  bool yaw_inited_{false};
  bool extend_inited_{false};

  // ---------- 方法 ----------
  void init();    // PID 参数初始化
  void update();  // 每控制周期调用一次：读取当前位置 → PID计算 → 写入电机指令

private:
  float clampAngle(float target, float current, float min_deg, float max_deg);
};
