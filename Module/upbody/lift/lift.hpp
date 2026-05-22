/**
 * @file lift.hpp
 * @author lxlx
 * @brief 电梯抬升机构模块 —— 双3508同步抬升R2、存储KFS
 * @version 0.1
 * @date 2026.5.19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 两个3508必须保持同步，位置差过大会损坏同步带机构！
 * @note 后续需加入同步补偿算法（交叉耦合/电子齿轮），当前仅做位置差监测
 * @versioninfo v0.1: 搭框架，双电机独立位置环 + 同步误差检测
 */

#pragma once

#include "Motor.hpp"
#include "pid_controller.h"

class Lift {
public:
  // ---------- 电机引用 ----------
  C620Motor *left_motor_{nullptr};   // 3508 左侧抬升
  C620Motor *right_motor_{nullptr};  // 3508 右侧抬升

  // ---------- 各电机 PID ----------
  PID_t left_pid_;
  PID_t right_pid_;

  // ---------- 目标角度（两侧共用同一个目标，保证同步） ----------
  float target_deg_{0.0f};

  // ---------- 角度限位 ----------
  float min_deg_{0.0f};
  float max_deg_{360.0f};

  // ---------- 同步误差阈值（单位：度，超出则报警） ----------
  float sync_error_threshold_{5.0f};

  // ---------- 首次初始化标志 ----------
  bool inited_{false};

  // ---------- 方法 ----------
  void init();
  void update();

  // 获取两侧位置差（绝对值），供外部监控
  float getSyncError() const;

private:
  float clampAngle(float target, float current, float min_deg, float max_deg);
};
