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
  PID_t platfrom_pos_pid_;  // 平台位置环，ref为平台高度（度），out为弧度/秒
  PID_t left_v_pid_;        // 平台左侧速度环，ref为左侧速度（弧度/秒），out为电流（mA）
  PID_t right_v_pid_;       // 平台右侧速度环，ref为右侧速度（弧度/秒），out为电流（mA）

  float sync_error;

  // ---------- 目标角度（两侧共用同一个目标，保证同步） ----------
  float target_deg_{0.0f};

  // ---------- 换算系数：电机输出轴1°对应同步带位移(mm) ----------
  static constexpr float kMmPerDeg = 130.0f / 360.0f;
  

  float ground_clearance_mm_{225.0f};  // encoder=0时平台离地高度
  float travel_max_mm_{257.3f};        // 最大抬升行程（1425° × 65/360）
  // min = ground_clearance, max = ground_clearance + travel_max

  // ---------- 同步误差阈值（单位：度，超出则报警） ----------
  float sync_error_threshold_{5.0f};
  // ---------- 首次初始化标志 ----------
  bool inited_{false};

  // ---------- 方法 ----------
  void init();
  void update();

  // 获取两侧位置差（绝对值），供外部监控
  float getSyncError() const;

  /// @brief 增减目标高度，正=升，负=降，单位mm
  void addTargetDelta(float delta_mm);

  /// @brief 获取当前高度（mm，相对值）
  float getCurrentHeightMm() const;

  /// @brief 获取同步误差（mm）
  float getSyncErrorMm() const;


private:
  float platform_pos_out_;
  uint8_t prescaler_cnt;
  float target_v_left_;
  float target_v_right_;
  float clampTarget(float target_mm, float current_mm, float min_mm, float max_mm);
};
