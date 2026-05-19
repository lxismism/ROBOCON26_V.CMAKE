/**
 * @file pos_ctrl_task.cpp
 * @author lxlx
 * @brief 上层机构7电机位置环控制任务 —— 通过三个机构模块统一管理
 * @version 0.2
 * @date 2026-5-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 上电首次循环以当前角度初始化所有目标值，避免电机跳变
 * @note Xbox控制：A键切换电机 / LT增角度 / RT减角度；后续接入自动化逻辑替代手动测试
 * @versioninfo v0.2: 三个模块统一调用，Xbox手动选电机测试
 */

#include "pos_ctrl_task.hpp"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "pick_hand.hpp"
#include "weapon_hand.hpp"
#include "lift.hpp"
#include "topic_pool.h"
#include "topics.hpp"

// ---------- 引用 com_config.cpp 中的机构模块对象 ----------
extern PickHand pick_hand;
extern WeaponHand weapon_hand;
extern Lift lift;

osThreadId_t PosCtrlTaskHandle;

// ---------- 订阅 Xbox 手柄数据 ----------
static TypedTopicSubscriber<pub_Xbox_Data> xbox_sub("xbox", 8);

// ---------- 常量 ----------
static constexpr uint16_t kTriggerThreshold = 512;
static constexpr float kStepAngle = 60.0f;
static constexpr int kMaxMotors = 7;

// ---------- 电机编号 ----------
enum MotorIndex : int {
  kPickerLift   = 0,  // pick_hand 抬升 (3508)
  kPickerYaw    = 1,  // pick_hand 云台 (2006)
  kPickerExtend = 2,  // pick_hand 伸缩 (2006)
  kWeaponLift   = 3,  // weapon_hand 抬升 (3508)
  kWeaponExtend = 4,  // weapon_hand 伸缩 (2006)
  kLift         = 5,  // lift 双电机同步 (3508×2)
  kMotorCount   = 6
};

// ---------- 电机名称数组（调试用，通过 DebugSerial 观察） ----------
static const char *motor_names[] = {
    "picker_lift", "picker_yaw", "picker_extend",
    "weapon_lift", "weapon_extend",
    "lift",         // 左右同步
};

// ---------- 控制状态 ----------
static int selected_motor = 0;
static bool last_up_pressed = false;
static bool last_down_pressed = false;
static bool last_lt_pressed = false;
static bool last_rt_pressed = false;


void posCtrlTask(void *argument) {
  (void)argument;

  if (!xbox_sub.IsValid()) {
    return;
  }

  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {
    // ===== 步骤1：Xbox 输入 & 边缘检测 =====
    pub_Xbox_Data xbox_data;
    if (xbox_sub.TryGet(&xbox_data)) {

      // --- A键：切换选中电机 ---
      // --- 方向键上下：切换选中电机 ---
      if (xbox_data.btnDirUp && !last_up_pressed) {
        selected_motor = (selected_motor + 1) % kMotorCount;
      }
      last_up_pressed = xbox_data.btnDirUp;

      if (xbox_data.btnDirDown && !last_down_pressed) {
        selected_motor = (selected_motor - 1 + kMotorCount) % kMotorCount;
      }
      last_down_pressed = xbox_data.btnDirDown;


      // --- LT：选中电机目标角度增加 ---
      bool lt_pressed = (xbox_data.trigLT > kTriggerThreshold);
      if (lt_pressed && !last_lt_pressed) {
        float step = kStepAngle;
        switch (selected_motor) {
          case kPickerLift:   pick_hand.lift_target_deg_   += step; break;
          case kPickerYaw:    pick_hand.yaw_target_deg_    += step; break;
          case kPickerExtend: pick_hand.extend_target_deg_ += step; break;
          case kWeaponLift:   weapon_hand.lift_target_deg_   += step; break;
          case kWeaponExtend: weapon_hand.extend_target_deg_ += step; break;
          case kLift:         lift.target_deg_               += step; break;
          default: break;
        }
      }
      last_lt_pressed = lt_pressed;

      // --- RT：选中电机目标角度减少 ---
      bool rt_pressed = (xbox_data.trigRT > kTriggerThreshold);
      if (rt_pressed && !last_rt_pressed) {
        float step = kStepAngle;
        switch (selected_motor) {
          case kPickerLift:   pick_hand.lift_target_deg_   -= step; break;
          case kPickerYaw:    pick_hand.yaw_target_deg_    -= step; break;
          case kPickerExtend: pick_hand.extend_target_deg_ -= step; break;
          case kWeaponLift:   weapon_hand.lift_target_deg_   -= step; break;
          case kWeaponExtend: weapon_hand.extend_target_deg_ -= step; break;
          case kLift:         lift.target_deg_               -= step; break;
          default: break;
        }
      }
      last_rt_pressed = rt_pressed;
    }

    // ===== 步骤2：三个机构模块各跑自己的位置环 =====
    pick_hand.update();
    weapon_hand.update();
    lift.update();

    vTaskDelayUntil(&currentTime, 5);
  }
}
