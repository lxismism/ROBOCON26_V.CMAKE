/**
 * @file pos_ctrl_task.cpp
 * @author lxlx
 * @brief 上身机构位置环执行任务 —— 订阅 upbody_cmd，消费后驱动7路电机 + GPIO + 舵机
 * @version 0.3
 * @date 2026-5-20
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 本任务只负责执行，决策由 control_task 完成
 * @note delta 累加到目标角度后由各模块 update() 内 clampAngle 限幅
 * @versioninfo v0.1: 7路电机位置环手动调试
 * @versioninfo v0.2: 接入三个机构模块统一管理
 * @versioninfo v0.3: 改为订阅 upbody_cmd，决策与执行分离
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

// ---------- 订阅上身控制指令 ----------
static TypedTopicSubscriber<pub_upbody_cmd> upbody_cmd_sub("upbody_cmd", 8);


void posCtrlTask(void *argument) {
  (void)argument;

  if (!upbody_cmd_sub.IsValid()) {
    return;
  }

  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {
    // ===== 步骤1：消费上身控制指令 =====
    pub_upbody_cmd upbody_msg;
    if (upbody_cmd_sub.TryGet(&upbody_msg)) {

      if (upbody_msg.active) {
        // --- 持续型：delta 累加到目标角度 ---
        pick_hand.lift_target_deg_   += upbody_msg.pick_lift_delta;
        pick_hand.yaw_target_deg_    += upbody_msg.pick_yaw_delta;
        pick_hand.extend_target_deg_ += upbody_msg.pick_extend_delta;

        weapon_hand.lift_target_deg_   += upbody_msg.weapon_lift_delta;
        weapon_hand.extend_target_deg_ += upbody_msg.weapon_extend_delta;

        lift.target_deg_ += upbody_msg.lift_delta;

        // --- 切换型：执行GPIO/舵机动作（仅上升沿单帧为true） ---
        if (upbody_msg.pump_toggle)
          pick_hand.pumpToggle();

        if (upbody_msg.valve_toggle)
          pick_hand.valveToggle();

        if (upbody_msg.claw_toggle)
          weapon_hand.clawToggle();

        if (upbody_msg.wrist_toggle)
          weapon_hand.wristFlip();
      }
      // active=false → 队友模式，不处理上身指令
    }

    // ===== 步骤2：三个机构模块各跑自己的位置环 =====
    pick_hand.update();
    weapon_hand.update();
    lift.update();

    vTaskDelayUntil(&currentTime, 5);
  }
}
