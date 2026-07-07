/**
 * @file control_process.hpp
 * @author 大帅将军 / lxism
 * @brief 控制流程层 —— 各模式下的决策逻辑，调用 control_action 中的动作函数
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 本模块负责"什么时候做"，具体"怎么做"由 control_action 负责
 */

#pragma once

#include "FreeRTOS.h"       // ← 加这行
#include "topic_pool.h"

// ===== 调试模式（原用户模式） =====

/**
 * @brief 调试模式 —— 所有上身机构独立按键控制
 *
 * 摇杆直驱底盘（场地坐标系），各按键分别控制：
 * 电梯、吸取手升降/云台/伸缩、武器手升降/伸缩、
 * 真空泵、电磁阀、夹爪、腕部舵机
 */
void Debug_Mode_Process(TypedTopicPublisher<pub_upbody_cmd>& pub, pub_upbody_cmd& msg);

// ===== 队友模式 =====

/** @brief 队友模式入口：摇杆处理 + MF / Normal 模式分发 */
void Chassis_RM_Data_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg);

/** @brief 普通手操 / 定位模式（headless 双模式切换） */
void Normal_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub);
/** @brief 武馆半自动网格定位模式 */
void MC_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg);

/** @brief 梅林半自动网格定位模式 */
void MF_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg);

/** @brief 九宫格半自动网格定位模式 */
void Arena_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg);


/** @brief XY 目标位置 → PID → 场地坐标系速度指令 */
void Aim_State_xy_Process();

/** @brief 目标角度 → PID → 角速度指令 */
void Aim_State_omega_Process();

// ===== 上层调试模式 =====

/**
 * @brief 上层调试模式 —— 底盘摇杆直驱 + 方向键触发动作链
 *
 * btnDirUp=吸取顶端KFS, btnDirRight=吸取中层KFS, btnDirDown=吸取底层KFS
 * 摇杆与调试模式的底盘控制相同，其余按键暂时置空
 */
void UpperDebug_Mode_Process(TypedTopicPublisher<pub_upbody_cmd>& pub, pub_upbody_cmd& msg);

/** @brief 工具函数 清零position定位*/
void Reset_position();

/** @brief 工具函数 归一化*/
float Warp_ToRange(float value,float min,float max);

/** @brief 返回预测yaw*/
float Get_predict_yaw();

float clamp(float value,float min,float max);
