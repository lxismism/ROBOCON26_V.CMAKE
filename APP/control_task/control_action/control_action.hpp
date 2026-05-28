/**
 * @file control_action.hpp
 * @author lxism
 * @brief 底层动作函数 —— 摇杆处理、坐标变换，供 control_process 调用
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 本模块不参与决策逻辑，只提供原子动作函数
 */

#pragma once

#include "FreeRTOS.h"       // ← 加这行，必须在 topic_pool.h 之前
#include "topic_pool.h"
#include <cmath>                          

// ===== 摇杆 + 坐标常量 =====            
inline constexpr uint16_t kJoyCenter = 32767;
inline constexpr uint16_t kJoyDeadZoneLeft = 3500;
inline constexpr uint16_t kJoyDeadZoneRight = 2000;
inline constexpr float kDegToRad = M_PI / 180.0f;

#ifndef ABS                               
#define ABS(x) ((x) < 0 ? -(x) : (x))     
#endif                                    

// ===== 辅助工具 =====                     

/** @brief 符号函数 —— 正数返回1，负数返回-1，零返回0 */
int8_t sign(double value);

// ===== 摇杆处理 =====                  

/**
 * @brief 单轴摇杆 → 速度（含死区处理）
 *
 * 将16位ADC摇杆原始值（0~65535，中心=32767）经死区过滤后，
 * 线性映射为 [-max_vel, +max_vel] 范围内的速度值
 *
 * @param raw       摇杆原始值
 * @param deadzone  死区阈值，偏离中心小于此值则输出0
 * @param max_vel   最大速度（m/s 或 rad/s）
 * @return float    速度值
 */
float JoyToVelocity(uint16_t raw, uint16_t deadzone, float max_vel);

// ===== 坐标变换 =====                  

/**
 * @brief 场地坐标系旋转（无头模式核心变换）
 *
 * 摇杆推"前"总是对应场地的前方，而不是机器人当前朝向的前方。
 * 本质：将速度向量 (vx, vy) 绕原点旋转 -yaw_deg 度
 *
 * @param vx       [in/out] x方向速度
 * @param vy       [in/out] y方向速度
 * @param yaw_deg  机器人当前偏航角（度）
 */
void ApplyFieldCentricRotation(float& vx, float& vy, float yaw_deg);
