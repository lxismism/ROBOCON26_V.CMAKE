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
#include "topics.hpp"   
#include <cmath>                          

// ===== 摇杆 + 坐标常量 =====            
inline constexpr uint16_t kJoyCenter = 32767;
inline constexpr uint16_t kJoyDeadZoneLeft = 3500;
inline constexpr uint16_t kJoyDeadZoneRight = 2000;
inline constexpr float kDegToRad = M_PI / 180.0f;

#ifndef ABS                               
#define ABS(x) ((x) < 0 ? -(x) : (x))   
#endif                                    

// ===== 动作渐变速度（mm/s 或 °/s） =====
inline constexpr float kPickLiftSpeed   = 160.0f;  // 吸取手抬升
inline constexpr float kPickYawSpeed    = 160.0f; // 云台旋转
inline constexpr float kPickExtendSpeed = 40.0f;
inline constexpr float kWeaponLiftSpeed   = 40.0f;   // 武器手抬升 mm/s
inline constexpr float kWeaponExtendSpeed = 30.0f;   // 武器手伸缩 mm/s
inline constexpr float kLiftSpeed         = 30.0f;   // 电梯抬升 mm/s


// ===== 渐变状态：记录当前中间目标和终点 =====
struct RampState {
    bool  active = false;
    int   phase  = 0;

    // 吸取手
    float cur_pick_lift_mm   = 0.0f;
    float cur_pick_yaw_deg   = 0.0f;
    float cur_pick_extend_mm = 0.0f;
    float end_pick_lift_mm   = 0.0f;
    float end_pick_yaw_deg   = 0.0f;
    float end_pick_extend_mm = 0.0f;

    // 武器手
    float cur_weapon_lift_mm   = 0.0f;
    float cur_weapon_extend_mm = 0.0f;
    float end_weapon_lift_mm   = 0.0f;
    float end_weapon_extend_mm = 0.0f;

    // 电梯
    float cur_lift_mm = 0.0f;
    float end_lift_mm = 0.0f;
};



// ===== 机器人全身姿态 =====

struct RobotPose {
    float pick_lift_mm;
    float pick_yaw_deg;
    float pick_extend_mm;
    float weapon_lift_mm;
    float weapon_extend_mm; 
    float lift_mm;
};

inline constexpr RobotPose kPose_KFS_Low  = {0.0f,   392.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_KFS_Mid  = {159.2f, 392.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_KFS_High = {352.6f, 392.0f, 0.0f, 347.0f, 0.0f, 0.0f};

inline constexpr RobotPose kPose_Home     = {0.0f,   0.0f,   0.0f,   0.0f, 0.0f, 0.0f};  // 复位

inline constexpr RobotPose kPose_Place1   = {390.4f, -292.0f, 70.59f, 347.0f, 0.0f, 240.0f};  // 放置远处
inline constexpr RobotPose kPose_Place2   = {390.0f, -99.0f, 0.0f, 347.0f, 0.0f, 240.0f};

// ===== 摇杆处理 =====

float JoyToVelocity(uint16_t raw, uint16_t deadzone, float max_vel);

// ===== 坐标变换 =====

void ApplyFieldCentricRotation(float& vx, float& vy, float yaw_deg);

// ===== 绝对姿态动作 =====

void Action_GoToPose(pub_upbody_cmd& msg, const RobotPose& pose);

// ===== 自包含动作（内部构造消息+发布，调用方一行搞定） =====
void Action_KFS_Low (TypedTopicPublisher<pub_upbody_cmd>& pub);
void Action_KFS_Mid (TypedTopicPublisher<pub_upbody_cmd>& pub);
void Action_KFS_High(TypedTopicPublisher<pub_upbody_cmd>& pub);

// ===== 渐变动作（每帧调用，自动按速度逼近终点） =====

void Ramp_Start(RampState& ramp, const RobotPose& pose);
bool Ramp_Step(RampState& ramp, float dt);
void Ramp_ToMsg(const RampState& ramp, pub_upbody_cmd& msg);
