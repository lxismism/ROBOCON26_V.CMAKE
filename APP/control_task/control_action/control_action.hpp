/**
 * @file control_action.hpp
 * @author lxism
 * @brief 底层动作函数 —— 摇杆处理、坐标变换、动作控制器，供 control_process 调用
 * @version 0.2
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention 动作函数在 ActionController 类中，process 层只需调用 GrabKFS/PlaceKFS/GetKFS/GoHome + Update
 */

#pragma once

#include "FreeRTOS.h"
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

// ===== 动作速度配置 =====
struct ActionSpeeds {
    float pick_lift     = 190.0f;   // 吸取手抬升 mm/s
    float pick_yaw      = 250.0f;   // 云台旋转 °/s
    float pick_extend   = 100.0f;    // 吸取手伸缩 mm/s
    float weapon_lift   = 160.0f;    // 武器手抬升 mm/s
    float weapon_extend = 100.0f;    // 武器手伸缩 mm/s
    float lift          = 40.0f;    // 电梯 mm/s
};

// ===== 动作优先级配置 =====
struct ActionPriorities {
    int pick_lift     = -1;   // -1 = 不受限，与其他轴并发
    int pick_yaw      = -1;
    int pick_extend   = -1;
    int weapon_lift   = -1;
    int weapon_extend = -1;
    int lift          = -1;
};

// ===== 渐变状态 =====
struct RampState {
    bool  active = false;
    uint8_t step_done_mask = 0x3F;

    ActionSpeeds    speeds;
    ActionPriorities priorities;

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

inline constexpr RobotPose kPose_Home     = {0.0f,   0.0f,   0.0f,   0.0f, 0.0f, 0.0f};

inline constexpr RobotPose kPose_Place1   = {78.1f, -302.0f, 160.59f, 347.0f, 0.0f, 100.0f};
inline constexpr RobotPose kPose_Place2   = {78.1f, -129.0f, 70.0f, 347.0f, 0.0f, 100.0f};
inline constexpr RobotPose kPose_Place3   = {280.0f, -190.5f, 0.0f, 347.0f, 0.0f, 0.0f};


inline constexpr RobotPose kPose_Grid9_Bot12 = {300.80f, 403.0f, 0.0f, 347.0f, 0.0f, 140.0f};
inline constexpr RobotPose kPose_Grid9_Bot3  = {300.80f, 778.0f, 0.0f, 347.0f, 0.0f, 140.0f};

inline constexpr RobotPose kPose_Get1   = {98.1f, -292.0f, 290.0f, 347.0f, 0.0f, 140.0f};
inline constexpr RobotPose kPose_Get2   = {98.1f, -129.0f, 120.0f, 347.0f, 0.0f, 140.0f};

inline constexpr RobotPose kPose_R2_First_Floor = {300.80f, 778.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_R2_Second_Floor = {300.80f, 778.0f, 0.0f, 347.0f, 0.0f, 257.3f};

// ===== 一步动作的完整配置 =====
struct ActionConfig {
    RobotPose       target;
    ActionSpeeds    speeds;
    ActionPriorities priorities;
    uint8_t step_done_mask = 0x3F;  // 位0=pick_lift, 1=pick_yaw, 2=pick_extend
                                      // 位3=weapon_lift, 4=weapon_extend, 5=lift
    bool skip_safety = false;        // true=跳过安全修正，用显式设的优先级
};

// ===== 动作控制器 =====
class ActionController {
public:
    // ---- 动作函数（process 层调用） ----
    void GrabKFS (const RobotPose& pose);
    void GrabKFS_Arena(const RobotPose& pose);
    void PlaceKFS(const RobotPose& pose);
    void GetKFS  (const RobotPose& pose);
    void R2MergePose(const RobotPose& pose);



    void GoHome();

    // ---- 步队列 ----
    void AddStep(const ActionConfig& config);
    void RunSteps();

    // ---- 区分步间和全完成 ---- 
    bool HasPending() const { return step_index_ < step_count_; }


    // ---- 每帧调用 ----
    void Update(float dt, TypedTopicPublisher<pub_upbody_cmd>& pub);

    // ---- 查询 ----
    bool IsActive() const;
    void SyncState(const RobotPose& current);


protected:
    void Start_(const ActionConfig& config);

private:
    // ---- 内部状态 ----
    RampState ramp_;
    static constexpr int kMaxSteps = 4;
    ActionConfig step_queue_[kMaxSteps];
    int step_count_ = 0;
    int step_index_ = 0;

    // ---- 内部实现 ----
    void Step_(float dt);
    void ToMsg_(pub_upbody_cmd& msg) const;
    void ApplySafety_(ActionConfig& config);
};

// ===== 摇杆处理 =====
float JoyToVelocity(uint16_t raw, uint16_t deadzone, float max_vel);

// ===== 坐标变换 =====
void ApplyFieldCentricRotation(float& vx, float& vy, float yaw_deg);
