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
#include <cstdint>
#include <cstddef>

// ===== 摇杆 + 坐标常量 =====
inline constexpr uint16_t kJoyCenter = 985;
inline constexpr uint16_t kJoyDeadZoneLeft = 100;
inline constexpr uint16_t kJoyDeadZoneRight = 100;
inline constexpr float kDegToRad = M_PI / 180.0f;

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum Height : size_t{
    Low = 0 ,
    Mid = 1 ,
    High = 2 ,
};

// ===== 动作速度配置 =====
struct ActionSpeeds {
    float pick_lift     = 350.0f;   // 吸取手抬升 mm/s
    float pick_yaw      = 500.0f;   // 云台旋转 °/s
    float pick_extend   = 280.0f;   // 吸取手伸缩 mm/s
    float weapon_lift   = 180.0f;   // 武器手抬升 mm/s
    float weapon_extend = 180.0f;   // 武器手伸缩 mm/s
    float lift          = 70.0f;    // 电梯 mm/s
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

    int8_t pending_pump_cmd = 0;
    int8_t pending_valve_cmd = 0;
    int8_t pump_cmd_at_done  = 0;
    int8_t valve_cmd_at_done = 0;


    // 底盘逼近
    bool chassis_approach_active = false;  // 当前步是否触发底盘前移逼近
    
    bool chassis_released = false;
    bool chassis_release_pending = false;

    uint16_t dwell_ms = 0;          // 本步驻留时间（毫秒）
    uint16_t dwell_timer_ms = 0;    // 驻留计时器（毫秒）


};





// ===== 机器人全身姿态 =====
struct RobotPose {
    float pick_lift_mm;
    float pick_yaw_deg;
    float pick_extend_mm;
    float weapon_lift_mm;
    float weapon_extend_mm;
    float lift_mm;
    int8_t name = -1;
};

enum Name : int8_t{
    Pose_Place0                  =   0 ,
    Pose_Place1                  =   1 ,
    Pose_Place2                  =   2 ,

    Pose_pick0                   =   3 ,
    Pose_pick1                   =   4 ,
    Pose_pick2                   =   5 ,

    Pose_Grid9_Bot12             =   6 ,
    Pose_Grid9_Bot3              =   7 ,

};

inline constexpr RobotPose kPose_KFS_Low  = {0.0f,   392.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_KFS_Mid  = {159.2f, 392.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_KFS_High = {352.6f, 392.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_Moving_In_MF = {352.6f, 0.0f, 0.0f, 347.0f, 0.0f, 0.0f};

inline constexpr RobotPose kPose_Home     = {0.0f,   0.0f,   0.0f,   0.0f, 0.0f, 0.0f};

inline constexpr RobotPose kPose_Place[3]   = {
    {78.1f, -312.0f, 150.59f, 347.0f, 0.0f, 100.0f,Pose_Place0},
    {78.1f, -139.0f, 140.0f, 347.0f, 0.0f, 100.0f,Pose_Place1},
    {370.0f, -220.5f, 0.0f, 347.0f, 0.0f, 0.0f,Pose_Place2}
};
inline constexpr RobotPose kPose_Pick[3] = {
    {0.0f  , 392.0f, 236.6f, 347.0f, 0.0f, 0.0f,Pose_pick0},
    {179.2f, 392.0f, 236.6f, 347.0f, 0.0f, 0.0f,Pose_pick1},
    {390.6f, 392.0f, 236.6f, 347.0f, 0.0f, 0.0f,Pose_pick2}
};

inline constexpr RobotPose kPose_Grid9_Bot12 = {320.80f, 403.0f, 0.0f, 347.0f, 0.0f, 0.0f, Pose_Grid9_Bot12};
inline constexpr RobotPose kPose_Grid9_Bot3  = {320.80f, 778.0f, 0.0f, 347.0f, 0.0f, 0.0f, Pose_Grid9_Bot3};

inline constexpr RobotPose kPose_Get1   = {98.1f, -312.0f, 201.6f, 347.0f, 0.0f, 140.0f};
inline constexpr RobotPose kPose_Get2   = {98.1f, -139.0f, 201.6f, 347.0f, 0.0f, 140.0f};

inline constexpr RobotPose kPose_Poke1  = {430.80f, 778.0f, 0.0f, 317.0f, 200.0f, 0.0f, -1};       // 腕部0°朝上，戳第一层
inline constexpr RobotPose kPose_Poke2  = {430.80f, 778.0f, 0.0f, 347.0f, 200.0f, 0.0f, -1};    // 腕部60°下翻，戳第二层


inline constexpr RobotPose kPose_R2_First_Floor = {300.80f, 778.0f, 0.0f, 347.0f, 0.0f, 0.0f};
inline constexpr RobotPose kPose_R2_Second_Floor = {300.80f, 778.0f, 0.0f, 347.0f, 0.0f, 257.3f};

extern RobotPose kPose_Now;   // 当前上身姿态


// ===== 一步动作的完整配置 =====
struct ActionConfig {
    RobotPose       target;
    ActionSpeeds    speeds;
    ActionPriorities priorities;
    uint8_t step_done_mask = 0x3F;  // 位0=pick_lift, 1=pick_yaw, 2=pick_extend
                                      // 位3=weapon_lift, 4=weapon_extend, 5=lift, 6=wrist

    bool skip_safety = false;

    // ===== 新增：泵/阀自动控制 =====
    int8_t pump_cmd       = 0;
    int8_t valve_cmd      = 0;
    int8_t pump_cmd_done  = 0;
    int8_t valve_cmd_done = 0;

    bool enable_chassis_approach = false;  // 本步期间触发底盘逼近
    bool release_chassis = false;
    uint16_t dwell_ms = 0;          // 到位后驻留时间（毫秒），0=不等



};


// ===== 动作控制器 =====
class ActionController {
public:
    // ---- 动作函数（process 层调用） ----
    void GoHome();

    //MF
    void GrabKFS (const RobotPose& pose);
    void PlaceKFS(const RobotPose& pose);
    void PickKFS(const RobotPose& pose_Grab, const RobotPose& pose_Place, bool close_pump_at_end = true);
    void Moving(const RobotPose& pose);

    //Arena
    void GetKFS  (const RobotPose& pose);
    void GrabKFS_Arena(const RobotPose& pose);
    void PokeWeapon(const RobotPose& pose, int wrist_preset);   // 九宫格武器子模式：摆到戳的姿态
    void PrepareWeapon();   // 武馆模式：武器手抬升缩回竖杆，渐变过渡

    
    void R2MergePose(const RobotPose& pose);

//-------------------------------------


    // ---- 步队列 ----
    void AddStep(const ActionConfig& config);
    void RunSteps();

    // ---- 区分步间和全完成 ---- 
    bool HasPending() const { return step_index_ < step_count_; }


    // ---- 每帧调用 ----
    void Update(float dt, TypedTopicPublisher<pub_upbody_cmd>& pub);

    // ---- 查询 ----
    bool IsActive() const;
    bool IsApproachPhase() const { return ramp_.active && ramp_.chassis_approach_active; }
    bool IsChassisReleased() const { return ramp_.chassis_released; }
    void SyncState(const RobotPose& current);



protected:
    void Start_(const ActionConfig& config);

private:
    // ---- 内部状态 ----
    RampState ramp_;
    static constexpr int kMaxSteps = 10;
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

int8_t sign (double value);
