/**
 * @file control_action.cpp
 * @author lxism
 * @brief 底层动作函数实现
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 */

#include "control_action.hpp"
// <cmath> 已通过 control_action.hpp 间接包含，无需重复         // ← 这行不需要写，只是说明

// ===== 辅助工具 =====

int8_t sign(double value) {
    if (value > 0) return 1;
    else if (value < 0) return -1;
    else return 0;
}

// ===== 摇杆处理 =====

float JoyToVelocity(uint16_t raw, uint16_t deadzone, float max_vel) {
    int32_t diff = (int32_t)raw - (int32_t)kJoyCenter;
    if (ABS(diff) <= (int32_t)deadzone) {
        return 0.0f;
    }
    return (float)(diff - sign(diff) * (int32_t)deadzone)
           / (float)(kJoyCenter - deadzone)
           * max_vel;
}

// ===== 坐标变换 =====

void ApplyFieldCentricRotation(float& vx, float& vy, float yaw_deg) {
    float angle_deg = atan2(vy, vx) / kDegToRad;
    float mag = sqrt(vx * vx + vy * vy);
    vx = mag * cos((angle_deg - yaw_deg) * kDegToRad);
    vy = mag * sin((angle_deg - yaw_deg) * kDegToRad);
}

// ===== 绝对姿态动作 =====

void Action_GoToPose(pub_upbody_cmd& msg, const RobotPose& pose) {
    msg.set_absolute_pose      = true;
    msg.pick_lift_target_mm    = pose.pick_lift_mm;
    msg.pick_yaw_target_deg    = pose.pick_yaw_deg;
    msg.pick_extend_target_mm  = pose.pick_extend_mm;
    msg.weapon_lift_target_mm  = pose.weapon_lift_mm;
    msg.weapon_extend_target_mm = pose.weapon_extend_mm;
    msg.lift_target_mm         = pose.lift_mm;
}

// ===== 自包含动作 =====

void Action_KFS_Low(TypedTopicPublisher<pub_upbody_cmd>& pub) {
    pub_upbody_cmd msg = {};
    msg.active = true;
    Action_GoToPose(msg, kPose_KFS_Low);
    pub.Publish(msg);
}

void Action_KFS_Mid(TypedTopicPublisher<pub_upbody_cmd>& pub) {
    pub_upbody_cmd msg = {};
    msg.active = true;
    Action_GoToPose(msg, kPose_KFS_Mid);
    pub.Publish(msg);
}

void Action_KFS_High(TypedTopicPublisher<pub_upbody_cmd>& pub) {
    pub_upbody_cmd msg = {};
    msg.active = true;
    Action_GoToPose(msg, kPose_KFS_High);
    pub.Publish(msg);
}

// ===== 渐变动作 =====
void Ramp_Start(PickRamp& ramp, const RobotPose& pose) {
    ramp.end_lift_mm   = pose.pick_lift_mm;
    ramp.end_yaw_deg   = pose.pick_yaw_deg;
    ramp.end_extend_mm = pose.pick_extend_mm;
    ramp.active        = true;

    // 自动策略：检测是否需要两段式
    bool going_to_danger  = (pose.pick_yaw_deg < 0.0f);                // 目标在负数区
    bool leaving_danger   = (ramp.cur_yaw_deg < 0.0f && pose.pick_yaw_deg >= 0.0f); // 从负数区离开

    if (going_to_danger) {
        ramp.phase = 1;  // 先抬升到位 → 再转云台
    } else if (leaving_danger) {
        ramp.phase = 2;  // 先转云台出负数区 → 再降抬升
    } else {
        ramp.phase = 0;  // 正常，所有轴同步
    }

    // 伸缩安全策略（优先级低于 yaw 安全策略）
    if (ramp.phase == 0) {
        // 目标缩回到安全位 → 先缩回
        if (pose.pick_extend_mm < 1.0f && ramp.cur_extend_mm > 1.0f)
            ramp.phase = 3;
        // 从缩回状态伸出且 yaw 在危险角 → 先转云台
        else if (ramp.cur_extend_mm < 1.0f && pose.pick_extend_mm >= 1.0f
                 && ramp.cur_yaw_deg <= 0.0f)
            ramp.phase = 4;
    }
    
}

bool Ramp_Step(PickRamp& ramp, float dt) {
    if (!ramp.active) return false;


        // 抬升
    bool lift_done = true;
    if (fabsf(ramp.cur_lift_mm - ramp.end_lift_mm) > 0.01f) {
        // phase=2 时锁定抬升，phase=3 时锁定抬升（先缩回）
        if (ramp.phase != 2 && ramp.phase != 3) {
            float lift_step = kPickLiftSpeed * dt;
            if (ramp.cur_lift_mm < ramp.end_lift_mm) {
                ramp.cur_lift_mm += lift_step;
                if (ramp.cur_lift_mm > ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
            } else {
                ramp.cur_lift_mm -= lift_step;
                if (ramp.cur_lift_mm < ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
            }
        }
        if (fabsf(ramp.cur_lift_mm - ramp.end_lift_mm) > 0.01f) lift_done = false;
    }

    // 云台
    bool yaw_done = true;
    if (fabsf(ramp.cur_yaw_deg - ramp.end_yaw_deg) > 0.1f) {
        // phase=1 时锁定云台（等抬升到安全高度），phase=3 时锁定云台
        if (ramp.phase != 1 && ramp.phase != 3) {
            float yaw_step = kPickYawSpeed * dt;
            if (ramp.cur_yaw_deg < ramp.end_yaw_deg) {
                ramp.cur_yaw_deg += yaw_step;
                if (ramp.cur_yaw_deg > ramp.end_yaw_deg) ramp.cur_yaw_deg = ramp.end_yaw_deg;
            } else {
                ramp.cur_yaw_deg -= yaw_step;
                if (ramp.cur_yaw_deg < ramp.end_yaw_deg) ramp.cur_yaw_deg = ramp.end_yaw_deg;
            }
        }
        if (fabsf(ramp.cur_yaw_deg - ramp.end_yaw_deg) > 0.1f) yaw_done = false;
    }


        // 伸缩（phase=4 时锁定——等 yaw > 0 再伸）
    bool extend_done = true;
    if (fabsf(ramp.cur_extend_mm - ramp.end_extend_mm) > 0.01f) {
        if (ramp.phase != 4) {
            float extend_step = kPickExtendSpeed * dt;
            if (ramp.cur_extend_mm < ramp.end_extend_mm) {
                ramp.cur_extend_mm += extend_step;
                if (ramp.cur_extend_mm >= ramp.end_extend_mm) ramp.cur_extend_mm = ramp.end_extend_mm;
                else extend_done = false;
            } else if (ramp.cur_extend_mm > ramp.end_extend_mm) {
                ramp.cur_extend_mm -= extend_step;
                if (ramp.cur_extend_mm <= ramp.end_extend_mm) ramp.cur_extend_mm = ramp.end_extend_mm;
                else extend_done = false;
            }
        }
        if (fabsf(ramp.cur_extend_mm - ramp.end_extend_mm) > 0.01f) extend_done = false;
    }


    // 判断 phase 切换（移到伸缩段后面）
    if (ramp.phase == 1 && lift_done)      ramp.phase = 0;
    if (ramp.phase == 2 && yaw_done)       ramp.phase = 0;
    if (ramp.phase == 3 && extend_done)    ramp.phase = 0;
    if (ramp.phase == 4 && ramp.cur_yaw_deg > 0.0f) ramp.phase = 0;

    if (lift_done && yaw_done && extend_done) ramp.active = false;
    return ramp.active;
}

void Ramp_ToMsg(const PickRamp& ramp, pub_upbody_cmd& msg) {
    msg.set_absolute_pose       = true;
    msg.pick_lift_target_mm     = ramp.cur_lift_mm;
    msg.pick_yaw_target_deg     = ramp.cur_yaw_deg;
    msg.pick_extend_target_mm   = ramp.cur_extend_mm;
    msg.weapon_lift_target_mm   = 0.0f;
    msg.weapon_extend_target_mm = 0.0f;
    msg.lift_target_mm          = 0.0f;
}
