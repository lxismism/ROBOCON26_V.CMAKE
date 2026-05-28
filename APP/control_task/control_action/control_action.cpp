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
    // cur_* 保持当前值——从"现在的位置"开始往终点走
}

bool Ramp_Step(PickRamp& ramp, float dt) {
    if (!ramp.active) return false;

    bool done = true;

    // 抬升
    float lift_step = kPickLiftSpeed * dt;
    if (ramp.cur_lift_mm < ramp.end_lift_mm) {
        ramp.cur_lift_mm += lift_step;
        if (ramp.cur_lift_mm >= ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
        else done = false;
    } else if (ramp.cur_lift_mm > ramp.end_lift_mm) {
        ramp.cur_lift_mm -= lift_step;
        if (ramp.cur_lift_mm <= ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
        else done = false;
    }

    // 云台
    float yaw_step = kPickYawSpeed * dt;
    if (ramp.cur_yaw_deg < ramp.end_yaw_deg) {
        ramp.cur_yaw_deg += yaw_step;
        if (ramp.cur_yaw_deg >= ramp.end_yaw_deg) ramp.cur_yaw_deg = ramp.end_yaw_deg;
        else done = false;
    } else if (ramp.cur_yaw_deg > ramp.end_yaw_deg) {
        ramp.cur_yaw_deg -= yaw_step;
        if (ramp.cur_yaw_deg <= ramp.end_yaw_deg) ramp.cur_yaw_deg = ramp.end_yaw_deg;
        else done = false;
    }

    // 伸缩
    float extend_step = kPickExtendSpeed * dt;
    if (ramp.cur_extend_mm < ramp.end_extend_mm) {
        ramp.cur_extend_mm += extend_step;
        if (ramp.cur_extend_mm >= ramp.end_extend_mm) ramp.cur_extend_mm = ramp.end_extend_mm;
        else done = false;
    } else if (ramp.cur_extend_mm > ramp.end_extend_mm) {
        ramp.cur_extend_mm -= extend_step;
        if (ramp.cur_extend_mm <= ramp.end_extend_mm) ramp.cur_extend_mm = ramp.end_extend_mm;
        else done = false;
    }

    if (done) ramp.active = false;
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
