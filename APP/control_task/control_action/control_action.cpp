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
void Ramp_Start(RampState& ramp, const RobotPose& pose) {
    // 吸取手
    ramp.end_pick_lift_mm   = pose.pick_lift_mm;
    ramp.end_pick_yaw_deg   = pose.pick_yaw_deg;
    ramp.end_pick_extend_mm = pose.pick_extend_mm;
    // 武器手
    ramp.end_weapon_lift_mm   = pose.weapon_lift_mm;
    ramp.end_weapon_extend_mm = pose.weapon_extend_mm;
    // 电梯
    ramp.end_lift_mm = pose.lift_mm;

    ramp.active = true;

    // 自动策略：检测拾取手是否需要两段式（只涉及 pick_hand）
    bool going_to_danger  = (pose.pick_yaw_deg < 0.0f);
    bool leaving_danger   = (ramp.cur_pick_yaw_deg < 0.0f && pose.pick_yaw_deg >= 0.0f);

    if (going_to_danger) {
        ramp.phase = 1;
    } else if (leaving_danger) {
        ramp.phase = 2;
    } else {
        ramp.phase = 0;
    }

    // 伸缩安全策略
    if (ramp.phase == 0) {
        if (pose.pick_extend_mm < 1.0f && ramp.cur_pick_extend_mm > 1.0f)
            ramp.phase = 3;
        else if (ramp.cur_pick_extend_mm < 1.0f && pose.pick_extend_mm >= 1.0f
                 && ramp.cur_pick_yaw_deg <= 0.0f)
            ramp.phase = 4;
    }
}

bool Ramp_Step(RampState& ramp, float dt) {
    if (!ramp.active) return false;

    // ---- 吸取手抬升 ----
    bool pick_lift_done = true;
    if (fabsf(ramp.cur_pick_lift_mm - ramp.end_pick_lift_mm) > 0.01f) {
        if (ramp.phase != 2 && ramp.phase != 3) {
            float step = kPickLiftSpeed * dt;
            if (ramp.cur_pick_lift_mm < ramp.end_pick_lift_mm) {
                ramp.cur_pick_lift_mm += step;
                if (ramp.cur_pick_lift_mm > ramp.end_pick_lift_mm) ramp.cur_pick_lift_mm = ramp.end_pick_lift_mm;
            } else {
                ramp.cur_pick_lift_mm -= step;
                if (ramp.cur_pick_lift_mm < ramp.end_pick_lift_mm) ramp.cur_pick_lift_mm = ramp.end_pick_lift_mm;
            }
        }
        if (fabsf(ramp.cur_pick_lift_mm - ramp.end_pick_lift_mm) > 0.01f) pick_lift_done = false;
    }

    // ---- 吸取手云台 ----
    bool pick_yaw_done = true;
    if (fabsf(ramp.cur_pick_yaw_deg - ramp.end_pick_yaw_deg) > 0.1f) {
        if (ramp.phase != 1 && ramp.phase != 3) {
            float step = kPickYawSpeed * dt;
            if (ramp.cur_pick_yaw_deg < ramp.end_pick_yaw_deg) {
                ramp.cur_pick_yaw_deg += step;
                if (ramp.cur_pick_yaw_deg > ramp.end_pick_yaw_deg) ramp.cur_pick_yaw_deg = ramp.end_pick_yaw_deg;
            } else {
                ramp.cur_pick_yaw_deg -= step;
                if (ramp.cur_pick_yaw_deg < ramp.end_pick_yaw_deg) ramp.cur_pick_yaw_deg = ramp.end_pick_yaw_deg;
            }
        }
        if (fabsf(ramp.cur_pick_yaw_deg - ramp.end_pick_yaw_deg) > 0.1f) pick_yaw_done = false;
    }

    // ---- 吸取手伸缩 ----
    bool pick_extend_done = true;
    if (fabsf(ramp.cur_pick_extend_mm - ramp.end_pick_extend_mm) > 0.01f) {
        if (ramp.phase != 4) {
            float step = kPickExtendSpeed * dt;
            if (ramp.cur_pick_extend_mm < ramp.end_pick_extend_mm) {
                ramp.cur_pick_extend_mm += step;
                if (ramp.cur_pick_extend_mm >= ramp.end_pick_extend_mm) ramp.cur_pick_extend_mm = ramp.end_pick_extend_mm;
                else pick_extend_done = false;
            } else {
                ramp.cur_pick_extend_mm -= step;
                if (ramp.cur_pick_extend_mm <= ramp.end_pick_extend_mm) ramp.cur_pick_extend_mm = ramp.end_pick_extend_mm;
                else pick_extend_done = false;
            }
        }
        if (fabsf(ramp.cur_pick_extend_mm - ramp.end_pick_extend_mm) > 0.01f) pick_extend_done = false;
    }

    // ---- 武器手抬升（无锁，始终自由）----
    bool weapon_lift_done = true;
    if (fabsf(ramp.cur_weapon_lift_mm - ramp.end_weapon_lift_mm) > 0.01f) {
        float step = kWeaponLiftSpeed * dt;
        if (ramp.cur_weapon_lift_mm < ramp.end_weapon_lift_mm) {
            ramp.cur_weapon_lift_mm += step;
            if (ramp.cur_weapon_lift_mm > ramp.end_weapon_lift_mm) ramp.cur_weapon_lift_mm = ramp.end_weapon_lift_mm;
            else weapon_lift_done = false;
        } else {
            ramp.cur_weapon_lift_mm -= step;
            if (ramp.cur_weapon_lift_mm < ramp.end_weapon_lift_mm) ramp.cur_weapon_lift_mm = ramp.end_weapon_lift_mm;
            else weapon_lift_done = false;
        }
    }

    // ---- 武器手伸缩（无锁）----
    bool weapon_extend_done = true;
    if (fabsf(ramp.cur_weapon_extend_mm - ramp.end_weapon_extend_mm) > 0.01f) {
        float step = kWeaponExtendSpeed * dt;
        if (ramp.cur_weapon_extend_mm < ramp.end_weapon_extend_mm) {
            ramp.cur_weapon_extend_mm += step;
            if (ramp.cur_weapon_extend_mm > ramp.end_weapon_extend_mm) ramp.cur_weapon_extend_mm = ramp.end_weapon_extend_mm;
            else weapon_extend_done = false;
        } else {
            ramp.cur_weapon_extend_mm -= step;
            if (ramp.cur_weapon_extend_mm < ramp.end_weapon_extend_mm) ramp.cur_weapon_extend_mm = ramp.end_weapon_extend_mm;
            else weapon_extend_done = false;
        }
    }

    // ---- 电梯（无锁）----
    bool lift_done = true;
    if (fabsf(ramp.cur_lift_mm - ramp.end_lift_mm) > 0.01f) {
        float step = kLiftSpeed * dt;
        if (ramp.cur_lift_mm < ramp.end_lift_mm) {
            ramp.cur_lift_mm += step;
            if (ramp.cur_lift_mm > ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
            else lift_done = false;
        } else {
            ramp.cur_lift_mm -= step;
            if (ramp.cur_lift_mm < ramp.end_lift_mm) ramp.cur_lift_mm = ramp.end_lift_mm;
            else lift_done = false;
        }
    }

    // phase 切换（只涉及 pick_hand）
    if (ramp.phase == 1 && pick_lift_done)      ramp.phase = 0;
    if (ramp.phase == 2 && pick_yaw_done)       ramp.phase = 0;
    if (ramp.phase == 3 && pick_extend_done)    ramp.phase = 0;
    if (ramp.phase == 4 && ramp.cur_pick_yaw_deg > 0.0f) ramp.phase = 0;

    if (pick_lift_done && pick_yaw_done && pick_extend_done
        && weapon_lift_done && weapon_extend_done && lift_done)
        ramp.active = false;
    return ramp.active;
}

void Ramp_ToMsg(const RampState& ramp, pub_upbody_cmd& msg) {
    msg.set_absolute_pose       = true;
    msg.pick_lift_target_mm     = ramp.cur_pick_lift_mm;
    msg.pick_yaw_target_deg     = ramp.cur_pick_yaw_deg;
    msg.pick_extend_target_mm   = ramp.cur_pick_extend_mm;
    msg.weapon_lift_target_mm   = ramp.cur_weapon_lift_mm;
    msg.weapon_extend_target_mm = ramp.cur_weapon_extend_mm;
    msg.lift_target_mm          = ramp.cur_lift_mm;
}

