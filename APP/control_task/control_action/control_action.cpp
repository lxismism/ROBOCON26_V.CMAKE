/**
 * @file control_action.cpp
 * @author lxism
 * @brief 底层动作函数实现 —— 摇杆处理、坐标变换、动作控制器
 * @version 0.2
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 */

#include "control_action.hpp"

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

// ====================================================================
//  ActionController 实现
// ====================================================================

// ---- 内部工具：单轴渐变逼近 ----

static void RampOneAxis(float& cur, float end, float step, float threshold, bool& done) {
    if (fabsf(cur - end) <= threshold) {
        done = true;
        return;
    }
    done = false;
    if (cur < end) {
        cur += step;
        if (cur >= end) cur = end;
    } else {
        cur -= step;
        if (cur <= end) cur = end;
    }
}

// ---- 内部工具：检查某轴是否被更高优先级阻塞 ----

static bool AxisBlocked(int my_prio, const bool done[], const int prios[], int n_axes) {
    if (my_prio == -1) return false;
    for (int i = 0; i < n_axes; i++) {
        if (prios[i] != -1 && prios[i] < my_prio && !done[i]) {
            return true;
        }
    }
    return false;
}

// ---- 安全修正：只保留"伸缩缩回优先"一条 ----

void ActionController::ApplySafety_(ActionConfig& config) {
    const RobotPose& pose = config.target;

    // 唯一通用规则：需缩回时先缩回
    if (pose.pick_extend_mm < 1.0f && ramp_.cur_pick_extend_mm > 1.0f) {
        config.priorities.pick_extend = 0;
        config.priorities.pick_lift   = 1;
        config.priorities.pick_yaw    = 1;
    }
}

// ---- protected: 启动一步渐变 ----

void ActionController::Start_(const ActionConfig& config) {
    ramp_.end_pick_lift_mm     = config.target.pick_lift_mm;
    ramp_.end_pick_yaw_deg     = config.target.pick_yaw_deg;
    ramp_.end_pick_extend_mm   = config.target.pick_extend_mm;
    ramp_.end_weapon_lift_mm   = config.target.weapon_lift_mm;
    ramp_.end_weapon_extend_mm = config.target.weapon_extend_mm;
    ramp_.end_lift_mm          = config.target.lift_mm;

    ramp_.speeds          = config.speeds;
    ramp_.priorities      = config.priorities;
    ramp_.step_done_mask  = config.step_done_mask;

    // 安全修正（除非显式跳过）
    if (!config.skip_safety) {
        ApplySafety_(const_cast<ActionConfig&>(config));
        ramp_.priorities = config.priorities;
    }

    ramp_.active = true;

    // 泵/阀切换暂存（本步首帧发出）
    ramp_.pending_pump_toggle  = config.pump_toggle;
    ramp_.pending_valve_toggle = config.valve_toggle;
    ramp_.pump_toggle_at_done  = config.pump_toggle_done;
    ramp_.valve_toggle_at_done = config.valve_toggle_done;

    ramp_.chassis_approach_active = config.enable_chassis_approach;
}



// ---- private: 每帧推进渐变 ----

void ActionController::Step_(float dt) {
    if (!ramp_.active) return;

    // ---- 检查各轴是否到位 ----
    bool pick_lift_done, pick_yaw_done, pick_extend_done;
    bool weapon_lift_done, weapon_extend_done, lift_done;

    pick_lift_done   = (fabsf(ramp_.cur_pick_lift_mm   - ramp_.end_pick_lift_mm)   <= 0.01f);
    pick_yaw_done    = (fabsf(ramp_.cur_pick_yaw_deg   - ramp_.end_pick_yaw_deg)   <= 0.1f);
    pick_extend_done = (fabsf(ramp_.cur_pick_extend_mm - ramp_.end_pick_extend_mm) <= 0.01f);
    weapon_lift_done   = (fabsf(ramp_.cur_weapon_lift_mm   - ramp_.end_weapon_lift_mm)   <= 0.01f);
    weapon_extend_done = (fabsf(ramp_.cur_weapon_extend_mm - ramp_.end_weapon_extend_mm) <= 0.01f);
    lift_done          = (fabsf(ramp_.cur_lift_mm          - ramp_.end_lift_mm)          <= 0.01f);

    bool done[6] = {
        pick_lift_done, pick_yaw_done, pick_extend_done,
        weapon_lift_done, weapon_extend_done, lift_done
    };
    int prios[6] = {
        ramp_.priorities.pick_lift, ramp_.priorities.pick_yaw,
        ramp_.priorities.pick_extend,
        ramp_.priorities.weapon_lift, ramp_.priorities.weapon_extend,
        ramp_.priorities.lift
    };

    // ---- 吸取手抬升 ----
    if (!pick_lift_done && !AxisBlocked(prios[0], done, prios, 6)) {
        RampOneAxis(ramp_.cur_pick_lift_mm, ramp_.end_pick_lift_mm,
                    ramp_.speeds.pick_lift * dt, 0.01f, pick_lift_done);
    }

    // ---- 吸取手云台 ----
    if (!pick_yaw_done && !AxisBlocked(prios[1], done, prios, 6)) {
        RampOneAxis(ramp_.cur_pick_yaw_deg, ramp_.end_pick_yaw_deg,
                    ramp_.speeds.pick_yaw * dt, 0.1f, pick_yaw_done);
    }

    // ---- 吸取手伸缩 ----
    if (!pick_extend_done && !AxisBlocked(prios[2], done, prios, 6)) {
        RampOneAxis(ramp_.cur_pick_extend_mm, ramp_.end_pick_extend_mm,
                    ramp_.speeds.pick_extend * dt, 0.01f, pick_extend_done);
    }

    // ---- 武器手抬升 ----
    if (!weapon_lift_done && !AxisBlocked(prios[3], done, prios, 6)) {
        RampOneAxis(ramp_.cur_weapon_lift_mm, ramp_.end_weapon_lift_mm,
                    ramp_.speeds.weapon_lift * dt, 0.01f, weapon_lift_done);
    }

    // ---- 武器手伸缩 ----
    if (!weapon_extend_done && !AxisBlocked(prios[4], done, prios, 6)) {
        RampOneAxis(ramp_.cur_weapon_extend_mm, ramp_.end_weapon_extend_mm,
                    ramp_.speeds.weapon_extend * dt, 0.01f, weapon_extend_done);
    }

    // ---- 电梯 ----
    if (!lift_done && !AxisBlocked(prios[5], done, prios, 6)) {
        RampOneAxis(ramp_.cur_lift_mm, ramp_.end_lift_mm,
                    ramp_.speeds.lift * dt, 0.01f, lift_done);
    }

    // ---- 全部完成 ----
    bool all_done = true;
    if (ramp_.step_done_mask & 0x01) all_done &= pick_lift_done;
    if (ramp_.step_done_mask & 0x02) all_done &= pick_yaw_done;
    if (ramp_.step_done_mask & 0x04) all_done &= pick_extend_done;
    if (ramp_.step_done_mask & 0x08) all_done &= weapon_lift_done;
    if (ramp_.step_done_mask & 0x10) all_done &= weapon_extend_done;
    if (ramp_.step_done_mask & 0x20) all_done &= lift_done;
    if (all_done) ramp_.active = false;
}

// ---- private: 渐变状态 → 消息 ----

void ActionController::ToMsg_(pub_upbody_cmd& msg) const {
    msg.set_absolute_pose       = true;
    msg.pick_lift_target_mm     = ramp_.cur_pick_lift_mm;
    msg.pick_yaw_target_deg     = ramp_.cur_pick_yaw_deg;
    msg.pick_extend_target_mm   = ramp_.cur_pick_extend_mm;
    msg.weapon_lift_target_mm   = ramp_.cur_weapon_lift_mm;
    msg.weapon_extend_target_mm = ramp_.cur_weapon_extend_mm;
    msg.lift_target_mm          = ramp_.cur_lift_mm;
}

// ====================================================================
//  Public 接口
// ====================================================================

bool ActionController::IsActive() const {
    return ramp_.active;
}

void ActionController::SyncState(const RobotPose& current) {
    ramp_.cur_pick_lift_mm     = current.pick_lift_mm;
    ramp_.cur_pick_yaw_deg     = current.pick_yaw_deg;
    ramp_.cur_pick_extend_mm   = current.pick_extend_mm;
    ramp_.cur_weapon_lift_mm   = current.weapon_lift_mm;
    ramp_.cur_weapon_extend_mm = current.weapon_extend_mm;
    ramp_.cur_lift_mm          = current.lift_mm;
}

// ---- 步队列 ----

void ActionController::AddStep(const ActionConfig& config) {
    if (step_count_ < kMaxSteps) {
        step_queue_[step_count_++] = config;
    }
}

void ActionController::RunSteps() {
    step_index_ = 0;
    // 第一步在 Update 中自动启动
}

// ---- 每帧更新 ----

void ActionController::Update(float dt, TypedTopicPublisher<pub_upbody_cmd>& pub) {
    if (!ramp_.active) {
        if (step_index_ < step_count_) {
            Start_(step_queue_[step_index_++]);
        } else {
            step_count_ = 0;
            return;
        }
    }
    Step_(dt);

    // 步完成时：将 done 标志转入 pending，下一帧发出
    if (!ramp_.active) {
        ramp_.pending_pump_toggle  |= ramp_.pump_toggle_at_done;
        ramp_.pending_valve_toggle |= ramp_.valve_toggle_at_done;
        ramp_.pump_toggle_at_done   = false;
        ramp_.valve_toggle_at_done  = false;

        if (step_index_ >= step_count_) {
            step_count_ = 0;  // 全完成，清零队列
        }
    }

    pub_upbody_cmd msg = {};
    msg.active = true;
    ToMsg_(msg);

    // 泵/阀切换（仅首帧发出，发出后清零）
    msg.pump_toggle  = ramp_.pending_pump_toggle;
    msg.valve_toggle = ramp_.pending_valve_toggle;
    ramp_.pending_pump_toggle  = false;
    ramp_.pending_valve_toggle = false;

    pub.Publish(msg);
}



// ---- 动作函数 ----

void ActionController::GoHome() {
    ActionConfig config;
    config.target = kPose_Home;
    config.speeds.pick_lift     = 150.0f;
    config.speeds.pick_yaw      = 150.0f;
    config.speeds.pick_extend   = 100.0f;
    config.speeds.weapon_lift   = 40.0f;
    config.speeds.weapon_extend = 40.0f;
    config.speeds.lift          = 40.0f;
    Start_(config);
}

//MF动作
void ActionController::GrabKFS(const RobotPose& pose) {
    ActionConfig config;
    config.target = pose;
    Start_(config);
}

void ActionController::PlaceKFS(const RobotPose& pose) {
    ActionConfig step1;
    step1.target = pose;
    step1.target.pick_lift_mm     = 372.6f;
    step1.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
    step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
    step1.priorities.pick_lift    = 0;
    step1.step_done_mask = 0x01;
    AddStep(step1);

    ActionConfig step2;
    step2.target = pose;
    step2.priorities.pick_yaw    = 0;
    step2.priorities.pick_extend = 1;
    step2.priorities.pick_lift   = 2;
    step2.priorities.lift        = -1;
    step2.skip_safety = true;
    step2.step_done_mask = 0x07;
    AddStep(step2);

    RunSteps();
}

void ActionController::PickKFS(const RobotPose& pose_Grab, const RobotPose& pose_Place, bool close_pump_at_end){
    // Step 1: 云台转到位 + 抬升降到抓取高度 → 步首开泵/阀
    ActionConfig step1;
    step1.target = pose_Grab;
    step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
    step1.priorities.pick_yaw     = 0;
    step1.priorities.pick_lift    = 1;
    step1.step_done_mask = 0x03;
    step1.pump_toggle       = true;
    step1.valve_toggle      = true;
    AddStep(step1);

    // Step 2: 伸缩伸出够到KFS → 底盘同步前移逼近
    ActionConfig step2;
    step2.target = pose_Grab;
    step2.priorities.pick_extend = 0;
    step2.step_done_mask = 0x04;
    step2.enable_chassis_approach = true;
    AddStep(step2);

    // Step 3: 电梯升到安全高度
    ActionConfig step3;
    step3.target = pose_Place;
    step3.target.pick_lift_mm     = 372.6f;
    step3.target.pick_yaw_deg     = pose_Grab.pick_yaw_deg;
    step3.target.pick_extend_mm   = pose_Grab.pick_extend_mm;
    step3.target.weapon_lift_mm   = ramp_.cur_weapon_lift_mm;
    step3.target.weapon_extend_mm = ramp_.cur_weapon_extend_mm;
    step3.priorities.pick_lift    = 1;   // 再抬吸取手
    step3.step_done_mask = 0x01;         // 只等吸取手抬升到位，电梯并发不阻塞
    AddStep(step3);

    // Step 4: 吸取手伸缩缩到最小（避免干涉）
    ActionConfig step4;
    step4.target = pose_Place;
    step4.target.pick_extend_mm   = 0.0f;
    step4.target.pick_lift_mm     = 372.6f;
    step4.target.pick_yaw_deg     = pose_Grab.pick_yaw_deg;
    step4.priorities.pick_extend  = 0;
    step4.step_done_mask = 0x04;
    step4.skip_safety = true;
    AddStep(step4);

    // Step 5: 吸取手云台转到放置角度
    ActionConfig step5;
    step5.target = pose_Place;
    step5.target.pick_extend_mm   = 0.0f;
    step5.target.pick_lift_mm     = 372.6f;
    step5.priorities.pick_yaw     = 0;
    step5.step_done_mask = 0x02;
    step5.skip_safety = true;
    AddStep(step5);

    // Step 6: 吸取手伸缩伸到放置位置
    ActionConfig step6;
    step6.target = pose_Place;
    step6.target.pick_lift_mm     = 372.6f;
    step6.priorities.pick_extend  = 0;
    step6.step_done_mask = 0x04;
    step6.skip_safety = true;
    AddStep(step6);

    // Step 7: 吸取手抬升降到放置高度 → 到位后关泵/阀
    ActionConfig step7;
    step7.target = pose_Place;
    step7.priorities.pick_lift    = 0;
    step7.step_done_mask = 0x01;
    step7.skip_safety = true;
    if (close_pump_at_end) {
        step7.pump_toggle_done  = true;
        step7.valve_toggle_done = true;
    }
    AddStep(step7);
    
    RunSteps();
}



void ActionController::Moving(const RobotPose& pose) {
    ActionConfig config;
    config.target = pose;
    Start_(config);
}

//Arena动作
void ActionController::GrabKFS_Arena(const RobotPose& pose) {
    if (ramp_.cur_pick_lift_mm < 300.0f) {
        ActionConfig step1;
        step1.target = pose;
        step1.target.pick_lift_mm     = 392.6f;
        step1.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
        step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
        step1.target.weapon_lift_mm   = ramp_.cur_weapon_lift_mm;
        step1.target.weapon_extend_mm = ramp_.cur_weapon_extend_mm;
        
        step1.speeds.pick_yaw         = 300.0f;

        step1.priorities.pick_lift    = 0;
        step1.step_done_mask = 0x01;
        AddStep(step1);
    }
    ActionConfig config;
    config.target = pose;
    config.priorities.pick_yaw  = 0;   // 先转云台
    config.priorities.pick_lift = 1;   // 再降高度
    AddStep(config);
    RunSteps();
}

void ActionController::GetKFS(const RobotPose& pose) {
    if (ramp_.cur_pick_extend_mm > 1.0f && pose.pick_extend_mm < 1.0f
        && pose.pick_lift_mm >= ramp_.cur_pick_lift_mm) {
        ActionConfig step1;
        step1.target = pose;
        step1.target.pick_lift_mm     = 392.6f;
        step1.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
        step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
        step1.step_done_mask = 0x01;
        AddStep(step1);
    }

    // 主步骤：yaw 转 → lift 降 → extend 伸
    ActionConfig config;
    config.target = pose;
    config.priorities.pick_yaw    = 0;
    config.speeds.pick_extend     = 150.0f;
    config.speeds.pick_yaw        = 300.0f;
    config.priorities.pick_lift   = 1;
    config.priorities.pick_extend = 2;
    AddStep(config);

    // 上抬 70mm 避开螺丝
    ActionConfig lift_up;
    lift_up.target = pose;
    lift_up.target.pick_lift_mm     = pose.pick_lift_mm + 70.0f;  // ← pose，不是 ramp_.cur
    lift_up.target.pick_yaw_deg     = pose.pick_yaw_deg;
    lift_up.target.pick_extend_mm   = pose.pick_extend_mm;
    lift_up.target.weapon_lift_mm   = pose.weapon_lift_mm;
    lift_up.target.weapon_extend_mm = pose.weapon_extend_mm;

    lift_up.speeds.pick_yaw         = 300.0f;

    lift_up.priorities.pick_lift    = 0;
    lift_up.step_done_mask = 0x01;
    AddStep(lift_up);

    // 缩回吸取手
    ActionConfig retract;
    retract.target = pose;
    retract.speeds.pick_extend    = 150.0f;
    retract.target.pick_extend_mm   = 0.0f;
    retract.target.pick_yaw_deg     = pose.pick_yaw_deg;
    retract.target.pick_lift_mm     = pose.pick_lift_mm + 70.0f;  // 保持抬高后的位置
    retract.target.weapon_lift_mm   = pose.weapon_lift_mm;
    retract.target.weapon_extend_mm = pose.weapon_extend_mm;
    retract.priorities.pick_extend  = 0;
    retract.step_done_mask = 0x04;
    AddStep(retract);

    RunSteps();
}

//withR2动作
void ActionController::R2MergePose(const RobotPose& pose) {
    ActionConfig config;
    // 其他轴保持当前位置不变，只改电梯
    config.target.pick_lift_mm     = ramp_.cur_pick_lift_mm;
    config.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
    config.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
    config.target.weapon_lift_mm   = ramp_.cur_weapon_lift_mm;
    config.target.weapon_extend_mm = ramp_.cur_weapon_extend_mm;
    config.target.lift_mm          = pose.lift_mm;
    config.step_done_mask = 0x20;  // 仅关注电梯到位
    Start_(config);
}
