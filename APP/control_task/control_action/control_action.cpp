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
#include "weapon_hand.hpp"
#include "pick_hand.hpp"
#include "lift.hpp"

RobotPose kPose_Now;


extern WeaponHand weapon_hand;
extern PickHand pick_hand;
extern Lift lift;


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
    ramp_.pending_pump_cmd  = config.pump_cmd;
    ramp_.pending_valve_cmd = config.valve_cmd;
    ramp_.pump_cmd_at_done  = config.pump_cmd_done;
    ramp_.valve_cmd_at_done = config.valve_cmd_done;

    ramp_.chassis_approach_active = config.enable_chassis_approach;
    ramp_.chassis_release_pending = config.release_chassis;

    ramp_.dwell_ms = config.dwell_ms;
    ramp_.dwell_timer_ms = 0;
    ramp_.done_stable_frames = config.done_stable_frames;   // ← 加
    ramp_.done_stable_cnt = 0;

          


}



// ---- private: 每帧推进渐变 ----
void ActionController::Step_(float dt) {
    if (!ramp_.active) return;

    // ---- 检查各轴是否到位 ----
    bool pick_lift_done, pick_yaw_done, pick_extend_done;
    bool weapon_lift_done, weapon_extend_done, lift_done;

    pick_lift_done   = (fabsf(ramp_.cur_pick_lift_mm   - ramp_.end_pick_lift_mm)   <= 1.01f);
    pick_yaw_done    = (fabsf(ramp_.cur_pick_yaw_deg   - ramp_.end_pick_yaw_deg)   <= 1.1f);
    pick_extend_done = (fabsf(ramp_.cur_pick_extend_mm - ramp_.end_pick_extend_mm) <= 1.01f);
    weapon_lift_done   = (fabsf(ramp_.cur_weapon_lift_mm   - ramp_.end_weapon_lift_mm)   <= 1.01f);
    weapon_extend_done = (fabsf(ramp_.cur_weapon_extend_mm - ramp_.end_weapon_extend_mm) <= 1.01f);
    lift_done          = (fabsf(ramp_.cur_lift_mm          - ramp_.end_lift_mm)          <= 1.01f);

    bool done[6] = {
        pick_lift_done, pick_yaw_done, pick_extend_done,
        weapon_lift_done, weapon_extend_done, lift_done
    };
    // 每轴稳定过滤：连续 N 帧判定到位才解阻塞
    static uint8_t axis_stable[6] = {0};
    for (int i = 0; i < 6; i++) {
        if (done[i]) axis_stable[i]++; else axis_stable[i] = 0;
        done[i] = (axis_stable[i] >= ramp_.done_stable_frames);
    }

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


    if (all_done && ramp_.chassis_release_pending) {
        ramp_.chassis_released = true;
    }

    if (all_done) {
        ramp_.done_stable_cnt++;
    if (ramp_.done_stable_cnt >= ramp_.done_stable_frames) {
            ramp_.dwell_timer_ms += (uint16_t)(dt * 1000.0f);
            if (ramp_.dwell_timer_ms >= ramp_.dwell_ms) {
                ramp_.active = false;
            }
        }
    } else {
        ramp_.done_stable_cnt = 0;          // 不稳定就清零
        ramp_.dwell_timer_ms = 0;
    }

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
    ramp_.chassis_released = false;
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
        if (ramp_.pump_cmd_at_done != 0) ramp_.pending_pump_cmd = ramp_.pump_cmd_at_done;
        if (ramp_.valve_cmd_at_done != 0) ramp_.pending_valve_cmd = ramp_.valve_cmd_at_done;
        ramp_.pump_cmd_at_done  = 0;
        ramp_.valve_cmd_at_done = 0;

        if (step_index_ >= step_count_) {
            step_count_ = 0;
        }
    }

    pub_upbody_cmd msg = {};
    msg.active = true;
    ToMsg_(msg);

    // 泵/阀切换（仅首帧发出，发出后清零）
    msg.pump_cmd  = ramp_.pending_pump_cmd;
    msg.valve_cmd = ramp_.pending_valve_cmd;
    ramp_.pending_pump_cmd  = 0;
    ramp_.pending_valve_cmd = 0;

    pub.Publish(msg);
}


//  ------------单轴动作函数-------------
void ActionController::YawTo(float yaw_deg) {
    RobotPose current;
    current.pick_lift_mm     = pick_hand.lift_target_deg_   * PickHand::kLiftMmPerDeg;
    current.pick_yaw_deg     = pick_hand.yaw_target_deg_;
    current.pick_extend_mm   = pick_hand.extend_target_deg_ * PickHand::kExtendMmPerDeg;
    current.weapon_lift_mm   = weapon_hand.lift_target_deg_   * WeaponHand::kLiftMmPerDeg;
    current.weapon_extend_mm = weapon_hand.extend_target_deg_ * WeaponHand::kExtendMmPerDeg;
    current.lift_mm          = lift.target_deg_ * Lift::kMmPerDeg;
    SyncState(current);

    ActionConfig config;
    config.target = current;
    config.target.pick_yaw_deg = yaw_deg / PickHand::kYawDegPerMotorDeg;
    config.step_done_mask = 0x02;   // 只等 yaw
    Start_(config);
}


// ---- 动作函数 ----

void ActionController::GoHome() {
    ActionConfig config;
    config.target = kPose_Home;
    Start_(config);
}

//MF动作
void ActionController::GrabKFS(const RobotPose& pose) {
    //到达吸取位置
    ActionConfig step1;
    step1.target = pose;
    step1.priorities.pick_yaw = 0;
    step1.priorities.pick_lift = 1;
    step1.priorities.pick_extend = 2;
    step1.pump_cmd  = 1;   // 开泵
    step1.valve_cmd = 1;   // 开阀
    step1.dwell_ms = 1850;
    step1.step_done_mask  = 0x03;
    AddStep(step1);

    //转到放置位置上方
    ActionConfig step2;
    step2.target = pose;
    step2.target.pick_lift_mm   = 462.80f;
    step2.target.pick_extend_mm = 0.0f;
    step2.target.pick_yaw_deg   = -139.0f;
    step2.skip_safety = true;          
    step2.priorities.pick_lift   = 0;
    step2.priorities.pick_extend = 0;
    step2.priorities.pick_yaw    = 1;
    step2.dwell_ms               = 1850;
    step2.step_done_mask  = 0x03;

    AddStep(step2);
    
    //伸出吸取手，然后降到放置高度
    ActionConfig step3;
    step3.target = pose;
    step3.target.pick_lift_mm   = 202.80f;
    step3.target.pick_extend_mm =  100.0f;
    step3.target.pick_yaw_deg   = -139.0f;
    step3.speeds.pick_yaw       = 300;
    step3.priorities.pick_extend= 0;
    step3.priorities.pick_lift  = 1;
    step3.skip_safety = true;       
    step3.dwell_ms    = 450;   
    step3.step_done_mask  = 0x03;

    AddStep(step3);    

    //关泵破真空
    ActionConfig step4;
    step4.target = pose;
    step4.target.pick_lift_mm   = 202.80f;
    step4.target.pick_extend_mm = 100.0f;
    step4.target.pick_yaw_deg   = -139.0f;
    step4.skip_safety = true;          
    step4.pump_cmd    = -1;
    step4.valve_cmd   = -1;
    step4.dwell_ms    = 850;
    AddStep(step4);    

    //先缩回，然后把吸取的点往下挪一挪
    ActionConfig step5;
    step5.target = pose;
    step5.target.pick_lift_mm   = 50.80f;
    step5.target.pick_extend_mm = 0.0f;
    step5.target.pick_yaw_deg   = -139.0f;
    step5.priorities.pick_extend= 0;
    step5.priorities.pick_lift  = 1;
    step5.skip_safety = true;          
    step5.step_done_mask  = 0x03;

    AddStep(step5);   

    //伸出吸取手吸取
    ActionConfig step6;
    step6.target = pose;
    step6.target.pick_lift_mm   = 50.80f;
    step6.target.pick_extend_mm = 140.0f;
    step6.target.pick_yaw_deg   = -139.0f;
    step6.pump_cmd              = 1;
    step6.valve_cmd             = 1;
    step6.dwell_ms              = 450;
    step6.skip_safety = true;          

    AddStep(step6);   

    //回中，即回到放置位置
    ActionConfig step7;
    step7.target = pose;
    step7.target.pick_lift_mm   = 462.80f;
    step7.target.pick_extend_mm = 0.0f;
    step7.target.pick_yaw_deg   = 392.0f;
    step7.priorities.pick_lift  = 0;
    step7.priorities.pick_extend= 1;
    step7.priorities.pick_yaw   = 2;
    step7.skip_safety = true;          

    AddStep(step7);   


    RunSteps();
}


void ActionController::PlaceKFS(const RobotPose& pose) {
    ActionConfig step1;
    step1.target = pose;
    step1.target.pick_lift_mm     = 382.6f;
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
    if(pose_Place.name == 1){
        // Step 1: 云台转到位 + 抬升降到抓取高度 → 步首开泵/阀
        ActionConfig step1;
        step1.target = pose_Grab;
        step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
        step1.priorities.pick_yaw     = 0;
        step1.priorities.pick_lift    = 1;
        step1.priorities.pick_extend  = 2;
        step1.step_done_mask = 0x03;
        step1.pump_cmd  = 1;   // 开泵
        step1.valve_cmd = 1;   // 开阀
        AddStep(step1);

        // Step 2: 伸缩伸出够到KFS → 底盘同步前移逼近
        ActionConfig step2;
        step2.target = pose_Grab;
        step2.speeds.pick_extend = 180.0f;
        step2.priorities.pick_extend = 0;
        step2.step_done_mask = 0x04;
        step2.enable_chassis_approach = true;
        step2.dwell_ms = 300;                    // ← 加：伸到头等200ms吸稳
        AddStep(step2);

        // Step 3: 电梯升到安全高度
        ActionConfig step3;
        step3.target = pose_Place;
        step3.target.pick_lift_mm     = 440.6f;
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
        step4.target.pick_lift_mm     = 440.6f;
        step4.target.pick_yaw_deg     = pose_Grab.pick_yaw_deg;
        step4.priorities.pick_extend  = 0;
        step4.step_done_mask = 0x04;
        step4.skip_safety = true;
        AddStep(step4);

        // Step 5a: 只转云台到 0°，完成后释放底盘
        ActionConfig step5a;
        step5a.target = pose_Place;
        step5a.target.pick_extend_mm   = 0.0f;
        step5a.target.pick_lift_mm     = 440.6f;
        step5a.target.pick_yaw_deg     = 0.0f;           // ← 转到 0°
        step5a.priorities.pick_yaw     = 0;
        step5a.step_done_mask = 0x02;                     // ← 只等 yaw
        step5a.skip_safety = true;
        step5a.release_chassis = true;                    // ← 解锁底盘！
        AddStep(step5a);

        // Step 5b: 抬升 + 伸出 + 转放置角度
        ActionConfig step5b;
        step5b.target = pose_Place;
        step5b.target.pick_extend_mm   = 0.0f;
        step5b.target.pick_lift_mm     = 440.6f;
        step5b.speeds.pick_yaw         = 250;
        step5b.priorities.pick_yaw     = 0;
        step5b.priorities.pick_lift    = 1;
        step5b.priorities.pick_extend  = 2;
        step5b.step_done_mask = 0x03;                     // ← 等 yaw + lift
        step5b.skip_safety = true;
        AddStep(step5b);


        // Step 6: 吸取手伸缩伸到放置位置
        ActionConfig step6;
        step6.target = pose_Place;
        step6.target.pick_lift_mm     = 200.6f;
        step6.priorities.pick_extend  = 0;
        step6.step_done_mask = 0x04;
        step6.skip_safety = true;
        step6.dwell_ms    = 1000;
        AddStep(step6);

        // Step 7a: 吸取手降到放置高度 → 到位立即关泵关阀
        ActionConfig step7a;
        step7a.target = pose_Place;
        step7a.priorities.pick_lift    = 0;
        step7a.step_done_mask = 0x01;
        step7a.skip_safety = true;
        if (close_pump_at_end) {
            step7a.pump_cmd_done  = -1;
            step7a.valve_cmd_done = -1;
        }
        AddStep(step7a);

        // Step 7b: 等 250ms 破真空（无运动）
        ActionConfig step7b;
        step7b.target = pose_Place;
        step7b.target.pick_lift_mm = pose_Place.pick_lift_mm;
        step7b.step_done_mask = 0x01;
        step7b.skip_safety = true;
        step7b.dwell_ms = 850;
        AddStep(step7b);


    } else{
        // Step 1: 云台转到位 + 抬升降到抓取高度 → 步首开泵/阀
        ActionConfig step1;
        step1.target = pose_Grab;
        step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
        step1.priorities.pick_yaw     = 0;
        step1.priorities.pick_lift    = 1;
        step1.priorities.pick_extend  = 2;
        step1.step_done_mask = 0x03;
        step1.pump_cmd  = 1;   // 开泵
        step1.valve_cmd = 1;   // 开阀
        AddStep(step1);

        // Step 2: 伸缩伸出够到KFS → 底盘同步前移逼近
        ActionConfig step2;
        step2.target = pose_Grab;
        step2.speeds.pick_extend = 180.0f;
        step2.priorities.pick_extend = 0;
        step2.step_done_mask = 0x04;
        step2.enable_chassis_approach = true;
        step2.dwell_ms = 300;                    // ← 加：伸到头等200ms吸稳
        AddStep(step2);

        // Step 3: 电梯升到安全高度
        ActionConfig step3;
        step3.target = pose_Place;
        step3.target.pick_lift_mm     = 440.6f;
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
        step4.target.pick_lift_mm     = 440.6f;
        step4.target.pick_yaw_deg     = pose_Grab.pick_yaw_deg;
        step4.priorities.pick_extend  = 0;
        step4.step_done_mask = 0x04;
        step4.skip_safety = true;
        AddStep(step4);

        // Step 5a: 只转云台到 0°，完成后释放底盘
        ActionConfig step5a;
        step5a.target = pose_Place;
        step5a.target.pick_extend_mm   = 0.0f;
        step5a.target.pick_lift_mm     = 440.6f;
        step5a.target.pick_yaw_deg     = 0.0f;           // ← 转到 0°
        step5a.priorities.pick_yaw     = 0;
        step5a.step_done_mask = 0x02;                     // ← 只等 yaw
        step5a.skip_safety = true;
        step5a.release_chassis = true;                    // ← 解锁底盘！
        AddStep(step5a);


        // Step 5b: 抬升 + 伸出 + 转放置角度
        ActionConfig step5b;
        step5b.target = pose_Place;
        step5b.target.pick_extend_mm   = 0.0f;
        step5b.target.pick_lift_mm     = 440.6f;
        step5b.priorities.pick_yaw     = 0;
        step5b.priorities.pick_lift    = 1;
        step5b.priorities.pick_extend  = 2;
        step5b.step_done_mask = 0x03;                     // ← 等 yaw + lift
        step5b.skip_safety = true;
        AddStep(step5b);


        // Step 6: 吸取手伸缩伸到放置位置
        ActionConfig step6;
        step6.target = pose_Place;
        step6.target.pick_lift_mm     = 440.6f;
        step6.priorities.pick_extend  = 0;
        step6.step_done_mask = 0x04;
        step6.skip_safety = true;
        step6.dwell_ms    = 1000;

        AddStep(step6);

        // Step 7a: 吸取手降到放置高度 → 到位立即关泵关阀
        ActionConfig step7a;
        step7a.target = pose_Place;
        step7a.priorities.pick_lift    = 0;
        step7a.step_done_mask = 0x01;
        step7a.skip_safety = true;
        if (close_pump_at_end) {
            step7a.pump_cmd_done  = -1;
            step7a.valve_cmd_done = -1;
        }
        AddStep(step7a);

        // Step 7b: 等 250ms 破真空（无运动）
        ActionConfig step7b;
        step7b.target = pose_Place;
        step7b.target.pick_lift_mm = pose_Place.pick_lift_mm;
        step7b.step_done_mask = 0x01;
        step7b.skip_safety = true;
        step7b.dwell_ms = 850;
        AddStep(step7b);


    }

    kPose_Now = pose_Place;   // 记录放置姿态（最后一个KFS对应name=2）
    RunSteps();

}

void ActionController::PreLoad() {
    ActionConfig config;
    config.target = kPose_PreLode;
    config.pump_cmd  = 1;
    config.valve_cmd = 1;
    Start_(config);
}


void ActionController::Moving(const RobotPose& pose) {
    ActionConfig config;
    config.target = pose;
    Start_(config);
}

//Arena动作
void ActionController::GrabKFS_Arena(const RobotPose& pose) {

    ActionConfig step1;
    step1.target = pose;
    step1.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
    step1.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
    step1.target.weapon_lift_mm   = ramp_.cur_weapon_lift_mm;
    step1.target.weapon_extend_mm = ramp_.cur_weapon_extend_mm;
    step1.priorities.pick_lift    = 0;
    step1.step_done_mask = 0x01;
    AddStep(step1);

    // Step 2a: 缩吸取手
    ActionConfig step2a;
    step2a.target = pose;
    step2a.target.pick_yaw_deg   = ramp_.cur_pick_yaw_deg;
    step2a.target.pick_lift_mm   = ramp_.cur_pick_lift_mm;
    step2a.priorities.pick_extend = 0;
    step2a.step_done_mask = 0x04;   // 只等 extend
    AddStep(step2a);

    // Step 2b: 转云台
    ActionConfig step2b;
    step2b.target = pose;
    step2b.target.pick_extend_mm = 0.0f;       // 已经缩回
    step2b.priorities.pick_yaw = 0;
    step2b.step_done_mask = 0x02;
    AddStep(step2b);


    // Step 2c: 降高度
    ActionConfig step2c;
    step2c.target = pose;
    step2c.priorities.pick_lift = 0;
    step2c.step_done_mask = 0x01;   // 只等 lift
    AddStep(step2c);

    RunSteps();

    kPose_Now = pose;   // 记录当前姿态
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

    // 主步骤：yaw 转 → lift 降 → extend 伸 → 步首自动开泵
    ActionConfig config;
    config.target = pose;
    config.priorities.pick_yaw    = 0;
    config.priorities.pick_lift   = 1;
    config.priorities.pick_extend = 2;
    config.speeds.pick_extend     = 250.0f;
    config.pump_cmd  = 1;   // 自动开泵
    config.valve_cmd = 1;   // 自动开阀
    config.dwell_ms  = 850;
    AddStep(config);


    // 上抬 70mm 避开螺丝
    ActionConfig lift_up;
    lift_up.target = pose;
    lift_up.target.pick_lift_mm     = pose.pick_lift_mm + 140.0f;  // ← pose，不是 ramp_.cur
    lift_up.target.pick_yaw_deg     = pose.pick_yaw_deg;
    lift_up.target.pick_extend_mm   = pose.pick_extend_mm;
    lift_up.target.weapon_lift_mm   = pose.weapon_lift_mm;
    lift_up.target.weapon_extend_mm = pose.weapon_extend_mm;
    lift_up.priorities.pick_lift    = 0;
    lift_up.step_done_mask = 0x01;
    AddStep(lift_up);

    // 缩回吸取手
    ActionConfig retract;
    retract.target = pose;
    retract.target.pick_extend_mm   = 0.0f;
    retract.target.pick_yaw_deg     = pose.pick_yaw_deg;
    retract.target.pick_lift_mm     = pose.pick_lift_mm + 70.0f;  // 保持抬高后的位置
    retract.target.weapon_lift_mm   = pose.weapon_lift_mm;
    retract.target.weapon_extend_mm = pose.weapon_extend_mm;
    retract.priorities.pick_extend  = 0;
    retract.step_done_mask = 0x04;
    AddStep(retract);

    kPose_Now = pose;    
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

void ActionController::PokeWeapon(const RobotPose& pose, int wrist_preset) {
    // wrist 不走渐变，直接设目标（和武馆模式 wristFlip 逻辑一致）
    switch (wrist_preset) {
        case 0: weapon_hand.wrist_target_rad_ = 0.087266f; break;   // 第一层：5°
        case 1: weapon_hand.wrist_target_rad_ = 0.9472f;   break;   // 第二层：~65°
        case 2: weapon_hand.wrist_target_rad_ = 1.57079f;  break;   // 90°
        default: break;
    }
    // 其他轴正常走渐变
    ActionConfig config;
    config.target = pose;
    config.speeds.weapon_extend = 180.0f;
    config.priorities.weapon_extend = -1;
    Start_(config);
}

void ActionController::PrepareWeapon() {
    // 先同步所有轴当前值，防突变
    RobotPose current;
    current.pick_lift_mm     = pick_hand.lift_target_deg_   * PickHand::kLiftMmPerDeg;
    current.pick_yaw_deg     = pick_hand.yaw_target_deg_;
    current.pick_extend_mm   = pick_hand.extend_target_deg_ * PickHand::kExtendMmPerDeg;
    current.weapon_lift_mm   = weapon_hand.lift_target_deg_   * WeaponHand::kLiftMmPerDeg;
    current.weapon_extend_mm = weapon_hand.extend_target_deg_ * WeaponHand::kExtendMmPerDeg;
    current.lift_mm          = lift.target_deg_ * Lift::kMmPerDeg;
    SyncState(current);

    // 目标：只改 weapon_lift 和 weapon_extend，其余保持 SyncState 后的值
    ActionConfig config;
    config.target.pick_lift_mm     = ramp_.cur_pick_lift_mm;
    config.target.pick_yaw_deg     = ramp_.cur_pick_yaw_deg;
    config.target.pick_extend_mm   = ramp_.cur_pick_extend_mm;
    config.target.weapon_lift_mm   = 349.2186275f;
    config.target.weapon_extend_mm = 0.0f;
    config.target.lift_mm          = ramp_.cur_lift_mm;
    config.step_done_mask = 0x18;   // 只等 weapon_lift + weapon_extend
    Start_(config);

}

