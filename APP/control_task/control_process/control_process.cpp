/**
 * @file control_process.cpp
 * @author 大帅将军 / lxism
 * @brief 控制流程层实现
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention btnSelect 切换模式：0=队友模式(现有逻辑), 1=调试模式(上身按键映射)
 * @note 持续型按键每帧累加步进量；切换型按键仅上升沿触发
 */
#include "control_task.h"
#include "control_process.hpp"
#include "control_action.hpp"
#include "pid_controller.h"
#include "chassis_task.h"
#include "topics.hpp"
#include <cmath>
#include <cstdint>

// ===== 上身控制常量（每帧步进量，1000Hz 控制频率） =====
static constexpr float kLiftStep = 0.2f;
static constexpr float kPickLiftStep = 0.4f;
static constexpr float kPickYawStep = 1.0f;
static constexpr float kPickExtendStep = 0.2f;
static constexpr float kWeaponLiftStep = 0.25f;
static constexpr float kWeaponExtendStep = 0.4f;
static constexpr uint16_t kTriggerThreshold = 512;

// ===== 外部变量（定义在 control_task.cpp，此处声明引用） =====

// 发布者
extern TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub;
extern pub_chassis_cmd xbox_cmd;


// 订阅者
extern TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub;
extern pub_Xbox_Data control_xbox_cmd;
extern pub_Xbox_Data control_xbox_cmd_Last;

extern TypedTopicSubscriber<pub_Position_Data> control_position_sub;
extern pub_Position_Data control_position_msg;
extern pub_Position_Data control_position;

// 定位修正
extern float position_correction_x;
extern float position_correction_y;
extern const float position_center_distance = 0.28f;

// 控制模式状态
extern RobotMode_t robot_mode;

extern bool headless_xy_mode;
extern bool headless_omega_mode;
// MF 模式相关
extern bool Normal_control_mode;
extern int8_t MF_x;
extern int8_t MF_y;
extern float MF_close_position_x;
extern float MF_close_position_y;
// Arena模式相关
extern int8_t Arena_x;
extern float Arena_close_position_y;

extern const FieldSide_t field_side;
extern const float robot_center_to_gimbal_x;

// 目标状态
extern pub_chassis_cmd robot_v_aim_cmd;
extern pub_chassis_cmd state_aim_cmd;

// PID
extern PID_t linear;
extern PID_t deg;

// 队友模式中间变量
extern float xbox_angle_deg;
extern float v_aim;
extern float error_x;
extern float error_y;
extern float state_xy_error;
extern float state_xy_angle_deg;
extern float xy_pid_output;

// ===== 梅林模式渐变状态 =====
static RampState mf_ramp;
static int8_t last_mf_action = -1;

static bool mf_placing = false;   // 放置进行中，禁止梯度打断

// ===== 九宫格模式渐变状态 =====
static RampState arena_ramp;
static int8_t last_arena_action = -1;



// =====================================================
//  调试模式（原用户模式）
// =====================================================
void Debug_Mode_Process(TypedTopicPublisher<pub_upbody_cmd>& pub, pub_upbody_cmd& msg) {
    // ---- 底盘：摇杆直驱 ----
    robot_v_aim_cmd.linear_x_ = JoyToVelocity(control_xbox_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    robot_v_aim_cmd.linear_y_ = -JoyToVelocity(control_xbox_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    robot_v_aim_cmd.omega_    = -JoyToVelocity(control_xbox_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

    // 无头旋转：场地坐标系 → 车身坐标系
    ApplyFieldCentricRotation(robot_v_aim_cmd.linear_x_, robot_v_aim_cmd.linear_y_, control_position.yaw);

    chassis_data_pub.Publish(robot_v_aim_cmd);

    // ---- 上身控制指令 ----
    msg = {};
    msg.active = true;

    // ▼ 持续型：trigLT 电梯上升 / trigRT 电梯下降
    if (control_xbox_cmd.trigLT > kTriggerThreshold)
        msg.lift_delta = kLiftStep;
    if (control_xbox_cmd.trigRT > kTriggerThreshold)
        msg.lift_delta = -kLiftStep;

    // ▼ 持续型：btnLB 武器手上升 / btnRB 武器手下降
    if (control_xbox_cmd.btnLB)
        msg.weapon_lift_delta = kWeaponLiftStep;
    if (control_xbox_cmd.btnRB)
        msg.weapon_lift_delta = -kWeaponLiftStep;

    // ▼ 持续型：btnDirUp 吸取手上升 / btnDirDown 吸取手下降
    if (control_xbox_cmd.btnDirUp)
        msg.pick_lift_delta = kPickLiftStep;
    if (control_xbox_cmd.btnDirDown)
        msg.pick_lift_delta = -kPickLiftStep;

    // ▼ 持续型：btnDirLeft 云台逆时针 / btnDirRight 云台顺时针
    if (control_xbox_cmd.btnDirLeft)
        msg.pick_yaw_delta = kPickYawStep;
    if (control_xbox_cmd.btnDirRight)
        msg.pick_yaw_delta = -kPickYawStep;

    // ▼ 持续型：右摇杆前推吸取手伸 / 后拉吸取手缩
    {
        int32_t rvert_diff = (int32_t)control_xbox_cmd.joyRVert - (int32_t)kJoyCenter;
        if (rvert_diff > (int32_t)kJoyDeadZoneRight)
            msg.pick_extend_delta = -kPickExtendStep;
        else if (rvert_diff < -(int32_t)kJoyDeadZoneRight)
            msg.pick_extend_delta = kPickExtendStep;
    }

    // ▼ 持续型：btnY 武器手伸 / btnA 武器手缩
    if (control_xbox_cmd.btnY)
        msg.weapon_extend_delta = kWeaponExtendStep;
    if (control_xbox_cmd.btnA)
        msg.weapon_extend_delta = -kWeaponExtendStep;

    // ▼ 切换型（上升沿触发）
    static bool last_btnX = false;
    static bool last_btnB = false;
    static bool last_btnLS = false;
    static bool last_btnRS = false;

    if (control_xbox_cmd.btnX && !last_btnX) {
        msg.pump_toggle  = true;
        msg.valve_toggle = true;
    }
    last_btnX = control_xbox_cmd.btnX;
    
    if (control_xbox_cmd.btnB && !last_btnB)
        msg.claw_toggle = true;
    last_btnB = control_xbox_cmd.btnB;


    if (control_xbox_cmd.btnLS && !last_btnLS)
        msg.claw_toggle = true;
    last_btnLS = control_xbox_cmd.btnLS;

    if (control_xbox_cmd.btnRS && !last_btnRS)
        msg.wrist_toggle = true;
    last_btnRS = control_xbox_cmd.btnRS;

    pub.Publish(msg);
}


// =====================================================
//  队友模式入口
// =====================================================
void Chassis_Xbox_Data_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    xbox_cmd.linear_x_ = JoyToVelocity(control_xbox_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    xbox_cmd.linear_y_ = -JoyToVelocity(control_xbox_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    xbox_cmd.omega_    = -JoyToVelocity(control_xbox_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

    // btnB 上升沿切换普通模式
    if (control_xbox_cmd.btnB != control_xbox_cmd_Last.btnB) {
        if (control_xbox_cmd.btnB == true) {
            Normal_control_mode = !Normal_control_mode;
        }
    }

    if(Normal_control_mode){
        last_mf_action = -1;
        Normal_control_Process();
    }else {
        switch (robot_mode) {
            case MF:{
                MF_control_Process(upbody_pub, upbody_msg);
                break;
            }

            case Arena:{
                Arena_control_Process(upbody_pub, upbody_msg);
                break;
            }
            
            default:
                break;
        }
    }
    
    //按Y清零定位
    if (control_xbox_cmd.btnY == 1 && control_xbox_cmd_Last.btnY == 0) {
        
        position_correction_x = -control_position_msg.x + position_center_distance*sin(control_position.yaw*kDegToRad);
        position_correction_y =  control_position_msg.y - position_center_distance*cos(control_position.yaw*kDegToRad);

        state_aim_cmd.linear_x_ = 0.0f;
        state_aim_cmd.linear_y_ = 0.0f;

    }
}



// =====================================================
//  普通手操 / 定位模式
// =====================================================
void Normal_control_Process() {
    // btnLS 上升沿切换 xy 模式
    if (control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS) {
        if (control_xbox_cmd.btnLS == true) {
            headless_xy_mode = !headless_xy_mode;
        }
    }

    // btnRS 上升沿切换 omega 模式
    if (control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS) {
        if (control_xbox_cmd.btnRS == true) {
            headless_omega_mode = !headless_omega_mode;
        }
    }

    if (headless_xy_mode) {
        // xy 手控模式：摇杆 → 速度，直接进入速度环 PID
        xbox_angle_deg = atan2(xbox_cmd.linear_y_, xbox_cmd.linear_x_) / kDegToRad;
        v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

        robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position.yaw) * kDegToRad);
        robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position.yaw) * kDegToRad);

        state_aim_cmd.linear_x_ = control_position.x;
        state_aim_cmd.linear_y_ = control_position.y;
    } else {
        // xy 定位模式：方向键控制目标位置，进入位置环 PID】

       

        
        // 上下控制
        if (control_xbox_cmd.btnDirUp == 1) {
            if (control_xbox_cmd_Last.btnDirUp == 0) {
                state_aim_cmd.linear_y_ = state_aim_cmd.linear_y_ + 1.0f;
            }
        } else if (control_xbox_cmd.btnDirDown == 1) {
            if (control_xbox_cmd_Last.btnDirDown == 0) {
                state_aim_cmd.linear_y_ = state_aim_cmd.linear_y_ - 1.0f;
            }
        }
        // 左右控制
        if (control_xbox_cmd.btnDirLeft == 1) {
            if (control_xbox_cmd_Last.btnDirLeft == 0) {
                state_aim_cmd.linear_x_ = state_aim_cmd.linear_x_ - 1.0f;
            }
        } else if (control_xbox_cmd.btnDirRight == 1) {
            if (control_xbox_cmd_Last.btnDirRight == 0) {
                state_aim_cmd.linear_x_ = state_aim_cmd.linear_x_ + 1.0f;
            }
        }
        

        Aim_State_xy_Process();
    }

    if (headless_omega_mode) {
        // omega 手控模式：右摇杆直接控制角速度
        robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
        state_aim_cmd.omega_ = control_position.yaw;
    } else {
        // omega 定位模式：右摇杆推到极限 → 设定目标角度
        if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > 30000) {
            if ((control_xbox_cmd.joyRHori - kJoyCenter) > 0) {
                state_aim_cmd.omega_ = -90.0f;
            } else {
                state_aim_cmd.omega_ = 90.0f;
            }
        } else if (ABS(control_xbox_cmd.joyRVert - kJoyCenter) > 30000) {
            if ((control_xbox_cmd.joyRVert - kJoyCenter) > 0) {
                state_aim_cmd.omega_ = 180.0f;
            } else {
                state_aim_cmd.omega_ = 0.0f;
            }
        }

        Aim_State_omega_Process();
    }
}


// =====================================================
//  梅林半自动网格定位
// =====================================================
void MF_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    // 在网格边缘时，方向键移动网格坐标
    if (MF_x == 0 || MF_x == 5) {
        if (control_xbox_cmd.btnDirUp == 1) {
            if (control_xbox_cmd_Last.btnDirUp == 0) {
                if (MF_y < 4) {
                    MF_y = MF_y + 1;
                }
            }
        } else if (control_xbox_cmd.btnDirDown == 1) {
            if (control_xbox_cmd_Last.btnDirDown == 0) {
                if (MF_y > 0) {
                    MF_y = MF_y - 1;
                }
            }
        }
    }

    if (MF_y == 0 || MF_y == 4) {
        if (control_xbox_cmd.btnDirLeft == 1) {
            if (control_xbox_cmd_Last.btnDirLeft == 0) {
                if (MF_x < 5) {
                    MF_x = MF_x + 1;
                }
            }
        } else if (control_xbox_cmd.btnDirRight == 1) {
            if (control_xbox_cmd_Last.btnDirRight == 0) {
                if (MF_x > 0) {
                    MF_x = MF_x - 1;
                }
            }
        }
    }

    /*上层机构执行*/
    switch ((int8_t)robot_position_MF[MF_x][MF_y][3]) {
        case 1:
            //最低高台高度
            if (last_mf_action != 1 && !mf_placing) {
                Ramp_Start(mf_ramp, kPose_KFS_Low);
                last_mf_action = 1;
            }
            break;
        case 2:
            //中间高台高度
            if (last_mf_action != 2 && !mf_placing) {
                Ramp_Start(mf_ramp, kPose_KFS_Mid);
                last_mf_action = 2;
            }
            break;
        case 3:
            //最高高台任务
            if (last_mf_action != 3 && !mf_placing) {
                Ramp_Start(mf_ramp, kPose_KFS_High);
                last_mf_action = 3;
            }
            break;
        default:
            last_mf_action = -1;
            break;
    }



     if(control_xbox_cmd.btnRB == 1){

        switch ((int16_t)robot_position_MF[MF_x][MF_y][2]) {
            case 0:

                MF_close_position_y = MF_close_position_y + 0.002f;
                break;

            case 90:

                MF_close_position_x = MF_close_position_x - 0.002f;
                break;

            case -90:

                 MF_close_position_x = MF_close_position_x + 0.002f;
                break;

            case 180:

                MF_close_position_y = MF_close_position_y - 0.002f;
                break;

            default:
                break;
        }

    }else{
        MF_close_position_x = 0.0f;
        MF_close_position_y = 0.0f;
    }

    state_aim_cmd.linear_x_ = robot_position_MF[MF_x][MF_y][0] + MF_close_position_x;
    state_aim_cmd.linear_y_ = robot_position_MF[MF_x][MF_y][1] + MF_close_position_y;
    Aim_State_xy_Process();

    state_aim_cmd.omega_    = robot_position_MF[MF_x][MF_y][2];
    Aim_State_omega_Process();

    
    // RB 按住：底盘前移(队友代码) + 吸取手前伸
    if (control_xbox_cmd.btnRB == 1 && !mf_placing && !mf_ramp.active) {
        pub_upbody_cmd tmp = {};
        tmp.active = true;
        tmp.pick_extend_delta = 1.2f;
        upbody_pub.Publish(tmp);
    }


    // 每帧推进渐变
    if (mf_ramp.active) {
        Ramp_Step(mf_ramp, 0.005f);
        upbody_msg = {};
        upbody_msg.active = true;
        Ramp_ToMsg(mf_ramp, upbody_msg);
        upbody_pub.Publish(upbody_msg);
    } else {
        mf_placing = false;
    }

    // 真空泵/阀（始终可用，不受渐变限制）
    static bool last_btnX_mf = false;
    if (control_xbox_cmd.btnX && !last_btnX_mf) {
        upbody_msg = {};
        upbody_msg.active = true;
        upbody_msg.pump_toggle  = true;
        upbody_msg.valve_toggle = true;
        upbody_pub.Publish(upbody_msg);
    }
    last_btnX_mf = control_xbox_cmd.btnX;

    // 放置（渐变空闲时响应）
    static bool last_btnA_mf  = false;
    static bool mf_place_toggle = false;
    if (!mf_ramp.active) {
        if (control_xbox_cmd.btnA && !last_btnA_mf) {
            mf_placing = true;
            Ramp_Start(mf_ramp, mf_place_toggle ? kPose_Place2 : kPose_Place1);
            mf_place_toggle = !mf_place_toggle;
        }
    }
    last_btnA_mf = control_xbox_cmd.btnA;

}


// =====================================================
//  九宫格半自动网格定位
// =====================================================
void Arena_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    if (control_xbox_cmd.btnDirRight == 1 && control_xbox_cmd_Last.btnDirRight == 0) {
        if (Arena_x < 2) {
            Arena_x = Arena_x + 1;
        }    
    } else if (control_xbox_cmd.btnDirLeft == 1 && control_xbox_cmd_Last.btnDirLeft == 0) {
        if (Arena_x > 0) {
            Arena_x = Arena_x - 1;
        }
    }

    if(control_xbox_cmd.btnRB == 1){
        Arena_close_position_y = Arena_close_position_y + 0.0015f;
    }else{
        Arena_close_position_y = 0.0f;
    }

    state_aim_cmd.linear_x_ = robot_position_Arena[Arena_x][0];
    state_aim_cmd.linear_y_ = robot_position_Arena[Arena_x][1] + Arena_close_position_y;
    Aim_State_xy_Process();

    state_aim_cmd.omega_    = robot_position_Arena[Arena_x][2];
    Aim_State_omega_Process();

    /*上层机构执行*/
    switch ((int16_t)state_aim_cmd.omega_) {
        case 0:
            if (last_arena_action != 0) {
                Ramp_Start(arena_ramp, kPose_Grid9_Bot12);
                last_arena_action = 0;
            }
            break;
        case -90:
            if (last_arena_action != -90) {
                Ramp_Start(arena_ramp, kPose_Grid9_Bot3);
                last_arena_action = -90;
            }
            break;
        default:
            last_arena_action = -1;
            break;
    }

    // 每帧推进渐变
    if (arena_ramp.active) {
        Ramp_Step(arena_ramp, 0.005f);
        upbody_msg = {};
        upbody_msg.active = true;
        Ramp_ToMsg(arena_ramp, upbody_msg);
        upbody_pub.Publish(upbody_msg);
    }
        // 真空泵/阀（始终可用）
    static bool last_btnX_arena = false;
    if (control_xbox_cmd.btnX && !last_btnX_arena) {
        pub_upbody_cmd toggle_msg = {};
        toggle_msg.active = true;
        toggle_msg.pump_toggle  = true;
        toggle_msg.valve_toggle = true;
        upbody_pub.Publish(toggle_msg);
    }
    last_btnX_arena = control_xbox_cmd.btnX;

}


// =====================================================
//  XY 位置 PID
// =====================================================
void Aim_State_xy_Process() {
    // 里程计定位修正
    position_correction_x = position_correction_x + 0.001f * xbox_cmd.linear_x_;
    position_correction_y = position_correction_y + 0.001f * xbox_cmd.linear_y_;

    // 计算目标位置与当前实际位置的误差，进入 PID
    error_x            = state_aim_cmd.linear_x_ - control_position.x;
    error_y            = state_aim_cmd.linear_y_ - control_position.y;
    state_xy_error     = sqrt(error_x * error_x + error_y * error_y);
    state_xy_angle_deg = atan2(error_y, error_x) / kDegToRad;
    xy_pid_output      = PID_Calculate(&linear, 0.0f, state_xy_error);
    robot_v_aim_cmd.linear_x_ = xy_pid_output * cos((state_xy_angle_deg - control_position.yaw) * kDegToRad);
    robot_v_aim_cmd.linear_y_ = xy_pid_output * sin((state_xy_angle_deg - control_position.yaw) * kDegToRad);
}


// =====================================================
//  角度 PID
// =====================================================
void Aim_State_omega_Process() {
    float error_dir = state_aim_cmd.omega_ - control_position.yaw;
    if (fabs(error_dir) < 1.5f) {
        error_dir = 0.0f;
    } else if (fabs(error_dir) > 180.0f) {
        if (error_dir > 0) error_dir = error_dir - 360.0f;
        else error_dir = error_dir + 360.0f;
    }

    robot_v_aim_cmd.omega_ = kDegToRad * PID_Calculate(&deg, control_position.yaw, control_position.yaw + error_dir);
}

// =====================================================
//  上层调试模式
// =====================================================
void UpperDebug_Mode_Process(TypedTopicPublisher<pub_upbody_cmd>& pub, pub_upbody_cmd& msg) {

    // ---- 底盘：摇杆直驱（始终运行）----
    robot_v_aim_cmd.linear_x_ = JoyToVelocity(control_xbox_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    robot_v_aim_cmd.linear_y_ = -JoyToVelocity(control_xbox_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    robot_v_aim_cmd.omega_    = -JoyToVelocity(control_xbox_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);
    ApplyFieldCentricRotation(robot_v_aim_cmd.linear_x_, robot_v_aim_cmd.linear_y_, control_position.yaw);
    chassis_data_pub.Publish(robot_v_aim_cmd);

    // ---- 泵/阀上升沿检测（先算好，不立即发）----
    static bool last_btnX = false;
    static bool last_btnB = false;
    bool pump_trigger  = control_xbox_cmd.btnX && !last_btnX;
    bool valve_trigger = control_xbox_cmd.btnB && !last_btnB;
    last_btnX = control_xbox_cmd.btnX;
    last_btnB = control_xbox_cmd.btnB;

    // ---- 渐变状态机 ----
    static RampState ramp;

    if (ramp.active) {
        // 每帧走一步，把中间目标 + 泵/阀写进同一条消息
        Ramp_Step(ramp, 0.005f);
        msg = {};
        msg.active = true;
        Ramp_ToMsg(ramp, msg);
        if (pump_trigger)  msg.pump_toggle  = true;
        if (valve_trigger) msg.valve_toggle = true;
        pub.Publish(msg);
    } else if (pump_trigger || valve_trigger) {
        // 渐变空闲时，泵/阀独立发一条消息
        msg = {};
        msg.active = true;
        if (pump_trigger)  msg.pump_toggle  = true;
        if (valve_trigger) msg.valve_toggle = true;
        pub.Publish(msg);
    }

    // ---- 动作链 + 复位（渐变空闲时才响应）----
    static bool last_btnDirUp    = false;
    static bool last_btnDirRight = false;
    static bool last_btnDirDown  = false;
    static bool last_btnY        = false;
    static bool last_btnA        = false; 

    static bool place_toggle     = false;  // false=Place1, true=Place2 

    if (!ramp.active) {
        if (control_xbox_cmd.btnDirUp    && !last_btnDirUp)    Ramp_Start(ramp, kPose_KFS_High);
        if (control_xbox_cmd.btnDirRight && !last_btnDirRight) Ramp_Start(ramp, kPose_KFS_Mid);
        if (control_xbox_cmd.btnDirDown  && !last_btnDirDown)  Ramp_Start(ramp, kPose_KFS_Low);
        if (control_xbox_cmd.btnY        && !last_btnY)        Ramp_Start(ramp, kPose_Home);
       
        if (control_xbox_cmd.btnA        && !last_btnA) {              // ← 加
            Ramp_Start(ramp, place_toggle ? kPose_Place2 : kPose_Place1);
            place_toggle = !place_toggle;
        }
    }

    last_btnDirUp    = control_xbox_cmd.btnDirUp;
    last_btnDirRight = control_xbox_cmd.btnDirRight;
    last_btnDirDown  = control_xbox_cmd.btnDirDown;
    last_btnY        = control_xbox_cmd.btnY;
    last_btnA        = control_xbox_cmd.btnA;

}
