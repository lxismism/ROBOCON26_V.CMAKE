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
#include "main.h"
#include "control_task.h"
#include "control_process.hpp"
#include "control_action.hpp"
#include "pid_controller.h"
#include "chassis_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include <cmath>
#include <cstdint>
#include "pick_hand.hpp"
#include "weapon_hand.hpp"
#include "lift.hpp"
#include "omni_ir.hpp"
#include "control_Traject.hpp"



// ===== 上身控制常量（每帧步进量，1000Hz 控制频率） =====
static constexpr float kLiftStep = 0.2f;
static constexpr float kPickLiftStep = 0.4f;
static constexpr float kPickYawStep = 1.0f;
static constexpr float kPickExtendStep = 0.5f;  //0.2
static constexpr float kWeaponLiftStep = 0.5f;  //0.25
static constexpr float kWeaponExtendStep = 0.4f;
static constexpr uint16_t kTriggerThreshold = 512;

// ===== MF 自动逼近参数（可配） =====
float kMF_ApproachDist  = 0.18f;   // 底盘前移距离 (m)，例如 0.07 = 7cm
float kMF_ApproachSpeed = 0.30f;   // 前移/后退速度 (m/s)，实测后改


// ===== 外部变量（定义在 control_task.cpp，此处声明引用） =====

// 发布者
extern TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub;
extern pub_chassis_cmd xbox_cmd;

extern TypedTopicPublisher<pub_ir_cmd> ir_cmd_pub;                
extern pub_ir_cmd ir_cmd;

// 订阅者
extern TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub;
extern pub_Xbox_Data control_xbox_cmd;
extern pub_Xbox_Data control_xbox_cmd_Last;

extern TypedTopicSubscriber<pub_Position_Data> control_position_sub;
extern pub_Position_Data control_position_msg;
extern pub_Position_Data control_position;

// 定位修正
extern float position_center_distance;
extern float position_correction_x;
extern float position_correction_y;


// 控制模式状态
extern RobotMode_t robot_mode;

extern float xbox_angle_deg;
extern float v_aim;


extern bool headless_xy_mode;
extern bool headless_omega_mode;
bool headless_mode = true;

extern bool Normal_control_mode;
// MC 模式相关
extern int8_t MC_y;
extern float MC_close_position_x;
extern bool MC_headless_xy_mode;
extern bool MC_headless_omega_mode;
// MF 模式相关
extern uint8_t MF_x;
extern uint8_t MF_y;
extern float MF_close_position_x;
extern float MF_close_position_y;
extern float MF_omega_correction;
extern bool MF_plan_record_Flag;
extern bool MF_plan_run_Flag;
extern bool MF_action_Flag;
extern bool MF_pick_Flag;
extern bool MF_xy_complete_Flag;
extern bool MF_omega_complete_Flag;
extern int8_t MF_omega_control_Flag;
extern uint8_t MF_plan_record_i;
extern uint8_t MF_plan_run_i;
extern MF_plan_t MF_plan_zero;
// Arena模式相关
extern int8_t Arena_x;
extern float Arena_close_position_y;
extern float Arena_close_position_y_Max;

extern const FieldSide_t field_side;
extern const float robot_center_to_gimbal_x;

extern PickHand pick_hand;
extern WeaponHand weapon_hand;
extern Lift lift;

// 目标状态
extern pub_chassis_cmd robot_v_aim_cmd;
extern pub_chassis_cmd state_now_cmd;
extern pub_chassis_cmd state_start_cmd;
extern pub_chassis_cmd state_target_cmd;
extern pub_chassis_cmd state_target_last_cmd;

// PID
extern PID_t lateral;
extern PID_t path;
extern PID_t omega;

// 队友模式中间变量
extern float Acc_path_SpeedUp;
extern float Acc_path_SpeedDown;
extern float path_plan_Max_Max;

extern float Acc_omega_SpeedUp;
extern float Acc_omega_SpeedDown;
extern float v_omega_plan_Max_Max;

extern float Acc_xy_dt; //加速计时器，单位s
extern uint32_t Acc_xy_DWT_CNT;
extern float Acc_omega_dt; //加速计时器，单位s
extern uint32_t Acc_omega_DWT_CNT;

extern float predict_yaw;
extern float yaw_delay_time;

static bool mf_placing = false;   // 放置进行中，禁止梯度打断

// ===== 九宫格模式渐变状态 =====
static ActionController upbody_ctrl;
static int8_t last_mf_action = -1;      
static int8_t last_arena_x = -1;

TrajectChassis Traject_chassis;


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
        if (control_xbox_cmd.btnLB && !control_xbox_cmd_Last.btnLB) {
            upbody_ctrl.GoHome();
        }
        Normal_control_Process();
    }else {
        // 模式切换时同步 ActionController 内部状态，防止姿态突变
        static RobotMode_t prev_robot_mode = MC;
        if (robot_mode != prev_robot_mode) {
            RobotPose current;
            current.pick_lift_mm     = pick_hand.lift_target_deg_   * PickHand::kLiftMmPerDeg;
            current.pick_yaw_deg     = pick_hand.yaw_target_deg_;
            current.pick_extend_mm   = pick_hand.extend_target_deg_ * PickHand::kExtendMmPerDeg;
            current.weapon_lift_mm   = weapon_hand.lift_target_deg_   * WeaponHand::kLiftMmPerDeg;
            current.weapon_extend_mm = weapon_hand.extend_target_deg_ * WeaponHand::kExtendMmPerDeg;
            current.lift_mm          = lift.target_deg_ * Lift::kMmPerDeg;
            upbody_ctrl.SyncState(current);
            prev_robot_mode = robot_mode;
        }

        switch (robot_mode) {
            case MC:{
                MC_control_Process(upbody_pub, upbody_msg);
                break;
            }

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
    
}



// =====================================================
//  普通手操 / 定位模式
// =====================================================
void Normal_control_Process() {

    // btnX 上身沿切换轨迹模式和手控模式
    if (control_xbox_cmd.btnX != control_xbox_cmd_Last.btnX) {
        if (control_xbox_cmd.btnX == true) {
            headless_mode = !headless_mode;
        }
    }
    if(headless_mode == true){
        // xy 手控模式：摇杆 → 速度，直接进入速度环 PID
        xbox_angle_deg = atan2(xbox_cmd.linear_y_, xbox_cmd.linear_x_) / kDegToRad;
        v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

        robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - state_now_cmd.omega_) * kDegToRad);
        robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - state_now_cmd.omega_) * kDegToRad);

        state_target_cmd.linear_x_ = state_now_cmd.linear_x_;
        state_target_cmd.linear_y_ = state_now_cmd.linear_y_;
        // omega 手控模式：右摇杆直接控制角速度
        robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
        state_target_cmd.omega_ = control_position.yaw;
    }else{
        // xy 定位模式：方向键控制目标位置，进入位置环 PID】

        // 上下控制
        if (control_xbox_cmd.btnDirUp == 1) {
            if (control_xbox_cmd_Last.btnDirUp == 0) {
                state_target_cmd.linear_y_ = state_target_cmd.linear_y_ + 3.0f;
                state_target_cmd.omega_ = Warp_ToRange(state_target_cmd.omega_ + 90.0f,-180.0f,180.0f);
            }
        } else if (control_xbox_cmd.btnDirDown == 1) {
            if (control_xbox_cmd_Last.btnDirDown == 0) {
                state_target_cmd.linear_y_ = state_target_cmd.linear_y_ - 3.0f;
                state_target_cmd.omega_ = Warp_ToRange(state_target_cmd.omega_ - 90.0f,-180.0f,180.0f);
            }
        }
        // 左右控制
        if (control_xbox_cmd.btnDirLeft == 1) {
            if (control_xbox_cmd_Last.btnDirLeft == 0) {
                state_target_cmd.linear_x_ = state_target_cmd.linear_x_ - 1.0f;
                state_target_cmd.omega_ = Warp_ToRange(state_target_cmd.omega_ + 90.0f,-180.0f,180.0f);
            }
        } else if (control_xbox_cmd.btnDirRight == 1) {
            if (control_xbox_cmd_Last.btnDirRight == 0) {
                state_target_cmd.linear_x_ = state_target_cmd.linear_x_ + 1.0f;
                state_target_cmd.omega_ = Warp_ToRange(state_target_cmd.omega_ - 90.0f,-180.0f,180.0f);
            }
        }

        predict_yaw = state_now_cmd.omega_ + (control_position.yaw_speed/kDegToRad)*yaw_delay_time;
        Traject_chassis.Set_Ref(state_target_cmd);
        Traject_chassis.Run(state_now_cmd);
        robot_v_aim_cmd = Traject_chassis.Get_output_b();

    }





    // // btnLS 上升沿切换 xy 模式
    // if (control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS) {
    //     if (control_xbox_cmd.btnLS == true) {
    //         headless_xy_mode = !headless_xy_mode;
    //     }
    // }

    // // btnRS 上升沿切换 omega 模式
    // if (control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS) {
    //     if (control_xbox_cmd.btnRS == true) {
    //         headless_omega_mode = !headless_omega_mode;
    //     }
    // }

    // if (headless_xy_mode) {
    //     // xy 手控模式：摇杆 → 速度，直接进入速度环 PID
    //     xbox_angle_deg = atan2(xbox_cmd.linear_y_, xbox_cmd.linear_x_) / kDegToRad;
    //     v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

    //     robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - state_now_cmd.omega_) * kDegToRad);
    //     robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - state_now_cmd.omega_) * kDegToRad);

    //     state_target_cmd.linear_x_ = state_now_cmd.linear_x_;
    //     state_target_cmd.linear_y_ = state_now_cmd.linear_y_;
    // } else {
    //     // xy 定位模式：方向键控制目标位置，进入位置环 PID】

    //     // 上下控制
    //     if (control_xbox_cmd.btnDirUp == 1) {
    //         if (control_xbox_cmd_Last.btnDirUp == 0) {
    //             state_target_cmd.linear_y_ = state_target_cmd.linear_y_ + 3.0f;
    //         }
    //     } else if (control_xbox_cmd.btnDirDown == 1) {
    //         if (control_xbox_cmd_Last.btnDirDown == 0) {
    //             state_target_cmd.linear_y_ = state_target_cmd.linear_y_ - 3.0f;
    //         }
    //     }
    //     // 左右控制
    //     if (control_xbox_cmd.btnDirLeft == 1) {
    //         if (control_xbox_cmd_Last.btnDirLeft == 0) {
    //             state_target_cmd.linear_x_ = state_target_cmd.linear_x_ - 1.0f;
    //         }
    //     } else if (control_xbox_cmd.btnDirRight == 1) {
    //         if (control_xbox_cmd_Last.btnDirRight == 0) {
    //             state_target_cmd.linear_x_ = state_target_cmd.linear_x_ + 1.0f;
    //         }
    //     }
        

    //     Aim_State_xy_Process();
    // }

    // if (headless_omega_mode) {
    //     // omega 手控模式：右摇杆直接控制角速度
    //     robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
    //     state_target_cmd.omega_ = control_position.yaw;
    // } else {
    //     // omega 定位模式：右摇杆推到极限 → 设定目标角度
    //     if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > 30000) {
    //         if ((control_xbox_cmd.joyRHori - kJoyCenter) > 0) {
    //             state_target_cmd.omega_ = -90.0f;
    //         } else {
    //             state_target_cmd.omega_ = 90.0f;
    //         }
    //     } else if (ABS(control_xbox_cmd.joyRVert - kJoyCenter) > 30000) {
    //         if ((control_xbox_cmd.joyRVert - kJoyCenter) > 0) {
    //             state_target_cmd.omega_ = 180.0f;
    //         } else {
    //             state_target_cmd.omega_ = 0.0f;
    //         }
    //     }

    //     Aim_State_omega_Process();
    // }
}


// =====================================================
//  武馆半自动网格定位
// =====================================================
void MC_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    // btnLS 上升沿切换 xy 模式
    if (control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS) {
        if (control_xbox_cmd.btnLS == true) {
            MC_headless_xy_mode = !MC_headless_xy_mode;
        }
    }

    // btnRS 上升沿切换 omega 模式
    if (control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS) {
        if (control_xbox_cmd.btnRS == true) {
            MC_headless_omega_mode = !MC_headless_omega_mode;
        }
    }

    if (control_xbox_cmd.btnDirUp == 1) {
        if (control_xbox_cmd_Last.btnDirUp == 0) {
            if (MC_y < 3) {
                MC_y = MC_y + 1;
            }
        }
    } else if (control_xbox_cmd.btnDirDown == 1) {
        if (control_xbox_cmd_Last.btnDirDown == 0) {
            if (MC_y > 0) {
                MC_y = MC_y - 1;
            }
        }
    }

    if(control_xbox_cmd.btnY == 1 && control_xbox_cmd_Last.btnY == 0){
        //在这里面配置红外发送
        ir_cmd.tx_data = CMD_RELEASE_CLAW;
        ir_cmd_pub.Publish(ir_cmd);
    }

    if (MC_headless_xy_mode) {
        // xy 手控模式：摇杆 → 速度，直接进入速度环 PID
        xbox_angle_deg = atan2(xbox_cmd.linear_y_, xbox_cmd.linear_x_) / kDegToRad;
        v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

        robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position.yaw) * kDegToRad);
        robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position.yaw) * kDegToRad);

        state_target_cmd.linear_x_ = state_now_cmd.linear_x_;
        state_target_cmd.linear_y_ = state_now_cmd.linear_y_;
    } else {
        state_target_cmd.linear_x_ = robot_position_MC[MC_y][0];
        state_target_cmd.linear_y_ = robot_position_MC[MC_y][1];
        Aim_State_xy_Process();
    }

    if (MC_headless_omega_mode) {
        // omega 定位模式：右摇杆推到极限 → 设定目标角度
        state_target_cmd.omega_ = 0.0f;
        Aim_State_omega_Process();

    } else {
        state_target_cmd.omega_  = robot_position_MC[MC_y][2];
        Aim_State_omega_Process();
    }

        // ---- 武器手控制 ----
    upbody_msg = {};
    upbody_msg.active = true;

    // 持续型：btnLB 缩 / btnRB 伸
    if (control_xbox_cmd.btnLB)
        upbody_msg.weapon_extend_delta = -kWeaponExtendStep;
    if (control_xbox_cmd.btnRB)
        upbody_msg.weapon_extend_delta = kWeaponExtendStep;

    // 持续型：右摇杆前推抬升 / 后拉下降（霍尔值线性映射速度）
    {
        int32_t rvert_diff = (int32_t)control_xbox_cmd.joyRVert - (int32_t)kJoyCenter;
        if (ABS(rvert_diff) > (int32_t)kJoyDeadZoneRight) {
            float ratio = (float)(ABS(rvert_diff) - (int32_t)kJoyDeadZoneRight)
                          / (float)(kJoyCenter - kJoyDeadZoneRight)
                          * (rvert_diff > 0 ? -1.0f : 1.0f);
            upbody_msg.weapon_lift_delta = ratio * kWeaponLiftStep;
        }
    }



    // 切换型：btnX 夹爪开合 / btnA 腕部翻转
    static bool last_btnX_mc = false;
    static bool last_btnA_mc = false;
    if (control_xbox_cmd.btnX && !last_btnX_mc)
        upbody_msg.claw_toggle = true;
    if (control_xbox_cmd.btnA && !last_btnA_mc)
        upbody_msg.wrist_toggle = true;
    last_btnX_mc = control_xbox_cmd.btnX;
    last_btnA_mc = control_xbox_cmd.btnA;

    upbody_pub.Publish(upbody_msg);

}

// =====================================================
//  梅林半自动网格定位
// =====================================================
void MF_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    // 在网格边缘时，方向键移动网格坐标

    uint8_t MF_x_Last = MF_x;
    uint8_t MF_y_Last = MF_y;
    static int mf_place_cycle = 0;   // 0=Place1, 1=Place2, 2=Place3
    static uint8_t last_moving_pose_i = 255;

    static float   mf_approach_offset = 0.0f;   // 底盘逼近偏移量 (mm)，渐变值
    static int16_t mf_approach_facing = 0;      // 逼近时锁定 KFS 朝向

    if(false){
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
                        MF_x = MF_x - field_side;
                    }
                }
            } else if (control_xbox_cmd.btnDirRight == 1) {
                if (control_xbox_cmd_Last.btnDirRight == 0) {
                    if (MF_x > 0) {
                        MF_x = MF_x + field_side;
                    }
                }
            }
        }
        
        /*上层机构执行*/
        switch ((int8_t)robot_position_MF[MF_x][MF_y][3]) {
            case 1:
                if (last_mf_action != 1 && !mf_placing) {
                    upbody_ctrl.GrabKFS(kPose_KFS_Low);
                    last_mf_action = 1;
                }
                break;
            case 2:
                if (last_mf_action != 2 && !mf_placing) {
                    upbody_ctrl.GrabKFS(kPose_KFS_Mid);
                    last_mf_action = 2;
                }
                break;
            case 3:
                if (last_mf_action != 3 && !mf_placing) {
                    upbody_ctrl.GrabKFS(kPose_KFS_High);
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

                    MF_close_position_y = MF_close_position_y + 0.0015f;
                    break;

                case 90:

                    MF_close_position_x = MF_close_position_x - 0.0015f;
                    break;

                case -90:

                    MF_close_position_x = MF_close_position_x + 0.0015f;
                    break;

                case 180:

                    MF_close_position_y = MF_close_position_y - 0.0015f;
                    break;

                default:
                    break;
            }

        }else{
            MF_close_position_x = 0.0f;
            MF_close_position_y = 0.0f;
        }

        
        state_target_cmd.linear_x_ = robot_position_MF[MF_x][MF_y][0] + MF_close_position_x;
        state_target_cmd.linear_y_ = robot_position_MF[MF_x][MF_y][1] + MF_close_position_y;
        state_target_cmd.omega_    = robot_position_MF[MF_x][MF_y][2];

        // RB 按住：底盘前移(队友代码) + 吸取手前伸
        if (control_xbox_cmd.btnRB == 1 && !mf_placing && !upbody_ctrl.IsActive()) {
            pub_upbody_cmd tmp = {};
            tmp.active = true;
            tmp.pick_extend_delta = 1.2f;
            upbody_pub.Publish(tmp);
        }


    // 每帧推进渐变
    upbody_ctrl.Update(0.005f, upbody_pub);
    if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending() && mf_placing) {
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
        if (!upbody_ctrl.IsActive()) {
            if (control_xbox_cmd.btnA && !last_btnA_mf) {
                mf_placing = true;
                upbody_ctrl.PlaceKFS(kPose_Place[mf_place_cycle]);
                mf_place_cycle = (mf_place_cycle + 1) % 3;
                HAL_GPIO_WritePin(PUMP_LIFT_GPIO_Port, PUMP_LIFT_Pin, GPIO_PIN_SET);

            }
        }
        last_btnA_mf = control_xbox_cmd.btnA;

    }else{

        bool MF_plan_run_Flag_Last = MF_plan_run_Flag;
        bool MF_plan_record_Flag_Last = MF_plan_record_Flag;

        if(control_xbox_cmd.btnX == true && control_xbox_cmd_Last.btnX == false){
            if(MF_plan_run_Flag == false){
                MF_plan_record_Flag = !MF_plan_record_Flag;
            }
        }

        if(MF_plan_record_Flag == true){
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
                            MF_x = MF_x - field_side;
                        }
                    }
                } else if (control_xbox_cmd.btnDirRight == 1) {
                    if (control_xbox_cmd_Last.btnDirRight == 0) {
                        if (MF_x > 0) {
                            MF_x = MF_x + field_side;
                        }
                    }
                }
            }

            if(MF_plan_record_i < 14){
                if(MF_x != MF_x_Last || MF_y != MF_y_Last){
                    if((MF_x == 0 || MF_x == 5) && (MF_y == 0 || MF_y == 4)){
                        MF_plan[MF_plan_record_i].MF_x = MF_x;
                        MF_plan[MF_plan_record_i].MF_y = MF_y;
                        MF_plan[MF_plan_record_i].is_picking = false;
                        MF_plan[MF_plan_record_i].is_valid = true;
                        MF_plan_record_i++;
                        MF_pick_Flag = false;
                    }else {
                        MF_pick_Flag = true;
                    }
                }
                if(MF_pick_Flag){
                    if(control_xbox_cmd.btnA == true && control_xbox_cmd_Last.btnA == false){
                        MF_plan[MF_plan_record_i].MF_x = MF_x;
                        MF_plan[MF_plan_record_i].MF_y = MF_y;
                        MF_plan[MF_plan_record_i].is_picking = true;
                        MF_plan[MF_plan_record_i].is_valid = true;
                        MF_plan_record_i++;
                        MF_pick_Flag = false;
                    }
                }
            }
        }else if(MF_plan_record_Flag_Last == true){
            if(MF_plan_record_i > 0){
                if(MF_x != MF_plan[MF_plan_record_i-1].MF_x || MF_y != MF_plan[MF_plan_record_i-1].MF_y){
                    MF_plan[MF_plan_record_i].MF_x = MF_x;
                    MF_plan[MF_plan_record_i].MF_y = MF_y;
                    MF_plan[MF_plan_record_i].is_picking = false;
                    MF_plan[MF_plan_record_i].is_valid = true;
                    MF_plan_record_i++;
                }
            }else if(MF_plan_record_i == 0){
                MF_plan[MF_plan_record_i].MF_x = MF_x;
                MF_plan[MF_plan_record_i].MF_y = MF_y;
                MF_plan[MF_plan_record_i].is_picking = false;
                MF_plan[MF_plan_record_i].is_valid = true;
                MF_plan_record_i++;
            }
            
            MF_plan_run_Flag = true;
        }

        if(MF_plan_run_Flag == true){

            // === 上身动作每帧推进（无论 MF_action_Flag 状态，有动作就必须跑） ===
            upbody_ctrl.Update(0.005f, upbody_pub);

            // === 动作完成检测：全部步执行完毕 → 收尾 ===
            if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending() && mf_placing) {
                mf_placing = false;
                MF_action_Flag = false;
                MF_plan[MF_plan_run_i].is_picking = false;
                MF_plan_run_i++;
                MF_xy_complete_Flag = false;
                MF_omega_complete_Flag = false;
                MF_omega_control_Flag = 0;
                last_mf_action = -1;  // 重置，下个拾取点不受限制
            }


            // === 底盘逼近偏移渐变（每帧运行，不受 MF_action_Flag 限制） ===
            {
                float target = (mf_placing && upbody_ctrl.IsApproachPhase()) ? kMF_ApproachDist : 0.0f;
                float step   = kMF_ApproachSpeed * 0.005f;  // dt = 5ms
                if (mf_approach_offset < target) {
                    mf_approach_offset += step;
                    if (mf_approach_offset > target) mf_approach_offset = target;
                } else if (mf_approach_offset > target) {
                    mf_approach_offset -= step;
                    if (mf_approach_offset < target) mf_approach_offset = target;
                }

                switch (mf_approach_facing) {
                    case 0:   MF_close_position_x = 0.0f;                     MF_close_position_y =  mf_approach_offset; break;
                    case 90:  MF_close_position_x = -mf_approach_offset;      MF_close_position_y = 0.0f;                   break;
                    case -90: MF_close_position_x =  mf_approach_offset;      MF_close_position_y = 0.0f;                   break;
                    case 180: MF_close_position_x = 0.0f;                     MF_close_position_y = -mf_approach_offset; break;
                    default:  MF_close_position_x = 0.0f;                     MF_close_position_y = 0.0f;                   break;
                }


                // 偏移叠加到底盘目标（动作执行期间 state_aim_cmd 也需要跟着变）
                if (MF_plan_run_i < MF_plan_record_i && MF_plan[MF_plan_run_i].is_valid) {
                    state_target_cmd.linear_x_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][0] + MF_close_position_x;
                    state_target_cmd.linear_y_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][1] + MF_close_position_y;
                }
            }

            if(MF_action_Flag == false){

                if((MF_plan_run_i < MF_plan_record_i) && (MF_plan[MF_plan_run_i].is_valid == true)){

                    state_target_cmd.linear_x_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][0];
                    state_target_cmd.linear_y_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][1];
                    if((int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] != (int16_t)state_target_cmd.omega_){
                        if(MF_omega_control_Flag == 0){
                            if(control_position.y < 1.3f-MF_omega_correction){
                                if(abs(Warp_ToRange(robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] - state_target_cmd.omega_,-180.0f,180.0f)) > 100){
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][2][2];
                                }else {
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                }
                                MF_omega_control_Flag = -1;
                            }else if(control_position.y > 3.7f+MF_omega_correction){
                                if(abs(Warp_ToRange(robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] - state_target_cmd.omega_,-180.0f,180.0f)) > 100){
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][2][2];
                                }else {
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                }
                                MF_omega_control_Flag = 1;
                            }
                        }else if(MF_omega_control_Flag == -1){
                            if(control_position.y > 3.7f-MF_omega_correction){
                            // if(true){
                                if(abs(Warp_ToRange(robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] - state_target_cmd.omega_,-180.0f,180.0f)) > 100){
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][2][2];
                                }else {
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                }
                            }
                        }else if(MF_omega_control_Flag == 1){
                            if(control_position.y < 1.3f+MF_omega_correction){
                                if(abs(Warp_ToRange(robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] - state_target_cmd.omega_,-180.0f,180.0f)) > 100){
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][2][2];
                                }else {
                                    state_target_cmd.omega_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                }
                            }
                        }
                    }

                    // 上身保持行进姿态（每个新路径点仅触发一次）
                    if (last_moving_pose_i != MF_plan_run_i
                        && !upbody_ctrl.IsActive()
                        && !upbody_ctrl.HasPending()
                        && !mf_placing) {
                        upbody_ctrl.Moving(kPose_Moving_In_MF);
                        last_moving_pose_i = MF_plan_run_i;
                    }

                    if(((state_target_cmd.linear_x_ - control_position.x)*(state_target_cmd.linear_x_ - control_position.x) + (state_target_cmd.linear_y_ - control_position.y)*(state_target_cmd.linear_y_ - control_position.y)) < 0.03*0.03){
                        static uint8_t count_xy = 0;
                        if(MF_xy_complete_Flag == false){
                            if(count_xy >= 10){
                                MF_xy_complete_Flag = true;
                                count_xy = 0 ;
                            }else {
                                count_xy ++;
                            }
                        }
                    }
                    if(Warp_ToRange(robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2] - control_position.yaw,-180.0f,180.0f) < 0.5){
                        static uint8_t count_omega = 0;
                        if(MF_omega_complete_Flag == false){
                            if(count_omega >= 10){
                                MF_omega_complete_Flag = true;
                                count_omega = 0;
                            }else {
                                count_omega ++;
                            }
                        }
                    }
                    
                    if(MF_omega_complete_Flag == true && MF_xy_complete_Flag == true){
                        if(MF_plan[MF_plan_run_i].is_picking == true){
                            /*上层机构执行*/
                            switch ((int8_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][3]) {
                                case 1:
                                    if (last_mf_action != 1 && !mf_placing) {
                                        bool close_pump = (mf_place_cycle != 2);  // 第3个KFS不关泵
                                        upbody_ctrl.PickKFS(kPose_Pick[Low], kPose_Place[mf_place_cycle], close_pump);
                                        last_mf_action = 1;
                                        MF_action_Flag = true;
                                        mf_placing = true;
                                        mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                        mf_place_cycle = (mf_place_cycle + 1) % 3;
                                    }
                                    break;
                                case 2:
                                    if (last_mf_action != 2 && !mf_placing) {
                                        bool close_pump = (mf_place_cycle != 2);
                                        upbody_ctrl.PickKFS(kPose_Pick[Mid], kPose_Place[mf_place_cycle], close_pump);
                                        last_mf_action = 2;
                                        MF_action_Flag = true;
                                        mf_placing = true;
                                        mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                        mf_place_cycle = (mf_place_cycle + 1) % 3;
                                    }
                                    break;
                                case 3:
                                    if (last_mf_action != 3 && !mf_placing) {
                                        bool close_pump = (mf_place_cycle != 2);
                                        upbody_ctrl.PickKFS(kPose_Pick[High], kPose_Place[mf_place_cycle], close_pump);
                                        last_mf_action = 3;
                                        MF_action_Flag = true;
                                        mf_placing = true;
                                        mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                        mf_place_cycle = (mf_place_cycle + 1) % 3;
                                    }
                                    break;

                                default:
                                    last_mf_action = -1;
                                    break;
                            }
                        } else {
                            // 非拾取点（纯路径点）：无需上身动作，直接推进
                            MF_plan_run_i++;
                            MF_xy_complete_Flag = false;
                            MF_omega_complete_Flag = false;
                            MF_omega_control_Flag = 0;
                        }
                    }

                }else {
                    MF_plan_record_i = 0;
                    MF_plan_run_i = 0;
                    MF_plan_run_Flag = false;
                    MF_pick_Flag = false;
                    MF_action_Flag = false;
                    last_mf_action = -1;
                    mf_approach_offset = 0.0f;
                    MF_close_position_x = 0.0f;
                    MF_close_position_y = 0.0f;
                    for(uint8_t i;i<15;i++){
                        MF_plan[i] = MF_plan_zero;
                    }
                }
            }
        }else if(MF_plan_run_Flag_Last == true){
        }

    }

    Aim_State_xy_Process();
    Aim_State_omega_Process();
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
        if(Arena_close_position_y < Arena_close_position_y_Max)Arena_close_position_y = Arena_close_position_y + 0.0015f;
    }else{
        if(Arena_close_position_y > 0.0f){
            Arena_close_position_y =  Arena_close_position_y - 0.0015;
        }else {
            Arena_close_position_y = 0.0f;
        }   
    }
    // ===== 九宫格子模式切换 =====
    static bool arena_r2_mode  = false;  // false=KFS放置模式, true=R2合体模式
    static bool arena_r2_floor = false;  // false=R2一楼, true=R2二楼

    // btnY 上升沿切换子模式
    static bool last_btnY_arena = false;
    if (control_xbox_cmd.btnY && !last_btnY_arena) {
        arena_r2_mode = !arena_r2_mode;
        if (arena_r2_mode) {
            arena_r2_floor = false;   // 进入合体模式默认一楼
        }
        last_arena_x = -1;            // 强制刷新上身姿态
    }

    last_btnY_arena = control_xbox_cmd.btnY;

    // btnA 上升沿检测（if/else 共用）
    static bool last_btnA_arena = false;
    
    if (!arena_r2_mode) {
        state_target_cmd.linear_x_ = robot_position_Arena[Arena_x][0];
        state_target_cmd.linear_y_ = robot_position_Arena[Arena_x][1] + Arena_close_position_y;
        Aim_State_xy_Process();

        state_target_cmd.omega_    = robot_position_Arena[Arena_x][2];
        Aim_State_omega_Process();

        /*上层机构执行*/
        if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending() && last_arena_x != Arena_x) {
            switch ((int16_t)state_target_cmd.omega_) {
                case 0:
                    upbody_ctrl.GrabKFS_Arena(kPose_Grid9_Bot12);
                    break;
                case -90:
                    upbody_ctrl.GrabKFS_Arena(kPose_Grid9_Bot3);
                    break;
            }
            last_arena_x = Arena_x;
        }


        // 每帧推进渐变
        upbody_ctrl.Update(0.005f, upbody_pub);


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

        // 获取KFS（渐变空闲时响应，先近后远）
        static bool get_toggle = false;  // false=Get2(近), true=Get1(远)
        if (!upbody_ctrl.IsActive()) {
            if (control_xbox_cmd.btnA && !last_btnA_arena) {
                upbody_ctrl.GetKFS(get_toggle ? kPose_Get1 : kPose_Get2);
                get_toggle = !get_toggle;
                last_arena_x = -1;
            }
        }
        
    } else {
        state_target_cmd.linear_x_ = robot_position_Arena_withR2[Arena_x][0];
        state_target_cmd.linear_y_ = robot_position_Arena_withR2[Arena_x][1] + Arena_close_position_y;
        Aim_State_xy_Process();

        state_target_cmd.omega_    = robot_position_Arena_withR2[Arena_x][2];
        Aim_State_omega_Process();

        /*上层机构执行*/
        // 进入合体模式或切换格子时，默认设为当前楼层姿态
        if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending() && last_arena_x != Arena_x) {
            upbody_ctrl.R2MergePose(arena_r2_floor ? kPose_R2_Second_Floor : kPose_R2_First_Floor);
            last_arena_x = Arena_x;
        }

        // btnA 上升沿切换一楼/二楼
        if (!upbody_ctrl.IsActive()) {
            if (control_xbox_cmd.btnA && !last_btnA_arena) {
                arena_r2_floor = !arena_r2_floor;
                upbody_ctrl.R2MergePose(arena_r2_floor ? kPose_R2_Second_Floor : kPose_R2_First_Floor);
            }
        }

        // 每帧推进渐变
        upbody_ctrl.Update(0.005f, upbody_pub);
    }
    last_btnA_arena = control_xbox_cmd.btnA;  // ← 移到此处，每帧更新一次

    
    
}


// =====================================================
//  XY 位置 PID
// =====================================================
void Aim_State_xy_Process() {



    static float tx = 0.0f;
    static float ty = 0.0f;
    static float nx = 0.0f;
    static float ny = 0.0f;

    // 里程计定位修正
    position_correction_x = position_correction_x + 0.001f * xbox_cmd.linear_x_;
    position_correction_y = position_correction_y + 0.001f * xbox_cmd.linear_y_;

    if(fabsf(state_target_cmd.linear_x_ - state_target_last_cmd.linear_x_) > 0.001f || fabsf(state_target_cmd.linear_y_ - state_target_last_cmd.linear_y_) > 0.001f){
        //设置初始点位和目标点位
        state_start_cmd.linear_x_ = state_now_cmd.linear_x_;
        state_start_cmd.linear_y_ = state_now_cmd.linear_y_;

        //设置路径基础信息
        float path_dx = state_target_cmd.linear_x_ - state_start_cmd.linear_x_;
        float path_dy = state_target_cmd.linear_y_ - state_start_cmd.linear_y_;
        float path_len = sqrt(path_dx*path_dx + path_dy*path_dy);

        if(path_len > 0.001f){
            //设置路径单位切向量 
            tx = path_dx/path_len;
            ty = path_dy/path_len;
            //设置法向量
            nx = -ty;
            ny =  tx;
        }

    }

    float path_error = tx*(state_target_cmd.linear_x_ - state_now_cmd.linear_x_) + ty*(state_target_cmd.linear_y_ - state_now_cmd.linear_y_);
    float lateral_error = nx*(state_target_cmd.linear_x_ - state_now_cmd.linear_x_) + ny*(state_target_cmd.linear_y_ - state_now_cmd.linear_y_);

    float path_pid_output = PID_Calculate(&path, 0.0f, path_error);
    float lateral_pid_output = PID_Calculate(&lateral, 0.0f, lateral_error);

    float path_plan_Max = sqrt(2.0f*Acc_path_SpeedDown*fabs(path_error));  //规划最大速度
    if(path_plan_Max > path_plan_Max_Max)path_plan_Max = path_plan_Max_Max;

    static uint32_t Acc_path_DWT_CNT = 0;
    static float Acc_path_dt = 0.0f;
    Acc_path_dt = DWT_GetDeltaT(&Acc_path_DWT_CNT);  //获取加速计时器增量，单位s

    static float path_plan_output = 0.0f;
    path_plan_output = path_plan_output + Acc_path_SpeedUp*Acc_path_dt;  //速度规划实际值更新
    if(path_plan_output > path_plan_Max){
        path_plan_output = path_plan_Max;
    }

    static float K_path_planTopid = 0.0f;
    if(fabsf(path_error) > 1.0f){
        K_path_planTopid = 1.0f;
    }else if(fabsf(path_error) > 0.5f){
        K_path_planTopid = (fabsf(path_error) - 0.5f) / 0.5f;
    }else{
        K_path_planTopid = 0.0f;
    }
    // K_path_planTopid = 0.0f;

    float path_real_output = path_pid_output*( 1 - K_path_planTopid ) + sign(path_error)*path_plan_output*K_path_planTopid;

    float vx_world = path_real_output*tx + lateral_pid_output*nx;
    float vy_world = path_real_output*ty + lateral_pid_output*ny;

    predict_yaw = state_now_cmd.omega_ + (control_position.yaw_speed/kDegToRad)*yaw_delay_time;

    float cos_yaw = cosf(predict_yaw*kDegToRad);
    float sin_yaw = sinf(predict_yaw*kDegToRad);

    robot_v_aim_cmd.linear_x_ = vx_world*cos_yaw + vy_world*sin_yaw;
    robot_v_aim_cmd.linear_y_ =-vx_world*sin_yaw + vy_world*cos_yaw;

    state_target_last_cmd.linear_x_ = state_target_cmd.linear_x_;
    state_target_last_cmd.linear_y_ = state_target_cmd.linear_y_;

}


// =====================================================
//  角度 PID
// =====================================================
void Aim_State_omega_Process() {
    float error_dir = Warp_ToRange(state_target_cmd.omega_ - state_now_cmd.omega_,-180.0f,180.0f);
    float state_omega_error = fabs(error_dir);

    float omega_pid_output = kDegToRad * PID_Calculate(&omega, 0.0f, 0.0f + error_dir);

    float v_omega_plan_Max = sqrt(2.0f*Acc_omega_SpeedDown*state_omega_error*kDegToRad);  //规划最大速度
    if(v_omega_plan_Max > v_omega_plan_Max_Max)v_omega_plan_Max = v_omega_plan_Max_Max;

    Acc_omega_dt = DWT_GetDeltaT(&Acc_omega_DWT_CNT);  //获取加速计时器增量，单位s

    static float v_omega_plan_output;
    v_omega_plan_output = v_omega_plan_output + Acc_omega_SpeedUp*Acc_omega_dt;  //速度规划实际值更新
    if(v_omega_plan_output > v_omega_plan_Max)v_omega_plan_output = v_omega_plan_Max;
    
    static float K_omega_planTopid;
    if(state_omega_error > 15.0f){
        K_omega_planTopid = 1.0f;
    }else if(state_omega_error > 7.5f){
        K_omega_planTopid = (state_omega_error - 7.5f) / 7.5f;
    }else{
        K_omega_planTopid = 0.0f;
    }
    // K_omega_planTopid = 0.0f;

    robot_v_aim_cmd.omega_ = (omega_pid_output*(1.0f - K_omega_planTopid) + sign(error_dir)*v_omega_plan_output*K_omega_planTopid);
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
    static ActionController debug_ctrl;

    debug_ctrl.Update(0.005f, pub);
    // 泵/阀独立发消息
    if (pump_trigger || valve_trigger) {
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

    static int place_cycle = 0;
    
    if (!debug_ctrl.IsActive()) {
        if (control_xbox_cmd.btnDirUp    && !last_btnDirUp)    debug_ctrl.GrabKFS(kPose_KFS_High);
        if (control_xbox_cmd.btnDirRight && !last_btnDirRight) debug_ctrl.GrabKFS(kPose_KFS_Mid);
        if (control_xbox_cmd.btnDirDown  && !last_btnDirDown)  debug_ctrl.GrabKFS(kPose_KFS_Low);
        if (control_xbox_cmd.btnY        && !last_btnY)        debug_ctrl.GoHome();

        if (control_xbox_cmd.btnA && !last_btnA) {
            debug_ctrl.PlaceKFS(kPose_Place[place_cycle]);
            place_cycle = (place_cycle + 1) % 3;
        }
    }


    last_btnDirUp    = control_xbox_cmd.btnDirUp;
    last_btnDirRight = control_xbox_cmd.btnDirRight;
    last_btnDirDown  = control_xbox_cmd.btnDirDown;
    last_btnY        = control_xbox_cmd.btnY;
    last_btnA        = control_xbox_cmd.btnA;

}

// =====================================================
//  定位清零函数
// =====================================================
void Reset_position(){
    position_correction_x = -control_position_msg.x + position_center_distance*sin(control_position.yaw*kDegToRad);
    position_correction_y =  control_position_msg.y - position_center_distance*cos(control_position.yaw*kDegToRad);

    state_target_cmd.linear_x_ = 0.0f;
    state_target_cmd.linear_y_ = 0.0f;
}

// =====================================================
//  一定范围内头尾相连函数
// =====================================================
float Warp_ToRange(float value,float min,float max){
    float range = max - min;
    while(value > max){
        value = value - range;
    }
    while(value < min){
        value = value + range;
    }
    return value;
}

// =====================================================
//  返回预测yaw角
// =====================================================
float Get_predict_yaw(){
    return predict_yaw;
}

// =====================================================
//  限值
// =====================================================
float clamp(float value,float min,float max){
    if(value > max){
        return max;
    }else if(value < min){
        return min;
    }else {
        return value;
    }
}