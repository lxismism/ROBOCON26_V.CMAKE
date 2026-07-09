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
#include "rm_pocket.hpp"
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
static constexpr float kPickExtendStep = 0.5f;
static constexpr float kWeaponLiftStep = 0.75f;
static constexpr float kWeaponExtendStep = 1.2f;
static constexpr uint16_t kTriggerThreshold = 512;

// ===== MF 自动逼近参数（可配） =====
float kMF_ApproachDist  = 0.23f;   // 底盘前移距离 (m)，例如 0.07 = 7cm
float kMF_ApproachSpeed = 0.30f;   // 前移/后退速度 (m/s)，实测后改

// ===== 外部变量（定义在 control_task.cpp，此处声明引用） =====

// 发布者
extern TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub;
extern pub_chassis_cmd xbox_cmd;
extern pub_chassis_cmd rm_cmd;

extern TypedTopicPublisher<pub_ir_cmd> omni_ir_cmd_pub;                
extern pub_ir_cmd omni_ir_cmd_push;

extern TypedTopicPublisher<pub_ir_cmd> whisper_ir_cmd_pub;                
extern pub_ir_cmd whisper_ir_cmd_push;

// 订阅者
extern TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub;
extern pub_Xbox_Data control_xbox_cmd;
extern pub_Xbox_Data control_xbox_cmd_Last;

extern TypedTopicSubscriber<pub_RC_Data> control_rc_sub;
extern pub_RC_Data control_rm_cmd;
extern pub_RC_Data control_rm_cmd_last;

extern TypedTopicSubscriber<pub_Position_Data> control_position_sub;
extern pub_Position_Data control_position_msg;
extern pub_Position_Data control_position;

// 定位修正
extern float position_center_distance;
extern float position_correction_x;
extern float position_correction_y;

extern float position_close_x;
extern float position_close_y;


// 控制模式状态
extern RobotMode_t robot_mode;
extern RobotMode_t robot_case;

extern float rm_angle_deg;
extern float v_aim;


extern bool headless_xy_mode;
extern bool headless_omega_mode;
bool headless_mode = true;

extern bool Normal_control_mode;

speed_data Normal_speed{1.2f,0.9f,1.5f,M_PI*0.5};
// MC 模式相关
extern int8_t MC_y;
extern float MC_close_position_x;
extern float MC_close_position_y;
extern bool MC_mode;
extern bool MC_mode_last;
speed_data MC_speed{1.2f,0.9f,1.5f,M_PI*0.5};
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
speed_data MF_speed{1.6f,1.0f,2.5f,M_PI*0.55};

extern uint8_t MF_pick_count;
extern uint8_t MF_pick_count_last;
// Arena模式相关
extern int8_t Arena_x;
extern float Arena_close_position_y;
extern float Arena_close_position_y_Max;
extern float Arena_close_position_step;
speed_data Arena_speed{0.9f,0.7f,1.8f,M_PI*0.45};

extern FieldSide_t field_side;
extern const float robot_center_to_gimbal_x;

extern PickHand pick_hand;
extern WeaponHand weapon_hand;
extern Lift lift;

uint8_t Arena_ir_count = 0;
ArenaMode_t Arena_mode = KFS;

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
    robot_v_aim_cmd.linear_y_ = JoyToVelocity(control_xbox_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
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
void Chassis_RM_Data_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    static float Chassis_RM_Data_Process_dt = 0.0f;
    static uint32_t Chassis_RM_Data_Process_DWT_CNT = 0;
    Chassis_RM_Data_Process_dt = DWT_GetDeltaT(&Chassis_RM_Data_Process_DWT_CNT);

    rm_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    rm_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    rm_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);


    static bool ir_press_single_Flag = false;
    static bool ir_press_double_Flag = false;
    static float press_ir_since_last = 60.0f;
    const  float DOUBLE_CLICK_TIME = 0.35f;
    press_ir_since_last = press_ir_since_last + Chassis_RM_Data_Process_dt;
    if(press_ir_since_last > 60.0f) press_ir_since_last = 60.0f;//过大计算无意义
    if(control_rm_cmd.swE != control_rm_cmd_last.swE) {
        if(control_rm_cmd.swE == RC_2_POS_SW_State_t::DOWN) {
            if(press_ir_since_last < DOUBLE_CLICK_TIME && ir_press_single_Flag == true){
                ir_press_single_Flag = false;
                ir_press_double_Flag = true;
            }else{
                ir_press_single_Flag = true;
            }
            press_ir_since_last = 0.0f;
        }
    }
    if(ir_press_single_Flag == true){
        if(press_ir_since_last >= DOUBLE_CLICK_TIME){
            //单击功能
            switch (robot_mode) {
                case MC : {
                    //在这里面配置红外发送
                    switch (control_rm_cmd.cursor){
                        case 0 : {
                            omni_ir_cmd_push.tx_data = CMD_MC_RELEASE_CLAW;
                            omni_ir_cmd_pub.Publish(omni_ir_cmd_push);
                            break;
                        }
                        case 1 : {
                            omni_ir_cmd_push.tx_data = CMD_MC_PICK_NEW;
                            omni_ir_cmd_pub.Publish(omni_ir_cmd_push);  
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                }
                case MF : {
                    if(control_rm_cmd.cursor == 2){
                        omni_ir_cmd_push.tx_data = CMD_MC_ENTER_MF;
                        omni_ir_cmd_pub.Publish(omni_ir_cmd_push);
                    }
                    break;
                }
                case Arena : {
                    switch (Arena_ir_count){
                        case 0 : {
                            omni_ir_cmd_push.tx_data = CMD_ENTER_ARENA;
                            omni_ir_cmd_pub.Publish(omni_ir_cmd_push);
                            break;
                        }
                        case 1 : {
                            switch (control_rm_cmd.cursor){
                                case 0 : {
                                    omni_ir_cmd_push.tx_data = CMD_AUTO_PUT_MIDDLE_LEFT;
                                    omni_ir_cmd_pub.Publish(omni_ir_cmd_push);  
                                    break;
                                }
                                case 1 : {
                                    omni_ir_cmd_push.tx_data = CMD_AUTO_PUT_MIDDLE_MIDDLE;
                                    omni_ir_cmd_pub.Publish(omni_ir_cmd_push);
                                    break;
                                }
                                case 2 : {
                                    omni_ir_cmd_push.tx_data = CMD_AUTO_PUT_MIDDLE_RIGHT;
                                    omni_ir_cmd_pub.Publish(omni_ir_cmd_push);  
                                    break;
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                        case 2 : {
                            switch (control_rm_cmd.cursor){
                                case 0 : {
                                    omni_ir_cmd_push.tx_data = CMD_JOINT;
                                    omni_ir_cmd_pub.Publish(omni_ir_cmd_push);  
                                    break;
                                }
                                case 1 : {
                                    whisper_ir_cmd_push.tx_data = CMD_RELEASE_KFS;
                                    whisper_ir_cmd_pub.Publish(whisper_ir_cmd_push);  
                                    break;
                                }
                                case 2 : {
                                    whisper_ir_cmd_push.tx_data = CMD_HOLD_KFS;
                                    whisper_ir_cmd_pub.Publish(whisper_ir_cmd_push);  
                                    break;
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                    }
                }
                default:
                    break;
            }
            ir_press_single_Flag = false;
        }
    }else if(ir_press_double_Flag == true){
        //双击功能
        if(robot_mode == Arena)Arena_ir_count ++;
        while(Arena_ir_count > 2)Arena_ir_count = Arena_ir_count - 3;
        ir_press_double_Flag = false;
    }


    switch (robot_case) {
        case RobotCase_t::Normal_case : {
            Normal_control_Process();
            break;
        }
        case RobotCase_t::Special : {
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
        default:
            break;
    }
}



// =====================================================
//  普通手操 / 定位模式
// =====================================================
void Normal_control_Process() {
    
    last_mf_action = -1;

    if (control_rm_cmd.trimLeft == RC_Trim_State_t::UP) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            upbody_ctrl.GoHome();
        }
    }

    // swA 上升沿切换 xy 模式
    if (control_rm_cmd.swA == RC_2_POS_SW_State_t::DOWN) {
        headless_xy_mode = false;
    }else {
        headless_xy_mode = true;
    }

    // swB 上升沿切换 omega 模式
    if (control_rm_cmd.swD == RC_2_POS_SW_State_t::DOWN) {
        headless_omega_mode = false;
    }else {
        headless_omega_mode = true;
    }

    if (control_rm_cmd.trimRight == RC_Trim_State_t::LEFT) {
        if (control_rm_cmd.trimRight_last == RC_Trim_State_t::MIDDLE) {
            field_side = Left;
            chassis_Map_Set();
        }
    }else if (control_rm_cmd.trimRight == RC_Trim_State_t::RIGHT) {
        if (control_rm_cmd_last.trimRight == RC_Trim_State_t::MIDDLE) {
            field_side = right;
            chassis_Map_Set();
        }
    }

    if (headless_xy_mode) {
        // xy 手控模式：摇杆 → 速度，直接进入速度环 PID
        rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
        v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);

        robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
        robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);

        state_target_cmd.linear_x_ = state_now_cmd.linear_x_;
        state_target_cmd.linear_y_ = state_now_cmd.linear_y_;
    } else {
        // xy 定位模式：方向键控制目标位置，进入位置环 PID
        // 上下控制
        if (control_rm_cmd.trimLeft == RC_Trim_State_t::UP) {
            if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                state_target_cmd.linear_y_ = state_target_cmd.linear_y_ + 3.0f;
            }
        } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::DOWN) {
            if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                state_target_cmd.linear_y_ = state_target_cmd.linear_y_ - 3.0f;
            }
        }
        // 左右控制
        if (control_rm_cmd.trimLeft == RC_Trim_State_t::LEFT) {
            if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                state_target_cmd.linear_x_ = state_target_cmd.linear_x_ - 1.0f;
            }
        } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::RIGHT) {
            if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                state_target_cmd.linear_x_ = state_target_cmd.linear_x_ + 1.0f;
            }
        }
        Aim_State_xy_Process();
    }

    if (headless_omega_mode) {
        // omega 手控模式：右摇杆直接控制角速度
        robot_v_aim_cmd.omega_ = rm_cmd.omega_;
        state_target_cmd.omega_ = control_position.yaw;
    } else {
        // omega 定位模式：右摇杆推到极限 → 设定目标角度
        if (ABS(control_rm_cmd.joyRHori - kJoyCenter) > 700) {
            if ((control_rm_cmd.joyRHori - kJoyCenter) > 0) {
                state_target_cmd.omega_ = -90.0f;
            } else {
                state_target_cmd.omega_ = 90.0f;
            }
        } else if (ABS(control_rm_cmd.joyRVert - kJoyCenter) > 700) {
            if ((control_rm_cmd.joyRVert - kJoyCenter) < 0) {
                state_target_cmd.omega_ = 180.0f;
            } else {
                state_target_cmd.omega_ = 0.0f;
            }
        }
        Aim_State_omega_Process();
    }

    Traject_chassis.Set_Ref(state_now_cmd,Normal,Normal_speed);
}


// =====================================================
//  武馆半自动网格定位
// =====================================================
void MC_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {
    static bool mc_prepare_wrist_pending = false;
    MC_mode_last = MC_mode;

    if (control_rm_cmd.trimLeft == RC_Trim_State_t::UP) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            if (MC_y < 3) {
                MC_y = MC_y + 1;
            }
        }
    } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::DOWN) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            if (MC_y > 0) {
                MC_y = MC_y - 1;
            }
        }
    }else if (control_rm_cmd.trimLeft == RC_Trim_State_t::LEFT) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            MC_mode = true;
            upbody_ctrl.PrepareWeapon();
            mc_prepare_wrist_pending = true;

        }
    } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::RIGHT) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            MC_mode = false;
        }
    }

    if (MC_mode) {
        static bool MC_Traj_complete_Flag = false;
        if(MC_mode_last != MC_mode){
            MC_Traj_complete_Flag = false;
        }
        if(MC_Traj_complete_Flag == false){
            state_target_cmd.linear_x_ = 0.90f*field_side;
            state_target_cmd.linear_y_ = 2.67f;
            state_target_cmd.omega_    = 0.0f;
            MC_close_position_x = MC_close_position_x + 0.001f * rm_cmd.linear_x_;
            MC_close_position_y = MC_close_position_y + 0.001f * rm_cmd.linear_y_;
            position_close_x = MC_close_position_x;
            position_close_y = MC_close_position_y;
            Traject_chassis.Run(state_now_cmd);
            robot_v_aim_cmd = Traject_chassis.Get_output_b();
            Traject_chassis.Set_Ref(state_target_cmd,Normal,MC_speed);

            if(Traject_chassis.PointTrack_omega_complete_Flag == true && Traject_chassis.PointTrack_linear_complete_Flag == true){
                MC_Traj_complete_Flag = true;
            }
        }else{
            rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
            v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);

            robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);

            robot_v_aim_cmd.omega_ = rm_cmd.omega_;

            state_target_cmd.linear_x_ = state_now_cmd.linear_x_;
            state_target_cmd.linear_y_ = state_now_cmd.linear_y_;
            state_target_cmd.omega_ = control_position.yaw;

            Traject_chassis.Set_Ref(state_now_cmd,Normal,Normal_speed);
        }
        
    } else {
        state_target_cmd.linear_x_ = robot_position_MC[MC_y][0];
        state_target_cmd.linear_y_ = robot_position_MC[MC_y][1];
        state_target_cmd.omega_    = robot_position_MC[MC_y][2];
        Traject_chassis.Set_Ref(state_target_cmd,Normal,MC_speed);
        MC_close_position_x = MC_close_position_x + 0.001f * rm_cmd.linear_x_;
        MC_close_position_y = MC_close_position_y + 0.001f * rm_cmd.linear_y_;
        position_close_x = MC_close_position_x;
        position_close_y = MC_close_position_y;
        Traject_chassis.Run(state_now_cmd);
        robot_v_aim_cmd = Traject_chassis.Get_output_b();
    }

    // Traject_chassis.Set_Ref(state_target_cmd,Normal,MC_speed);
    // MC_close_position_x = MC_close_position_x + 0.001f * rm_cmd.linear_x_;
    // MC_close_position_y = MC_close_position_y + 0.001f * rm_cmd.linear_y_;
    // position_close_x = MC_close_position_x;
    // position_close_y = MC_close_position_y;
    // Traject_chassis.Run(state_now_cmd);
    // robot_v_aim_cmd = Traject_chassis.Get_output_b();

        // ---- 武器手控制 ----
    // 上身：动作优先推进，空闲时手操
    upbody_ctrl.Update(0.005f, upbody_pub);

    // PrepareWeapon 完成后翻转达妙
    if (!upbody_ctrl.IsActive() && mc_prepare_wrist_pending) {
        weapon_hand.wrist_target_rad_ = 0.087266f;
        mc_prepare_wrist_pending = false;
    }

    if (upbody_ctrl.IsActive()) return;


    upbody_msg = {};
    upbody_msg.active = true;

    // 持续型：右摇杆水平推武器手伸缩（霍尔值线性映射速度，2倍速）
    {
        int32_t rhori_diff = (int32_t)control_rm_cmd.joyRHori - (int32_t)kJoyCenter;
        if (ABS(rhori_diff) > (int32_t)kJoyDeadZoneRight) {
            float ratio = (float)(ABS(rhori_diff) - (int32_t)kJoyDeadZoneRight)
                          / (float)(kJoyCenter - kJoyDeadZoneRight)
                          * (rhori_diff > 0 ? 1.0f : -1.0f);
            upbody_msg.weapon_extend_delta = ratio * kWeaponExtendStep;
        }
    }

    // 持续型：右摇杆前推抬升 / 后拉下降（霍尔值线性映射速度）
    {
        int32_t rvert_diff = -((int32_t)control_rm_cmd.joyRVert - (int32_t)kJoyCenter);
        if (ABS(rvert_diff) > (int32_t)kJoyDeadZoneRight) {
            float ratio = (float)(ABS(rvert_diff) - (int32_t)kJoyDeadZoneRight)
                          / (float)(kJoyCenter - kJoyDeadZoneRight)
                          * (rvert_diff > 0 ? -1.0f : 1.0f);
            upbody_msg.weapon_lift_delta = ratio * kWeaponLiftStep;
        }
    }

    // 切换型：swD 夹爪开合 / swA 腕部翻转
    if (control_rm_cmd.swD != control_rm_cmd_last.swD){
        upbody_msg.claw_toggle = true;
    }
    if (control_rm_cmd.swA != control_rm_cmd_last.swA){
        upbody_msg.wrist_toggle = true;
    }

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

    static bool mf_chassis_freed = false;   // 底盘已解锁出发

    bool MF_plan_run_Flag_Last = MF_plan_run_Flag;
    bool MF_plan_record_Flag_Last = MF_plan_record_Flag;

    if(control_rm_cmd.swD == RC_2_POS_SW_State_t::DOWN){
        if(MF_plan_run_Flag == false){
            MF_plan_record_Flag = true;
        }else {
            MF_plan_record_Flag = false;
        }
    }else {
        MF_plan_record_Flag = false;
    }

    if(MF_plan_record_Flag == true){
        if (MF_x == 0 || MF_x == 5) {
            if (control_rm_cmd.trimLeft == RC_Trim_State_t::UP) {
                if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                    if (MF_y < 4) {
                        MF_y = MF_y + 1;
                    }
                }
            } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::DOWN) {
                if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                    if (MF_y > 0) {
                        MF_y = MF_y - 1;
                    }
                }
            }
        }

        if (MF_y == 0 || MF_y == 4) {
            if (control_rm_cmd.trimLeft == RC_Trim_State_t::LEFT) {
                if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                    if (MF_x - field_side <= 5 && MF_x - field_side >= 0) {
                        MF_x = MF_x - field_side;
                    }
                }
            } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::RIGHT) {
                if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
                    if (MF_x + field_side >= 0 && MF_x + field_side <= 5) {
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
            if(MF_pick_Flag && MF_pick_count < 3){
                if(control_rm_cmd.swE != control_rm_cmd_last.swE){
                    if(control_rm_cmd.swE == RC_2_POS_SW_State_t::DOWN){
                        MF_plan[MF_plan_record_i].MF_x = MF_x;
                        MF_plan[MF_plan_record_i].MF_y = MF_y;
                        MF_plan[MF_plan_record_i].is_picking = true;
                        MF_plan[MF_plan_record_i].is_valid = true;
                        MF_plan_record_i++;
                        MF_pick_Flag = false;

                        MF_pick_count ++;                     
                        MF_pick_count_last = MF_pick_count;
                    }
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

        // === 完成 / 解锁检测（同一把锁，两阶段） ===
        if (mf_placing) {
            bool chassis_ready = upbody_ctrl.IsChassisReleased();
            bool all_done = !upbody_ctrl.IsActive() && !upbody_ctrl.HasPending();

            if (chassis_ready || all_done) {
                if (!mf_chassis_freed) {
                    // 阶段1：推进路径索引 + 底盘出发
                    mf_chassis_freed = true;
                    MF_action_Flag = false;
                    MF_plan_run_i++;
                    MF_xy_complete_Flag = false;
                    MF_omega_complete_Flag = false;
                    MF_omega_control_Flag = 0;
                    last_mf_action = -1;
                    mf_approach_offset = 0.0f;
                    MF_close_position_x = 0.0f;
                    MF_close_position_y = 0.0f;
                    position_close_x = 0.0f;
                    position_close_y = 0.0f;
                }
                if (all_done) {
                    // 阶段2：上身全完成 → 收尾
                    mf_placing = false;
                    mf_chassis_freed = false;
                }
            }
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
                position_close_x = MF_close_position_x;
                position_close_y = MF_close_position_y;
            }
        }



        if(MF_action_Flag == false){

            if((MF_plan_run_i < MF_plan_record_i) && (MF_plan[MF_plan_run_i].is_valid == true)){

                state_target_cmd.linear_x_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][0];
                state_target_cmd.linear_y_ = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][1];
                state_target_cmd.omega_    = robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                Traject_chassis.Set_Ref(state_target_cmd,MF,MF_speed);
 
                // 上身保持行进姿态（每个新路径点仅触发一次）
                if (last_moving_pose_i != MF_plan_run_i
                    && !upbody_ctrl.IsActive()
                    && !upbody_ctrl.HasPending()
                    && !mf_placing) {
                    upbody_ctrl.Moving(kPose_Moving_In_MF);
                    weapon_hand.wrist_target_rad_ = 1.4835298f;   // 85°，竖杆防碰撞
                    last_moving_pose_i = MF_plan_run_i;

                }
                
                if(Traject_chassis.PointTrack_omega_complete_Flag == true && Traject_chassis.PointTrack_linear_complete_Flag == true){
                //  if(true){
                     if(MF_plan[MF_plan_run_i].is_picking == true){
                        // 根据 MF_pick_count 选择放置姿态序列
                        static const RobotPose kPlace_1[] = {kPose_Place1_2};
                        static const RobotPose kPlace_2[] = {kPose_Place1_1, kPose_Place1_2};
                        const RobotPose* place_poses;
                        int place_max;
                        switch (MF_pick_count) {
                            case 1:  place_poses = kPlace_1; place_max = 1; break;
                            case 2:  place_poses = kPlace_2; place_max = 2; break;
                            default: place_poses = kPose_Place; place_max = 3; break;
                        }

                        /*上层机构执行*/
                        switch ((int8_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][3]) {
                            case 1:
                                if (last_mf_action != 1 && !mf_placing) {
                                    bool close_pump = (mf_place_cycle != place_max - 1);  // 第3个KFS不关泵
                                    upbody_ctrl.PickKFS(kPose_Pick[Low],  place_poses[mf_place_cycle], close_pump);   // case 1
                                    last_mf_action = 1;
                                    MF_action_Flag = true;
                                    mf_placing = true;
                                    mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                    mf_place_cycle = (mf_place_cycle + 1) % place_max;
                                }
                                break;
                            case 2:
                                if (last_mf_action != 2 && !mf_placing) {
                                    bool close_pump = (mf_place_cycle != place_max - 1);
                                    upbody_ctrl.PickKFS(kPose_Pick[Mid],  place_poses[mf_place_cycle], close_pump);   // case 2
                                    last_mf_action = 2;
                                    MF_action_Flag = true;
                                    mf_placing = true;
                                    mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                    mf_place_cycle = (mf_place_cycle + 1) % place_max;
                                }
                                break;
                            case 3:
                                if (last_mf_action != 3 && !mf_placing) {
                                    bool close_pump = (mf_place_cycle != place_max - 1);
                                    upbody_ctrl.PickKFS(kPose_Pick[High], place_poses[mf_place_cycle], close_pump);   // case 3
                                    last_mf_action = 3;
                                    MF_action_Flag = true;
                                    mf_placing = true;
                                    mf_approach_facing = (int16_t)robot_position_MF[MF_plan[MF_plan_run_i].MF_x][MF_plan[MF_plan_run_i].MF_y][2];
                                    mf_place_cycle = (mf_place_cycle + 1) % place_max;
                                }
                                break;

                            default:
                                last_mf_action = -1;
                                break;
                        }
                    } else {
                        // 非拾取点（纯路径点）：无需上身动作，直接推进
                        MF_plan_run_i++;
                    }
                }

            } else if (!mf_placing) {
                MF_plan_record_i = 0;
                MF_plan_run_i = 0;
                MF_plan_run_Flag = false;
                MF_pick_Flag = false;

                MF_pick_count = 0;
                mf_place_cycle = 0;
                MF_action_Flag = false;
                last_mf_action = -1;
                mf_approach_offset = 0.0f;
                MF_close_position_x = 0.0f;
                MF_close_position_y = 0.0f;
                for(uint8_t i = 0;i<15;i++){
                    MF_plan[i] = MF_plan_zero;
                }
            }
        }
    }else if(MF_plan_run_Flag_Last == true){
    }

    position_correction_x = position_correction_x + 0.001f * rm_cmd.linear_x_;
    position_correction_y = position_correction_y + 0.001f * rm_cmd.linear_y_;
    Traject_chassis.Run(state_now_cmd);
    robot_v_aim_cmd = Traject_chassis.Get_output_b();
    // Aim_State_xy_Process();
    // Aim_State_omega_Process();
}

 
// =====================================================
//  九宫格半自动网格定位
// =====================================================
void Arena_control_Process(TypedTopicPublisher<pub_upbody_cmd>& upbody_pub, pub_upbody_cmd& upbody_msg) {

    if (control_rm_cmd.trimLeft == RC_Trim_State_t::RIGHT) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            if (Arena_x - field_side <= 2 && Arena_x - field_side >= 0) {
                Arena_x = Arena_x - field_side;
            }
        }
    } else if (control_rm_cmd.trimLeft == RC_Trim_State_t::LEFT) {
        if (control_rm_cmd_last.trimLeft == RC_Trim_State_t::MIDDLE) {
            if (Arena_x + field_side >= 0 && Arena_x + field_side <= 2) {
                Arena_x = Arena_x + field_side;
            }
        }
    }

    // ===== 九宫格子模式切换 =====
    static bool arena_r2_floor = false;   // false=R2一楼, true=R2二楼

    if (control_rm_cmd.trimRight == RC_Trim_State_t::UP) {
        if (control_rm_cmd.trimRight_last == RC_Trim_State_t::MIDDLE) {
            Arena_mode = WithR2;
            arena_r2_floor = false;   // 进入合体模式默认一楼
            last_arena_x = -1;
            weapon_hand.wrist_target_rad_ = 1.4835298f;   // ← 加这行，达妙竖杆
        }

    }else if (control_rm_cmd.trimRight == RC_Trim_State_t::RIGHT) {
        if (control_rm_cmd_last.trimRight == RC_Trim_State_t::MIDDLE) {
            Arena_mode = KFS;
            // if (arena_r2_mode) {
            //     arena_r2_floor = false;   // 进入合体模式默认一楼
            // }
            last_arena_x = -1;
            weapon_hand.wrist_target_rad_ = 1.4835298f;   // ← 加这行，达妙竖杆
        }
    }else if (control_rm_cmd.trimRight == RC_Trim_State_t::DOWN) {
        if (control_rm_cmd_last.trimRight == RC_Trim_State_t::MIDDLE) {
            Arena_mode = Weapon;
            // if (arena_r2_mode) {
            //     arena_r2_floor = false;   // 进入合体模式默认一楼
            // }
            last_arena_x = -1;            // 强制刷新上身姿态
        }
    }

    if(control_rm_cmd.swD == RC_2_POS_SW_State_t::DOWN){
        if(Arena_close_position_y < Arena_close_position_y_Max)Arena_close_position_y = Arena_close_position_y + Arena_close_position_step;
    }else{
        if(control_rm_cmd_last.swD == RC_2_POS_SW_State_t::DOWN){
            //按键下降沿
            if(Arena_mode != Weapon){
                pub_upbody_cmd pump_off = {};
                pump_off.active = true;
                pump_off.pump_cmd  = -1;
                pump_off.valve_cmd = -1;
                upbody_pub.Publish(pump_off);
            }
        }
        if(Arena_close_position_y > 0.0f){
            Arena_close_position_y =  Arena_close_position_y - Arena_close_position_step;
        }else {
            Arena_close_position_y = 0.0f;
        }
    }
    
    if (Arena_mode == KFS) {
        state_target_cmd.linear_x_ = robot_position_Arena[Arena_x][0];
        state_target_cmd.linear_y_ = robot_position_Arena[Arena_x][1] + Arena_close_position_y;

        state_target_cmd.omega_    = robot_position_Arena[Arena_x][2];

        Arena_close_position_step = 0.0015f;

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

        // 获取KFS（渐变空闲时响应，先近后远）
        static bool get_toggle = false;  // false=Get2(近), true=Get1(远)
        if (!upbody_ctrl.IsActive()) {
            if (control_rm_cmd.swA != control_rm_cmd_last.swA) {
                if(control_rm_cmd.swA == RC_2_POS_SW_State_t::DOWN){
                    const RobotPose& get_pose = (MF_pick_count_last == 2) ? kPose_2Get
                                                : (get_toggle ? kPose_Get1 : kPose_Get2);
                    upbody_ctrl.GetKFS(get_pose);
                    get_toggle = !get_toggle;
                    last_arena_x = -1;    
                }
            }
        }
        
    } else if(Arena_mode == WithR2){
        state_target_cmd.linear_x_ = robot_position_Arena_withR2[Arena_x][0];
        state_target_cmd.linear_y_ = robot_position_Arena_withR2[Arena_x][1];
        position_close_y = Arena_close_position_y;

        state_target_cmd.omega_    = robot_position_Arena_withR2[Arena_x][2];

        Arena_close_position_step = 0.0008f;

        /*上层机构执行*/
        // 进入合体模式或切换格子时，默认设为当前楼层姿态
        if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending() && last_arena_x != Arena_x) {
            upbody_ctrl.R2MergePose(arena_r2_floor ? kPose_R2_Second_Floor : kPose_R2_First_Floor);
            last_arena_x = Arena_x;
        }

        // 右摇杆上下切换一楼/二楼
        if (ABS(control_rm_cmd.joyRVert - kJoyCenter) > 700) {
            if ((control_rm_cmd.joyRVert - kJoyCenter) > 0) {
                arena_r2_floor = true;
            } else {
                arena_r2_floor = false;
            }
            upbody_ctrl.R2MergePose(arena_r2_floor ? kPose_R2_Second_Floor : kPose_R2_First_Floor);
        }

        //右摇杆左右切换云台180度和45度（）也可以是其他适宜角度
        if (!upbody_ctrl.IsActive() && ABS(control_rm_cmd.joyRHori - kJoyCenter) > 700) {
            if ((control_rm_cmd.joyRHori - kJoyCenter) > 0) {
                upbody_ctrl.YawTo(45.0f);
            } else {
                upbody_ctrl.YawTo(180.0f);
            }
        }


        // 每帧推进渐变
        upbody_ctrl.Update(0.005f, upbody_pub);

    }else if(Arena_mode == Weapon){
        state_target_cmd.linear_x_ = robot_position_Arena_useWeapon[Arena_x][0];
        state_target_cmd.linear_y_ = robot_position_Arena_useWeapon[Arena_x][1];
        position_close_y = Arena_close_position_y;

        state_target_cmd.omega_    = robot_position_Arena_useWeapon[Arena_x][2];

        Arena_close_position_step = 0.0015f;

	    // 右摇杆上下切换武器第一/第二层
	    static bool weapon_floor = false;       // false=第一层, true=第二层
	    static bool last_weapon_floor = false;
	    if (ABS(control_rm_cmd.joyRVert - kJoyCenter) > 700) {
	        if ((control_rm_cmd.joyRVert - kJoyCenter) > 0) {
	            weapon_floor = true;            // 武器抬到第二层
	        } else {
	            weapon_floor = false;           // 武器抬到第一层
	        }
	    }
	    // 模式进入 / 格子切换 / 楼层切换 → 触发PokeWeapon
		    if (!upbody_ctrl.IsActive() && !upbody_ctrl.HasPending()
		        && (last_arena_x != Arena_x || weapon_floor != last_weapon_floor)) {
		        upbody_ctrl.PokeWeapon(weapon_floor ? kPose_Poke2 : kPose_Poke1,
		                               weapon_floor ? 1 : 0);
		        last_arena_x = Arena_x;
		        last_weapon_floor = weapon_floor;
		    }


	    // 每帧推进渐变
	    upbody_ctrl.Update(0.005f, upbody_pub);
	}
    Traject_chassis.Set_Ref(state_target_cmd,Normal,Arena_speed);
    position_correction_x = position_correction_x + 0.001f * rm_cmd.linear_x_;
    position_correction_y = position_correction_y + 0.001f * rm_cmd.linear_y_;
    Traject_chassis.Run(state_now_cmd);
    robot_v_aim_cmd = Traject_chassis.Get_output_b();
}


// =====================================================
//  XY 位置 PID
// =====================================================
void Aim_State_xy_Process() {



    static float tx = 1.0f;
    static float ty = 0.0f;
    static float nx = 0.0f;
    static float ny = 1.0f;

    // 里程计定位修正
    position_correction_x = position_correction_x + 0.001f * rm_cmd.linear_x_;
    position_correction_y = position_correction_y + 0.001f * rm_cmd.linear_y_;

    if(fabsf(state_target_cmd.linear_x_ - state_target_last_cmd.linear_x_) > 0.0015f || fabsf(state_target_cmd.linear_y_ - state_target_last_cmd.linear_y_) > 0.0015f){
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

// =====================================================
//  计算
// =====================================================