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
static constexpr float kPickLiftStep = 0.3f;
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

// 控制模式状态
extern bool headless_xy_mode;
extern bool headless_omega_mode;
extern bool MF_control_mode;
extern int8_t MF_x;
extern int8_t MF_y;

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

    if (control_xbox_cmd.btnX && !last_btnX)
        msg.pump_toggle = true;
    last_btnX = control_xbox_cmd.btnX;

    if (control_xbox_cmd.btnB && !last_btnB)
        msg.valve_toggle = true;
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
void Chassis_Xbox_Data_Process() {
    xbox_cmd.linear_x_ = JoyToVelocity(control_xbox_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    xbox_cmd.linear_y_ = -JoyToVelocity(control_xbox_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
    xbox_cmd.omega_    = -JoyToVelocity(control_xbox_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

    // btnB 上升沿切换梅林模式
    if (control_xbox_cmd.btnB != control_xbox_cmd_Last.btnB) {
        if (control_xbox_cmd.btnB == true) {
            MF_control_mode = !MF_control_mode;
            /* ======= 若是进入MF控制模式，计算最近的MF点位，并自动移动过去 ====== */
            // if(MF_control_mode)
            // {
            //   int8_t closest_x = 0;
            //   int8_t closest_y = 0;
            //   float min_distance = 100.0f;//初始化一个较大的距离值
            //   for(int x=0;x<6;x++){
            //     for(int y=0;y<5;y++){
            //       if(robot_position_MF[x][y][0] != 0.0f && robot_position_MF[x][y][1] != 0.0f){
            //         float diatance_now = (state_aim_cmd.linear_x_ - robot_position_MF[x][y][0])*(state_aim_cmd.linear_x_ - robot_position_MF[x][y][0])
            //                            + (state_aim_cmd.linear_y_ - robot_position_MF[x][y][1])*(state_aim_cmd.linear_y_ - robot_position_MF[x][y][1]);
            //         if(diatance_now < min_distance){
            //         closest_x = x;
            //         closest_y = y;
            //         }
            //       }
            //     }
            //   }
            //   MF_x= closest_x;
            //   MF_y= closest_y;
            // }
            /* ======= 逻辑结束 ====== */

        }
    }

    if (MF_control_mode) {
        MF_control_Process();
    } else {
        Normal_control_Process();
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

       

        if (control_xbox_cmd.btnY == 1) {
            if (control_xbox_cmd_Last.btnY == 0) {
                state_aim_cmd.linear_x_ = 0.0f;
                state_aim_cmd.linear_y_ = 0.0f;
            }
        } else {
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
void MF_control_Process() {
    // 在网格边缘时，方向键移动网格坐标
    if (MF_x == 0 || MF_x == 5) {
        if (control_xbox_cmd.btnDirUp == 1) {
            if (control_xbox_cmd_Last.btnDirUp == 0) {
                if (MF_y < 5) {
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
                if (MF_x < 6) {
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

            break;
        case 2:
            //中间高台高度

            break;
        case 3:
            //最高高台任务

            break;
        default:
            break;
    }

    if(control_xbox_cmd.btnRB == 1){
        //设置伸出去为目标状态

        //车辆缓慢向前移动
        switch ((int16_t)robot_position_MF[MF_x][MF_y][2]) {
            case 0:
                //0度任务
                robot_v_aim_cmd.linear_x_ = 0.0f;
                robot_v_aim_cmd.linear_y_ = 0.05f;
                break;

            case 90:
                //90度任务
                robot_v_aim_cmd.linear_x_ = -0.05f;
                robot_v_aim_cmd.linear_y_ = 0.0f;
                break;

            case -90:
                //-90度任务
                robot_v_aim_cmd.linear_x_ = 0.05f;
                robot_v_aim_cmd.linear_y_ = 0.0f;
                break;

            case 180:
                //180度任务
                robot_v_aim_cmd.linear_x_ = 0.0f;
                robot_v_aim_cmd.linear_y_ = -0.05f;
                break;

            default:
                break;
        }

    }else{
        //设置0为目标状态

        //只有当按钮未按下时进行点跟踪
        state_aim_cmd.linear_x_ = -robot_position_MF[MF_x][MF_y][0];
        state_aim_cmd.linear_y_ = robot_position_MF[MF_x][MF_y][1];
        Aim_State_xy_Process();
    }

    //角度跟踪
    state_aim_cmd.omega_    = robot_position_MF[MF_x][MF_y][2];
    Aim_State_omega_Process();

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
