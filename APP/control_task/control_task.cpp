/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief 控制任务 —— Xbox按键 → 底盘 + 上身控制指令决策层
 * @version 0.3
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention btnSelect 切换模式：0=队友模式(现有逻辑), 1=调试模式(上身按键映射)
 * @note 持续型按键每帧累加步进量；切换型按键仅上升沿触发
 * @versioninfo v0.3: 模块化拆分 —— 流程函数移至 control_process，动作函数移至 control_action
 * @versioninfo v0.2: 新增上身控制指令 + btnSelect模式切换
 */

#include "main.h"
#include "control_task.h"
#include "control_process.hpp"       // ← 新增：流程层
#include "control_action.hpp"        // ← 新增：动作层（常量 + 工具函数）
#include "pid_controller.h"
#include "chassis_task.h"
#include "rm_pocket.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>
#include <cstdint>

osThreadId_t ControlTaskHandle;

// ===== 发布者 =====
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd xbox_cmd{};

static TypedTopicPublisher<pub_upbody_cmd> upbody_cmd_pub("upbody_cmd");      
static pub_upbody_cmd upbody_cmd_msg{};                                   

TypedTopicPublisher<pub_ir_cmd> ir_cmd_pub("ir_cmd");                
pub_ir_cmd ir_cmd{};                                               

// static TypedTopicPublisher<QR_code_cmd_t> qr_code_cmd_pub("qr_code_cmd");   
// static QR_code_cmd_t qr_code_cmd{};


// ===== 订阅者 =====
TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};
pub_Xbox_Data control_xbox_cmd_Last{};

TypedTopicSubscriber<pub_RC_Data> control_rc_sub("rc", 8);
pub_RC_Data control_rc_cmd{};

TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};
pub_Position_Data control_position{};

// 定位修正参数
float position_center_distance = 0.25f;
float position_correction_x = 0.0f;
float position_correction_y = -position_center_distance;


// TypedTopicSubscriber<pub_ir_data> control_ir_sub("ir_data", 8);
// pub_ir_data control_ir_msg{};

// TypedTopicSubscriber<QR_code_data_t> qr_code_data_sub("qr_code_data", 8);
// QR_code_data_t control_qr_code_data{};

// ===== 控制状态变量 =====
RobotMode_t robot_mode = MC; // 当前机器人模式，默认为MF

float xbox_angle_deg;
float v_aim;

bool headless_xy_mode = true;
bool headless_omega_mode = true;

bool Normal_control_mode = true;

int8_t MC_y = 0;
float MC_close_position_x = 0.0f;
bool MC_headless_xy_mode = false;
bool MC_headless_omega_mode = false;
const float MC_position_correction_y = 0.03f;


uint8_t MF_x = 0;
uint8_t MF_y = 0;
float MF_close_position_x = 0.0f;
float MF_close_position_y = 0.0f;
float MF_omega_correction = 0.6f;
bool MF_plan_record_Flag = false;
bool MF_plan_run_Flag = false;
bool MF_action_Flag = false;
bool MF_pick_Flag = false;
bool MF_xy_complete_Flag = false;
bool MF_omega_complete_Flag = false;
int8_t MF_omega_control_Flag = 0;
uint8_t MF_plan_record_i = 0;
uint8_t MF_plan_run_i = 0;
MF_plan_t MF_plan_zero = {0,0,0,0};

int8_t Arena_x = 0;
float Arena_close_position_y = 0.0f;
float Arena_close_position_y_Max = 0.64f;

const FieldSide_t field_side = Left;
const float robot_center_to_gimbal_x = 0.4f;

float Acc_path_SpeedUp = 2.9f; //加速度，单位m/s^2
float Acc_path_SpeedDown = 1.8f; //加速度，单位m/s^2
float path_plan_Max_Max = 1.9f; //规划最大速度

float Acc_omega_SpeedUp = M_PI*1.2f; //加速度，单位m/s^2
float Acc_omega_SpeedDown = M_PI*0.5f; //加速度，单位m/s^2
float v_omega_plan_Max_Max = M_PI*0.45f;

float Acc_xy_dt = 0.0f; //加速计时器，单位s
uint32_t Acc_xy_DWT_CNT = 0;
float Acc_omega_dt = 0.0f; //加速计时器，单位s
uint32_t Acc_omega_DWT_CNT = 0;

float predict_yaw = 0.0f;
float yaw_delay_time = 0.0085f;

pub_chassis_cmd robot_v_aim_cmd{};
pub_chassis_cmd state_now_cmd{};
pub_chassis_cmd state_start_cmd{};
pub_chassis_cmd state_target_cmd{};
pub_chassis_cmd state_target_last_cmd{};

// 车体目标角度环 PID
PID_t lateral{.Kp = 5.1f,.Ki = 0.00f,.Kd = 0.3f,.MaxOut = 0.95*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
PID_t path{.Kp = 4.68f,.Ki = 0.03f,.Kd = 0.75f,.MaxOut = 0.95*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
PID_t omega{.Kp = 2.10f,.Ki = 0.22f,.Kd = 0.08f,.MaxOut = MAX_VELOCITY_ANGULAR*0.75*180.0/M_PI,.IntegralLimit = 50000.0f,.DeadBand = 0.1f,.Improve = Integral_Limit};


void controlInit() {
    if (!chassis_data_pub.IsValid()) {
        return;
    }
    if (!upbody_cmd_pub.IsValid()) {
        return;
    }
    if (!control_xbox_sub.IsValid()) {
        return;
    }

    if (!control_rc_sub.IsValid()) {
        return;
    }

    if (!control_position_sub.IsValid()) {
        return;
    }
    // if (!control_ir_sub.IsValid()) {
    //     return;
    // }
    if (!ir_cmd_pub.IsValid()) {
        return;
    }
    // if(!qr_code_cmd_pub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_data_sub.IsValid()) {
    //     return;
    // }
}

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    PID_Init(&lateral);
    PID_Init(&path);
    PID_Init(&omega);

    controlInit();
    uint32_t last_time = HAL_GetTick();
    for (;;) {
        //test begin
        if(control_rc_sub.TryGet(&control_rc_cmd)) {
            if(control_rc_cmd.swA == RC_2_POS_SW_State_t::UP) {
                // 处理 swA 按钮按下
            }
        }
        //test end

        /* 从Position订阅者中获取数据 */
        if (control_position_sub.TryGet(&control_position_msg)) {
            control_position.frame_id = control_position_msg.frame_id;
            control_position.yaw = -control_position_msg.yaw;
            control_position.yaw_speed = control_position_msg.yaw_speed;
            control_position.x = -control_position_msg.x + position_center_distance*sin(control_position.yaw*kDegToRad) - position_correction_x ;
            control_position.y =  control_position_msg.y - position_center_distance*cos(control_position.yaw*kDegToRad) - position_correction_y ;

            state_now_cmd.linear_x_ = control_position.x;
            state_now_cmd.linear_y_ = control_position.y;
            state_now_cmd.omega_ = control_position.yaw;
        }

        /* 从xbox数据订阅者中获取数据 */
        if (control_xbox_sub.TryGet(&control_xbox_cmd)) {

            if(control_xbox_cmd.btnSelect == 1 && control_xbox_cmd_Last.btnSelect == 0) {
                robot_mode = MC;
                Reset_position();
            }else if(control_xbox_cmd.btnShare == 1 && control_xbox_cmd_Last.btnShare == 0) {
                robot_mode = MF;
                Reset_position();
            }else if(control_xbox_cmd.btnStart == 1 && control_xbox_cmd_Last.btnStart == 0){
                robot_mode = Arena;
                Reset_position();
            }
            Chassis_Xbox_Data_Process(upbody_cmd_pub, upbody_cmd_msg);
    

            // 保留本次xbox数据
            control_xbox_cmd_Last = control_xbox_cmd;
            chassis_data_pub.Publish(robot_v_aim_cmd);
        }

        // if(HAL_GetTick() - last_time >= 2000) {
        //     uint8_t test_data = 0x2B; // 示例数据
        //     ir_cmd.tx_data = test_data;
        //     ir_cmd_pub.Publish(ir_cmd);
        //     last_time = HAL_GetTick();
        // }


        //if(qr_code_data_sub.TryGet(&control_qr_code_data)) {
          //二维码输入数据处理放这
          //测试
          //示例：接收到0x01二维码数据后，发布显示0x01二维码命令
          // if(control_qr_code_data.QR_type == 0x01)
          // {
          //   qr_code_cmd.QR_type = 0x01;            //<-
          //   qr_code_cmd_pub.Publish(qr_code_cmd);  //<-发布显示0x01二维码命令核心代码
          // }
        //}


        vTaskDelayUntil(&currentTime, 5);
    }
}
