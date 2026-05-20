/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief 控制任务 —— Xbox按键 → 底盘 + 上身控制指令决策层
 * @version 0.2
 * @date 2026-05-20
 *
 * @copyright Copyright (c) 2026
 *
 * @attention btnSelect 切换模式：0=队友模式(现有逻辑), 1=用户模式(上身按键映射)
 * @note 持续型按键每帧累加步进量；切换型按键仅上升沿触发
 * @versioninfo v0.2: 新增上身控制指令 + btnSelect模式切换
 */

#include "control_task.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>

osThreadId_t ControlTaskHandle;

// ---------- 发布者 ----------
static TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
static pub_chassis_cmd xbox_cmd{};

static TypedTopicPublisher<pub_upbody_cmd> upbody_cmd_pub("upbody_cmd");
static pub_upbody_cmd upbody_cmd_msg{};

// ---------- 订阅者 ----------
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};
pub_Xbox_Data control_xbox_cmd_Last{};

static TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};

static float control_position_x = 0.0f;
static float control_position_y = 0.0f;
static float control_position_yaw = 0.0f;
static float control_position_yaw_speed = 0.0f;
static uint8_t control_position_frame_id = 0;

static float xbox_angle_deg;
static float v_aim;

static bool headless_xy_mode = true;
static bool headless_omega_mode = true;

static float error_x;
static float error_y;
static float state_xy_error;
static float state_xy_angle_deg;
static float xy_pid_output;

static pub_chassis_cmd robot_v_aim_cmd{
    .linear_x_ = 0.0f,
    .linear_y_ = 0.0f,
    .omega_ = 0.0f
};
static pub_chassis_cmd state_aim_cmd{
    .linear_x_ = 0.0f,
    .linear_y_ = 0.0f,
    .omega_ = 0.0f
};

// PID
static PID_t linear{.Kp = 1.68f,.Ki = 0.5f,.Kd = 0.0022f,.MaxOut = MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = Derivative_On_Measurement};
static PID_t deg{.Kp = 3.0f,.Ki = 0.8f,.Kd = 0.024f,.MaxOut = MAX_VELOCITY_ANGULAR*180.0/M_PI,.DeadBand = 0.3f,.Improve = Derivative_On_Measurement};

// 摇杆常量
static constexpr uint16_t kJoyCenter = 32767;
static constexpr uint16_t kJoyDeadZoneLeft = 3500;
static constexpr uint16_t kJoyDeadZoneRight = 2000;
static constexpr float kDegToRad = M_PI / 180.0f;

// 上身控制常量
static constexpr float kUpbodyStep = 1.0f;           // 持续按键每帧步进 (°/帧)
static constexpr uint16_t kTriggerThreshold = 512;   // 扳机触发阈值

static int8_t sign(double value) {
    if (value > 0) return 1;
    else if (value < 0) return -1;
    else return 0;
}

// 处理xbox数据 → 底盘控制指令（队友模式/无头模式用）
void Xbox_Data_Process()
{
  if (ABS(control_xbox_cmd.joyLHori - kJoyCenter) > kJoyDeadZoneLeft)
  {
    xbox_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - kJoyCenter - sign(control_xbox_cmd.joyLHori - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
  }
  else
  {
    xbox_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLVert - kJoyCenter) > kJoyDeadZoneLeft)
  {
    xbox_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - kJoyCenter - sign(control_xbox_cmd.joyLVert - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
  }
  else
  {
    xbox_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > kJoyDeadZoneRight)
  {
    xbox_cmd.omega_ = -(int)(control_xbox_cmd.joyRHori - kJoyCenter - sign(control_xbox_cmd.joyRHori - kJoyCenter)*kJoyDeadZoneRight) / ((float)(kJoyCenter - kJoyDeadZoneRight)) * MAX_VELOCITY_ANGULAR;
  }
  else
  {
    xbox_cmd.omega_ = 0.0f;
  }

  {
    if(control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS)
    {
      if(control_xbox_cmd.btnLS == true){
        headless_xy_mode = !headless_xy_mode;
      }
    }
    control_xbox_cmd_Last.btnLS = control_xbox_cmd.btnLS;

    if(control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS)
    {
      if(control_xbox_cmd.btnRS == true){
        headless_omega_mode = !headless_omega_mode;
      }
    }
    control_xbox_cmd_Last.btnRS = control_xbox_cmd.btnRS;

    if(headless_xy_mode)
    {
      xbox_angle_deg = atan2(xbox_cmd.linear_y_,xbox_cmd.linear_x_)/kDegToRad;
      v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

      robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position_yaw)*kDegToRad);
      robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position_yaw)*kDegToRad);

      state_aim_cmd.linear_x_ = control_position_x;
      state_aim_cmd.linear_y_ = control_position_y;
    }
    else
    {
      if(control_xbox_cmd.btnY == 1){
        state_aim_cmd.linear_x_ = 0.0f;
        state_aim_cmd.linear_y_ = 0.0f;
      }else if(control_xbox_cmd.btnX == 1){
        state_aim_cmd.linear_x_ = 0.0f;
        state_aim_cmd.linear_y_ = 0.5f;
      }else if(control_xbox_cmd.btnA == 1){
        state_aim_cmd.linear_x_ = 0.0f;
        state_aim_cmd.linear_y_ = 1.0f;
      }else if(control_xbox_cmd.btnB == 1){
        state_aim_cmd.linear_x_ = 0.0f;
        state_aim_cmd.linear_y_ = 1.5f;
      }

      error_x            = state_aim_cmd.linear_x_ - control_position_x;
      error_y            = state_aim_cmd.linear_y_ - control_position_y;
      state_xy_error     = sqrt(error_x*error_x + error_y*error_y);
      state_xy_angle_deg = atan2(error_y,error_x)/kDegToRad;
      xy_pid_output      = PID_Calculate(&linear,0.0f,state_xy_error);
      robot_v_aim_cmd.linear_x_  = xy_pid_output * cos((state_xy_angle_deg - control_position_yaw)*kDegToRad);
      robot_v_aim_cmd.linear_y_  = xy_pid_output * sin((state_xy_angle_deg - control_position_yaw)*kDegToRad);
    }

    if(headless_omega_mode)
    {
      robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
      state_aim_cmd.omega_ = control_position_yaw ;
    }
    else
    {
      if(control_xbox_cmd.btnY == 1){
        state_aim_cmd.omega_ = 0.0f;
      }else if(control_xbox_cmd.btnX == 1){
        state_aim_cmd.omega_ = 90.0f;
      }else if(control_xbox_cmd.btnA == 1){
        state_aim_cmd.omega_ = 180.0f;
      }else if(control_xbox_cmd.btnB == 1){
        state_aim_cmd.omega_ = -90.0f;
      }
      float error_dir = state_aim_cmd.omega_ - control_position_yaw;
      if (fabs(error_dir) < 1.5f){
        error_dir = 0.0f;
      }else if(fabs(error_dir) > 180.0 ){
        if(error_dir > 0) error_dir = error_dir - 360.0 ;
        else error_dir = error_dir + 360.0 ;
      }

      robot_v_aim_cmd.omega_ = kDegToRad*PID_Calculate(&deg,control_position_yaw,control_position_yaw + error_dir);
    }

  }

}

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
    if (!control_position_sub.IsValid()) {
        return;
    }
}

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    PID_Init(&linear);
    PID_Init(&deg);

    // 模式状态
    static bool control_mode = false;    // false=队友模式, true=用户模式
    static bool last_select = false;     // btnSelect上一帧状态

    // 切换型按键上一帧状态（边缘检测用）
    static bool last_btnX = false;
    static bool last_btnB = false;
    static bool last_btnLB = false;
    static bool last_btnRB = false;

    controlInit();
    for (;;) {
        /* 从Position订阅者中获取数据 */
        if (control_position_sub.TryGet(&control_position_msg)) {
            control_position_frame_id = control_position_msg.frame_id;
            control_position_x = -control_position_msg.x;
            control_position_y = control_position_msg.y;
            control_position_yaw = -control_position_msg.yaw;
            control_position_yaw_speed = control_position_msg.yaw_speed;
        }

        /* 从xbox数据订阅者中获取数据 */
        if (control_xbox_sub.TryGet(&control_xbox_cmd)) {

            // ===== btnSelect 模式切换（上升沿触发） =====
            if (control_xbox_cmd.btnSelect && !last_select) {
                control_mode = !control_mode;
            }
            last_select = control_xbox_cmd.btnSelect;

            // ==========================================
            //  用户模式 (mode = 1)
            // ==========================================
            if (control_mode) {

                // ---- 底盘：摇杆直驱 ----
                if (ABS(control_xbox_cmd.joyLHori - kJoyCenter) > kJoyDeadZoneLeft)
                {
                    robot_v_aim_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - kJoyCenter - sign(control_xbox_cmd.joyLHori - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
                }
                else { robot_v_aim_cmd.linear_x_ = 0.0f; }

                if (ABS(control_xbox_cmd.joyLVert - kJoyCenter) > kJoyDeadZoneLeft)
                {
                    robot_v_aim_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - kJoyCenter - sign(control_xbox_cmd.joyLVert - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
                }
                else { robot_v_aim_cmd.linear_y_ = 0.0f; }

                if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > kJoyDeadZoneRight)
                {
                    robot_v_aim_cmd.omega_ = -(int)(control_xbox_cmd.joyRHori - kJoyCenter - sign(control_xbox_cmd.joyRHori - kJoyCenter)*kJoyDeadZoneRight) / ((float)(kJoyCenter - kJoyDeadZoneRight)) * MAX_VELOCITY_ANGULAR;
                }
                else { robot_v_aim_cmd.omega_ = 0.0f; }
                // 无头旋转：场地坐标系 → 车身坐标系
                float joy_angle_deg = atan2(robot_v_aim_cmd.linear_y_, robot_v_aim_cmd.linear_x_) / kDegToRad;
                float joy_mag = sqrt(robot_v_aim_cmd.linear_x_ * robot_v_aim_cmd.linear_x_ + robot_v_aim_cmd.linear_y_ * robot_v_aim_cmd.linear_y_);
                robot_v_aim_cmd.linear_x_ = joy_mag * cos((joy_angle_deg - control_position_yaw) * kDegToRad);
                robot_v_aim_cmd.linear_y_ = joy_mag * sin((joy_angle_deg - control_position_yaw) * kDegToRad);

                chassis_data_pub.Publish(robot_v_aim_cmd);

                // ---- 上身控制指令 ----
                upbody_cmd_msg = {};
                upbody_cmd_msg.active = true;


                // ▼ 持续型：trigLT电梯上升 / trigRT电梯下降
                if (control_xbox_cmd.trigLT > kTriggerThreshold)
                    upbody_cmd_msg.lift_delta = kUpbodyStep;
                if (control_xbox_cmd.trigRT > kTriggerThreshold)
                    upbody_cmd_msg.lift_delta = -kUpbodyStep;

                // ▼ 持续型：btnLS武器手上升 / btnRS武器手下降
                if (control_xbox_cmd.btnLS)
                    upbody_cmd_msg.weapon_lift_delta = kUpbodyStep;
                if (control_xbox_cmd.btnRS)
                    upbody_cmd_msg.weapon_lift_delta = -kUpbodyStep;

                // ▼ 持续型：btnDirUp吸取手上升 / btnDirDown吸取手下降
                if (control_xbox_cmd.btnDirUp)
                    upbody_cmd_msg.pick_lift_delta = kUpbodyStep;
                if (control_xbox_cmd.btnDirDown)
                    upbody_cmd_msg.pick_lift_delta = -kUpbodyStep;

                // ▼ 持续型：btnDirLeft云台逆时针 / btnDirRight云台顺时针
                if (control_xbox_cmd.btnDirLeft)
                    upbody_cmd_msg.pick_yaw_delta = kUpbodyStep;
                if (control_xbox_cmd.btnDirRight)
                    upbody_cmd_msg.pick_yaw_delta = -kUpbodyStep;

                // ▼ 持续型：btnY武器手伸 / btnA武器手缩
                if (control_xbox_cmd.btnY)
                    upbody_cmd_msg.weapon_extend_delta = kUpbodyStep;
                if (control_xbox_cmd.btnA)
                    upbody_cmd_msg.weapon_extend_delta = -kUpbodyStep;

                // ▼ 切换型：btnX吸盘真空泵（上升沿触发）
                if (control_xbox_cmd.btnX && !last_btnX)
                    upbody_cmd_msg.pump_toggle = true;
                last_btnX = control_xbox_cmd.btnX;

                // ▼ 切换型：btnB吸盘电磁阀（上升沿触发）
                if (control_xbox_cmd.btnB && !last_btnB)
                    upbody_cmd_msg.valve_toggle = true;
                last_btnB = control_xbox_cmd.btnB;

                // ▼ 切换型：btnLB夹爪开合（上升沿触发）
                if (control_xbox_cmd.btnLB && !last_btnLB)
                    upbody_cmd_msg.claw_toggle = true;
                last_btnLB = control_xbox_cmd.btnLB;

                // ▼ 切换型：btnRB腕部舵机翻转（上升沿触发）
                if (control_xbox_cmd.btnRB && !last_btnRB)
                    upbody_cmd_msg.wrist_toggle = true;
                last_btnRB = control_xbox_cmd.btnRB;

                upbody_cmd_pub.Publish(upbody_cmd_msg);

            // ==========================================
            //  队友模式 (mode = 0)：现有逻辑不变
            // ==========================================
            } else {
                Xbox_Data_Process();
                chassis_data_pub.Publish(robot_v_aim_cmd);
            }
        }

        vTaskDelayUntil(&currentTime, 5);
    }
}
