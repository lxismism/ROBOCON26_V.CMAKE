/**
 * @file control_task.cpp
 * @author 大帅将军
 * @brief 控制任务，遥控/上位机 接口都接入到此，解析后向其他任务发布可能的控制指令
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "control_task.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>

osThreadId_t ControlTaskHandle;

// 发布底盘控制指令
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
static pub_chassis_cmd xbox_cmd{};

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
static pub_Xbox_Data control_xbox_cmd{};
static pub_Xbox_Data control_xbox_cmd_Last{};

/* 订阅Position定位信息 */
static TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
static pub_Position_Data control_position_msg{};

static float control_position_x = 0.0f;
static float control_position_y = 0.0f;
static float control_position_yaw = 0.0f;
static float control_position_yaw_speed = 0.0f;
static uint8_t control_position_frame_id = 0;

static float xbox_angle_deg;
static float v_aim;
  
bool headless_xy_mode = true;//无头模式标志位，默认开启
bool headless_omega_mode = true;//无头模式标志位，默认开启

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

//车体目标角度环pid
static PID_t linear{.Kp = 1.68f,.Ki = 0.5f,.Kd = 0.0022f,.MaxOut = MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = Derivative_On_Measurement};
static PID_t deg{.Kp = 3.0f,.Ki = 0.8f,.Kd = 0.024f,.MaxOut = MAX_VELOCITY_ANGULAR*180.0/M_PI,.DeadBand = 0.3f,.Improve = Derivative_On_Measurement};

// 摇杆常量
static constexpr uint16_t kJoyCenter = 32767;
static constexpr uint16_t kJoyDeadZoneLeft = 3500;
static constexpr uint16_t kJoyDeadZoneRight = 2000;
static constexpr float kDegToRad = M_PI / 180.0f;

static int8_t sign(double value) {
    if (value > 0) return 1;
    else if (value < 0) return -1;
    else return 0;
}


// 处理xbox数据，处理为底盘控制指令
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
    xbox_cmd.omega_ = -(int)(control_xbox_cmd.joyRHori - kJoyCenter - sign(control_xbox_cmd.joyRHori - kJoyCenter)*kJoyDeadZoneRight) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_ANGULAR;
  }
  else
  {
    xbox_cmd.omega_ = 0.0f;
  }

    // 无头模式：将场地坐标系速度旋转为车身坐标系速度
  {
    //切换xy模式
    if(control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS)
    {
      if(control_xbox_cmd.btnLS == true){
        headless_xy_mode = !headless_xy_mode;
      }
    }
    control_xbox_cmd_Last.btnLS = control_xbox_cmd.btnLS;
    
    //切换omega模式
    if(control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS)
    {
      if(control_xbox_cmd.btnRS == true){
        headless_omega_mode = !headless_omega_mode;
      }
    }
    control_xbox_cmd_Last.btnRS = control_xbox_cmd.btnRS;


    // if(xbox_cmd.linear_y_ != 0.0f || xbox_cmd.linear_x_ != 0.0f )
    if(headless_xy_mode)
    {
        xbox_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - kJoyCenter
            - sign(control_xbox_cmd.joyLHori - kJoyCenter) * kJoyDeadZoneLeft)
            / (float)(kJoyCenter - kJoyDeadZoneLeft) * MAX_VELOCITY_LINEAR;
    }
    else
    {
        xbox_cmd.linear_x_ = 0.0f;
    }

    // if(xbox_cmd.omega_ != 0.0f)
    if(headless_omega_mode)
    {
      robot_v_Aim_cmd.omega_ = xbox_cmd.omega_;
      state_Aim_cmd.omega_ = control_position_yaw ;
    }
    else
    {
        xbox_cmd.linear_y_ = 0.0f;
    }

    // 旋转
    if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > kJoyDeadZoneRight)
    {
        xbox_cmd.omega_ = -(int)(control_xbox_cmd.joyRHori - kJoyCenter
            - sign(control_xbox_cmd.joyRHori - kJoyCenter) * kJoyDeadZoneRight)
            / (float)(kJoyCenter - kJoyDeadZoneRight) * MAX_VELOCITY_ANGULAR;
    }
    else
    {
        xbox_cmd.omega_ = 0.0f;
    }

    // 场地坐标系 → 车身坐标系 转换
    {
        if (xbox_cmd.omega_ != 0.0f || xbox_cmd.linear_x_ != 0.0f)
        {
            // 摇杆有输入：直接遥控，同时更新目标位置为当前位置
            xbox_angle_deg = atan2(xbox_cmd.linear_y_, xbox_cmd.linear_x_) / kDegToRad;
            v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_
                       + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

            robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position_yaw) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position_yaw) * kDegToRad);

            state_aim_cmd.linear_x_ = control_position_x;
            state_aim_cmd.linear_y_ = control_position_y;
        }
        else
        {
            // 摇杆归零：按键选择目标点，位置环PID自动导航
            if (control_xbox_cmd.btnY == 1) {
                state_aim_cmd.linear_x_ = 0.0f;
                state_aim_cmd.linear_y_ = 0.0f;
            } else if (control_xbox_cmd.btnX == 1) {
                state_aim_cmd.linear_x_ = 0.0f;
                state_aim_cmd.linear_y_ = 0.5f;
            } else if (control_xbox_cmd.btnA == 1) {
                state_aim_cmd.linear_x_ = 0.0f;
                state_aim_cmd.linear_y_ = 1.0f;
            } else if (control_xbox_cmd.btnB == 1) {
                state_aim_cmd.linear_x_ = 0.0f;
                state_aim_cmd.linear_y_ = 1.5f;
            }

            error_x = state_aim_cmd.linear_x_ - control_position_x;
            error_y = state_aim_cmd.linear_y_ - control_position_y;
            state_xy_error = sqrt(error_x * error_x + error_y * error_y);
            state_xy_angle_deg = atan2(error_y, error_x) / kDegToRad;
            xy_pid_output = PID_Calculate(&linear_pid, 0.0f, state_xy_error);

            robot_v_aim_cmd.linear_x_ = xy_pid_output * cos((state_xy_angle_deg - control_position_yaw) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = xy_pid_output * sin((state_xy_angle_deg - control_position_yaw) * kDegToRad);
        }

        if (xbox_cmd.omega_ != 0.0f)
        {
            robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
            state_aim_cmd.omega_ = control_position_yaw;
        }
        else
        {
            // 摇杆归零：按键选择目标角度
            if (control_xbox_cmd.btnY == 1) {
                state_aim_cmd.omega_ = 0.0f;
            } else if (control_xbox_cmd.btnX == 1) {
                state_aim_cmd.omega_ = 90.0f;
            } else if (control_xbox_cmd.btnA == 1) {
                state_aim_cmd.omega_ = 180.0f;
            } else if (control_xbox_cmd.btnB == 1) {
                state_aim_cmd.omega_ = -90.0f;
            }

            float error_dir = state_aim_cmd.omega_ - control_position_yaw;

            // 角度死区
            if (fabs(error_dir) < 1.5f) {
                error_dir = 0.0f;
            }
            // 角度归一化到 [-180°, 180°]
            ANGLE_LIMIT(error_dir, -180.0f, 180.0f);

            // PID输出为度/s，转换为rad/s后发布
            robot_v_aim_cmd.omega_ = kDegToRad * PID_Calculate(&angle_pid, control_position_yaw, control_position_yaw + error_dir);
        }
    }
}


void controlInit() {
    if (!chassis_data_pub.IsValid()) {
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

    PID_Init(&linear_pid);
    PID_Init(&angle_pid);

    controlInit();
    for (;;) {
        /* 从Position订阅者中获取数据 */
        if (control_position_sub.TryGet(&control_position_msg)) {
            control_position_frame_id = control_position_msg.frame_id;
            // Position模块x轴正向与控制系统x轴正向相反，取负号
            control_position_x = -control_position_msg.x;
            control_position_y = control_position_msg.y;
            // Position模块yaw顺时针为正，控制系统yaw逆时针为正，取负号
            control_position_yaw = -control_position_msg.yaw;
            control_position_yaw_speed = control_position_msg.yaw_speed;
        }

        /* 从xbox数据订阅者中获取数据 */
        if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
            Xbox_Data_Process();
            chassis_data_pub.Publish(robot_v_aim_cmd);
        }

        vTaskDelayUntil(&currentTime, 5);
    }
}
