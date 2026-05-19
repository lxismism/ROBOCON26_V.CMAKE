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
static TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
static pub_chassis_cmd xbox_cmd{};

//发布红外控制命令
static TypedTopicPublisher<pub_ir_cmd> ir_cmd_pub("ir_cmd");
static pub_ir_cmd ir_cmd{};

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};
pub_Xbox_Data control_xbox_cmd_Last{};

/* 订阅Position定位信息 */
static TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};

/* 订阅IR_data信息 */
static TypedTopicSubscriber<pub_ir_data> control_ir_sub("ir_data", 8);
pub_ir_data control_ir_msg{};

static float control_position_x = 0.0f;
static float control_position_y = 0.0f;
static float control_position_yaw = 0.0f;
static float control_position_yaw_speed = 0.0f;
static uint8_t control_position_frame_id = 0;

static float xbox_angle_deg;
static float v_aim;
  
static bool headless_xy_mode = true;//无头模式标志位，默认开启
static bool headless_omega_mode = true;//无头模式标志位，默认开启

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
  // ===== 第1步：解析摇杆 → xbox_cmd（场地坐标系下的速度指令） =====
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

    // ===== 第2步：无头模式切换 + 坐标系旋转 =====
  {
    //切换xy模式（按下左摇杆LS切换）
    if(control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS)
    {
      if(control_xbox_cmd.btnLS == true){
        headless_xy_mode = !headless_xy_mode;
      }
    }
    control_xbox_cmd_Last.btnLS = control_xbox_cmd.btnLS;
    
    //切换omega模式（按下右摇杆RS切换）
    if(control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS)
    {
      if(control_xbox_cmd.btnRS == true){
        headless_omega_mode = !headless_omega_mode;
      }
    }
    control_xbox_cmd_Last.btnRS = control_xbox_cmd.btnRS;


    // ----- XY方向控制 -----
    if(headless_xy_mode)
    {
      // 无头模式：将场地坐标系速度旋转为车身坐标系速度
      xbox_angle_deg = atan2(xbox_cmd.linear_y_,xbox_cmd.linear_x_)/kDegToRad;
      v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

      robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position_yaw)*kDegToRad);
      robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position_yaw)*kDegToRad);
      
      //覆盖定位信息（有摇杆输入时，目标位置跟踪当前实际位置，避免松开摇杆后回弹）
      state_aim_cmd.linear_x_ = control_position_x;
      state_aim_cmd.linear_y_ = control_position_y;
    }
    else
    {
      // 摇杆归零，进入目标点位置环PID自动导航
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

    // ----- Omega旋转控制 -----
    if(headless_omega_mode)
    {
      // 无头模式：摇杆直通
      robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
      state_aim_cmd.omega_ = control_position_yaw ;
    }
    else
    {
      // 摇杆归零，进入目标角度环PID
      if(control_xbox_cmd.btnY == 1){
        state_aim_cmd.omega_ = 0.0f;
      }else if(control_xbox_cmd.btnX == 1){
        state_aim_cmd.omega_ = 90.0f;
      }else if(control_xbox_cmd.btnA == 1){
        state_aim_cmd.omega_ = 180.0f;
      }else if(control_xbox_cmd.btnB == 1){
        state_aim_cmd.omega_ = -90.0f;
      }
      //计算角度误差并进行PID控制
      float error_dir = state_aim_cmd.omega_ - control_position_yaw;
      if (fabs(error_dir) < 1.5f){
        error_dir = 0.0f;
      }else if(fabs(error_dir) > 180.0 ){
        if(error_dir > 0) error_dir = error_dir - 360.0 ;
        else error_dir = error_dir + 360.0 ;
      }

      // PID输出为度/s，转换为rad/s后发布
      robot_v_aim_cmd.omega_ = kDegToRad*PID_Calculate(&deg,control_position_yaw,control_position_yaw + error_dir);
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
    if (!control_ir_sub.IsValid()) {
        return;
    }
    if (!ir_cmd_pub.IsValid()) {
        return;
    }
}

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    PID_Init(&linear);
    PID_Init(&deg);

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

        if(control_ir_sub.TryGet(&control_ir_msg)) {
          //红外信号处理放这
          //测试
          if(control_ir_msg.data1 == 0x2B && control_ir_msg.data2 == 0xFC)
          {
            ir_cmd.tx_data[0] = 0x67; //示例：接收到特定红外信号后，发送0x01命令
            ir_cmd.tx_data[1] = 0x78; //示例：接收到特定红外信号后，发送0x02命令
            ir_cmd_pub.Publish(ir_cmd);
          }
        }

        vTaskDelayUntil(&currentTime, 5);
    }
}
