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
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>

osThreadId_t ControlTaskHandle;

// 发布底盘控制指令
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd xbox_cmd{};

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox",8);
pub_Xbox_Data control_xbox_cmd{};

/* 订阅IMU姿态传感器数据 */
static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8);
pub_imu_data imu_data{};

/* 订阅Position定位信息 */
static TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};

//测试时发现陀螺仪有绝对正向，故在此以上电时的正向为相对正向
static bool headless_ref_ready = false;
static float ref_yaw_rad = 0.0f;

static float control_position_x = 0.0f;
static float control_position_y = 0.0f;
static float control_position_yaw = 0.0f;
static float control_position_yaw_speed = 0.0f;
static uint8_t control_position_frame_id = 0;

pub_chassis_cmd Wheelv_Aim_cmd{
  .linear_x_ = 0.0f,
  .linear_y_ = 0.0f,
  .omega_ = 0.0f
};
pub_chassis_cmd State_Aim_cmd{
  .linear_x_ = 0.0f,
  .linear_y_ = 0.0f,
  .omega_ = 0.0f
};

//车体目标角度环pid
PID_t Deg{
  .Kp = 30.0f,
  .Ki = 0.01f,
  .Kd = 0.00002f,
  .MaxOut = 5000.0f,
  .DeadBand = 0.0f,
  .Improve = NONE
};





//陀螺仪的超时常量
static TickType_t last_imu_tick = 0;
static constexpr TickType_t kImuTimeoutTicks = pdMS_TO_TICKS(200);

int8_t sign(double value) {
  if(value > 0) return 1;
  else if(value < 0) return -1;
  else return 0;
}


// 处理xbox数据，处理为底盘控制指令
void Xbox_Data_Process()
{
  if (ABS(control_xbox_cmd.joyLHori - 32767) > 3500)
  {
    xbox_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - 32767 - sign(control_xbox_cmd.joyLHori - 32767)*3500) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    xbox_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLVert - 32767) > 3500)
  {
    xbox_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - 32767 - sign(control_xbox_cmd.joyLVert - 32767)*3500) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    xbox_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - 32767) > 2000)
  {
    xbox_cmd.omega_ = (int)(control_xbox_cmd.joyRHori - 32767 - sign(control_xbox_cmd.joyRHori - 32767)*2000);
  }
  else
  {
    xbox_cmd.omega_ = 0.0f;
  }

    // 无头模式：将场地坐标系速度旋转为车身坐标系速度
  {
    float delta_yaw = imu_data.yaw_rad - ref_yaw_rad;
    if(delta_yaw > 180.0f) {
      delta_yaw -= 360.0f;
    } else if(delta_yaw < -180.0f) {
      delta_yaw += 360.0f;
    }
    float xbox_angle_rad = atan2(xbox_cmd.linear_y_,xbox_cmd.linear_x_)/(M_PI/180.0f);
    float V_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

    Wheelv_Aim_cmd.linear_x_ = V_aim * cos((xbox_angle_rad - delta_yaw)*(M_PI/180.0f));
    Wheelv_Aim_cmd.linear_y_ = V_aim * sin((xbox_angle_rad - delta_yaw)*(M_PI/180.0f));

    //计算目标角度
    if(control_xbox_cmd.btnY == 1){
        State_Aim_cmd.omega_ = 0;
      }else if(fabs(xbox_cmd.omega_) > 14382.0f){
          State_Aim_cmd.omega_ = State_Aim_cmd.omega_ + (xbox_cmd.omega_ - sign(xbox_cmd.omega_)*16382.0f)/32767.0f; 
          while(State_Aim_cmd.omega_ > 180){
            State_Aim_cmd.omega_ = State_Aim_cmd.omega_ - 360.0;
          }
          while(State_Aim_cmd.omega_ < -180){
            State_Aim_cmd.omega_ = State_Aim_cmd.omega_ + 360.0; 
          }
    }

    //计算角度误差并进行PID控制
    float_t error_Dir = State_Aim_cmd.omega_ - control_position_yaw;
     if (fabs(error_Dir) < 1.5f){
        error_Dir = 0.0f;
      }else if(fabs(error_Dir) > 180.0 ){
      if(error_Dir > 0) error_Dir = error_Dir - 360.0 ;
      else error_Dir = error_Dir + 360.0 ;
    }
    Wheelv_Aim_cmd.omega_ = ((double)3.14159f/180.0)*PID_Calculate(&Deg,control_position_yaw,control_position_yaw + error_Dir);

  }


}

void controlInit() { 
  if (!chassis_data_pub.IsValid()) {
    // 发布者初始化失败
    return;
  }
  if (!control_xbox_sub.IsValid()) {
        // 订阅失败
    return;
  }
  if (!control_position_sub.IsValid()) {
    // 订阅失败
    return;
  }
  if (!imu_data_sub.IsValid()) {
    return;
  }

}

void controlTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  PID_Init(&Deg);

  controlInit();
  for (;;) {
    
    /* 不断获取最新的IMU数据，保持imu_data为最新值 */
    if (imu_data_sub.TryGet(&imu_data)) {
      last_imu_tick = xTaskGetTickCount();
      if (!headless_ref_ready) {
        ref_yaw_rad = imu_data.yaw_rad;
        headless_ref_ready = true;
      }
    } else if ((xTaskGetTickCount() - last_imu_tick) > kImuTimeoutTicks) {
      imu_data.yaw_rad = 0.0f;
    }

    /* 从Position订阅者中获取数据 */
    if (control_position_sub.TryGet(&control_position_msg)) {
      control_position_frame_id = control_position_msg.frame_id;
      control_position_x = control_position_msg.x;
      control_position_y = control_position_msg.y;
      control_position_yaw = - control_position_msg.yaw;//取负号是因为Position模块中定义的yaw角是顺时针为正，而我们控制系统中定义的yaw角是逆时针为正
      control_position_yaw_speed = control_position_msg.yaw_speed;
    }

    /* 从xbox数据订阅者中获取数据 */
    if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
      Xbox_Data_Process();
      chassis_data_pub.Publish(Wheelv_Aim_cmd);
    }
    vTaskDelayUntil(&currentTime, 5);
  }
}
