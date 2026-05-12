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
pub_chassis_cmd chassis_cmd{};

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox",8);
pub_Xbox_Data control_xbox_cmd{};

/* 订阅IMU姿态传感器数据 */
static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8);
pub_imu_data imu_data{};

//测试时发现陀螺仪有绝对正向，故在此以上电时的正向为相对正向
static bool headless_ref_ready = false;
static float ref_yaw_rad = 0.0f;



//陀螺仪的超时常量
static TickType_t last_imu_tick = 0;
static constexpr TickType_t kImuTimeoutTicks = pdMS_TO_TICKS(200);

// 处理xbox数据，处理为底盘控制指令
void Xbox_Data_Process()
{
  if (ABS(control_xbox_cmd.joyLHori - 32767) > 2000)
  {
    chassis_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLVert - 32767) > 2000)
  {
    chassis_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - 32767) > 2000)
  {
    chassis_cmd.omega_ = (int)(control_xbox_cmd.joyRHori - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.omega_ = 0.0f;
  }

    // 无头模式：将场地坐标系速度旋转为车身坐标系速度
  {
    float delta_yaw = imu_data.yaw_rad - ref_yaw_rad;
    float cos_yaw = cosf(delta_yaw);
    float sin_yaw = sinf(delta_yaw);


    float vx_field = chassis_cmd.linear_x_;
    float vy_field = chassis_cmd.linear_y_;

    chassis_cmd.linear_x_ =  vx_field * cos_yaw + vy_field * sin_yaw;
    chassis_cmd.linear_y_ = -vx_field * sin_yaw + vy_field * cos_yaw;
    // omega_ 不变 —— 自转始终绕车身中心，不需要变换
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

  if (!imu_data_sub.IsValid()) {
    return;
  }

}

void controlTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

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



    /* 从xbox数据订阅者中获取数据 */
    if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
      Xbox_Data_Process();
      chassis_data_pub.Publish(chassis_cmd);
    }
    vTaskDelayUntil(&currentTime, 5);
  }
}
