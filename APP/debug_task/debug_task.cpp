/**
 * @file debug_task.cpp
 * @author 大帅将军
 * @brief 调试任务，测试用，后续可能会删除
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "debug_task.h"
#include "Motor.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"

#include "task.h"

#include "com_config.h"
#include "pid_controller.h"
#include "pm20s.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

osThreadId_t Debug_TaskHandle;

// 任务剩余栈空间监控变量（单位：byte）
extern osThreadId_t CAN1_Send_TaskHandle;
extern osThreadId_t CAN2_Send_TaskHandle;
extern osThreadId_t CAN3_Send_TaskHandle;
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t PosCtrlTaskHandle;
extern osThreadId_t uart2ProcessTaskHandle;
extern osThreadId_t uart3ProcessTaskHandle;
extern osThreadId_t uart4ProcessTaskHandle;
extern osThreadId_t usbcdcProcessTaskHandle;

uint32_t CAN1_Send_m = 0;
uint32_t CAN2_Send_m = 0;
uint32_t CAN3_Send_m = 0;
uint32_t Debug_m = 0;
uint32_t Chassis_m = 0;
uint32_t Control_m = 0;
uint32_t PosCtrl_m = 0;
uint32_t Uart2Process_m = 0;
uint32_t Uart3Process_m = 0;
uint32_t Uart4Process_m = 0;
uint32_t UsbcdcProcess_m = 0;




extern DM4310Motor arm4310_motor;
float arm4310_pos = 0.0f;
float arm4310_speed = 0.0f;
bool change = 0;
extern C620Motor arm3508_motor;
// PID_t arm3508_motor_speedpid = {
//   .Kp = 150.0, .Ki = 10.0, .Kd = 0.0, .MaxOut = 12000, .DeadBand = 0.1, .Improve = NONE};
PID_t arm3508_motor_pospid = {
  .Kp = 400.0, .Ki = 80.0, .Kd = 20.0, .MaxOut = 12000, .DeadBand = 0.1, .Improve = NONE};
  float arm3508_pid_out = 0.0f;
  float arm3508_speed = 0.0f;
  float target_speed = 0.0f;
#ifdef DEBUG_ARM
extern C610Motor arm2006_motor;
extern C620Motor arm3508_motor;

extern TIM_HandleTypeDef htim2;

PM20sServo debug_servo(htim2, TIM_CHANNEL_3);

static TypedTopicSubscriber<pub_Xbox_Data> debug_xbox_sub("xbox", 8);
pub_Xbox_Data debug_xbox_cmd{};

namespace {
constexpr float kServoMinAngleDeg = 0.0f;
constexpr float kServoMaxAngleDeg = 180.0f;
constexpr float kServoStepDeg = 0.1f;
}

float debug_servo_angle_deg = 90.0f;

float arm2006_speed = 0.0f;
float arm3508_speed = 0.0f;
float arm2006_pid_out = 0.0f;
float arm3508_pid_out = 0.0f;

PID_t arm2006_motor_pid = {
  .Kp = 100.0, .Ki = 10.0, .Kd = 0.0, .MaxOut = 20000, .DeadBand = 0.3, .Improve = NONE};
PID_t arm3508_motor_pid = {
  .Kp = 200.0, .Ki = 50.0, .Kd = 0.8, .MaxOut = 12000, .DeadBand = 0.1, .Improve = NONE};

#endif


static inline void debugInit(void) {
  // PID_Init(&arm3508_motor_speedpid);
  PID_Init(&arm3508_motor_pospid);
#ifdef DEBUG_ARM
  PID_Init(&arm2006_motor_pid);
  PID_Init(&arm3508_motor_pid);

  debug_servo.init();
  debug_servo.setServoAngle(debug_servo_angle_deg);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

  if (!debug_xbox_sub.IsValid()) {
    return;
  }
#endif

}


void debugTask(void *argument) {
  (void)argument;
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();
  debugInit();
  
  for (;;) {
        // 更新各任务剩余栈空间（单位：byte）
        CAN1_Send_m = osThreadGetStackSpace(CAN1_Send_TaskHandle);
        CAN2_Send_m = osThreadGetStackSpace(CAN2_Send_TaskHandle);
        CAN3_Send_m = osThreadGetStackSpace(CAN3_Send_TaskHandle);
        Debug_m = osThreadGetStackSpace(Debug_TaskHandle);
        Chassis_m = osThreadGetStackSpace(ChassisTaskHandle);
        Control_m = osThreadGetStackSpace(ControlTaskHandle);
        PosCtrl_m = osThreadGetStackSpace(PosCtrlTaskHandle);
        Uart2Process_m = osThreadGetStackSpace(uart2ProcessTaskHandle);
        Uart3Process_m = osThreadGetStackSpace(uart3ProcessTaskHandle);
        Uart4Process_m = osThreadGetStackSpace(uart4ProcessTaskHandle);
        UsbcdcProcess_m = osThreadGetStackSpace(usbcdcProcessTaskHandle);



#ifdef DEBUG_ARM
    if (debug_xbox_sub.TryGet(&debug_xbox_cmd)) {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,
                        debug_xbox_cmd.btnY ? GPIO_PIN_SET : GPIO_PIN_RESET);

      if (debug_xbox_cmd.btnDirUp) {
        debug_servo_angle_deg += kServoStepDeg;
        if (debug_servo_angle_deg > kServoMaxAngleDeg) {
          debug_servo_angle_deg = kServoMaxAngleDeg;
        }
      }

      if (debug_xbox_cmd.btnDirDown) {
        debug_servo_angle_deg -= kServoStepDeg;
        if (debug_servo_angle_deg < kServoMinAngleDeg) {
          debug_servo_angle_deg = kServoMinAngleDeg;
        }
      }

      if (debug_xbox_cmd.btnX) {
        arm2006_speed = 300.0f;
      } else if (debug_xbox_cmd.btnB) {
        arm2006_speed = -300.0f;
      } else {
        arm2006_speed = 0.0f;
      }

      if (debug_xbox_cmd.btnLB) {
        arm3508_speed = -50.0f;
      } else if (debug_xbox_cmd.btnRB) {
        arm3508_speed = 50.0f;
      } else {
        arm3508_speed = 0.0f;
      }
    }

    debug_servo.setServoAngle(debug_servo_angle_deg);

    arm2006_pid_out = PID_Calculate(&arm2006_motor_pid,
                                    arm2006_motor.getRawCurrentSpeed(),
                                    arm2006_speed);
    arm3508_pid_out = PID_Calculate(&arm3508_motor_pid,
                                    arm3508_motor.getRawCurrentSpeed(),
                                    arm3508_speed);
    arm2006_motor.setMotorCmd(arm2006_pid_out);
    arm3508_motor.setMotorCmd(arm3508_pid_out);
  #endif
    // arm3508_pid_out = PID_Calculate(&arm3508_motor_speedpid,
    //                                 arm3508_motor.getRawCurrentSpeed(),
    //                                 arm3508_speed);
    arm3508_pid_out = PID_Calculate(&arm3508_motor_pospid,
                                    arm3508_motor.getCurrentSumPos(),
                                    arm3508_speed);
    // arm3508_motor.setMotorCmd(arm3508_pid_out);
    if (change == 0)arm4310_motor.posWithSpeedControl(arm4310_pos, arm4310_speed);
    else arm4310_motor.speedControl(arm4310_speed);  
    vTaskDelayUntil(&currentTime, 1);
  }
}