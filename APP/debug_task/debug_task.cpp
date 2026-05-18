/**
 * @file debug_task.cpp
 * @author 大帅将军，lx下士
 * @brief 调试任务，监控各任务剩余栈空间
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

#include "task.h"

osThreadId_t Debug_TaskHandle;

// ---------- 任务剩余栈空间监控变量（单位：byte） ----------
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

static uint32_t CAN1_Send_m = 0;
static uint32_t CAN2_Send_m = 0;
static uint32_t CAN3_Send_m = 0;
static uint32_t Debug_m = 0;
static uint32_t Chassis_m = 0;
static uint32_t Control_m = 0;
static uint32_t PosCtrl_m = 0;
static uint32_t Uart2Process_m = 0;
static uint32_t Uart3Process_m = 0;
static uint32_t Uart4Process_m = 0;
static uint32_t UsbcdcProcess_m = 0;


void debugTask(void *argument) {
  (void)argument;
  TickType_t currentTime = xTaskGetTickCount();

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

    vTaskDelayUntil(&currentTime, 100); // 每100ms更新一次
  }
}
