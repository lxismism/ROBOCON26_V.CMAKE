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

extern osThreadId_t uart5ProcessTaskHandle;       // ← 新增
extern osThreadId_t uart1ProcessTaskHandle;       // ← 新增
extern osThreadId_t uart6ProcessTaskHandle;       // ← 新增
extern osThreadId_t uart8ProcessTaskHandle;       // ← 新增
extern osThreadId_t uart9ProcessTaskHandle;       // ← 新增
extern osThreadId_t uart10ProcessTaskHandle;      // ← 新增
extern osThreadId_t usbcdcSendTaskHandle;         // ← 新增
extern osThreadId_t DebugSerialTaskHandle;        // ← 新增
extern osThreadId_t omniIrSendTaskHandle;         // ← 新增
extern osThreadId_t whisperIrSendTaskHandle;      // ← 新增

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

static uint32_t Uart5Process_m = 0;               // ← 新增
static uint32_t Uart1Process_m = 0;               // ← 新增
static uint32_t Uart6Process_m = 0;               // ← 新增
static uint32_t Uart8Process_m = 0;               // ← 新增
static uint32_t Uart9Process_m = 0;               // ← 新增
static uint32_t Uart10Process_m = 0;              // ← 新增
static uint32_t UsbcdcSend_m = 0;                 // ← 新增
static uint32_t DebugSerial_m = 0;                // ← 新增
static uint32_t OmniIrSend_m = 0;                 // ← 新增
static uint32_t WhisperIrSend_m = 0;              // ← 新增


void debugTask(void *argument) {
  (void)argument;
  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {
    CAN1_Send_m     = osThreadGetStackSpace(CAN1_Send_TaskHandle);
    CAN2_Send_m     = osThreadGetStackSpace(CAN2_Send_TaskHandle);
    CAN3_Send_m     = osThreadGetStackSpace(CAN3_Send_TaskHandle);
    Debug_m         = osThreadGetStackSpace(Debug_TaskHandle);
    Chassis_m       = osThreadGetStackSpace(ChassisTaskHandle);
    Control_m       = osThreadGetStackSpace(ControlTaskHandle);
    PosCtrl_m       = osThreadGetStackSpace(PosCtrlTaskHandle);
    Uart2Process_m  = osThreadGetStackSpace(uart2ProcessTaskHandle);
    Uart3Process_m  = osThreadGetStackSpace(uart3ProcessTaskHandle);
    Uart4Process_m  = osThreadGetStackSpace(uart4ProcessTaskHandle);
    UsbcdcProcess_m = osThreadGetStackSpace(usbcdcProcessTaskHandle);

    Uart5Process_m  = osThreadGetStackSpace(uart5ProcessTaskHandle);    // ←
    Uart1Process_m  = osThreadGetStackSpace(uart1ProcessTaskHandle);    // ←
    Uart6Process_m  = osThreadGetStackSpace(uart6ProcessTaskHandle);    // ←
    Uart8Process_m  = osThreadGetStackSpace(uart8ProcessTaskHandle);    // ←
    Uart9Process_m  = osThreadGetStackSpace(uart9ProcessTaskHandle);    // ←
    Uart10Process_m = osThreadGetStackSpace(uart10ProcessTaskHandle);   // ←
    UsbcdcSend_m    = osThreadGetStackSpace(usbcdcSendTaskHandle);      // ←
    DebugSerial_m   = osThreadGetStackSpace(DebugSerialTaskHandle);     // ←
    OmniIrSend_m    = osThreadGetStackSpace(omniIrSendTaskHandle);      // ←
    WhisperIrSend_m = osThreadGetStackSpace(whisperIrSendTaskHandle);   // ←

    vTaskDelayUntil(&currentTime, 100);
  }
}
