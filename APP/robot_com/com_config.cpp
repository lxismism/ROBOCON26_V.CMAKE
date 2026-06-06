/**
 * @file com_config.cpp
 * @author Keten (2863861004@qq.com) lxlx
 * @brief 全局通信配置，包含can设备、串口设备、协议解析等
 * @version 0.2
 * @date 2026-04-21 2026-05-19(lxlx)
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "pid_controller.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"

#include "Canbus.hpp"
#include "Motor.hpp"
#include "pick_hand.hpp"
#include "weapon_hand.hpp"
#include "lift.hpp"
#include "Position.hpp"
#include "ROSCom.hpp"
#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "XboxRemote.hpp"
#include "pm20s.hpp"
#include "tim.h"
#include "WitMotionImu.hpp"
#include "topics.hpp"
#include "topic_pool.h"
#include "usart.h"

#include "chassis_solution.hpp" //访问底盘控制器用于串口调参
#include "lift.hpp"             //访问lift模块用于串口调参

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdio.h>

osThreadId_t CAN1_Send_TaskHandle;
osThreadId_t CAN2_Send_TaskHandle;
osThreadId_t CAN3_Send_TaskHandle;
osThreadId_t uart2ProcessTaskHandle;
osThreadId_t uart3ProcessTaskHandle;
osThreadId_t uart4ProcessTaskHandle;
osThreadId_t uart5ProcessTaskHandle;
osThreadId_t uart10ProcessTaskHandle;
osThreadId_t uart10SendTaskHandle;
osThreadId_t usbcdcProcessTaskHandle;
osThreadId_t DebugSerialTaskHandle;
osThreadId_t usbcdcSendTaskHandle;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CanBus fdcan1_bus(hfdcan1);
CanBus fdcan2_bus(hfdcan2);
CanBus fdcan3_bus(hfdcan3);

// can设备

// 底盘电机
C620Motor chassis_motor1(&fdcan3_bus, 0x201, 0, 0x200, 0);
C620Motor chassis_motor2(&fdcan3_bus, 0x202, 0, 0x200, 0);
C620Motor chassis_motor3(&fdcan3_bus, 0x203, 0, 0x200, 0);
C620Motor chassis_motor4(&fdcan3_bus, 0x204, 0, 0x200, 0);

//
// ---------- 上层机构电机 (CAN1) ----------
// 0x1FF 组 (0x205-0x208): 2006 电机
C610Motor picker_yaw_motor(&fdcan1_bus, 0x205, 0, 0x1FF, 0);     // pick_hand 云台旋转
C610Motor picker_extend_motor(&fdcan1_bus, 0x206, 0, 0x1FF, 0);  // pick_hand 伸缩
C610Motor weapon_extend_motor(&fdcan1_bus, 0x207, 0, 0x1FF, 0);  // weapon_hand 伸缩

// 0x208 预留

// ---------- 上层机构电机 (CAN2) ----------

// 0x200 组 (0x201-0x204): 3508 电机
C620Motor lift_left_motor(&fdcan2_bus, 0x201, 0, 0x200, 0);      // lift 左侧
C620Motor lift_right_motor(&fdcan2_bus, 0x202, 0, 0x200, 0);     // lift 右侧
C620Motor picker_lift_motor(&fdcan2_bus, 0x203, 0, 0x200, 0);    // pick_hand 抬升
C620Motor weapon_lift_motor(&fdcan2_bus, 0x204, 0, 0x200, 0);    // weapon_hand 抬升

DM4310Motor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,
                         DM4310Motor::PosWithSpeed);

// ---------- 上层机构模块对象 ----------
PickHand pick_hand;
WeaponHand weapon_hand;
Lift lift;
// ---------- 腕部舵机（TIM13_CH1, PF8） ----------
extern TIM_HandleTypeDef htim13;
PM20sServo wrist_servo(htim13, TIM_CHANNEL_1);



// 串口外设（回调+信号量唤醒处理线程进行解包）
void onUart3RxCb(const uint8_t *data, size_t len, void *user);

void onUsbRxCb(const uint8_t *data, size_t len, void *user);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart10;

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];
UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma), uart3_tx_dma,
                    sizeof(uart3_tx_dma), onUart3RxCb, nullptr);
osSemaphoreId_t uart3_rx_semphore = NULL;

// IMU姿态传感器（USART2）
void onUart2RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[128];
DMA_BUFFER_ATTR static uint8_t uart2_tx_dma[64];
UartPort uart2_port(&huart2, uart2_rx_dma, sizeof(uart2_rx_dma), uart2_tx_dma,
                    sizeof(uart2_tx_dma), onUart2RxCb, nullptr);
osSemaphoreId_t uart2_rx_semphore = NULL;

// Position（USART4）
void onUart4RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart4_rx_dma[128];
DMA_BUFFER_ATTR static uint8_t uart4_tx_dma[64];
UartPort uart4_port(&huart4, uart4_rx_dma, sizeof(uart4_rx_dma), uart4_tx_dma,
                    sizeof(uart4_tx_dma), onUart4RxCb, nullptr);
osSemaphoreId_t uart4_rx_semphore = NULL;

//debug串口
void onUart5RxCb(const uint8_t *data, size_t len, void *user); //仅用于实例化不报错

DMA_BUFFER_ATTR static uint8_t uart5_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart5_tx_dma[512];
UartPort uart5_port(&huart5, uart5_rx_dma, sizeof(uart5_rx_dma), uart5_tx_dma,
                    sizeof(uart5_tx_dma), onUart5RxCb, nullptr);
osSemaphoreId_t uart5_rx_semphore = NULL;

void onUart10RxCb(const uint8_t *data, size_t len, void *user); //仅用于实例化不报错

DMA_BUFFER_ATTR static uint8_t uart10_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart10_tx_dma[64];
UartPort uart10_port(&huart10, uart10_rx_dma, sizeof(uart10_rx_dma), uart10_tx_dma,
                     sizeof(uart10_tx_dma), onUart10RxCb, nullptr);
osSemaphoreId_t uart10_rx_semphore = NULL;

// IMU姿态传感器解析器 及 Topic发布者
WitMotionImu wit_imu;
TypedTopicPublisher<pub_imu_data> imu_data_pub("imu_data");
pub_imu_data imu_msg{};

// Xbox控制器（基于uart3）
XboxRemote xbox_remote(uart3_port);
TypedTopicPublisher<pub_Xbox_Data> xbox_data_pub("xbox");
pub_Xbox_Data xbox_msg;

// Position模块（基于uart4）
Position position;
TypedTopicPublisher<pub_Position_Data> Position_data_pub("position");
pub_Position_Data Position_msg{};

// 上身控制指令发布者（control_task 决策 → pos_ctrl_task 执行）
TypedTopicPublisher<pub_upbody_cmd> upbody_cmd_pub("upbody_cmd");
pub_upbody_cmd upbody_cmd_msg{};
//IR模块（基于uart10）
TypedTopicPublisher<pub_ir_data> ir_data_pub("ir_data");
pub_ir_data ir_data{};
TickType_t last_data_received_time = 0; // 上次接收到数据的时间

TypedTopicSubscriber<pub_ir_cmd> ir_cmd_sub("ir_cmd", 8);
pub_ir_cmd ir_cmd{};

TypedTopicSubscriber<QR_code_cmd_t> qr_code_cmd_sub("qr_code_cmd", 8);
QR_code_cmd_t qr_code_cmd{};

TypedTopicPublisher<QR_code_data_t> qr_code_data_pub("qr_code_data");
QR_code_data_t qr_code_data{};


// usb
osSemaphoreId_t usbcdc_rx_semphore = NULL;
ROSProtocol ros_protocol(nullptr, &UsbPort::Instance());

uint8_t comServiceInit() {
  // can外设初始化
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan1);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan2);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan3);

  // can 总线初始化
  fdcan1_bus.init();
  fdcan2_bus.init();
  fdcan3_bus.init();

  chassis_motor1.init();
  chassis_motor2.init();
  chassis_motor3.init();
  chassis_motor4.init();

  // ---- 上层机构电机初始化 ----
  picker_yaw_motor.init();
  picker_extend_motor.init();
  weapon_extend_motor.init();

  lift_left_motor.init(100);
  lift_right_motor.init(100);
  picker_lift_motor.init();
  weapon_lift_motor.init();

  arm4310_motor.init();

  // ---- 底盘电机初始化 ----
  fdcan3_bus.registerDevice(&chassis_motor1);
  fdcan3_bus.registerDevice(&chassis_motor2);
  fdcan3_bus.registerDevice(&chassis_motor3);
  fdcan3_bus.registerDevice(&chassis_motor4);


  // ---- 将上身3个2006电机注册到 CAN1 总线 ----
  fdcan1_bus.registerDevice(&picker_yaw_motor);
  fdcan1_bus.registerDevice(&picker_extend_motor);
  fdcan1_bus.registerDevice(&weapon_extend_motor);

  // ---- 将上身4个3508注册到 CAN2 总线 ----
  fdcan2_bus.registerDevice(&lift_left_motor);
  fdcan2_bus.registerDevice(&lift_right_motor);
  fdcan2_bus.registerDevice(&picker_lift_motor);
  fdcan2_bus.registerDevice(&weapon_lift_motor);

  fdcan2_bus.registerDevice(&arm4310_motor);

  // ---- 绑定机构模块的电机指针 ----
  pick_hand.lift_motor_ = &picker_lift_motor;
  pick_hand.yaw_motor_ = &picker_yaw_motor;
  pick_hand.extend_motor_ = &picker_extend_motor;

  weapon_hand.lift_motor_ = &weapon_lift_motor;
  weapon_hand.extend_motor_ = &weapon_extend_motor;

  lift.left_motor_ = &lift_left_motor;
  lift.right_motor_ = &lift_right_motor;

  // ---- 机构模块 PID 初始化 ----
  pick_hand.init();
  weapon_hand.init();
  lift.init();
  // ---- 舵机初始化 & 绑定 ----
  wrist_servo.init();
  weapon_hand.wrist_servo_ = &wrist_servo;
  
  
  // 串口外设

  uart10_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart10_rx_semphore == NULL || uart10_port.startRxDmaIdle() != HAL_OK) {
    return 1;
  }

  uart5_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart5_rx_semphore == NULL || uart5_port.startRxDmaIdle() != HAL_OK) {
    return 1;
  }

  uart3_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart3_rx_semphore == NULL || uart3_port.startRxDmaIdle() != HAL_OK) {
    return 1;
  }

  uart4_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart4_rx_semphore == NULL || uart4_port.startRxDmaIdle() != HAL_OK) {
    return 1;
  }

  uart2_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart2_rx_semphore == NULL || uart2_port.startRxDmaIdle() != HAL_OK) {
    return 1;
  }
  // Xbox控制器初始化
  xbox_remote.init();

  // position模块初始化
  position.init();

  // usb 外设
  usbcdc_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (usbcdc_rx_semphore == NULL) {
    return 1;
  }
  ros_protocol.init();
  UsbPort::Instance().SetRxCallback(onUsbRxCb, NULL);
  return 0;
}

void onUart10RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart10_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart10_rx_semphore);
  }
}

void onUart5RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart5_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart5_rx_semphore);
  }
}

void onUart4RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart4_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart4_rx_semphore);
  }
}

void onUart3RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart3_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart3_rx_semphore);
  }
}

void onUart2RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart2_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart2_rx_semphore);
  }
}

void onUsbRxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && usbcdc_rx_semphore != NULL) {
    (void)osSemaphoreRelease(usbcdc_rx_semphore);
  }
}

void can1SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t group_1FF_ids[4] = {0x205, 0x206, 0x207, 0x208};

  for (;;) {
    // ---- 帧: 0x1FF → 2006电机组 (云台旋转/伸缩) ----
    pack.id = 0x1FF;
    int16_t commands_1FF[4] = {0};
    commands_1FF[0] = static_cast<int16_t>(picker_yaw_motor.cmdTrans());
    commands_1FF[1] = static_cast<int16_t>(picker_extend_motor.cmdTrans());
    commands_1FF[2] = static_cast<int16_t>(weapon_extend_motor.cmdTrans());
    commands_1FF[3] = static_cast<int16_t>(0); // 0x208 预留

    packDJIMotorCanMsg(pack.id, group_1FF_ids, commands_1FF, 4, pack.data, len);
    fdcan1_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1);
  }
}


void can2SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t group_200_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // ---- 帧: 0x200 → 3508电机组 (抬升) ----
    pack.id = 0x200;
    int16_t commands_200[4] = {0};
    commands_200[0] = static_cast<int16_t>(lift_left_motor.cmdTrans());
    commands_200[1] = static_cast<int16_t>(lift_right_motor.cmdTrans());
    commands_200[2] = static_cast<int16_t>(picker_lift_motor.cmdTrans());
    commands_200[3] = static_cast<int16_t>(weapon_lift_motor.cmdTrans());

    packDJIMotorCanMsg(pack.id, group_200_ids, commands_200, 4, pack.data, len);
    fdcan2_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1);
  }
}


void can3SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;  
  const uint32_t chassis_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // 一帧固定打包 4 个槽位：0x201~0x204
    pack.id = 0x200; // DJI Group 2

    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};
    commands[0] = static_cast<int16_t>(chassis_motor1.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(chassis_motor2.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(chassis_motor3.cmdTrans()); // 0x203   
    commands[3] = static_cast<int16_t>(chassis_motor4.cmdTrans()); // 0x204
    packDJIMotorCanMsg(pack.id, chassis_motor_ids, commands, 4, pack.data, len);
    // arm3508_motor.manager_->addCanMsg(pack);
    fdcan3_bus.addCanMsg(pack);
    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void uart3RxProcessTask(void *argument) {
  (void)argument;
  if(!xbox_data_pub.IsValid()) {
    return;
  }
  for (;;) {
    (void)osSemaphoreAcquire(uart3_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart3_port.Read(packet)) {
      // 逐字节送进Xbox协议解析器
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = xbox_remote.processByte(packet.data[i]);
        if (frame_id != 0) {
          // 帧解析完成，可以在这里获取控制器数据并做业务处理
          const auto &ctrl_data = xbox_remote.getControllerData();
          xbox_msg.btnY = ctrl_data.btnY;
          xbox_msg.btnA = ctrl_data.btnA;
          xbox_msg.btnB = ctrl_data.btnB;
          xbox_msg.btnX = ctrl_data.btnX;
          xbox_msg.btnLB = ctrl_data.btnLB;
          xbox_msg.btnRB = ctrl_data.btnRB;
          xbox_msg.btnLS = ctrl_data.btnLS;
          xbox_msg.btnRS = ctrl_data.btnRS;
          xbox_msg.btnSelect = ctrl_data.btnSelect;
          xbox_msg.btnShare = ctrl_data.btnShare;
          xbox_msg.btnStart = ctrl_data.btnStart;
          xbox_msg.btnXbox = ctrl_data.btnXbox;
          xbox_msg.trigLT = ctrl_data.trigLT;
          xbox_msg.trigRT = ctrl_data.trigRT;
          xbox_msg.btnDirUp = ctrl_data.btnDirUp;
          xbox_msg.btnDirDown = ctrl_data.btnDirDown;
          xbox_msg.btnDirLeft = ctrl_data.btnDirLeft;
          xbox_msg.btnDirRight = ctrl_data.btnDirRight;
          xbox_msg.joyLHori = ctrl_data.joyLHori;
          xbox_msg.joyLVert = ctrl_data.joyLVert;
          xbox_msg.joyRHori = ctrl_data.joyRHori;
          xbox_msg.joyRVert = ctrl_data.joyRVert;
          xbox_data_pub.Publish(xbox_msg);
        }
      }
    }
  }
}

//暂时先做发送
void uart5RxProcessTask(void *argument) {
  (void)argument;
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

//IR
void uart10RxProcessTask(void *argument) {
  (void)argument;
  if(!ir_data_pub.IsValid()) {
    return;
  }

  typedef enum {
    wait_for_data_HEAD,
    wait_for_data1,
    wait_for_data2,
    wait_for_data3,
    wait_for_data_END
  }ir_data_rx_state_t;

  ir_data_rx_state_t ir_data_rx_state = wait_for_data_HEAD;

  for(;;)
  {
    (void)osSemaphoreAcquire(uart10_rx_semphore, osWaitForever);
    
    UartPort::Packet packet{};

    while(uart10_port.Read(packet)) {
      // 处理接收到的IR数据
      for(uint16_t i = 0; i < packet.len; ++i) {
        if(packet.data[i] == 0xAA)   ir_data_rx_state = wait_for_data1;
        else
        {
          switch (ir_data_rx_state) {
            case wait_for_data_HEAD:
              break;
            case wait_for_data1:
              ir_data.data1 = packet.data[i];
              ir_data.data2 = 0;
              ir_data_rx_state = wait_for_data2;
              break;
            case wait_for_data2:
              ir_data.data2 = packet.data[i];
              ir_data_rx_state = wait_for_data3;
              break;
            case wait_for_data3:
              ir_data.data3 = packet.data[i];
              ir_data_rx_state = wait_for_data_END;
              //last_data_received_time = xTaskGetTickCount();
              //ir_data_pub.Publish(ir_data);
              break;
            case wait_for_data_END:
              if(packet.data[i] == 0xBB) {
                last_data_received_time = xTaskGetTickCount();
                if(ir_data.data1 == ir_data.data2 && ir_data.data2 == ir_data.data3)
                  ir_data_pub.Publish(ir_data);
              }
              ir_data_rx_state = wait_for_data_HEAD;
              break;
          }
        }   
      }
    }
  }
}

void uart10SendTask(void *argument) {
  (void)argument;

  if(!ir_cmd_sub.IsValid()) {
    return;
  }
  if(!ir_data_pub.IsValid()) {
    return;
  }

  uint8_t tx[5];
  tx[0] = 0xAA; // 数据帧头
  tx[4] = 0xBB; // 数据帧尾

  TickType_t currentTime = xTaskGetTickCount();
  TickType_t last_transmit_time = 0;

  typedef enum {
    wait_for_transmit,
    wait_for_data,
  } ir_data_tx_state_t;
  ir_data_tx_state_t ir_data_tx_state = wait_for_data;

  for(;;)
  {
    switch (ir_data_tx_state) {
      case wait_for_transmit:
        if(currentTime - last_transmit_time >= 150 && currentTime - last_data_received_time >= 150)
          ir_data_tx_state = wait_for_data;
        break;
      case wait_for_data:
        if (ir_cmd_sub.TryGet(&ir_cmd)) {
          tx[1] = ir_cmd.tx_data[0];
          tx[2] = ir_cmd.tx_data[1];
          tx[3] = ir_cmd.tx_data[2];
          uart10_port.writeDma(tx, sizeof(tx));
          last_transmit_time = xTaskGetTickCount();
          ir_data_tx_state = wait_for_transmit;
        }
        break;
    }
    vTaskDelayUntil(&currentTime, 10);
  }
}

void DebugSerialTask(void *argument) {
  (void)argument;
  static char debug_buffer[256];
  static char title[12];

  extern OmniChassis Omnichassis_solver;
  extern pub_chassis_cmd state_Aim_cmd;
  extern pub_Position_Data control_position_msg;
  extern pub_Position_Data control_position;
  extern Lift lift;
  extern pub_chassis_cmd robot_v_aim_cmd;

  const PID_t& pid_LU = Omnichassis_solver.pid(OmniChassis::kLeftUp);
  const PID_t& pid_RU = Omnichassis_solver.pid(OmniChassis::kRightUp);
  const PID_t& pid_LD = Omnichassis_solver.pid(OmniChassis::kLeftDown);
  const PID_t& pid_RD = Omnichassis_solver.pid(OmniChassis::kRightDown);

  TickType_t currentTime = xTaskGetTickCount();

  for(;;)
  {
    //电机在线检测
    title[0] =  chassis_motor1.isOffline() ? 'X' : 'O';
    title[1] =  chassis_motor2.isOffline() ? 'X' : 'O';
    title[2] =  chassis_motor3.isOffline() ? 'X' : 'O';
    title[3] =  chassis_motor4.isOffline() ? 'X' : 'O';
    title[4] =  picker_yaw_motor.isOffline() ? 'X' : 'O';
    title[5] =  picker_extend_motor.isOffline() ? 'X' : 'O';
    title[6] =  weapon_extend_motor.isOffline() ? 'X' : 'O';
    title[7] =  lift_left_motor.isOffline() ? 'X' : 'O';
    title[8] =  lift_right_motor.isOffline() ? 'X' : 'O';
    title[9] =  picker_lift_motor.isOffline() ? 'X' : 'O';
    title[10] = weapon_lift_motor.isOffline() ? 'X' : 'O';

    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%03d,%d.%03d,%d.%02d\n",
    //                                           static_cast<int>(pid_LU.Ref), (static_cast<int>(abs(pid_LU.Ref * 100)))%100,
    //                                           static_cast<int>(pid_LU.Measure), (static_cast<int>(abs(pid_LU.Measure * 100)))%100,
    //                                           static_cast<int>(pid_RU.Ref), (static_cast<int>(abs(pid_RU.Ref * 100)))%100,
    //                                           static_cast<int>(pid_RU.Measure), (static_cast<int>(abs(pid_RU.Measure * 100)))%100,
    //                                           static_cast<int>(pid_LD.Ref), (static_cast<int>(abs(pid_LD.Ref * 100)))%100,
    //                                           static_cast<int>(pid_LD.Measure), (static_cast<int>(abs(pid_LD.Measure * 100)))%100,
    //                                           static_cast<int>(pid_RD.Ref), (static_cast<int>(abs(pid_RD.Ref * 100)))%100,
    //                                           static_cast<int>(pid_RD.Measure), (static_cast<int>(abs(pid_RD.Measure * 100)))%100,
    //                                           static_cast<int>(control_position.x), (static_cast<int>(abs(control_position.x * 1000)))%1000,
    //                                           static_cast<int>(control_position.y), (static_cast<int>(abs(control_position.y * 1000)))%1000,
    //                                           static_cast<int>(control_position.yaw), (static_cast<int>(abs(control_position.yaw * 100)))%100

    int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d,%d.%02d,%d.%02d\n",
                                              static_cast<int>(robot_v_aim_cmd.linear_x_), (static_cast<int>(abs(robot_v_aim_cmd.linear_x_ * 100)))%100,
                                              static_cast<int>(robot_v_aim_cmd.linear_y_), (static_cast<int>(abs(robot_v_aim_cmd.linear_y_ * 100)))%100,
                                              static_cast<int>(robot_v_aim_cmd.omega_), (static_cast<int>(abs(robot_v_aim_cmd.omega_ * 100)))%100


    // );
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "%splatform: %d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
                                              title,
                                              static_cast<int>(lift.platfrom_pos_pid_.Ref), (static_cast<int>(abs(lift.platfrom_pos_pid_.Ref * 100)))%100,
                                              static_cast<int>(lift.platfrom_pos_pid_.Measure), (static_cast<int>(abs(lift.platfrom_pos_pid_.Measure * 100)))%100,
                                              static_cast<int>(lift.left_v_pid_.Ref), (static_cast<int>(abs(lift.left_v_pid_.Ref * 100)))%100,
                                              static_cast<int>(lift.left_v_pid_.Measure), (static_cast<int>(abs(lift.left_v_pid_.Measure * 100)))%100,
                                              static_cast<int>(lift.right_v_pid_.Ref), (static_cast<int>(abs(lift.right_v_pid_.Ref * 100)))%100,
                                              static_cast<int>(lift.right_v_pid_.Measure), (static_cast<int>(abs(lift.right_v_pid_.Measure * 100)))%100
    );
    // HAL_UART_Transmit_DMA(&huart5, (const uint8_t *)debug_buffer, sizeof(debug_buffer));
    uart5_port.writeDma(reinterpret_cast<const uint8_t*>(debug_buffer), len);
    vTaskDelayUntil(&currentTime, 10);//10ms发送一次
  }
}

void usbCdcProcessTask(void *argument) {

  (void)argument;

  for (;;) {
    (void)osSemaphoreAcquire(usbcdc_rx_semphore, osWaitForever);

    UsbPort::Packet packet{};
    while (UsbPort::Instance().Read(packet)) {
      // 逐个字节解析
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = ros_protocol.processData(packet.data[i]);
        if (frame_id != 0) {
          switch (frame_id) {
            case static_cast<uint8_t>(ROSProtocol::package_id::QR_CODE_BAG): {
              // 发布二维码类型
              const auto &qr_types = ros_protocol.getQRCodeBagData().QR_type;
              qr_code_data.QR_type = qr_types;
              qr_code_data_pub.Publish(qr_code_data);
              //重复应答
              // uint8_t rev[64] = {0};
              // memcpy(rev, &ros_protocol.getQRCodeBagData(), sizeof(ros_protocol.getQRCodeBagData()));
              // UsbPort::Instance().WriteAsync(rev, sizeof(ros_protocol.getQRCodeBagData()));
              break;
            }
            default:
              break;
          }

        }
      }
    }
  }
}

void usbCdcSendTask(void *argument) {
  (void)argument;

  TickType_t currentTime = xTaskGetTickCount();

  if(!qr_code_cmd_sub.IsValid()) {
    return;
  }

  if(!qr_code_data_pub.IsValid()) {
    return;
  }

  for (;;) {
    if(qr_code_cmd_sub.TryGet(&qr_code_cmd)) {
      uint8_t tx[64] = {0};
      uint8_t frame_length = ros_protocol.packQRMsg(tx, qr_code_cmd.QR_type);
      UsbPort::Instance().WriteAsync(tx, frame_length);
    }
    vTaskDelayUntil(&currentTime, 10); // 每10ms发送一次
  }
}

void uart2RxProcessTask(void *argument) {
  (void)argument;

  for (;;) {
    (void)osSemaphoreAcquire(uart2_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart2_port.Read(packet)) {
      // 逐字节喂给IMU协议解析器
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_type = wit_imu.processByte(packet.data[i]);
        if (frame_type == 0x53) {
          // 一帧角度数据解析完毕，发布到Topic总线
          const auto &data = wit_imu.getImuData();
          imu_msg.yaw_rad = data.yaw_rad;
          imu_msg.pitch_rad = data.pitch_rad;
          imu_msg.roll_rad = data.roll_rad;
          imu_data_pub.Publish(imu_msg);
        }
      }
    }

  }
}

void uart4RxProcessTask(void *argument) {
  (void)argument;
  if (!Position_data_pub.IsValid()) {
    return;
  }

  for (;;) {
    (void)osSemaphoreAcquire(uart4_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart4_port.Read(packet)) {
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = position.processByte(packet.data[i]);
        if (frame_id != 0) {
          const auto &pos_data = position.getData();
          Position_msg.frame_id = pos_data.frame_id;
          Position_msg.payload_length = pos_data.payload_length;
          Position_msg.frame_count = pos_data.frame_count;
          Position_msg.x = pos_data.x;
          Position_msg.y = pos_data.y;
          Position_msg.yaw = pos_data.yaw;
          Position_msg.yaw_speed = pos_data.yaw_speed;
          Position_data_pub.Publish(Position_msg);
        }
      }
    }
  }
}