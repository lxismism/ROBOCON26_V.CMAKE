/**
 * @file com_config.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 全局通信配置，包含can设备、串口设备、协议解析等
 * @version 0.1
 * @date 2026-04-21
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
#include "Position.hpp"
#include "ROSCom.hpp"
#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "XboxRemote.hpp"
#include "WitMotionImu.hpp"
#include "topics.hpp"
#include "topic_pool.h"
#include "usart.h"

#include "chassis_solution.hpp" //访问底盘控制器用于串口调参

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
osThreadId_t usbcdcProcessTaskHandle;
osThreadId_t DebugSerialTaskHandle;

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
C610Motor arm2006_motor(&fdcan2_bus, 0x205, 0, 0x1FF, 0);
C620Motor arm3508_motor(&fdcan2_bus, 0x206, 0, 0x1FF, 0);
DM4310Motor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,
                         DM4310Motor::PosWithSpeed);

// 串口外设（回调+信号量唤醒处理线程进行解包）
void onUart3RxCb(const uint8_t *data, size_t len, void *user);

void onUsbRxCb(const uint8_t *data, size_t len, void *user);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

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
DMA_BUFFER_ATTR static uint8_t uart5_tx_dma[128];
UartPort uart5_port(&huart5, uart5_rx_dma, sizeof(uart5_rx_dma), uart5_tx_dma,
                    sizeof(uart5_tx_dma), onUart5RxCb, nullptr);
osSemaphoreId_t uart5_rx_semphore = NULL;

// IMU姿态传感器解析器 及 Topic发布者
WitMotionImu wit_imu;
TypedTopicPublisher<pub_imu_data> imu_data_pub("imu_data");
pub_imu_data imu_msg{};

// Xbox控制器（基于uart3）
XboxRemote xbox_remote(uart3_port);
TypedTopicPublisher<pub_Xbox_Data> xbox_data_pub("xbox");
pub_Xbox_Data xbox_msg;

// Position模块（基于uart4）
Position Position;
TypedTopicPublisher<pub_Position_Data> Position_data_pub("position");
pub_Position_Data Position_msg{};

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

  arm2006_motor.init();
  arm3508_motor.init();
  arm4310_motor.init();

  fdcan3_bus.registerDevice(&chassis_motor1);
  fdcan3_bus.registerDevice(&chassis_motor2);
  fdcan3_bus.registerDevice(&chassis_motor3);
  fdcan3_bus.registerDevice(&chassis_motor4);


  fdcan2_bus.registerDevice(&arm2006_motor);
  fdcan2_bus.registerDevice(&arm3508_motor);
  fdcan2_bus.registerDevice(&arm4310_motor);

  // 串口外设
  uart5_rx_semphore = osSemaphoreNew(1, 0, NULL);
  uart5_port.startRxDmaIdle();

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

  // Position模块初始化
  Position.init();

  // usb 外设
  usbcdc_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (usbcdc_rx_semphore == NULL) {
    return 1;
  }
  ros_protocol.init();
  UsbPort::Instance().SetRxCallback(onUsbRxCb, NULL);
  return 0;
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

  for (;;) {

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void can2SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t arm_motor_ids[4] = {0x205, 0x206, 0x207, 0x208};
  for (;;) {
    pack.id = 0x1FF; // DJI Group 2
    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};

    // arm motor
    commands[0] = static_cast<int16_t>(arm2006_motor.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(arm3508_motor.cmdTrans()); // 0x203
    commands[2] = static_cast<int16_t>(0); // 0x203
    commands[3] = static_cast<int16_t>(0); // 0x204
    packDJIMotorCanMsg(pack.id, arm_motor_ids, commands, 4, pack.data, len);
    fdcan2_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
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
          xbox_msg.btnLB = ctrl_data.btnLB;
          xbox_msg.btnRB = ctrl_data.btnRB;
          xbox_msg.trigLT = ctrl_data.trigLT;
          xbox_msg.trigRT = ctrl_data.trigRT;
          xbox_msg.btnDirUp = ctrl_data.btnDirUp;
          xbox_msg.btnDirDown = ctrl_data.btnDirDown;
          xbox_msg.btnDirLeft = ctrl_data.btnDirLeft;
          xbox_msg.btnDirRight = ctrl_data.btnDirRight;
          xbox_msg.btnB = ctrl_data.btnB;
          xbox_msg.btnX = ctrl_data.btnX;
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

//暂时先做发
void uart5RxProcessTask(void *argument) {
  (void)argument;
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

void DebugSerialTask(void *argument) {
  (void)argument;
  static char debug_buffer[100];

  extern OmniChassis Omnichassis_solver;

  const PID_t& pid_LU = Omnichassis_solver.pid(OmniChassis::kLeftUp);
  const PID_t& pid_RU = Omnichassis_solver.pid(OmniChassis::kRightUp);
  const PID_t& pid_LD = Omnichassis_solver.pid(OmniChassis::kLeftDown);
  const PID_t& pid_RD = Omnichassis_solver.pid(OmniChassis::kRightDown);

  TickType_t currentTime = xTaskGetTickCount();

  for(;;)
  {
    currentTime = xTaskGetTickCount();
    snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d, %d.%02d, %d.%02d, %d.%02d\r\n",
                                              static_cast<int>(pid_LU.Ref), static_cast<int>(pid_LU.Ref * 100),
                                              static_cast<int>(pid_LU.Measure), static_cast<int>(pid_LU.Measure * 100),
                                              static_cast<int>(pid_RU.Ref), static_cast<int>(pid_RU.Ref * 100),
                                              static_cast<int>(pid_RU.Measure), static_cast<int>(pid_RU.Measure * 100),
                                              static_cast<int>(pid_LD.Ref), static_cast<int>(pid_LD.Ref * 100),
                                              static_cast<int>(pid_LD.Measure), static_cast<int>(pid_LD.Measure * 100),
                                              static_cast<int>(pid_RD.Ref), static_cast<int>(pid_RD.Ref * 100),
                                              static_cast<int>(pid_RD.Measure), static_cast<int>(pid_RD.Measure * 100)
    );
    uart5_port.writeDma(reinterpret_cast<uint8_t*>(debug_buffer), sizeof(debug_buffer));
    vTaskDelayUntil(&currentTime, 5);//1ms发送一次
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
          uint8_t rev[64] = {0};
          memcpy(rev, ros_protocol.getSensorBagData().i16_data,
                 sizeof(ros_protocol.getSensorBagData().i16_data));
          memcpy(rev + sizeof(ros_protocol.getSensorBagData().i16_data),
                 ros_protocol.getSensorBagData().f_data,
                 sizeof(ros_protocol.getSensorBagData().f_data));
          UsbPort::Instance().WriteAsync(
              rev, sizeof(ros_protocol.getSensorBagData()));
        }
      }
    }
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
        uint8_t frame_id = Position.processByte(packet.data[i]);
        if (frame_id != 0) {
          const auto &pos_data = Position.getData();
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