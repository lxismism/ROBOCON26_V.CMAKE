/**
 * @file topic_pool.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief
 * 模块所依赖的数据类型结构体，有些模块会依赖这些数据类型结构体进行数据传输，所以移植module层都
 *        必须携带这个包
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "fdcan.h"
#include "portmacro.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)

typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

typedef enum {
    Left = -1,
    right = 1,
}FieldSide_t;

typedef enum {
    MC = 1,
    MF = 2,
    Arena = 3,
}RobotMode_t;

typedef struct {
  bool btnY;
  bool btnA;
  bool btnB;
  bool btnX;
  bool btnLB;
  bool btnRB;
  bool btnLS;
  bool btnRS;
  bool btnSelect;
  bool btnShare;
  bool btnStart;
  bool btnXbox;
  uint16_t trigLT;
  uint16_t trigRT;
  bool btnDirUp;
  bool btnDirDown;
  bool btnDirLeft;
  bool btnDirRight;

  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;

} pub_Xbox_Data;

// IMU姿态传感器数据 —— 无头模式用
typedef struct {
  float yaw_rad;   // 偏航角，单位：弧度，范围 -π ~ +π
  float pitch_rad; // 俯仰角
  float roll_rad;  // 滚转角
} pub_imu_data;


// 底盘运动指令
typedef struct {
  float linear_x_;
  float linear_y_;
  float omega_;
} pub_chassis_cmd;

//Position模块数据结构体
typedef struct {
  uint8_t frame_id;
  uint8_t payload_length;
  uint32_t frame_count;
  float x;
  float y;
  float yaw;
  float yaw_speed;

} pub_Position_Data;

// 上身机构控制指令（control_task → pos_ctrl_task）
typedef struct {
  bool active;  // true = 用户模式生效，false = 队友模式（忽略本指令）

  // 每帧步进增量（持续按住时累加）
  // 单位：mm/帧（pick_yaw_delta 除外，为 °/帧）
  float pick_lift_delta;
  float pick_yaw_delta;      // 单位：°/帧（云台旋转，非直线运动）
  float pick_extend_delta;
  float weapon_lift_delta;
  float weapon_extend_delta;
  float lift_delta;

  // 切换型命令（仅上升沿有效，pos_ctrl_task 执行后清零）
  bool pump_toggle;
  bool valve_toggle;
  bool claw_toggle;
  bool wrist_toggle;

  // ===== 全身绝对姿态模式（set_absolute_pose=true 时生效，忽略delta） =====
  bool set_absolute_pose;
  float pick_lift_target_mm;
  float pick_yaw_target_deg;
  float pick_extend_target_mm;
  float weapon_lift_target_mm;
  float weapon_extend_target_mm;
  float lift_target_mm;


} pub_upbody_cmd;



// IR模块数据结构体
typedef struct {
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
} pub_ir_data;

typedef struct {
  uint8_t tx_data[3];
} pub_ir_cmd;

typedef struct {
  uint8_t QR_type;
} QR_code_cmd_t;

typedef struct  {
  uint8_t QR_type;
} QR_code_data_t;

#pragma pack()