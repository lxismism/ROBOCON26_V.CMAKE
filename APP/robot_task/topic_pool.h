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
#include "rm_pocket.hpp"
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
    Normal = 0,
    MC = 1,
    MF = 2,
    Arena = 3,
}RobotMode_t;

typedef enum {
    KFS,
    Weapon,
    WithR2,
}ArenaMode_t;

typedef enum {
    Normal_case = 1,
    Special = 2,
    Debug = 3,
}RobotCase_t;

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

typedef struct {
  //摇杆，值域172-1810 建议映射值：min200 max1780 mid~=985 死区+-100
  uint16_t joyLHori;  //左小
  uint16_t joyLVert;  //下小
  uint16_t joyRHori;  //左小
  uint16_t joyRVert;  //下小

  RC_2_POS_SW_State_t swA;        //阴刻有SA的两端开关
  RC_2_POS_SW_State_t swA_last;
  RC_3_POS_SW_State_t swB;        //阴刻有SB的三段开关
  RC_3_POS_SW_State_t swB_last;
  RC_3_POS_SW_State_t swC;        //阴刻有SC的三段开关
  RC_3_POS_SW_State_t swC_last;
  RC_2_POS_SW_State_t swD;        //阴刻有SD的两段开关
  RC_2_POS_SW_State_t swD_last;
  RC_2_POS_SW_State_t swE;        //阴刻有SE的按钮
  RC_2_POS_SW_State_t swE_last;

  RC_Trim_State_t trimLeft;        //左微调按钮
  RC_Trim_State_t trimLeft_last;
  RC_Trim_State_t trimRight;       //右微调按钮
  RC_Trim_State_t trimRight_last;

  //电位器 往左推小，值域172-1810 实际可能取不到端点
  uint16_t pot;                   //阴刻有S1的拨盘

  //左微调按钮控制的光标，-9~+9
  int8_t x_cnt;
  int8_t y_cnt;

  //拨盘电位器控制的光标，最左0，中间1，最右2
  int8_t cursor;
} pub_RC_Data;

// IMU姿态传感器数据 —— 无头模式用
typedef struct {
  float yaw_rad;   // 偏航角，单位：弧度，范围 -π ~ +π
  float pitch_rad; // 俯仰角
  float roll_rad;  // 滚转角
} pub_imu_data;

typedef struct{
  float Acc_linear;
  float Dec_linear;
  float v_Max;

  float Acc_omega;
  float Dec_omega;
  float w_Max;
} speed_plan;

typedef struct {
  bool chassis_motor1;
  bool chassis_motor2;
  bool chassis_motor3;
  bool chassis_motor4;
  bool picker_yaw_motor;
  bool picker_extend_motor;
  bool weapon_extend_motor;
  bool lift_left_motor;
  bool lift_right_motor;
  bool picker_lift_motor;
  bool weapon_lift_motor;
} pub_motor_status;

typedef struct {
  bool isSending;
} pub_omni_ir_status;

// 发布底盘运动指令
typedef struct {
  float linear_x_;
  float linear_y_;
  float omega_;
} pub_chassis_cmd;

// 底盘速度
typedef struct {
  float vx;
  float vy;
  float w;
} chassis_speed;

// 底盘定位
typedef struct {
  float x;
  float y;
  float yaw;
} chassis_position;

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
  int8_t pump_cmd = 0;     // 0=无操作, 1=开, -1=关
  int8_t valve_cmd = 0;
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
  uint8_t data;
} pub_ir_data;

typedef struct {
  uint8_t tx_data;
} pub_ir_cmd;

typedef struct {
  uint8_t QR_type;
} QR_code_cmd_t;

typedef struct  {
  uint8_t QR_type;
} QR_code_data_t;

#pragma pack()