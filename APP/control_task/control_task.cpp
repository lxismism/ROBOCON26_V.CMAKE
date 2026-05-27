/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief 控制任务 —— Xbox按键 → 底盘 + 上身控制指令决策层
 * @version 0.2
 * @date 2026-05-20
 *
 * @copyright Copyright (c) 2026
 *
 * @attention btnSelect 切换模式：0=队友模式(现有逻辑), 1=用户模式(上身按键映射)
 * @note 持续型按键每帧累加步进量；切换型按键仅上升沿触发
 * @versioninfo v0.2: 新增上身控制指令 + btnSelect模式切换
 */

#include "control_task.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>

osThreadId_t ControlTaskHandle;

// ---------- 发布者 ----------
static TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
static pub_chassis_cmd xbox_cmd{};

static TypedTopicPublisher<pub_upbody_cmd> upbody_cmd_pub("upbody_cmd");
static pub_upbody_cmd upbody_cmd_msg{};

//发布红外控制命令
static TypedTopicPublisher<pub_ir_cmd> ir_cmd_pub("ir_cmd");
static pub_ir_cmd ir_cmd{};

static TypedTopicPublisher<QR_code_cmd_t> qr_code_cmd_pub("qr_code_cmd");
static QR_code_cmd_t qr_code_cmd{};

// ---------- 订阅者 ----------

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};
pub_Xbox_Data control_xbox_cmd_Last{};

static TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};
pub_Position_Data control_position{};
//定位修正参数
static float position_correction_x = 0.0f;
static float position_correction_y = 0.0f;
static const float position_center_distance = 0.28f;

/* 订阅IR_data信息 */
static TypedTopicSubscriber<pub_ir_data> control_ir_sub("ir_data", 8);
pub_ir_data control_ir_msg{};

static TypedTopicSubscriber<QR_code_data_t> qr_code_data_sub("qr_code_data", 8);
QR_code_data_t control_qr_code_data{};



static float xbox_angle_deg;
static float v_aim;
  
static bool headless_xy_mode = true;//手操模式与定位模式切换，默认手操
static bool headless_omega_mode = true;//手操模式与定位模式切换，默认手操

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
static PID_t linear{.Kp = 1.68f,.Ki = 0.11f,.Kd = 0.0f,.MaxOut = 0.75*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
static PID_t deg{.Kp = 1.70f,.Ki = 0.32f,.Kd = 0.000001f,.MaxOut = MAX_VELOCITY_ANGULAR*0.5*180.0/M_PI,.DeadBand = 0.3f,.Improve = NONE};

// 摇杆常量
static constexpr uint16_t kJoyCenter = 32767;
static constexpr uint16_t kJoyDeadZoneLeft = 3500;
static constexpr uint16_t kJoyDeadZoneRight = 2000;
static constexpr float kDegToRad = M_PI / 180.0f;

// 上身控制常量
static constexpr float kUpbodyStep = 1.0f;           // 持续按键每帧步进 (°/帧)
static constexpr float kPickExtendStep = 0.3f; // 吸取手伸缩步进 (°/帧) ，减速比大，需慢速
static constexpr uint16_t kTriggerThreshold = 512;   // 扳机触发阈值

static int8_t sign(double value) {
    if (value > 0) return 1;
    else if (value < 0) return -1;
    else return 0;
}

// 处理xbox数据 → 底盘控制指令（队友模式/无头模式用）
void Xbox_Data_Process()
{
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

  {
    if(control_xbox_cmd.btnLS != control_xbox_cmd_Last.btnLS)
    {
      if(control_xbox_cmd.btnLS == true){
        headless_xy_mode = !headless_xy_mode;
      }
    }
    
    //切换omega模式（按下右摇杆RS切换）
    if(control_xbox_cmd.btnRS != control_xbox_cmd_Last.btnRS)
    {
      if(control_xbox_cmd.btnRS == true){
        headless_omega_mode = !headless_omega_mode;
      }
    }

    if(headless_xy_mode)
    {
      xbox_angle_deg = atan2(xbox_cmd.linear_y_,xbox_cmd.linear_x_)/kDegToRad;
      v_aim = sqrt(xbox_cmd.linear_x_ * xbox_cmd.linear_x_ + xbox_cmd.linear_y_ * xbox_cmd.linear_y_);

      robot_v_aim_cmd.linear_x_ = v_aim * cos((xbox_angle_deg - control_position.yaw)*kDegToRad);
      robot_v_aim_cmd.linear_y_ = v_aim * sin((xbox_angle_deg - control_position.yaw)*kDegToRad);

      state_aim_cmd.linear_x_ = control_position.x;
      state_aim_cmd.linear_y_ = control_position.y;
    }
    else
    {
      // 定位模式：摇杆负责里程计误差修正，方向键控制目标位置，进入目标位置环PID
      if(control_xbox_cmd.btnY == 1){//按Y复位回零点
        if(control_xbox_cmd_Last.btnY == 0){
          state_aim_cmd.linear_x_ = 0.0f;
          state_aim_cmd.linear_y_ = 0.0f;
        }
      }
      else{
        //上下控制
        if(control_xbox_cmd.btnDirUp == 1){
          if(control_xbox_cmd_Last.btnDirUp == 0){
            state_aim_cmd.linear_y_ = state_aim_cmd.linear_y_ + 1.2f;
          }
        }else if(control_xbox_cmd.btnDirDown == 1){
          if(control_xbox_cmd_Last.btnDirDown == 0){
            state_aim_cmd.linear_y_ = state_aim_cmd.linear_y_ - 1.2f;
          }
        }
        
        //左右控制
        if(control_xbox_cmd.btnDirLeft == 1){
          if(control_xbox_cmd_Last.btnDirLeft == 0){
            state_aim_cmd.linear_x_ = state_aim_cmd.linear_x_ - 1.2f;
          }
        }else if(control_xbox_cmd.btnDirRight == 1){
          if(control_xbox_cmd_Last.btnDirRight == 0){
            state_aim_cmd.linear_x_ = state_aim_cmd.linear_x_ + 1.2f;
          }
        }
      }

      //里程计定位修正
      position_correction_x = position_correction_x + 0.001f*xbox_cmd.linear_x_;
      position_correction_y = position_correction_y + 0.001f*xbox_cmd.linear_y_;

      //计算目标位置与当前实际位置的误差，进入PID
      error_x            = state_aim_cmd.linear_x_ - control_position.x;
      error_y            = state_aim_cmd.linear_y_ - control_position.y;
      state_xy_error     = sqrt(error_x*error_x + error_y*error_y);
      state_xy_angle_deg = atan2(error_y,error_x)/kDegToRad;
      xy_pid_output      = PID_Calculate(&linear,0.0f,state_xy_error);
      robot_v_aim_cmd.linear_x_  = xy_pid_output * cos((state_xy_angle_deg - control_position.yaw)*kDegToRad);
      robot_v_aim_cmd.linear_y_  = xy_pid_output * sin((state_xy_angle_deg - control_position.yaw)*kDegToRad);
    }

    if(headless_omega_mode)
    {
      robot_v_aim_cmd.omega_ = xbox_cmd.omega_;
      state_aim_cmd.omega_ = control_position.yaw ;
    }
    else
    {
      // // 摇杆归零，进入目标角度环PID
      // if(control_xbox_cmd.btnY == 1){
      //   state_aim_cmd.omega_ = 0.0f;
      // }else if(control_xbox_cmd.btnX == 1){
      //   state_aim_cmd.omega_ = -90.0f;
      // }else if(control_xbox_cmd.btnA == 1){
      //   state_aim_cmd.omega_ = 180.0f;
      // }else if(control_xbox_cmd.btnB == 1){
      //   state_aim_cmd.omega_ = 90.0f;
      // }
      //通过右摇杆控制车辆执行4种朝向（0，90，180，-90）
      if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > 30000)
      {
        if((control_xbox_cmd.joyRHori - kJoyCenter) > 0){
          state_aim_cmd.omega_ = -90.0f;
        }else{
          state_aim_cmd.omega_ = 90.0f;
        }
      }else if (ABS(control_xbox_cmd.joyRVert - kJoyCenter) > 30000)
      {
        if((control_xbox_cmd.joyRVert - kJoyCenter) > 0){
          state_aim_cmd.omega_ = 180.0f;
        }else{
          state_aim_cmd.omega_ = 0.0f;
        }
      }

      //计算角度误差并进行PID控制
      float error_dir = state_aim_cmd.omega_ - control_position.yaw;
      if (fabs(error_dir) < 1.5f){
        error_dir = 0.0f;
      }else if(fabs(error_dir) > 180.0 ){
        if(error_dir > 0) error_dir = error_dir - 360.0 ;
        else error_dir = error_dir + 360.0 ;
      }

      robot_v_aim_cmd.omega_ = kDegToRad*PID_Calculate(&deg,control_position.yaw,control_position.yaw + error_dir);
    }

  }
  
  // ===== 第3步：保留本次xbox数据 =====
  control_xbox_cmd_Last = control_xbox_cmd;

}

void controlInit() {
    if (!chassis_data_pub.IsValid()) {
        return;
    }
    if (!upbody_cmd_pub.IsValid()) {
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
    if(!qr_code_cmd_pub.IsValid()) {
        return;
    }
    if(!qr_code_data_sub.IsValid()) {
        return;
    }
}

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    PID_Init(&linear);
    PID_Init(&deg);

    // 模式状态
    static bool control_mode = false;    // false=队友模式, true=用户模式
    static bool last_select = false;     // btnSelect上一帧状态

    // 切换型按键上一帧状态（边缘检测用）
    static bool last_btnX = false;
    static bool last_btnB = false;
    static bool last_btnLS = false;
    static bool last_btnRS = false;

    controlInit();
    for (;;) {
        /* 从Position订阅者中获取数据 */
        if (control_position_sub.TryGet(&control_position_msg)) {
            control_position.frame_id = control_position_msg.frame_id;
            control_position.yaw = -control_position_msg.yaw;
            control_position.yaw_speed = control_position_msg.yaw_speed;
            control_position.x = -control_position_msg.x + position_center_distance*sin(control_position.yaw*kDegToRad) - position_correction_x ;
            control_position.y =  control_position_msg.y - position_center_distance*cos(control_position.yaw*kDegToRad) - position_correction_y ;
        }

        /* 从xbox数据订阅者中获取数据 */
        if (control_xbox_sub.TryGet(&control_xbox_cmd)) {

            // ===== btnSelect 模式切换（上升沿触发） =====
            if (control_xbox_cmd.btnSelect && !last_select) {
                control_mode = !control_mode;
            }
            last_select = control_xbox_cmd.btnSelect;

            // ==========================================
            //  用户模式 (mode = 1)
            // ==========================================
            if (control_mode) {

                // ---- 底盘：摇杆直驱 ----
                if (ABS(control_xbox_cmd.joyLHori - kJoyCenter) > kJoyDeadZoneLeft)
                {
                    robot_v_aim_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - kJoyCenter - sign(control_xbox_cmd.joyLHori - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
                }
                else { robot_v_aim_cmd.linear_x_ = 0.0f; }

                if (ABS(control_xbox_cmd.joyLVert - kJoyCenter) > kJoyDeadZoneLeft)
                {
                    robot_v_aim_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - kJoyCenter - sign(control_xbox_cmd.joyLVert - kJoyCenter)*kJoyDeadZoneLeft) / ((float)(kJoyCenter - kJoyDeadZoneLeft)) * MAX_VELOCITY_LINEAR;
                }
                else { robot_v_aim_cmd.linear_y_ = 0.0f; }

                if (ABS(control_xbox_cmd.joyRHori - kJoyCenter) > kJoyDeadZoneRight)
                {
                    robot_v_aim_cmd.omega_ = -(int)(control_xbox_cmd.joyRHori - kJoyCenter - sign(control_xbox_cmd.joyRHori - kJoyCenter)*kJoyDeadZoneRight) / ((float)(kJoyCenter - kJoyDeadZoneRight)) * MAX_VELOCITY_ANGULAR;
                }
                else { robot_v_aim_cmd.omega_ = 0.0f; }
                // 无头旋转：场地坐标系 → 车身坐标系
                float joy_angle_deg = atan2(robot_v_aim_cmd.linear_y_, robot_v_aim_cmd.linear_x_) / kDegToRad;
                float joy_mag = sqrt(robot_v_aim_cmd.linear_x_ * robot_v_aim_cmd.linear_x_ + robot_v_aim_cmd.linear_y_ * robot_v_aim_cmd.linear_y_);
                robot_v_aim_cmd.linear_x_ = joy_mag * cos((joy_angle_deg - control_position.yaw) * kDegToRad);
                robot_v_aim_cmd.linear_y_ = joy_mag * sin((joy_angle_deg - control_position.yaw) * kDegToRad);

                chassis_data_pub.Publish(robot_v_aim_cmd);

                // ---- 上身控制指令 ----
                upbody_cmd_msg = {};
                upbody_cmd_msg.active = true;


                // ▼ 持续型：trigLT电梯上升 / trigRT电梯下降
                if (control_xbox_cmd.trigLT > kTriggerThreshold)
                    upbody_cmd_msg.lift_delta = kUpbodyStep;
                if (control_xbox_cmd.trigRT > kTriggerThreshold)
                    upbody_cmd_msg.lift_delta = -kUpbodyStep;

                // ▼ 持续型：btnLB武器手上升 / btnRB武器手下降
                if (control_xbox_cmd.btnLB)
                    upbody_cmd_msg.weapon_lift_delta = kUpbodyStep;
                if (control_xbox_cmd.btnRB)
                    upbody_cmd_msg.weapon_lift_delta = -kUpbodyStep;
                  
                // ▼ 持续型：btnDirUp吸取手上升 / btnDirDown吸取手下降
                if (control_xbox_cmd.btnDirUp)
                    upbody_cmd_msg.pick_lift_delta = kUpbodyStep;
                if (control_xbox_cmd.btnDirDown)
                    upbody_cmd_msg.pick_lift_delta = -kUpbodyStep;

                // ▼ 持续型：btnDirLeft云台逆时针 / btnDirRight云台顺时针
                if (control_xbox_cmd.btnDirLeft)
                    upbody_cmd_msg.pick_yaw_delta = kUpbodyStep;
                if (control_xbox_cmd.btnDirRight)
                    upbody_cmd_msg.pick_yaw_delta = -kUpbodyStep;

                // ▼ 持续型：右摇杆前推吸取手伸 / 后拉吸取手缩
                {
                    int32_t rvert_diff = (int32_t)control_xbox_cmd.joyRVert - (int32_t)kJoyCenter;
                    if (rvert_diff > (int32_t)kJoyDeadZoneRight)
                        upbody_cmd_msg.pick_extend_delta = -kPickExtendStep;
                    else if (rvert_diff < -(int32_t)kJoyDeadZoneRight)
                        upbody_cmd_msg.pick_extend_delta = kPickExtendStep;
                }

                // ▼ 持续型：btnY武器手伸 / btnA武器手缩
                if (control_xbox_cmd.btnY)
                    upbody_cmd_msg.weapon_extend_delta = kUpbodyStep;
                if (control_xbox_cmd.btnA)
                    upbody_cmd_msg.weapon_extend_delta = -kUpbodyStep;

                // ▼ 切换型：btnX吸盘真空泵（上升沿触发）
                if (control_xbox_cmd.btnX && !last_btnX)
                    upbody_cmd_msg.pump_toggle = true;
                last_btnX = control_xbox_cmd.btnX;

                // ▼ 切换型：btnB吸盘电磁阀（上升沿触发）
                if (control_xbox_cmd.btnB && !last_btnB)
                    upbody_cmd_msg.valve_toggle = true;
                last_btnB = control_xbox_cmd.btnB;

                // ▼ 切换型：btnLS夹爪开合（上升沿触发）
                if (control_xbox_cmd.btnLS && !last_btnLS)
                    upbody_cmd_msg.claw_toggle = true;
                last_btnLS = control_xbox_cmd.btnLS;

                // ▼ 切换型：btnRS腕部舵机翻转（上升沿触发）
                if (control_xbox_cmd.btnRS && !last_btnRS)
                    upbody_cmd_msg.wrist_toggle = true;
                last_btnRS = control_xbox_cmd.btnRS;

                upbody_cmd_pub.Publish(upbody_cmd_msg);

            // ==========================================
            //  队友模式 (mode = 0)：现有逻辑不变
            // ==========================================
            } else {
                Xbox_Data_Process();
                chassis_data_pub.Publish(robot_v_aim_cmd);
            }
        }

        if(control_ir_sub.TryGet(&control_ir_msg)) {
          //红外信号处理放这
          //测试
          // if(control_ir_msg.data1 == 0x2B && control_ir_msg.data2 == 0xFC)
          // {
          //   ir_cmd.tx_data[0] = 0x67; //示例：接收到特定红外信号后，发送0x01命令
          //   ir_cmd.tx_data[1] = 0x78; //示例：接收到特定红外信号后，发送0x02命令
          //   ir_cmd_pub.Publish(ir_cmd);
          // }
        }

        //if(qr_code_data_sub.TryGet(&control_qr_code_data)) {
          //二维码输入数据处理放这
          //测试
          //示例：接收到0x01二维码数据后，发布显示0x01二维码命令
          // if(control_qr_code_data.QR_type == 0x01)
          // {
          //   qr_code_cmd.QR_type = 0x01;            //<-
          //   qr_code_cmd_pub.Publish(qr_code_cmd);  //<-发布显示0x01二维码命令核心代码
          // }
        //}

        vTaskDelayUntil(&currentTime, 5);
    }
}
