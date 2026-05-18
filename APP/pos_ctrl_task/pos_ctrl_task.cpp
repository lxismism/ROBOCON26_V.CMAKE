/**
 * @file pos_ctrl_task.cpp
 * @brief 3508电机位置环控制任务实现
 *
 * 数据流：
 *   Xbox → UART3 → uart3RxProcessTask → publish("xbox")
 *     → posCtrlTask 订阅 → 边缘检测LT/RT → 更新目标角度
 *     → PID_Calculate(measure=当前角度, ref=目标角度)
 *     → arm3508_motor.setMotorCmd(pid输出)
 *     → can2SendTask 每1ms打包发送到CAN总线
 */
#include "pos_ctrl_task.hpp"



#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "Motor.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

// ---------- 引用 com_config.cpp 中已存在的全局电机对象 ----------
extern C620Motor arm3508_motor;
extern C610Motor arm2006_motor;



osThreadId_t PosCtrlTaskHandle;


// ---------- 订阅 Xbox 手柄数据 ----------
static TypedTopicSubscriber<pub_Xbox_Data> xbox_sub("xbox", 8);

// ---------- 位置环 PID ----------
static PID_t pos_pid;
static PID_t pos_pid_2006;

// ---------- 控制状态（static 保证在函数多次调用间保持值） ----------
static float target_pos_deg = 0.0f;
static bool last_lt_pressed = false;
static bool last_rt_pressed = false;
static bool pos_inited = false;

static float target_pos_deg_2006 = 0.0f;   // 2006电机目标角度
static bool last_lb_pressed = false;        // LB按键上一次状态
static bool last_rb_pressed = false;        // RB按键上一次状态
static bool pos_inited_2006 = false;        // 2006电机首次初始化标志


// ---------- 常量 ----------
static constexpr uint16_t kTriggerThreshold = 512;
static constexpr float kStepAngle = 60.0f;

static float debug_current_angle = 0.0f;

void posCtrlTask(void *argument) {
  (void)argument;

  // ---- PID 参数初始化 ----
  PID_Init(&pos_pid);
  pos_pid.Kp = 11.5f;
  pos_pid.Ki = 4.0f;
  pos_pid.Kd = 0.61f;
  pos_pid.MaxOut = 8000.0f;
  pos_pid.IntegralLimit = 5000.0f;
  pos_pid.DeadBand = 0.5f;
  pos_pid.Improve = Integral_Limit | Derivative_On_Measurement;

    // 2006电机 PID 参数初始化
  PID_Init(&pos_pid_2006);
  pos_pid_2006.Kp = 8.0f;
  pos_pid_2006.Ki = 2.0f;
  pos_pid_2006.Kd = 0.3f;
  pos_pid_2006.MaxOut = 5000.0f;
  pos_pid_2006.IntegralLimit = 3000.0f;
  pos_pid_2006.DeadBand = 0.5f;
  pos_pid_2006.Improve = Integral_Limit | Derivative_On_Measurement;


  if (!xbox_sub.IsValid()) {
    return;
  }

  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {
    // -------- 步骤1：获取手柄数据，边缘检测 --------
    pub_Xbox_Data xbox_data;
    if (xbox_sub.TryGet(&xbox_data)) {

       // 3508电机：LT增加目标角度，RT减少目标角度
      bool lt_pressed = (xbox_data.trigLT > kTriggerThreshold);
      bool rt_pressed = (xbox_data.trigRT > kTriggerThreshold);

      if (lt_pressed && !last_lt_pressed) {
        target_pos_deg += kStepAngle;
      }
      if (rt_pressed && !last_rt_pressed) {
        target_pos_deg -= kStepAngle;
      }

      last_lt_pressed = lt_pressed;
      last_rt_pressed = rt_pressed;

            // 2006电机：LB增加目标角度，RB减少目标角度
      bool lb_pressed = xbox_data.btnLB;
      bool rb_pressed = xbox_data.btnRB;

      if (lb_pressed && !last_lb_pressed) {
        target_pos_deg_2006 += kStepAngle;
      }
      if (rb_pressed && !last_rb_pressed) {
        target_pos_deg_2006 -= kStepAngle;
      }

      last_lb_pressed = lb_pressed;
      last_rb_pressed = rb_pressed;


    }

    // -------- 步骤2：首次初始化，以当前角度为目标避免跳变 --------
    if (!pos_inited) {
      target_pos_deg = arm3508_motor.getCurrentSumPos();
      pos_inited = true;
    }
        // 2006电机首次上电，以当前位置为目标，避免角度跳变
    if (!pos_inited_2006) {
      target_pos_deg_2006 = arm2006_motor.getCurrentSumPos();
      pos_inited_2006 = true;
    }


    // -------- 步骤3：位置环PID --------
    float current_pos = arm3508_motor.getCurrentSumPos();

    debug_current_angle = current_pos;

    float pid_out = PID_Calculate(&pos_pid, current_pos, target_pos_deg);
    arm3508_motor.setMotorCmd(pid_out);

        // 2006电机位置环PID
    float current_pos_2006 = arm2006_motor.getCurrentSumPos();
    float pid_out_2006 = PID_Calculate(&pos_pid_2006, current_pos_2006, target_pos_deg_2006);
    arm2006_motor.setMotorCmd(pid_out_2006);


    vTaskDelayUntil(&currentTime, 5);
  }
}
