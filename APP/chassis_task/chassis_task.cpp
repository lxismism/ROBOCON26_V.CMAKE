/**
 * @file chassis_task.cpp
 * @author 大帅将军
 * @brief 底盘任务实现
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "chassis_task.h"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

#include <array>

const FieldSide_t field_side = Left;
const float robot_center_to_gimbal = 0.4f;
const float MC_position_correction_y = 0.03f;
const float robot_center_to_Weapon = 0.4187f;

// 任务句柄
osThreadId_t ChassisTaskHandle;

// 底盘电机实例声明
extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

//将实例移出命名空间以在debug处访问
OmniChassis Omnichassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                chassis_motor4);
namespace {//omni底盘解算器实例
// OmniChassis Omnichassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
//                 chassis_motor4);
// 每个轮子的PID参数配置
const std::array<OmniChassis::SpeedPidParam, OmniChassis::kWheelCount>
    kOmniWheelSpeedPidParams = {
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 左上
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 右上
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 左下
        // OmniChassis::SpeedPidParam(105.0f, 75000.0f, 0.002f, 16000.0f, 0.5f, NONE), // 右下
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左上
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右上
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左下
        // OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右下
        OmniChassis::SpeedPidParam(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 左上
        OmniChassis::SpeedPidParam(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 右上
        OmniChassis::SpeedPidParam(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 左下
        OmniChassis::SpeedPidParam(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT), // 右下
    };

const std::array<OmniChassis::SpeedPidParam, OmniChassis::kWheelCount>
    kOmniWheelAnglePidParams = {
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左上
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右上
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 左下
        OmniChassis::SpeedPidParam(1200.0f, 750.0f, 0.0f, 16000.0f, 0.5f, NONE), // 右下
    };
}

static inline void chassisInit() {
    Omnichassis_solver.configureSpeedPid(kOmniWheelSpeedPidParams);
    Omnichassis_solver.configureAnglePid(kOmniWheelAnglePidParams);
    if (!chassis_cmd_sub.IsValid()) {
        return;
    }
}

void chassisTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    chassisInit();

    for (;;) {

        if (chassis_cmd_sub.TryGet(&chassis_chassis_cmd)) {
        }
        // 无论有没有新命令，都持续执行解算
        Omnichassis_solver.run(chassis_chassis_cmd);
        vTaskDelayUntil(&currentTime, 1);
    }
}

const float robot_position_MF[6][5][4] = {//用于在梅林半自动控制车辆移动
  //[MF_x][MF_y] = {aim_real_position_x, aim_real_position_y, aim_real_position_yaw, 云台高度}
{
        {2.1f*field_side, 0.1f                                      , 0.0f  , 2.0f},
        {2.1f*field_side, 1.3f - field_side*robot_center_to_gimbal, -90.0f*field_side , 2.0f},
        {2.1f*field_side, 2.5f - field_side*robot_center_to_gimbal, -90.0f*field_side , 1.0f},
        {2.1f*field_side, 3.7f - field_side*robot_center_to_gimbal, -90.0f*field_side , 2.0f},
        {2.1f*field_side, 4.9f                                      , 180.0f, 2.0f}

    },
    {
        {3.3f*field_side + robot_center_to_gimbal, 0.1f, 0.0f, 2.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {3.3f*field_side - robot_center_to_gimbal, 4.9f, 180.0f, 2.0f}
    },
    {
        {4.5f*field_side + robot_center_to_gimbal, 0.1f, 0.0f, 1.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {4.5f*field_side - robot_center_to_gimbal, 4.9f, 180.0f, 3.0f}
    },
    {
        {5.7f*field_side + robot_center_to_gimbal, 0.1f, 0.0f, 2.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {5.7f*field_side - robot_center_to_gimbal, 4.9f, 180.0f, 2.0f}
    },
    {
        {6.9f*field_side + robot_center_to_gimbal, 0.1f, 0.0f, 1.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {0.0f*field_side                           , 0.0f, 0.0f, 0.0f},
        {6.9f*field_side - robot_center_to_gimbal, 4.9f, 180.0f, 1.0f}
    },
    {
        {8.1f*field_side, 0.1f                                      , 0.0f  , 2.0f},
        {8.1f*field_side, 1.3f + field_side*robot_center_to_gimbal, 90.0f*field_side, 1.0f},
        {8.1f*field_side, 2.5f + field_side*robot_center_to_gimbal, 90.0f*field_side, 2.0f},
        {8.1f*field_side, 3.7f + field_side*robot_center_to_gimbal, 90.0f*field_side, 1.0f},
        {8.1f*field_side, 4.9f                                      , 180.0f, 2.0f}
    }
};


const float robot_position_Arena[3][3] = {//用于在九宫格自动控制车辆移动
  //[Arena_x] = {aim_real_position_x, aim_real_position_y, aim_real_position_yaw}
    {-0.227f*field_side - field_side*robot_center_to_gimbal, 4.399f - 0.5f, -90.0f*( field_side + 1)/2},

    {-0.727f*field_side + robot_center_to_gimbal           , 4.399f - 0.5f, 0.0f},

    {-1.227f*field_side + field_side*robot_center_to_gimbal, 4.399f - 0.5f, -90.0f*(-field_side + 1)/2}
};

const float robot_position_Arena_withR2[3][3] = {//用于在九宫格自动控制车辆移动
  //[Arena_x] = {aim_real_position_x, aim_real_position_y, aim_real_position_yaw}
    {-0.227f*field_side , 4.399f - 0.5f, 0.0f},

    {-0.727f*field_side , 4.399f - 0.5f, 0.0f},

    {-1.227f*field_side , 4.399f - 0.5f, 0.0f}
};

const float robot_position_Arena_useWeapon[3][3] = {//用于在九宫格自动控制车辆移动
  //[Arena_x] = {aim_real_position_x, aim_real_position_y, aim_real_position_yaw}
    {-0.227f*field_side - robot_center_to_Weapon*(field_side + 1)/2  , 4.399f - 0.5f, 0.0f + 30.0f*(-field_side + 1)/2},

    {-0.727f*field_side - robot_center_to_Weapon , 4.399f - 0.5f, 0.0f},

    {-1.227f*field_side - robot_center_to_Weapon*(-field_side + 1)/2 , 4.399f - 0.5f, 0.0f + 30.0f*(field_side + 1)/2}
};

const float robot_position_MC[4][3] = {//用于在武馆自动控制车辆移动
  //[Arena_x] = {aim_real_position_x, aim_real_position_y, aim_real_position_yaw}
    {0.5f*field_side, 1.8f + robot_center_to_Weapon - MC_position_correction_y, 90.0f*field_side},

    {0.5f*field_side, 2.0f + robot_center_to_Weapon - MC_position_correction_y, 90.0f*field_side},

    {0.5f*field_side, 2.2f + robot_center_to_Weapon - MC_position_correction_y, 90.0f*field_side},

    {0.5f*field_side, 2.4f + robot_center_to_Weapon - MC_position_correction_y, 90.0f*field_side}
};



MF_plan_t MF_plan[15] = {
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};