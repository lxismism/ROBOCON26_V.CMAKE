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
#include "topic_pool.h"
#include "topics.hpp"

#include <array>

// 任务句柄
osThreadId_t ChassisTaskHandle;

// 底盘电机实例声明
extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

// 底盘解算器实例声明
namespace {
OmniChassis chassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                           chassis_motor4);

const std::array<OmniChassis::SpeedPidParam, OmniChassis::kWheelCount>
    kWheelPidParams = {
    OmniChassis::SpeedPidParam(100.0f, 0.01f, 0.05f, 8000.0f, 0.5f, NONE),
    OmniChassis::SpeedPidParam(100.0f, 0.01f, 0.05f, 8000.0f, 0.5f, NONE),
    OmniChassis::SpeedPidParam(100.0f, 0.01f, 0.05f, 8000.0f, 0.5f, NONE),
    OmniChassis::SpeedPidParam(100.0f, 0.01f, 0.05f, 8000.0f, 0.5f, NONE),
};
} // namespace

static inline void chassisInit() {
    chassis_solver.configureSpeedPid(kWheelPidParams);

    if (!chassis_cmd_sub.IsValid()) {
        return;
    }
}

void chassisTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    chassisInit();

    for (;;) {
        if (chassis_cmd_sub.TryGet(&chassis_chassis_cmd)) {
            // Process the received chassis command
        }
        chassis_solver.run(chassis_chassis_cmd);
        vTaskDelayUntil(&currentTime, 5);
    }
}
