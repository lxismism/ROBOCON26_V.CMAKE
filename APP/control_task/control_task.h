/**
 * @file control_task.h
 * @author 大帅将军
 * @brief
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "topics.hpp"
#include "topic_pool.h"

#define MAX_VELOCITY_LINEAR 1.5f // 最大速度，单位m/s，根据实际情况调整
#define MAX_VELOCITY_ANGULAR 3.14f // 最大速度，单位rad/s，根据实际情况调整

extern const FieldSide_t field_side;
extern const float robot_center_to_gimbal_x;

void controlInit();

void controlTask(void *argument);
