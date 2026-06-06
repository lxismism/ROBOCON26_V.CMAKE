/**
 * @file chassis_task.h
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

void chassisTask(void *argument);

typedef struct {
    uint8_t MF_x;
    uint8_t MF_y;
    bool is_picking;
    bool is_valid;
} MF_plan_t;

extern const float robot_position_MC[4][3];
extern const float robot_position_MF[6][5][4];
extern const float robot_position_Arena[3][3];
extern MF_plan_t MF_plan[10] ;

