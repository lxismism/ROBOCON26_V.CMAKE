/**
 * @file pos_ctrl_task.hpp
 * @brief 电机位置环控制任务
 *
 * 订阅Xbox手柄数据，通过边缘检测LT/RT按键，
 * 累加目标角度，用单级PID（位置→电流）控制3508电机转到目标角度。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void posCtrlTask(void *argument);

#ifdef __cplusplus
}
#endif
