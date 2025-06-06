/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "referee_init.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "user_lib.h"

#include "bsp_log.h"
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "referee_UI.h"
#include "UI.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "motor_task.h"
float ins_dt = 0, ui_dt = 0;
float cmd_dt = 0;
//! 任务直接在cubeMX中配置,不再使用这种方式
__attribute__((noreturn)) void StartINSTASK(void *argument) // 陀螺仪任务
{
    UNUSED(argument);
    static uint32_t ins_time;

    LOGINFO("[freeRTOS] INS Task Start");
    while (1) {
        INS_Task();
        ins_dt = 1000 * DWT_GetDeltaT(&ins_time);
        if (ins_dt > 1.2f)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]ms");
        LOGERROR("StartINSTASK Task Running");
        osDelay(1);
    }
}

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{

    static uint32_t cmd_time;
    // static float cmd_dt;
    for (;;) {
        cmd_dt = 1000 * DWT_GetDeltaT(&cmd_time);
        if (cmd_dt > 1.2f) LOGERROR("[freeRTOS] CMD Task is being DELAY! dt = []ms");
        RobotCMDTask();
        LOGERROR("_RobotCMDTask Task Running");
        osDelay(5);
    }
}

__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    for (;;) {
        GimbalTask();
        LOGERROR("_GimbalTask Task Running");
        osDelay(1);
    }
}

__attribute__((noreturn)) void _ShootTask(void *argument)
{
    for (;;) {
        ShootTask();
        LOGERROR("_ShootTask Task Running");
        osDelay(5);
    }
}

__attribute__((noreturn)) void motorControlTask(void *argument)
{
    for (;;) {
        MotorControlTask();
        LOGERROR("motorControlTask Task Running");
        osDelay(1);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    for (;;) {
        DaemonTask();
        LOGERROR("_DaemonTask Task Running");
        osDelay(1);
    }
}

__attribute__((noreturn)) void _My_UIGraphRefresh(void *argument)
{
    for (;;) {
        My_UIGraphRefresh();
        LOGERROR("UI Task Running");
        osDelay(50);
    }
}
