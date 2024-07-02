/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "tool.h"
#include "power_calc.h"


extern referee_info_t referee_info;
static referee_info_t *referee_data; // 用于获取裁判系统的数据

static SuperCapInstance *cap;                                       // 超级电容

// 跟随模式底盘的pid

/* 用于自旋变速策略的时间变量 */
// static float t;

void ChassisInit()
{

    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0X427, // 超级电容默认接收id
            .rx_id      = 0x300, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    PowerControlInit(referee_info.GameRobotState.chassis_power_limit + referee_data->PowerHeatData.chassis_power_buffer * 0.3 - 20, 1.0f / REDUCTION_RATIO_WHEEL);
}