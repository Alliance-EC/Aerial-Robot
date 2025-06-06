/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "dji_motor.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数

#define YAW_ECD_GREATER_THAN_4096 1//1    // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD         2093 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_POS_UP_LIMIT_ECD    1975 // 3750 // 云台竖直方向高处限位编码器值,若对云台有机械改动需要修改
#define PITCH_POS_DOWN_LIMIT_ECD  2549 // 4455 // 云台竖直方向低处限位编码器值,若对云台有机械改动需要修改
#define PITCH_FEED_TYPE           1    // 云台PITCH轴反馈值来源:编码器为0,陀螺仪为1
#define PITCH_INS_FEED_TYPE       1    // 云台PITCH轴陀螺仪反馈:角度值为0,弧度制为1
#define PITCH_ECD_UP_ADD          0    // 云台抬升时编码器变化趋势,增为1,减为0 (陀螺仪变化方向应相同)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 45    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         8     // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE             350   // 320.5   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            350   // 320.5   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           153   // 轮子半径
#define REDUCTION_RATIO_WHEEL  13.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换


// 模拟小电脑负重 652.2
// 其他参数(尽量所有参数集中到此文件)
#define BUZZER_SILENCE 1 // 蜂鸣器静音,1为静音,0为正常

#define IMU_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改传感器参数
#ifndef IMU_DEF_PARAM_WARNING
#define IMU_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !IMU_DEF_PARAM_WARNING

// 陀螺仪校准数据，开启陀螺仪校准后可从INS中获取
#define BMI088_PRE_CALI_GYRO_X_OFFSET -0.00275911391f //-0.00275911391f //-0.00296019088f    // 0.00190102309f    //-0.00221039914f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.00438436493f  //-0.00304090488f//-0.00350466394f     //-0.00241432083f // 0.00354450056f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.00304090488f // 0.00438436493f  // 0.00435080705f      // 0.00466383668f   // 0.000225723968f
// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f
// 设置陀螺仪数据相较于云台的yaw,pitch,roll的方向
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                 \
        {0.0f, 0.0f, 1.0f},              \
    {                                    \
        0.0f, -1.0f, 0.0f                 \
    }

#define INS_YAW_ADDRESS_OFFSET   2 // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 1 // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  0 // 陀螺仪数据相较于云台的roll的方向

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;


// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_RC_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_PC_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_VISION_MODE,
} gimbal_mode_e;

//自瞄状态设置
typedef enum {
    AutoShooting_Off = 0,    // 自瞄关闭
    AutoShooting_Open,            // 自瞄开启
    AutoShooting_Find,         // 自瞄识别到目标
} auto_shoot_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
    FRICTION_REVERSE, // 摩擦轮倒转，防塞弹
} friction_mode_e;

typedef enum {
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum {
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;



/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float yaw_last;
    float pitch_last;
    int gimbal_yaw_max;
    int gimbal_yaw_min;
    int gimbal_pitch_max;
    int gimbal_pitch_min;
    gimbal_mode_e gimbal_mode;
    float delta_yaw;
    float delta_pitch;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{ 
    gimbal_mode_e gimbal_mode;
    DJIMotorInstance * yaw_motor;
    DJIMotorInstance *pitch_motor;
    DJIMotorInstance *loader_motor;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    float yaw_limit;
    float pitch_limit;
    int8_t fly_mode;
    int8_t delete_warnning;
} Ui_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
        Bullet_Speed_limit_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    
    uint32_t set_shoot_count;//允许发弹量
    
    float delta_angle_abs;//开始刹车的区间
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */



typedef struct
{
    INS_Instance *gimbal_imu_data;
    DJIMotorInstance * yaw_motor;
    DJIMotorInstance *pitch_motor;

    float yaw_init_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
    DJIMotorInstance *loader_motor;
} Shoot_Upload_Data_s;

typedef struct
{ 
    gimbal_mode_e gimbal_mode;
    DJIMotorInstance * yaw_motor;
    DJIMotorInstance *pitch_motor;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
} Ui_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H