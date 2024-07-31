#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#define YAW_ANGLE_MAX   40
#define YAW_ANGLE_MIN   -60
#define PITCH_ANGLE_MAX -11
#define PITCH_ANGLE_MIN -32
#define YAW_MID_ANGLE   33.5
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H