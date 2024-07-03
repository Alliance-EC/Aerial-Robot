#include "UI.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "super_cap.h"

extern referee_info_t referee_info;                         // 裁判系统数据
extern Referee_Interactive_info_t Referee_Interactive_info; // 绘制UI所需的数据

uint8_t UI_Seq;                           // 包序号，供整个referee文件使用
static Graph_Data_t UI_shoot_dot[10];     // 射击准线
static Graph_Data_t UI_Deriction_line[4]; // 射击准线
static Graph_Data_t UI_Energy[3];         // 电容能量条
Graph_Data_t UI_Rectangle[10];            // 矩形
static Graph_Data_t UI_Circle_t[10];      // 圆形
static Graph_Data_t UI_Arco_t[10];        // 圆弧
static Graph_Data_t UI_Number_t[10];      // 数字
static String_Data_t UI_State_sta[10];    // 机器人状态,静态只需画一次
// static String_Data_t UI_State_dyn[6];							// 机器人状态,动态先add才能change

static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

uint8_t Super_condition;    // 超电的开关状态
float Super_condition_volt; // 超电的电压

float Pitch_Angle; // pitch角度（角度制)
float Yaw_Angle;   // yaw角度（角度制）

extern auto_shoot_mode_e AutoShooting_flag; // 自动射击标志位


/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color       = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Cilent_ID         = 0x0100 + referee_info.GameRobotState.robot_id; // 计算客户端ID
    referee_info.referee_id.Robot_ID          = referee_info.GameRobotState.robot_id;          // 计算机器人ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

char Send_Once_Flag = 0; // 初始化标志
uint32_t Rect_De[4] = {1540, 555, 1660, 645};
int16_t AIM_Rect_X, AIM_Rect_Y; // 自瞄框中心点的坐标信息
int16_t AIM_Rect_half_length = 50;
int16_t AIM_Rec_Color;

void My_UIGraphRefresh()
{
    DeterminRobotID();
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // const float arc = 45.0f; // 弧长
    // const uint16_t Mechangle_offset = 10546;
    // float mid_point_angle = fmod(720.0f - (yaw_angle - YAW_CHASSIS_ALIGN_ECD) * (360.0f / 8192.0f), 360.0f);
    // float angle_start = fmod(mid_point_angle + 360.0f - arc / 2.0f, 360.0f);
    // float angle_end = fmod(mid_point_angle + arc / 2.0f, 360.0f);

    if (Send_Once_Flag == 0) {
        Send_Once_Flag = 1;
        // 清空UI          GameRobotState
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 6);
        // 射击准点
        // 射击线
        //  UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH/2,SCREEN_WIDTH/2,SCREEN_LENGTH/2,SCREEN_WIDTH/2-500);
        //  UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-80,SCREEN_WIDTH/2-90,SCREEN_LENGTH/2+80,SCREEN_WIDTH/2-90);
        //  UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-40,SCREEN_WIDTH/2-220,SCREEN_LENGTH/2+40,SCREEN_WIDTH/2-220);
        //  UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 + 20, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 210);
        //  UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-90,SCREEN_WIDTH/2-40,SCREEN_LENGTH/2+90,SCREEN_WIDTH/2-40);
        //  UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-70,SCREEN_WIDTH/2-120,SCREEN_LENGTH/2+70,SCREEN_WIDTH/2-120);

        // 自瞄指示圈
        UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_ADD, 9, UI_Color_White, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250);
        // 位置标定线
        UILineDraw(&UI_Deriction_line[0], "sq0", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22 + 30, SCREEN_WIDTH / 2 - 47, SCREEN_LENGTH / 2 - 22 + 5, SCREEN_WIDTH / 2 - 47);
        UILineDraw(&UI_Deriction_line[1], "sq1", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 + 30, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 + 5);
        UILineDraw(&UI_Deriction_line[2], "sq2", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22 - 5, SCREEN_WIDTH / 2 - 47, SCREEN_LENGTH / 2 - 22 - 30, SCREEN_WIDTH / 2 - 47);
        UILineDraw(&UI_Deriction_line[3], "sq3", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 - 5, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 - 30);
        // 小陀螺
        // sprintf(UI_State_sta[0].show_Data,"Rotate");
        // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 660, 100, "Rotate");
        // UICharRefresh(&referee_info.referee_id,UI_State_sta[0]);
        UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 20, 700, 160, 8);
        // 摩擦轮
        //  sprintf(UI_State_sta[2].show_Data,"Fric");
        //  UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 1160,100, "Fric");
        //  UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
        UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 20, 1180, 160, 8);  // 摩擦轮是否开启显示
        UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_ADD, 9, UI_Color_Orange, 20, 1280, 160, 8); // 摩擦轮是否正常显示
        // 电容
        // 显示电容是否正常开启
        UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_ADD, 9, UI_Color_White, 20, 580, 160, 8);

        sprintf(UI_State_sta[3].show_Data, "SuperCap");
        UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 80, 800, "SuperCap");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);
        UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_White, 4, 80, 700, 480, 740);
        //UILineDraw(&UI_Energy[1], "sn1", UI_Graph_ADD, 9, UI_Color_Green, 20, 80, 720, (uint32_t)((Super_condition_volt * Super_condition_volt - 144) / 532 * 400 + 80), 720); // 超电电压在12V-26V之间
        // 初始自瞄框
        /*
        if((NUC_Data.yaw_offset==0) & (NUC_Data.pit_offset==0))
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_ADD,9,(int)AIM_Rec_Color+1,2,960,540,961,541);
        }
        else
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_ADD,9,(int)AIM_Rec_Color+1,2,AIM_Rect_X-AIM_Rect_half_length,AIM_Rect_Y-AIM_Rect_half_length,AIM_Rect_X+AIM_Rect_half_length,AIM_Rect_Y+AIM_Rect_half_length);
        }
        */

        // 云台朝向圆弧
        // if (mid_point_angle > 360.0f - arc / 2.0f || mid_point_angle < arc / 2.0f)
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_ADD, 8, UI_Color_Green, angle_start, 360, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_ADD, 8, UI_Color_Green, 0, angle_end, 8, 960, 540, 100, 100);
        // }
        // else
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_ADD, 8, UI_Color_Green, angle_start, mid_point_angle, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_ADD, 8, UI_Color_Green, mid_point_angle, angle_end, 8, 960, 540, 100, 100);
        // }
        // pitch角度
        sprintf(UI_State_sta[4].show_Data, "Pitch");
        UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 300, 700, "Pitch");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

        UIFloatDraw(&UI_Number_t[0], "sm1", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 5, 3, 300 + 100, 700, Pitch_Angle * 1000);

        // 射击准点
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Deriction_line[0], UI_Deriction_line[1], UI_Deriction_line[2], UI_Deriction_line[3], UI_State_sta[0], UI_State_sta[2], UI_State_sta[4]);
        // 将位置标定线，小陀螺，弹舱盖，摩擦轮，电容一共7个图形打包一块发
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Deriction_line[0], UI_Deriction_line[1], UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2], UI_State_sta[5], UI_Rectangle[1]);
        // UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0],UI_Circle_t[2],UI_Circle_t[3],UI_Rectangle[1],&UI_Energy[1]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Circle_t[3], UI_Circle_t[4], UI_Energy[1], UI_Number_t[0], UI_Circle_t[5]);

    }

    else {
        // 自瞄指示圈
        if (AutoShooting_flag == AutoShooting_Find) {
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_Green, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250);
        } else if (AutoShooting_flag == AutoShooting_Open) {
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_Orange, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250);
        } else {
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_White, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250);
        }
        // 底盘模式
        if (Referee_Interactive_info.chassis_mode == CHASSIS_ROTATE) {
            UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_Green, 20, 700, 160, 8);
        } else if (Referee_Interactive_info.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_White, 20, 700, 160, 8);
        }
        // 摩擦轮
        // if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
        // {
        //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 20, 1180, 160, 8);
        // } else if (Referee_Interactive_info.friction_mode == FRICTION_OFF) // 摩擦轮关闭
        // {
        //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 20, 1180, 160, 8);
        // }
        if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
        {
            UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 20, 1180, 160, 8);
        } else // 摩擦轮关闭
        {
            UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 20, 1180, 160, 8);
        }

        if (Referee_Interactive_info.shoot_mode == SHOOT_ON) // 发射模式开启
        {
            UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Green, 20, 1280, 160, 8);
        } else if (Referee_Interactive_info.shoot_mode == SHOOT_OFF) // 发射模式关闭
        {
            UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Orange, 20, 1280, 160, 8);
        }

        // 电容是否工作
        if (Super_condition == 0) {
            UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_Green, 20, 580, 160, 8);
        } else {
            UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 20, 580, 160, 8);
        }

        // 电容
        UILineDraw(&UI_Energy[1], "sn1", UI_Graph_Change, 9, UI_Color_Green, 20, 80, 720, (uint32_t)((Super_condition_volt * Super_condition_volt - 144) / 532 * 200 + 80), 720); // 超电电压在12V-26V之间

        UIFloatDraw(&UI_Number_t[0], "sm1", UI_Graph_Change, 9, UI_Color_Yellow, 20, 5, 3, 300 + 100, 700, (uint32_t)(Pitch_Angle * 1000));
        // 云台朝向圆弧,中供弹暂时不需要
        // if (mid_point_angle > 360.0f - arc / 2.0f || mid_point_angle < arc / 2.0f)
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_Change, 8, UI_Color_Green, angle_start, 360, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_Change, 8, UI_Color_Green, 0, angle_end, 8, 960, 540, 100, 100);
        // }
        // else
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_Change, 8, UI_Color_Green, angle_start, mid_point_angle, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_Change, 8, UI_Color_Green, mid_point_angle, angle_end, 8, 960, 540, 100, 100);
        // }
        // 动态自瞄框
        /*等待自瞄接口
        if((NUC_Data.yaw_offset==0) & (NUC_Data.pit_offset==0))
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_Change,9,(int)AIM_Rec_Color+1,2,960,540,961,541  );
        }
            else
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_Change,9,(int)AIM_Rec_Color+1,2,AIM_Rect_X-AIM_Rect_half_length,AIM_Rect_Y-AIM_Rect_half_length,AIM_Rect_X+AIM_Rect_half_length,AIM_Rect_Y+AIM_Rect_half_length);
        }
        */

        // 发送4个指示圈+超电剩余电压条
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Circle_t[3], UI_Circle_t[4], UI_Number_t[0], UI_Circle_t[5], UI_Energy[1]);
    }
}
