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
#include "robot_def.h"
#include "UI_Interface.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define PITCH_ZERO 30.368
#define YAW_ZERO 11
#define SQRT_2 1.4142135

#define LAST 1

#define Gimbal_Yaw_Float 0
#define Gimbal_Pitch_Float 2

#define Shoot_Friction_Circle_l 4
#define Shoot_Friction_Circle_r 6
#define Shoot_Loader_Circle     8


#define Fan_Circle_l1           12
#define Fan_Circle_r1           14
#define Fan_Circle_l2           16
#define Fan_Circle_r2           18
#define Shoot_Line              20
#define Mode_Rectangle          22
#define Music_Rectangle_1         24
#define Music_Rectangle_2         26
#define Music_Rectangle_3         28

#define LENGTH 40
static Subscriber_t *ui_sub;

static Ui_Ctrl_Cmd_s ui_cmd_recv;                         // 发射反馈信息订阅者


extern referee_info_t referee_info;                         // 裁判系统数据
extern Referee_Interactive_info_t Referee_Interactive_info; // 绘制UI所需的数据

uint8_t UI_Seq;                           // 包序号，供整个referee文件使用
static Graph_Data_t UI[LENGTH];           // 射击准线
static Graph_Data_t UI_Deriction_line[4]; // 射击准线
static Graph_Data_t UI_Energy[3];         // 电容能量条
Graph_Data_t UI_Rectangle[10];            // 矩形
static Graph_Data_t UI_Oval[8];         // 
static Graph_Data_t UI_Circle_t[10];      // 圆形
static Graph_Data_t UI_Arco_t[10];        // 圆弧
static Graph_Data_t UI_Number_t[10];      // 数字
static String_Data_t UI_State_sta[10];    
// static String_Data_t UI_State_dyn[6];							// 机器人状态,动态先add才能change


static uint16_t Gimbal_start_x;
static uint16_t Gimbal_start_y;
static uint16_t shoot_fric_x;
static uint16_t shoot_fric_y;
static uint16_t shoot_loader_x;
static uint16_t shoot_loader_y;
static uint16_t shoot_loader_del_x;
static uint16_t shoot_loader_del_y;
static uint16_t aerial_x;
static uint16_t aerial_y;
static uint16_t mode_start_x;
static uint16_t mode_start_y;

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


void ui_init(){
    ui_sub = SubRegister("ui_cmd", sizeof(Ui_Ctrl_Cmd_s));
    // srand((unsigned int)timeGetTime(NULL));
    // 云台ui绝对位置
    Gimbal_start_x = 300;
    Gimbal_start_y=700;
    // 摩擦轮ui绝对位置
    shoot_fric_x  = 1180;
    shoot_fric_y  = 160;
    //拨弹盘ui绝对位置
    shoot_loader_x=1315;
    shoot_loader_y=760;
    shoot_loader_del_x=10;
    //飞机模型位置
    aerial_x=1580;
    aerial_y=560;
    mode_start_x=960;
    mode_start_y=300;
    DeterminRobotID();
    //飞机模型
}

void My_UIGraphRefresh()
{
    DeterminRobotID();
    SubGetMessage(ui_sub, &ui_cmd_recv);

    if (Send_Once_Flag == 0) {
        Send_Once_Flag = 1;
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 0);

        UICircleDraw(&UI[Fan_Circle_l1], "sc4", UI_Graph_ADD, 8, UI_Color_White, 10, aerial_x, aerial_y, 25);
        UICircleDraw(&UI[Fan_Circle_l2], "sc5", UI_Graph_ADD, 8, UI_Color_White, 10, aerial_x, aerial_y - 120, 25);
        UICircleDraw(&UI[Fan_Circle_r1], "sc6", UI_Graph_ADD, 8, UI_Color_White, 10, aerial_x + 120, aerial_y, 25);
        UICircleDraw(&UI[Fan_Circle_r2], "sc7", UI_Graph_ADD, 8, UI_Color_White, 10, aerial_x + 120, aerial_y - 120, 25);
        UILineDraw(&UI[Shoot_Line], "sq0", UI_Graph_ADD, 6, UI_Color_White, 8, aerial_x + 60, aerial_y - 60, aerial_x + 60, aerial_y + 60);
        // 摩擦轮
        UICircleDraw(&UI[Shoot_Friction_Circle_l], "sc2", UI_Graph_ADD, 7, UI_Color_White, 30, shoot_fric_x, shoot_fric_y, 15);       // 摩擦轮是否开启显示
        UICircleDraw(&UI[Shoot_Friction_Circle_r], "sc3", UI_Graph_ADD, 7, UI_Color_White, 30, shoot_fric_x + 100, shoot_fric_y, 15); // 摩擦轮是否正常显示

        sprintf(UI_State_sta[1].show_Data, "PC");
        UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 7, UI_Color_Cyan, 30, 4, mode_start_x - 200, mode_start_y, "PC");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);

        sprintf(UI_State_sta[0].show_Data, "Vision");
        UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 7, UI_Color_Cyan, 30, 4, mode_start_x, mode_start_y, "Vision");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);
        
        sprintf(UI_State_sta[2].show_Data, "Loader");
        UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 7, UI_Color_Cyan, 30, 4, shoot_loader_x - 200, shoot_loader_y, "Loader");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);

        UIRectangleDraw(&UI[Mode_Rectangle], "rc0", UI_Graph_ADD, 6, UI_Color_White, 8, mode_start_x, mode_start_y, mode_start_x, mode_start_y);
        UIRectangleDraw(&UI[Music_Rectangle_1], "rc1", UI_Graph_ADD, 6, UI_Color_White, 8, mode_start_x, mode_start_y, mode_start_x, mode_start_y);
        UIRectangleDraw(&UI[Music_Rectangle_2], "rc2", UI_Graph_ADD, 6, UI_Color_White, 8, mode_start_x, mode_start_y, mode_start_x, mode_start_y);
        UIRectangleDraw(&UI[Music_Rectangle_3], "rc3", UI_Graph_ADD, 6, UI_Color_White, 8, mode_start_x, mode_start_y, mode_start_x, mode_start_y);

        UICircleDraw(&UI[Shoot_Loader_Circle], "so4", UI_Graph_ADD, 9, UI_Color_White - ui_cmd_recv.load_mode, 10, shoot_loader_x + 40, shoot_loader_y - 16, 25);
        // pitch角度
        sprintf(UI_State_sta[4].show_Data, "Pitch");
        UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 7, UI_Color_Cyan, 30, 4, Gimbal_start_x + 150, Gimbal_start_y, "Pitch");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

        UIFloatDraw(&UI[Gimbal_Pitch_Float], "sm1", UI_Graph_ADD, 7, UI_Color_White, 30, 5, 3, Gimbal_start_x + 300, Gimbal_start_y, (ui_cmd_recv.pitch_motor->measure.total_angle + PITCH_ZERO) * 1000);

        // yaw角度
        sprintf(UI_State_sta[3].show_Data, "Yaw");
        UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 7, UI_Color_Cyan, 30, 4, Gimbal_start_x + 250, Gimbal_start_y + 100, "Yaw");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);

        UIFloatDraw(&UI[Gimbal_Yaw_Float], "sm2", UI_Graph_ADD, 7, UI_Color_White, 30, 5, 3, Gimbal_start_x + 390, Gimbal_start_y + 100, (int)(ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) * 1000);

        UIGraphRefresh(&referee_info.referee_id, 7, UI[0], UI[2], UI[4], UI[6], UI[8], UI[10], UI[12]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI[14], UI[16], UI[18], UI[20], UI[22], UI[24], UI[26]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI[28], UI[30], UI[32], UI[34], UI[36], UI[38], UI[38]);
    }
    else {
        //判断是否需要刷新，若需要刷新，修改operatre为CHANGE，待刷新完成之后再改回ADD
        for (int i = 1; i <= LENGTH;) {
            Is_change(&(UI[i-1]),&(UI[i]));
            i += 2;
        }

        //todo：满七个刷新一次，未满七个的有两种方法：一是凑2，5；二是直接1发。需要计算凑的周期以达到效率最高，发送队列中要有查重，发完之后队列出队
        //MY_High_Refresh();
        
        //pitch
        if (ui_cmd_recv.gimbal_mode == GIMBAL_PC_MODE){
            UIRectangleDraw(&UI[Mode_Rectangle], "rc0", UI_Graph_Change, 6, UI_Color_White, 8, mode_start_x-220, mode_start_y-45, mode_start_x-125, mode_start_y+20);
        }
        else if (ui_cmd_recv.gimbal_mode ==GIMBAL_VISION_MODE){
            UIRectangleDraw(&UI[Mode_Rectangle], "rc0", UI_Graph_Change, 6, UI_Color_White, 8, mode_start_x-10, mode_start_y-45, mode_start_x+180, mode_start_y+20);
        }
        UIRectangleDraw(&UI[Music_Rectangle_1], "rc1", UI_Graph_Change, 6, UI_Color_Cyan, 25, SCREEN_LENGTH - 100 * (rand()%9), SCREEN_WIDTH-240, SCREEN_LENGTH, SCREEN_WIDTH-220);
        UIRectangleDraw(&UI[Music_Rectangle_2], "rc2", UI_Graph_Change, 6, UI_Color_Cyan, 25, SCREEN_LENGTH - 70 * (rand() % 8), SCREEN_WIDTH - 290, SCREEN_LENGTH, SCREEN_WIDTH - 270);
        UIRectangleDraw(&UI[Music_Rectangle_3], "rc3", UI_Graph_Change, 6, UI_Color_Cyan, 25, SCREEN_LENGTH - 50 * (rand() % 10), SCREEN_WIDTH - 340, SCREEN_LENGTH, SCREEN_WIDTH - 320);

        UIFloatDraw(&UI[Gimbal_Pitch_Float], "sm1", UI_Graph_Change, 7, UI_Color_Purplish_red + 4 * ui_cmd_recv.pitch_limit, 30, 5, 3, Gimbal_start_x + 300, Gimbal_start_y, (ui_cmd_recv.pitch_motor->measure.total_angle + PITCH_ZERO) * 1000);
        // yaw角度
        UIFloatDraw(&UI[Gimbal_Yaw_Float], "sm2", UI_Graph_Change, 7, UI_Color_Purplish_red + 4 * ui_cmd_recv.yaw_limit, 30, 5, 3, Gimbal_start_x + 390, Gimbal_start_y + 100, (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) * 1000);

        //摩擦轮
        UICircleDraw(&UI[Shoot_Friction_Circle_l], "sc2", UI_Graph_Change, 7, UI_Color_White - 2 * ui_cmd_recv.friction_mode, 30, shoot_fric_x, shoot_fric_y, 8); // 摩擦轮是否开启显示
        UICircleDraw(&UI[Shoot_Friction_Circle_r], "sc3", UI_Graph_Change, 7, UI_Color_White - 2 * ui_cmd_recv.friction_mode, 30, shoot_fric_x + 100, shoot_fric_y, 8); // 摩擦轮是否正常显示

        UICircleDraw(&UI[Shoot_Loader_Circle], "so4", UI_Graph_Change, 9, UI_Color_White - ui_cmd_recv.load_mode, 10, shoot_loader_x + 40, shoot_loader_y -16,25);

        UICircleDraw(&UI[Fan_Circle_l1], "sc4", UI_Graph_Change, 8, UI_Color_Cyan, 10, aerial_x + 60 + 60 *cos(0.25 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), aerial_y - 60 + 60 * sin(0.25 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), 25);
        UICircleDraw(&UI[Fan_Circle_l2], "sc5", UI_Graph_Change, 8, UI_Color_Cyan, 10, aerial_x + 60 + 60 * cos(0.75 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), aerial_y - 60 + 60 * sin(0.75 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), 25);
        UICircleDraw(&UI[Fan_Circle_r1], "sc6", UI_Graph_Change, 8, UI_Color_Purplish_red, 10, aerial_x + 60 + 60 * cos(1.25 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), aerial_y - 60 + 60 * sin(1.25 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), 25);
        UICircleDraw(&UI[Fan_Circle_r2], "sc7", UI_Graph_Change, 8, UI_Color_Purplish_red, 10, aerial_x + 60 + 60 * cos(1.75 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), aerial_y - 60 + 60 * sin(1.75 * PI + (ui_cmd_recv.yaw_motor->measure.total_angle + YAW_ZERO) / 360.0 * 2 * PI), 25);
        // UIGraphRefresh(&referee_info.referee_id, 7, UI[2], UI[3], UI[4], UI[5], UI[6], UI[7], UI[1]);
        // UIGraphRefresh(&referee_info.referee_id, 7, UI[0], UI[1], UI[2], UI[3], UI[4], UI[5], UI[0]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI[Fan_Circle_l1], UI[Fan_Circle_l2], UI[Fan_Circle_r1], UI[Fan_Circle_r2], UI[Gimbal_Yaw_Float], UI[Gimbal_Pitch_Float], UI[Shoot_Line]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI[Shoot_Loader_Circle], UI[Mode_Rectangle], UI[Music_Rectangle_1], UI[Music_Rectangle_2], UI[Music_Rectangle_3], UI[Shoot_Friction_Circle_l], UI[Shoot_Friction_Circle_r]);
        }

    for (int i=1;i<=LENGTH;){
        memcpy(&(UI[i]), &(UI[i - 1]), sizeof(Graph_Data_t));
        i+=2;
    }
}



