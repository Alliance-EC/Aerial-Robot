// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "tool.h"
#include "super_cap.h"
#include "AHRS_MiddleWare.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"


#define YAW_ANGLE_MAX 41
#define YAW_ANGLE_MIN -97
#define PITCH_ANGLE_MAX -8
#define PITCH_ANGLE_MIN -35

#if PITCH_FEED_TYPE                                                  // Pitch 电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else                                                                   // PITCH电机反馈数据源为编码器
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif


/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
#endif                                 // ONE_BOARD



static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

HostInstance *host_instance; // 上位机接口

static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位
static uint8_t vision_send_data[23]; // 给视觉上位机发送的数据-四元数
// 这里的四元数以wxyz的顺序

static Publisher_t *ui_cmd_pub;            // 云台控制消息发布者
static Ui_Ctrl_Cmd_s ui_cmd_send;      // 传递给云台的控制信息
static Subscriber_t *ui_feed_sub; // 发射反馈信息订阅者
static Ui_Upload_Data_s ui_fetch_data; // 从云台获取的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

Referee_Interactive_info_t Referee_Interactive_info;    // 发送给UI绘制的数据
auto_shoot_mode_e AutoShooting_flag = AutoShooting_Off; // 自动射击标志位
extern char Send_Once_Flag;                             // 初始化UI标志


float limit_yaw_max = YAW_ANGLE_MAX, limit_yaw_min = YAW_ANGLE_MIN;
float limit_pitch_max = PITCH_ANGLE_MAX, limit_pitch_min = PITCH_ANGLE_MIN;
float delta_pitch =0;
float delta_yaw   = 0;
int remote_work_condition = 0; // 遥控器是否离线判断

float rec_yaw, rec_pitch;

float pc_limit_yaw=0,pc_limit_pitch=0;
float rc_limit_yaw=0,rc_limit_pitch=0;
bool shoot_cmd; // 接受上位机的火控指令

int Cover_Open_Flag; // 弹舱打开标志位

int Shoot_Mode_Flag; // 发射模式标志位
int Shoot_Run_Flag;  // 摩擦轮标志位

static void RobotReset() ;
static void EmergencyHandler() ;
static void Shoot_control() ;

void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    vision_recv_data[8] = 1;
}
void RobotCMDInit()
{

    rc_data                = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 8,
    };
    host_instance = HostInit(&host_conf); // 视觉通信串口

    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    ui_cmd_pub   = PubRegister("ui_cmd", sizeof(Ui_Ctrl_Cmd_s));
    ui_feed_sub =SubRegister("ui_feed", sizeof(Ui_Upload_Data_s));
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RC_CONTROL_MODE()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;    // 发射机构常开
    shoot_cmd_send.shoot_rate = 23;          // 射频默认25Hz

    if ((rc_data[TEMP].rc.dial > 440)) {
            Send_Once_Flag = 0; // UI重新发送
        }
    // 左侧开关为[上]右侧开关为[下]，且接收到上位机的相对角度,视觉模式
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_up(rc_data[TEMP].rc.switch_left))) {
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            Shoot_control(gimbal_cmd_send.gimbal_mode);
            gimbal_cmd_send.gimbal_mode = GIMBAL_VISION_MODE;

            // 使用相对角度控制
            memcpy(&rec_yaw, vision_recv_data, sizeof(float));
            memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));
            if ((rec_yaw == 0 && rec_pitch == 0) || vision_recv_data[8] == 0) {
                // 视觉未识别到目标,纯遥控器拨杆控制
                // 按照摇杆的输出大小进行角度增量,增益系数需调整
                // 控制云台旋转
            gimbal_cmd_send.gimbal_mode=GIMBAL_RC_MODE;
            } else {
                gimbal_cmd_send.gimbal_mode = GIMBAL_VISION_MODE;
            }
        } else {
            gimbal_cmd_send.gimbal_mode = GIMBAL_RC_MODE;
            if (switch_is_mid(rc_data[TEMP].rc.switch_right) && switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[上],右侧开关状态[中],底盘和云台分离,摩擦轮启动
            {
                shoot_cmd_send.friction_mode = FRICTION_ON;
                Shoot_control(gimbal_cmd_send.gimbal_mode);
            } 
        }
    }


static void PC_CONTROL_MODE()
    {
        // 按住鼠标右键且视觉识别到目标
        shoot_cmd_send.shoot_mode = SHOOT_ON; // 发射机构常开
        shoot_cmd_send.shoot_rate = 23;       // 射频默认25Hz
        gimbal_cmd_send.gimbal_mode=GIMBAL_PC_MODE;

        //控制云台
        if (rc_data[TEMP].mouse.press_r) {
            // 相对角度控制
            memcpy(&rec_yaw, vision_recv_data, sizeof(float));
            memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));
            // if ((rec_yaw == 0 && rec_pitch == 0) || vision_recv_data[8] == 0) {
            //     gimbal_cmd_send.gimbal_mode = GIMBAL_PC_MODE;
            // } else {
                gimbal_cmd_send.gimbal_mode=GIMBAL_VISION_MODE;
            //}
        } else {
            gimbal_cmd_send.gimbal_mode=GIMBAL_PC_MODE;
        }
        gimbal_cmd_send.yaw   = yaw_control;
        gimbal_cmd_send.pitch = pitch_control;

        
        
        //控制UI
        if (rc_data[TEMP].key[KEY_PRESS].r) {
            Send_Once_Flag = 0; // UI重新发送
        }

        if (rc_data[TEMP].key[KEY_PRESS].c){
            shoot_cmd_send.friction_mode =FRICTION_ON;
        }
        if (rc_data[TEMP].key[KEY_PRESS].x){
            shoot_cmd_send.friction_mode =FRICTION_OFF;
        }

        Shoot_control(gimbal_cmd_send.gimbal_mode);
        RobotReset(); // 机器人复位处理
    }
    
/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */

static void RobotReset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
}

/**
 * @brief 更新UI数据
 */
void UpDateUI()
{
    ui_cmd_send.yaw_motor=gimbal_fetch_data.yaw_motor;
    ui_cmd_send.pitch_motor=gimbal_fetch_data.pitch_motor;
    ui_cmd_send.yaw_limit=pc_limit_yaw;
    ui_cmd_send.pitch_limit=pc_limit_pitch;
    ui_cmd_send.friction_mode=shoot_cmd_send.friction_mode;
    ui_cmd_send.load_mode    = shoot_cmd_send.load_mode;
    ui_cmd_send.gimbal_mode=gimbal_cmd_send.gimbal_mode;
    ui_cmd_send.loader_motor=shoot_fetch_data.loader_motor;
}

static void Shoot_control(int mode){
    if (mode == GIMBAL_RC_MODE){
    if (rc_data[TEMP].rc.dial > 440) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    } else if (rc_data[TEMP].rc.dial < -440) {
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    }

    else if (mode == GIMBAL_PC_MODE){
        if (rc_data[TEMP].mouse.press_l){
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        }
        else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    }
}
static void Gimbal_control(int mode){

//键鼠控制
    if (mode ==GIMBAL_PC_MODE){
        pc_limit_yaw=brake_calc(limit_yaw_max, 20, gimbal_fetch_data.yaw_motor, limit_yaw_min, -rc_data[TEMP].mouse.x);
        pc_limit_pitch=brake_calc(limit_pitch_max, 5, gimbal_fetch_data.pitch_motor, limit_pitch_min, rc_data[TEMP].mouse.y);
        yaw_control -= rc_data[TEMP].mouse.x / 300.0f*pc_limit_yaw;
        pitch_control += rc_data[TEMP].mouse.y / 30000.0f*pc_limit_pitch;
    }

//遥控器控制
    else if (mode == GIMBAL_RC_MODE){
        // if (gimbal_cmd_send.gimbal_yaw_max && rc_data[TEMP].rc.rocker_l_ < 0) {
        // } else if (gimbal_cmd_send.gimbal_yaw_min && rc_data[TEMP].rc.rocker_l_ > 0) {
        // } else {
        //     ramp_init(&yaw_limit_ramp, 20);
        // }
        rc_limit_yaw=brake_calc(limit_yaw_max, 10, gimbal_fetch_data.yaw_motor, limit_yaw_min, (float)rc_data[TEMP].rc.rocker_l_*(-1));
        rc_limit_pitch=brake_calc(limit_pitch_max, 5, gimbal_fetch_data.pitch_motor, limit_pitch_min, -(float)rc_data[TEMP].rc.rocker_l1);
        yaw_control -= 0.0012f * (float)rc_data[TEMP].rc.rocker_l_ * rc_limit_yaw;
        pitch_control -= 0.00002f * (float)rc_data[TEMP].rc.rocker_l1*rc_limit_pitch;
    }

//视觉控制
    else if (mode == GIMBAL_VISION_MODE){
        yaw_control   = (gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET] + rec_yaw);
        pitch_control = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] + rec_pitch;
    }

    gimbal_cmd_send.yaw_last   = gimbal_cmd_send.yaw;
    gimbal_cmd_send.pitch_last = gimbal_cmd_send.pitch;

    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;

    delta_yaw   = (gimbal_cmd_send.yaw - gimbal_cmd_send.yaw_last)*8;
    delta_pitch = (gimbal_cmd_send.pitch - gimbal_cmd_send.pitch_last)*2.5;
    // if (delta_yaw == 0 && delta_pitch==0){
    //     // gimbal_keep();
    // }
}
/**
 * @brief 视觉发送任务，将数据发送给上位机
 *
 */
void VisionTask()
{
    static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x12};
    memcpy(vision_send_data, frame_head, 4); // 帧头

    memcpy(vision_send_data + 4, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4); // 四元数

    memcpy(vision_send_data + 20, &referee_info.GameRobotState.robot_id, sizeof(uint8_t)); // 机器人ID

    vision_send_data[22] = 0;
    for (size_t i = 0; i < 22; i++)
        vision_send_data[22] += vision_send_data[i];
    HostSend(host_instance, vision_send_data, 23);
}



/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_up(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆双[上],键鼠控制
        PC_CONTROL_MODE();
    else if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right)) { //双[下]，急停
        EmergencyHandler(); 
        // memcpy(&rec_yaw, vision_recv_data, sizeof(float));
        // memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));
        // yaw_control   = (gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] + rec_yaw / DEGREE_2_RAD);
        // pitch_control = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] + rec_pitch;
    } else {
        RC_CONTROL_MODE();
        //shoot_cmd_send.friction_mode = FRICTION_ON;
    }

    Gimbal_control(gimbal_cmd_send.gimbal_mode);
    UpDateUI();
        
    remote_work_condition = RemoteControlIsOnline();

    if (remote_work_condition == 0) {
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.friction_mode  = FRICTION_OFF;
        shoot_cmd_send.load_mode      = LOAD_STOP;
    }

    
        // 设置视觉发送数据,还需增加加速度和角速度数据

        // 推送消息,双板通信,视觉通信等
        // 其他应用所需的控制数据在RC_CONTROL_MODEmode和mousekeysetmode中完成设置

#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    VisionTask();
    }

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode  = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.shoot_mode    = SHOOT_OFF;
    shoot_cmd_send.load_mode     = LOAD_STOP;

    LOGERROR("[CMD] emergency stop!");
}