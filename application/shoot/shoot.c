#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "UI.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader;
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;

float delta_friction=0;
float delta_speed_r=0;
float delta_speed_l = 0;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
// 用来控制UI的刷新频率
static uint8_t UI_timer = 0;

bool friction_mode_last =1;
void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp      = 4.5 ,//0.3
                .Ki      = 0, //10
                .Kd      = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 20000,
            },
            // .current_PID = {
            //     .Kp            = 0.9, // 2.0, // 0.7
            //     .Ki            = 0.0,   // 0.1
            //     .Kd            = 0,
            //     .Improve       = PID_Integral_Limit,
            //     .IntegralLimit = 10000,
            //     .MaxOut        = 20000,
            // },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2;
    friction_l                            = DJIMotorInit(&friction_config);
    
    friction_config.can_init_config.tx_id                             = 3; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag=MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 6,
        },
        .controller_param_init_config = {
            // .angle_PID = {
            //     // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
            //     .Kp     = 10, // 10
            //     .Ki     = 1,  // 1,
            //     .Kd     = 0,
            //     .MaxOut = 200,
            // },
            .speed_PID = {
                .Kp            = 2.0, // 10
                .Ki            = 0,  // 0.5, // 1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit ,
                .IntegralLimit = 5000,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = SPEED_LOOP,// | SPEED_LOOP ,//| ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_cmd_recv.shoot_mode = SHOOT_ON; // 初始化后摩擦轮进入准备模式,也可将右拨杆拨至上一次来手动开启

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);

    DJIMotorOuterLoop(friction_l, SPEED_LOOP); // 切换到速度环
    DJIMotorOuterLoop(friction_r, SPEED_LOOP); // 切换到速度环=
}

uint16_t Block_Time;            // 堵转时间
float speed_record[6] = {0}; // 第五个为最近的射速 第六个为平均射速
float Shoot_Speed;           // 射速
float speed_adj;             // 摩擦轮调速
uint16_t loder_reverse = 1;
/**
 * @brief 堵转，弹速检测
 *
 */
static void ShootCtrl(){
//     //堵转时间检测
//     //平均射速
//     //摩擦轮调速
    if(abs(loader->measure.speed_aps) < loader->motor_controller.speed_PID.Ref - 2000){
        Block_Time++;
    }
    else{
        Block_Time = 0;
    }
    if(Block_Time >= 1000){
        loder_reverse = -1;
    }
    else{
        loder_reverse = 1;
    }
}

float loader_angle;
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // if (UI_timer < 10) {
    //     UI_timer++;
    // } else {
    //     UI_timer = 0;
    //     My_UIGraphRefresh();
    // }

    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJIMotorSetRef(friction_l, 0); // 42500
        DJIMotorSetRef(friction_r, 0);
        // DJIMotorStop(friction_l);
        // DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    } else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    loader_angle=loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE;/////// // 拨盘电机参考值设定
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode) {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_1_BULLET:                                                               // 激活能量机关/干扰对方用,英雄用.
            DJIMotorOuterLoop(loader, SPEED_LOOP);                                        // 切换到角度环
            shoot_cmd_recv.shoot_rate = 2.5;                                                  // 设定射速为1,即单发
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8); // 控制量增加一发弹丸的角度
            // DJIMotorOuterLoop(loader, ANGLE_LOOP);                                        // 切换到角度环
            // DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();                                        // 记录触发指令的时间
            dead_time      = 150;                                                         // 完成1发弹丸发射的时间
            break; 
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            ShootCtrl();
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, (shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8)*loder_reverse);
            // DJIMotorSetRef(loader, 27000);

            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // ...
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
    
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        if (friction_mode_last==FRICTION_OFF){
            ramp_init(&shoot_ramp_l, RAMP_TIME);
            ramp_init(&shoot_ramp_r, RAMP_TIME);
        }
        DJIMotorSetRef(friction_l, 42500*ramp_calc(&shoot_ramp_l)); // 42500
        DJIMotorSetRef(friction_r, 42500*ramp_calc(&shoot_ramp_r));
    } 
    else if (shoot_cmd_recv.friction_mode == FRICTION_REVERSE) {
        // DJIMotorSetRef(friction_l, -150);
        // DJIMotorSetRef(friction_r, 150);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) // 关闭摩擦轮
    {
        if (friction_mode_last == FRICTION_ON) {
            ramp_init(&shoot_ramp_l, RAMP_TIME);
            ramp_init(&shoot_ramp_r, RAMP_TIME);
        }

        DJIMotorSetRef(friction_r, 42500 * (1 - ramp_calc(&shoot_ramp_r)));
        DJIMotorSetRef(friction_l, 42500 * (1 - ramp_calc(&shoot_ramp_l)));
    }
    delta_friction=friction_l->measure.speed_aps+ friction_r->measure.speed_aps;
    delta_speed_l = friction_l->measure.speed_aps -42500;
    delta_speed_r = friction_r->measure.speed_aps +42500;
    friction_mode_last = shoot_cmd_recv.friction_mode;
    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
     PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
