/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                               // 包序号，供整个referee文件使用

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测

// syhtod 正式上车后需删除
static void robot_mode_change(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *Referee_Interactive_init(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    return referee_recv_info;
}

void Referee_Interactive_task()
{
    My_UI_Refresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static uint32_t shoot_line_location[10] = {540, 960, 430, 450, 470, 495, 515, 560, 580};//射击基准线位置

static Graph_Data_t UI_position_line[10]; // 车身位姿 

static Graph_Data_t UI_Energy[3];      // 电容能量条

static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
 
static Graph_Data_t UI_aim_circle[2]; //瞄准准圈

void My_UI_init()
{
    if(Interactive_data->ui_mode == UI_KEEP)
    {
        return;
    }
    while (referee_recv_info->GameRobotState.robot_id == 0)
    {
        RefereeSend(NULL,0);
    }
    DeterminRobotID();
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    Line_Draw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 930, shoot_line_location[0], 960, shoot_line_location[0]); // 横向基准线
    Line_Draw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740); // 纵向基准线
    Line_Draw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[2], 960, shoot_line_location[2]); //
    Line_Draw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Main, 2, 960, shoot_line_location[3], 990, shoot_line_location[3]);
    Line_Draw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[4], 960, shoot_line_location[4]);
    Line_Draw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Main, 2, 960, shoot_line_location[5], 990, shoot_line_location[5]);
    Line_Draw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[6], 960, shoot_line_location[6]);
    UI_ReFresh(&referee_recv_info->referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4] ,UI_shoot_line[5],UI_shoot_line[6]);
    
    Line_Draw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Main, 2, 960, shoot_line_location[7], 990, shoot_line_location[7]);
    Line_Draw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[8], 960, shoot_line_location[8]);
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_shoot_line[7], UI_shoot_line[8]);



    Circle_Draw(&UI_aim_circle[0], "sc0",UI_Graph_ADD ,7,UI_Color_Main, 3,960,540,400);
    UI_ReFresh(&referee_recv_info->referee_id, 1, UI_aim_circle[0]);
    // 绘制车辆状态标志指示
    Char_Draw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 150, 750, "chassis:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    Char_Draw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 150, 700, "lid:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    Char_Draw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 150, 650, "shoot:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    Char_Draw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 150, 600, "frict:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    Char_Draw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 150, 550, "pitch:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 750, "zeroforce");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 700, "open");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 650, "stop");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 600, "off");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    Float_Draw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_White, 18, 1,2, 270, 550,0);
    UI_ReFresh(&referee_recv_info->referee_id, 1, UI_State_dyn[4]);

    // 底盘功率显示，静态
    Char_Draw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    Rectangle_Draw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UI_ReFresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    Float_Draw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    Line_Draw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}



static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    
    Mode_Change_Check(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 750, "zeroforce");
            break;
        case CHASSIS_ROTATE:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 750, "rotate   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_NO_FOLLOW:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 750, "nofollow ");
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 750, "follow   ");
            break;
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {   
        switch (_Interactive_data->lid_mode)
        {
        case LID_CLOSE:
        {
            Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 700, "close");
            break;
        }
        case LID_OPEN:
        {
            Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 700,  "open " );
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        switch (_Interactive_data->shoot_mode)
        {
        case LOAD_STOP:
        {
            Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 650, "stop    ");
            break;
        }
        case LOAD_1_BULLET:
        {
            Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 650, "one     ");
            break;
        }
        case LOAD_BURSTFIRE:
        {
            Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, "burst   ");
            break;
        }
        case LOAD_3_BULLET:
        {
            Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, "three   ");
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        switch (_Interactive_data->friction_mode)
        {
        case FRICTION_OFF:
        {
            Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 600,"off");
            break;
        }
        case FRICTION_ON:
        {
            Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 600,"on ");
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        Float_Draw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        Line_Draw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)720 + (pow(_Interactive_data->Chassis_Power_Data.chassis_power_mx,2)/576) * 300, 160);
        UI_ReFresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    //Pitch
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        Float_Draw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Purplish_red, 18, 1,2, 270, 550,_Interactive_data->Pitch_angle*1000);
        UI_ReFresh(&referee_recv_info->referee_id, 1, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    if (_Interactive_data->Pitch_angle != _Interactive_data->Pitch_last_angle)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->Pitch_last_angle = _Interactive_data->Pitch_angle;
    }
}
