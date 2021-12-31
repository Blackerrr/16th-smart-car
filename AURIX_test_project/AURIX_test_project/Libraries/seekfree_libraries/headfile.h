/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		headfile
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
 
#ifndef _headfile_h
#define _headfile_h




#include "SEEKFREE_PRINTF.h"

#include "zf_assert.h"
#include "stdio.h"
#include "math.h"
//官方头文件
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"

//------逐飞科技单片机外设驱动头文件
#include "zf_gpio.h"
#include "zf_gtm_pwm.h"
#include "zf_uart.h"
#include "zf_ccu6_pit.h"
#include "zf_stm_systick.h"
#include "zf_spi.h"
#include "zf_eru.h"
#include "zf_eru_dma.h"
#include "zf_vadc.h"
#include "zf_gpt12.h"
#include "zf_eeprom.h"

//------逐飞科技产品驱动头文件
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_FUN.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_VIRSCO.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_MMA8451.h"
#include "SEEKFREE_L3G4200D.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "SEEKFREE_7725.h"
#include "SEEKFREE_RDA5807.h"

//------自定义文件
#include "pid.h"
#include "change.h"    // 上位机文件
#include "key.h"
#include "motor.h"
#include "image.h"
#include "circle.h"
#include "ringq.h"
#include "kalman_filter.h"
#include "show.h"
#include "adc.h"
#include "keyboard.h"
#include "init.h"
#include "icm20602.h"
#include "adrc.h"
#include "sdcard.h"
#include "S_CV.h"
#include"S_CVS.h"
#include "my_DP.h"
#include "vofa.h"
#include "fuzzy.h"





//定义拨码开关
#define sw1    P22_0
#define sw2    P22_1
#define sw3    P22_2
#define sw4    P22_3

#define MOTOR_DEBUG            0            // 若要调试电机，直接将其该为 1 即可
#define SERVO_DEBUG            0
#define RINGQ_DEBUG            0
#define RETURN_TO_ONE_DEBUG    1            /*是否开启电感归一化*/
#define MOTOR_USE_POSITION_PID 0            /*电机控制是否为位置式PID*/
#define PARAMETER_FLASH_DEBUG  1            /* 若为1，则开启参数存入flash */
#define CPU1_USE_CAMERA        1            /*CPU1是否开启摄像头*/
#define USING_ICM20602         1            /*是否开启陀螺仪*/
#define STAP_CAR_DETECT        1            /*是否开启停车检测*/
#define USING_ERROR_FILTER     0            /*是否对误差进行平滑滤波*/
#define USING_NIMING_REPORT    0            /*是否使用匿名上位机*/
#define USING_VCAN_REPORT      0            /*是否使用山外上位机*/

#define LAD         adc[1]    // 左水平电感
#define LMAD        adc[2]    // 左垂直电感
#define MAD         adc[4]    // 中间水平电感
#define RMAD        adc[6]    // 右垂直电感
#define RAD         adc[7]    // 右水平电感

#define lean_AD1     adc[3]  // 后排左电感
#define lean_AD2     adc[5]  // 后排右电感
#define _MAD         adc[8]  // 中间垂直电感

#define SERVO_MID   2966
#define BEEP_PIN    P33_10         //定义蜂鸣器引脚
#define REED_PIN    P20_10          // 干黄管引脚
#define RoundMaxNum 10
#define PARAMETER_USING_FLASH   0  // 0 程序里直接赋值   1 从flash 里读参数

#define EncoderFilterLength  5
//#define LostLineLimit        30

typedef struct{
        int circle, circle_left, circle_right;    // 环岛标志位
        int ramp_flag;                            // 坡道标志位
//        int garage_flag;                          // 车库标志位
        int buzzer_flag;                          // 蜂鸣器标志位  响一声
        int two_buzzer_flag;                      // 蜂鸣器 响两声
        int flash_update_flag;                    // flash 更新标志
        int return_to_one_flag;                   // 归一化标志
        int curve_flag;                           // 弯道标志
        int straightLineFlag;                     // 直道标志
        int front_lost_line;                      // 长前瞻丢线标志
        int back_lost_line;                       // 短前瞻丢线标志
        int single_line_Flag;                     // 单线标志
        int tft_flag;                             // 0 不显示 ，1 电磁  2 原始图像  3 显示摄像头处理后的图像
        int tft_clear_flag;                       // 清屏标志
        int cpu0_using_tft;                       // cpu0 使用屏幕标志
        int cpu1_using_tft;                       // cpu1 使用屏幕表示
        int long_road_to_curve;
        int PID_parameter_debug;                  // 0: 普通动态PD  1  弯道PD  2 小环PD  3 大环PD 4  三叉动态PD  5 电机PID
        int speed_debug;                          // 五个速度档位
        int tri_road_in;                          // 在三叉路口内标志
        int camera_tri_road_flag;                 // 摄像头检测到三叉标志位
        int crossroadFlag;                        // 十字标志位
        int tri_road_detect_flag;
        int already_in_tri_road;                  // 表示已经进入一次三叉路口
        int using_lean_error;
        int enter_garage;                         // 入库标志位
        int enable_enter_garage;
        int motor_reverse;

} FLAG;

extern  FLAG flag;

typedef struct{
        float mid_error;             // 中间电感计算偏差
        float H_error;               // 水平电感计算偏差
        float V_error;               // 垂直电感计算偏差
        float lean_error;            // 八字电感偏差
        float error;                 // 最终偏差
        float last_error;            // 上一个偏差
        float error_delta;           // 偏差变化率
        float camera_error;          // 摄像头计算偏差

} ERROR;

extern ERROR error;


typedef struct{
        uint32 closed_speed[5];
        uint32 curve_low_speed, curve_high_speed;
} SPEED;
extern SPEED speed;


typedef struct{
    float straightKd;           // 直道D值
    float curve_kp;             // 长前瞻丢线Kp
    float curve_kd;             // 长前瞻丢线Kd
    float LRTC_kp;              // 长直道入弯kp   动态
    float LRTC_kd;              // 长直道入弯kd
    float smallRingKp;
    float smallRingKd;
    float bigRingKp;
    float bigRingKd;
    float triRoadA;
    float triToadB;
    float triRoadKd;
    float basic_kp;            /* 弯道内动态kp */
    float basic_kd;            /* 弯道内Kd*/
} PID;

extern  PID pid;


typedef struct{
        uint32 flag;     // 车库标志位
        uint32 status;   // 车库状态位
        uint32 dir;      // 出库方向
        uint8  angle;    // 入库时陀螺仪积分角度
} GARAGE;
extern GARAGE garage;

//extern volatile _pid  servo_pid;
//extern volatile _pid  motor_pid;

extern volatile int16 duty;
extern volatile int32 moto;
extern volatile uint8 motor_start;
extern volatile uint32 motor_target;
extern volatile uint8 motor_reverse_level;
extern volatile int16 encoder;
extern volatile int16 EncoderData[EncoderFilterLength];
extern volatile uint16 delay_20, delay_5000, delay_100;      // 50ms, 5000ms 延时标志位
extern volatile uint16  ad_value, voltage;        // 电池电压
extern volatile uint16 adc[10], adc_original[10], adc_max[10], adc_min[10], ADRingALLmax;   // 双线值
extern volatile uint16 adcSingleLineMax[10], adcSingleLineMin[10];                          // 单线值

extern volatile uint16 ringq_length;
                                // 最终偏差
extern volatile float mul;                                  // key调节系数

extern volatile float dt;

//extern volatile kalman_struct kalman1;
//extern volatile kalman_struct kalman2;

extern volatile float yaw, pitch;                   // 航向角，俯仰角
extern volatile uint8 ramp_ticks;        // 坡道标志位
extern volatile uint8 buzzer_flag;

extern volatile uint32 timevar;                     // 计时时间

extern volatile int16 servo_limit;

extern volatile uint8 round_status[RoundMaxNum];    // 0:左大环  1：右大环  2 左小环  // 右小环
extern volatile uint8 round_num;
extern volatile volatile uint8 count_round;
extern volatile uint8 round_flag;                   // 环岛状态量标志位

extern volatile float A, B;
extern volatile uint8 var;
extern volatile uint8 tri_road_status;   // 0 表示第一次进三叉， 1表示第2次进三叉
extern volatile uint8 tri_road_dir;

//extern volatile uint8 lost_line_dir;                // 丢线方向   1: 左丢线   2 : 右丢线
extern volatile uint8 reed_value;                   // 干簧管
extern uint32 current_time;
extern uint8 last_dir;
extern uint8 curve_status;

extern uint16 encoder_circle_count;

extern volatile uint16 bigEncoderValue, smallEncoderValue;      // 大小环岛编码器计步值
extern volatile uint16 round_angle_value;                       // 进环角度积分值
extern volatile uint16 outround_encoder_value;                  // 出环编码器积分值
extern volatile uint16 round_mad;                               // 环岛中间电感标定值


/*************************************三叉参数******************************************/
extern volatile uint8 tri_road_mad_value;                      // 入三叉中间电感值
extern volatile uint8 out_tri_road_value;                      // 出三叉电感阈值
extern uint8 in_tri_road_angle;                                   // 入三叉陀螺仪角度
extern uint8 out_tri_road_angle;                                   // 出三叉陀螺仪角度

extern uint8  ramp_angle;                                       // 坡道识别角度
extern uint16 Lostlinelimit;
#endif

