/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		headfile
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
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
//�ٷ�ͷ�ļ�
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"

//------��ɿƼ���Ƭ����������ͷ�ļ�
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

//------��ɿƼ���Ʒ����ͷ�ļ�
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

//------�Զ����ļ�
#include "pid.h"
#include "change.h"    // ��λ���ļ�
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





//���岦�뿪��
#define sw1    P22_0
#define sw2    P22_1
#define sw3    P22_2
#define sw4    P22_3

#define MOTOR_DEBUG            0            // ��Ҫ���Ե����ֱ�ӽ����Ϊ 1 ����
#define SERVO_DEBUG            0
#define RINGQ_DEBUG            0
#define RETURN_TO_ONE_DEBUG    1            /*�Ƿ�����й�һ��*/
#define MOTOR_USE_POSITION_PID 0            /*��������Ƿ�Ϊλ��ʽPID*/
#define PARAMETER_FLASH_DEBUG  1            /* ��Ϊ1��������������flash */
#define CPU1_USE_CAMERA        1            /*CPU1�Ƿ�������ͷ*/
#define USING_ICM20602         1            /*�Ƿ���������*/
#define STAP_CAR_DETECT        1            /*�Ƿ���ͣ�����*/
#define USING_ERROR_FILTER     0            /*�Ƿ��������ƽ���˲�*/
#define USING_NIMING_REPORT    0            /*�Ƿ�ʹ��������λ��*/
#define USING_VCAN_REPORT      0            /*�Ƿ�ʹ��ɽ����λ��*/

#define LAD         adc[1]    // ��ˮƽ���
#define LMAD        adc[2]    // ��ֱ���
#define MAD         adc[4]    // �м�ˮƽ���
#define RMAD        adc[6]    // �Ҵ�ֱ���
#define RAD         adc[7]    // ��ˮƽ���

#define lean_AD1     adc[3]  // ��������
#define lean_AD2     adc[5]  // �����ҵ��
#define _MAD         adc[8]  // �м䴹ֱ���

#define SERVO_MID   2966
#define BEEP_PIN    P33_10         //�������������
#define REED_PIN    P20_10          // �ɻƹ�����
#define RoundMaxNum 10
#define PARAMETER_USING_FLASH   0  // 0 ������ֱ�Ӹ�ֵ   1 ��flash �������

#define EncoderFilterLength  5
//#define LostLineLimit        30

typedef struct{
        int circle, circle_left, circle_right;    // ������־λ
        int ramp_flag;                            // �µ���־λ
//        int garage_flag;                          // �����־λ
        int buzzer_flag;                          // ��������־λ  ��һ��
        int two_buzzer_flag;                      // ������ ������
        int flash_update_flag;                    // flash ���±�־
        int return_to_one_flag;                   // ��һ����־
        int curve_flag;                           // �����־
        int straightLineFlag;                     // ֱ����־
        int front_lost_line;                      // ��ǰհ���߱�־
        int back_lost_line;                       // ��ǰհ���߱�־
        int single_line_Flag;                     // ���߱�־
        int tft_flag;                             // 0 ����ʾ ��1 ���  2 ԭʼͼ��  3 ��ʾ����ͷ������ͼ��
        int tft_clear_flag;                       // ������־
        int cpu0_using_tft;                       // cpu0 ʹ����Ļ��־
        int cpu1_using_tft;                       // cpu1 ʹ����Ļ��ʾ
        int long_road_to_curve;
        int PID_parameter_debug;                  // 0: ��ͨ��̬PD  1  ���PD  2 С��PD  3 ��PD 4  ���涯̬PD  5 ���PID
        int speed_debug;                          // ����ٶȵ�λ
        int tri_road_in;                          // ������·���ڱ�־
        int camera_tri_road_flag;                 // ����ͷ��⵽�����־λ
        int crossroadFlag;                        // ʮ�ֱ�־λ
        int tri_road_detect_flag;
        int already_in_tri_road;                  // ��ʾ�Ѿ�����һ������·��
        int using_lean_error;
        int enter_garage;                         // ����־λ
        int enable_enter_garage;
        int motor_reverse;

} FLAG;

extern  FLAG flag;

typedef struct{
        float mid_error;             // �м��м���ƫ��
        float H_error;               // ˮƽ��м���ƫ��
        float V_error;               // ��ֱ��м���ƫ��
        float lean_error;            // ���ֵ��ƫ��
        float error;                 // ����ƫ��
        float last_error;            // ��һ��ƫ��
        float error_delta;           // ƫ��仯��
        float camera_error;          // ����ͷ����ƫ��

} ERROR;

extern ERROR error;


typedef struct{
        uint32 closed_speed[5];
        uint32 curve_low_speed, curve_high_speed;
} SPEED;
extern SPEED speed;


typedef struct{
    float straightKd;           // ֱ��Dֵ
    float curve_kp;             // ��ǰհ����Kp
    float curve_kd;             // ��ǰհ����Kd
    float LRTC_kp;              // ��ֱ������kp   ��̬
    float LRTC_kd;              // ��ֱ������kd
    float smallRingKp;
    float smallRingKd;
    float bigRingKp;
    float bigRingKd;
    float triRoadA;
    float triToadB;
    float triRoadKd;
    float basic_kp;            /* ����ڶ�̬kp */
    float basic_kd;            /* �����Kd*/
} PID;

extern  PID pid;


typedef struct{
        uint32 flag;     // �����־λ
        uint32 status;   // ����״̬λ
        uint32 dir;      // ���ⷽ��
        uint8  angle;    // ���ʱ�����ǻ��ֽǶ�
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
extern volatile uint16 delay_20, delay_5000, delay_100;      // 50ms, 5000ms ��ʱ��־λ
extern volatile uint16  ad_value, voltage;        // ��ص�ѹ
extern volatile uint16 adc[10], adc_original[10], adc_max[10], adc_min[10], ADRingALLmax;   // ˫��ֵ
extern volatile uint16 adcSingleLineMax[10], adcSingleLineMin[10];                          // ����ֵ

extern volatile uint16 ringq_length;
                                // ����ƫ��
extern volatile float mul;                                  // key����ϵ��

extern volatile float dt;

//extern volatile kalman_struct kalman1;
//extern volatile kalman_struct kalman2;

extern volatile float yaw, pitch;                   // ����ǣ�������
extern volatile uint8 ramp_ticks;        // �µ���־λ
extern volatile uint8 buzzer_flag;

extern volatile uint32 timevar;                     // ��ʱʱ��

extern volatile int16 servo_limit;

extern volatile uint8 round_status[RoundMaxNum];    // 0:���  1���Ҵ�  2 ��С��  // ��С��
extern volatile uint8 round_num;
extern volatile volatile uint8 count_round;
extern volatile uint8 round_flag;                   // ����״̬����־λ

extern volatile float A, B;
extern volatile uint8 var;
extern volatile uint8 tri_road_status;   // 0 ��ʾ��һ�ν����棬 1��ʾ��2�ν�����
extern volatile uint8 tri_road_dir;

//extern volatile uint8 lost_line_dir;                // ���߷���   1: ����   2 : �Ҷ���
extern volatile uint8 reed_value;                   // �ɻɹ�
extern uint32 current_time;
extern uint8 last_dir;
extern uint8 curve_status;

extern uint16 encoder_circle_count;

extern volatile uint16 bigEncoderValue, smallEncoderValue;      // ��С�����������Ʋ�ֵ
extern volatile uint16 round_angle_value;                       // �����ǶȻ���ֵ
extern volatile uint16 outround_encoder_value;                  // ��������������ֵ
extern volatile uint16 round_mad;                               // �����м��б궨ֵ


/*************************************�������******************************************/
extern volatile uint8 tri_road_mad_value;                      // �������м���ֵ
extern volatile uint8 out_tri_road_value;                      // ����������ֵ
extern uint8 in_tri_road_angle;                                   // �����������ǽǶ�
extern uint8 out_tri_road_angle;                                   // �����������ǽǶ�

extern uint8  ramp_angle;                                       // �µ�ʶ��Ƕ�
extern uint16 Lostlinelimit;
#endif

