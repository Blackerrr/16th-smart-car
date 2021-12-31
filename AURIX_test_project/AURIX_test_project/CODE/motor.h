#ifndef __MOTOR_H
#define __MOTOR_H

#include "headfile.h"
#include "math.h"



extern uint8  servo_deviation_flag;    // 0 ˮƽ���ѭ��  1 ��ֱ���ѭ��
extern uint8 angle_compresation_flag;

extern uint8 tri_road_delay_count, tri_road_delay_flag;    // ����·�ڴ�����ʱ

#define MOTOR2_A      ATOM0_CH2_P21_4   //����2�����תPWM����
#define MOTOR2_B      ATOM0_CH3_P21_5   //����2�����תPWM����

#define MOTOR2_DIR   P21_5              //����2��������������
#define MOTOR2_PWM   ATOM0_CH2_P21_4    //����2���PWM����

#define S_MOTOR_PIN   ATOM1_CH1_P33_9       //����������

#define L_value 580;
#define R_value 920;


void Motor_init(void);
void Motor_control(int32 encoder);
void Servo_control(void);
float get_absf(float value);
void update_H_error(void);
void update_V_error(void);
void update_MAD_error(void);
void stop_car_detect(void);
float my_sqrt(float number);
void encoder_handle(void);
void tri_road_handle(void);
int16 RecurrenceFilter (int16 DATA);
void CalculateServoKp(void);
#endif
