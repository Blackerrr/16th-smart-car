#ifndef __PID_H
#define	__PID_H
#include "headfile.h"
#include <stdio.h>
#include <stdlib.h>


typedef struct
{
    float target_val;               //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_next;          		//������һ��ƫ��ֵ
	float err_last;                 // ��������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

void  PID_param_init(_pid *pid);
void  set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void  set_pid(_pid *pid, float p, float i, float d);
float IncPID_realize(_pid *pid, float temp_val);
float PosPID_realize(_pid *pid, float temp_val);

#endif
