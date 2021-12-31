#ifndef __PID_H
#define	__PID_H
#include "headfile.h"
#include <stdio.h>
#include <stdlib.h>


typedef struct
{
    float target_val;               //目标值
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_next;          		//定义上一个偏差值
	float err_last;                 // 定义上上一次偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
}_pid;

void  PID_param_init(_pid *pid);
void  set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void  set_pid(_pid *pid, float p, float i, float d);
float IncPID_realize(_pid *pid, float temp_val);
float PosPID_realize(_pid *pid, float temp_val);

#endif
