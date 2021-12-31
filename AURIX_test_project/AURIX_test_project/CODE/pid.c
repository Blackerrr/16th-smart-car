#include "pid.h"
#include "math.h"
#include "string.h"



void PID_param_init(_pid *pid)
{
    memset(pid, 0, sizeof(pid));

/*  pid->target_val = 0.0;
    pid->actual_val = 0.0;
    pid->integral = 0.0;
    pid->err = 0.0;
    pid->err_next = 0.0;
    pid->err_last = 0.0;
    pid->Kp = 0.0;
    pid->Ki = 0.0;
    pid->Kd = 0.0; */
}

void set_pid_target(_pid *pid, float temp_val)
{
    pid->target_val = temp_val; // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
  * @note   ��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
    return pid->target_val; // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ���ñ��������֡�΢��ϵ��
  * @param  p������ϵ�� P
  * @param  i������ϵ�� i
  * @param  d��΢��ϵ�� d
  * @note   ��
  * @retval ��
  */
void set_pid(_pid *pid, float p, float i, float d)
{
    pid->Kp = p; // ���ñ���ϵ�� P
    pid->Ki = i; // ���û���ϵ�� I
    pid->Kd = d; // ����΢��ϵ�� D
}

// ����ʽ pid ��ɢ��ʽ
// pid -> actual_val + =Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
float IncPID_realize(_pid *pid, float temp_val)
{
    pid->err = (pid->target_val - temp_val);
    pid->actual_val += pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last); // ����ʽ(*pid)
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->actual_val;
}


// λ�� pid
float PosPID_realize(_pid *pid, float temp_val)
{
    pid->err = (pid->target_val - temp_val);
/* #if 1                                        // �����޷�
    pid->integral += pid->err;
    if(pid->integral > 150)
        pid->integral = 150;
    else if(pid -> integral < -70)
        pid->integral = -40;
#endif */
    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid-> err_next);
    pid->err_next = pid->err;
    return pid->actual_val;
}
