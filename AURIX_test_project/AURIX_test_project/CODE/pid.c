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
    pid->target_val = temp_val; // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
  * @note   无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
    return pid->target_val; // 设置当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
  * @note   无
  * @retval 无
  */
void set_pid(_pid *pid, float p, float i, float d)
{
    pid->Kp = p; // 设置比例系数 P
    pid->Ki = i; // 设置积分系数 I
    pid->Kd = d; // 设置微分系数 D
}

// 增量式 pid 离散公式
// pid -> actual_val + =Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
float IncPID_realize(_pid *pid, float temp_val)
{
    pid->err = (pid->target_val - temp_val);
    pid->actual_val += pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last); // 增量式(*pid)
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->actual_val;
}


// 位置 pid
float PosPID_realize(_pid *pid, float temp_val)
{
    pid->err = (pid->target_val - temp_val);
/* #if 1                                        // 积分限幅
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
