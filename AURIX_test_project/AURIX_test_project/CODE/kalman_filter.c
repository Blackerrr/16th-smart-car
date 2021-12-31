#include "kalman_filter.h"

/**
 *kalman_init - 卡尔曼滤波器初始化
 *@kalman_lcw： 卡尔曼滤波器结构体
 *@init_x：        待测量的初始值
 *@init_p：        后验状态估计值误差的方差的初始值
 */
void kalman_init(kalman_struct *kalman_lcw, float init_x, float init_p)
{
    kalman_lcw->x = init_x;//待测量的初始值，如有中值一般设成中值（如陀螺仪）
    kalman_lcw->p = init_p;//后验状态估计值误差的方差的初始值
    kalman_lcw->A = 1;
    kalman_lcw->H = 1;
    kalman_lcw->q = 25;//10e-2;//10e-6;//2e2;////predict noise convariance 预测（过程）噪声方差 实验发现修改这个值会影响收敛速率
    kalman_lcw->r = 100;//10e-5;//测量（观测）噪声方差。以陀螺仪为例，测试方法是：
    //保持陀螺仪不动，统计一段时间内的陀螺仪输出数据。数据会近似正态分布，
    //按3σ原则，取正态分布的(3σ)^2作为r的初始化值
}

/**
 *kalman_filter - 卡尔曼滤波器
 *@kalman_lcw:卡尔曼结构体
 *@measure；测量值
 *返回滤波后的值
 */
float kalman_filter(kalman_struct *kalman_lcw, float measure)
{
    /* Predict */
    kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
    kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    kalman_lcw->gain = kalman_lcw->p * kalman_lcw->H / (kalman_lcw->p * kalman_lcw->H * kalman_lcw->H + kalman_lcw->r);
    kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure - kalman_lcw->H * kalman_lcw->x);
    kalman_lcw->p = (1 - kalman_lcw->gain * kalman_lcw->H) * kalman_lcw->p;

    return kalman_lcw->x;
}

/**********************************************
* 电磁卡尔曼滤波
***********************************************/
float KalmanFilter_Elect(float curr_elect_val,float last_elect_val)
{
  static float Q_curr = 1.0;//0.1           //Q增大，动态响应增大，过程噪声的协方差
  static float Q_last = 0.0001;             //过程噪声的协方差，过程噪声的协方差为一个一行两列矩阵
  static float R_elect = 10.0;              //测量噪声的协方差 即测量偏差

  static float Pk[2][2] = { {1, 0}, {0, 1 }};

  static float Pdot[4] = {0,0,0,0};

  static float q_bias = 0.0;
  static float elect_err = 0.0;
  static float PCt_0 = 0.0;
  static float PCt_1 = 0.0;
  static float E = 0.0;
  static float K_0 = 0.0, K_1 = 0.0, t_0 = 0.0, t_1 = 0.0;

  Pdot[0] = Q_curr - Pk[0][1] - Pk[1][0];       //Pk-先验估计误差协方差的微分
  Pdot[1] = -Pk[1][1];
  Pdot[2] = -Pk[1][1];
  Pdot[3] = Q_last;

  Pk[0][0] += Pdot[0] * dt;             //Pk-先验估计误差的协方差微分的积分
  Pk[0][1] += Pdot[1] * dt;             //先验估计误差协方差
  Pk[1][0] += Pdot[2] * dt;
  Pk[1][1] += Pdot[3] * dt;

  elect_err = curr_elect_val - last_elect_val;          //偏差 = 测量值 - 预测值，先验估计

  PCt_0 = Pk[0][0];
  PCt_1 = Pk[1][0];

  E = R_elect + PCt_0;

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = Pk[0][1];

  Pk[0][0] -= K_0 * t_0;                    //后验估计误差协方差
  Pk[0][1] -= K_0 * t_1;
  Pk[1][0] -= K_1 * t_0;
  Pk[1][1] -= K_1 * t_1;

  curr_elect_val += K_0 * elect_err;                //后验估计 更新最优电磁值 最优电磁值 = 预测值 + 卡尔曼增益*(测量值-预测值)
  q_bias += K_1 * elect_err;                        //后验估计 更新误差

  return curr_elect_val;

}


