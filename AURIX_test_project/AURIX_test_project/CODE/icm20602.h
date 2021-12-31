#ifndef __ICM20602_H
#define __ICM20602_H

#include "headfile.h"

typedef struct{
        float  x, y, z;
} GYRO_t;

typedef struct{
        float  x, y, z;
} ANGLE_t;

typedef struct{
        float x, y, z;
} ACC_t;

extern GYRO_t icm_gyro;          // 陀螺仪原始数据
extern GYRO_t last_icm_gyro;     // 陀螺仪上次数据
extern GYRO_t icm_gyro_offset;   // 陀螺仪零偏
extern GYRO_t kalman_icm_gyro;   // 卡尔曼滤波之后的陀螺仪数据

extern ACC_t icm_acc;            // 加速度
extern ACC_t  last_icm_acc;      // 加速度
extern ACC_t  icm_acc_offset;    // 加速度
extern ACC_t  kalman_icm_acc;    // 卡尔曼滤波之后的加速度计数据

extern ANGLE_t
    Attitude_Angle, // 当前角度
    Last_Angle,     // 上次角度
    Target_Angle;   // 目标角度

extern float angle;
extern uint8 gyro_accumulate_status;

#define START_ANGLE_ACCUMULATE gyro_accumulate_status = 1, yaw = 0, hubu_angle = 0
#define STOP_ANGLE_ACCUMULATE gyro_accumulate_status = 0, yaw = 0, hubu_angle = 0

#define PI 3.14159265f
extern float hubu_angle;
extern float last_angle;


void get_icm_offset(void);
void Icm_Data_Filter(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void icm20602_data_update(void);
float angle_calc(float angle_m, float gyro_m);
void yijielvbo(float *angle_now, float *angle_m, float *gyro_m);
#endif
