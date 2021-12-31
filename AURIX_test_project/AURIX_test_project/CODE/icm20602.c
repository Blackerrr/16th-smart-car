#include "icm20602.h"
#include "math.h"
#include "stdlib.h"



GYRO_t
    icm_gyro,          // 陀螺仪原始数据
    last_icm_gyro,     // 陀螺仪上次数据
    icm_gyro_offset,   // 陀螺仪零偏
    kalman_icm_gyro;   // 卡尔曼滤波之后的陀螺仪数据

ACC_t
    icm_acc,            // 加速度原始数据
    last_icm_acc,      // 加速度上次数据
    icm_acc_offset,    // 加速度零偏
    kalman_icm_acc;    // 卡尔曼滤波之后的加速度计数据

ANGLE_t
    Attitude_Angle, // 当前角度
    Last_Angle,     // 上次角度
    Target_Angle;   // 目标角度


#define AcceRatio   16384.0f
#define GyroRatio   16.4f
#define Gyro_Gr     0.0010653   // 角速度变成弧度  此参数对应陀螺2000度每秒
#define ACC_FILTER_NUM 5        // 加速度计滤波深度
#define GYRO_FILTER_NUM 7       // 陀螺仪滤波深度
int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];  // 滤波缓存数组
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];

uint8 gyro_accumulate_status;                         // 偏航角计算标志


/*
 * 函数名：Data_Filter
 * 描述  ：获取陀螺仪零偏
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void get_icm_offset(void)
{
    for(uint8 i = 0; i < 100; i++)
    {
        get_icm20602_gyro_spi();       //±2000 dps      16.4 LSB/dps
        get_icm20602_accdata_spi();    //±8g            4096 LSB/g

        icm_gyro_offset.x += (float)icm_gyro_x;
        icm_gyro_offset.y += (float)icm_gyro_y;
        icm_gyro_offset.z += (float)icm_gyro_z;

        icm_acc_offset.x += (float)icm_acc_x;
        icm_acc_offset.y += (float)icm_acc_y;
//        icm_acc_offset.z += (float)icm_acc_z;
        systick_delay_ms(STM0, 5);
    }
    icm_gyro_offset.x /= 100.0f;
    icm_gyro_offset.y /= 100.0f;
    icm_gyro_offset.z /= 100.0f;

    icm_acc_offset.x /= 100.0f;
    icm_acc_offset.y /= 100.0f;
//    icm_acc_offset.z /= 100.0f;
}

/*
 * 函数名：Data_Filter
 * 描述  ：获取数据并进行数据滑动滤波
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Icm_Data_Filter(void)  // 数据滤波
{

    uint8 i=0;
    static float pre_ax = 0.0f;
    static float pre_ay = 0.0f;
    static float pre_az = 0.0f;
    static unsigned int first_flag = 0;
    static unsigned int filter_cnt = 0;    //计算加速度计滤波的次数

    long temp_accel_x = 0; //用来存放加速度计X轴原生数据的累加和
    long temp_accel_y = 0; //用来存放加速度计Y轴原生数据的累加和
    long temp_accel_z = 0; //用来存放加速度计Z轴原生数据的累加和

    static short accel_x_buffer[10] = {0}; //用来存放加速度计X轴最近10个数据的数组
    static short accel_y_buffer[10] = {0}; //用来存放加速度计Y轴最近10个数据的数组
    static short accel_z_buffer[10] = {0}; //用来存放加速度计Z轴最近10个数据的数组
    get_icm20602_accdata_spi();    //±8g            4096 LSB/g
    get_icm20602_gyro_spi();       //±2000 dps      16.4 LSB/dps

    if(first_flag == 0) //如果第一次进来该函数，则对用来做平均的数组进行初始化
    {
        first_flag = 1; //以后不再进来
        for(i=0;i<10;i++)
        {
            accel_x_buffer[i] = icm_acc_x - icm_acc_offset.x;
            accel_y_buffer[i] = icm_acc_y - icm_acc_offset.y;
            accel_z_buffer[i] = icm_acc_z - icm_acc_offset.z;
        }
    }
    else  //如果不是第一次了
    {
        accel_x_buffer[filter_cnt] = icm_acc_x - icm_acc_offset.x;
        accel_y_buffer[filter_cnt] = icm_acc_y - icm_acc_offset.y;
        accel_z_buffer[filter_cnt] = icm_acc_z - icm_acc_offset.z;

        filter_cnt ++;
        if(filter_cnt == 10)
        {
            filter_cnt = 0;
        }
    }

    for(i=0;i<10;i++)
    {
        temp_accel_x += accel_x_buffer[i];
        temp_accel_y += accel_y_buffer[i];
        temp_accel_z += accel_z_buffer[i];
    }

    icm_acc.x = (float)temp_accel_x / 10.0f / AcceRatio;
    icm_acc.y = (float)temp_accel_y / 10.0f / AcceRatio;
    icm_acc.z = (float)temp_accel_z / 10.0f / AcceRatio;

    icm_gyro.x = (float)(icm_gyro_x - icm_gyro_offset.x) / GyroRatio;
    icm_gyro.y = (float)(icm_gyro_y - icm_gyro_offset.y) / GyroRatio;
    icm_gyro.z = (float)(icm_gyro_z - icm_gyro_offset.z) / GyroRatio;


    /******************* 加速度低通滤波 ************************/
    icm_acc.x = 0.02f * icm_acc.x + 0.98f * pre_ax;
    pre_ax = icm_acc.x;

    icm_acc.y = 0.02f * icm_acc.y + 0.98f * pre_ay;
    pre_ay = icm_acc.y;

    icm_acc.z = 0.02f * icm_acc.z + 0.98f * pre_az;
    pre_az = icm_acc.z;

}


/*
 * 函数名：Get_Attitude
 * 描述  ：姿态解算
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Get_Attitude(void) // 姿态解算
{
    IMUupdate(icm_gyro.x*Gyro_Gr*GyroRatio,
              icm_gyro.y*Gyro_Gr*GyroRatio,
              icm_gyro.z*Gyro_Gr*GyroRatio,
              icm_acc.x * AcceRatio,
              icm_acc.y * AcceRatio,
              icm_acc.z * AcceRatio);    // 姿态解算出欧拉角
}


//===============================四元素============================================
#define Kp 1.6f //10.0f                 // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f//1.2f // //0.008f     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                    // half the sample period采样周期的一半, 采样周期8ms，
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;  // scaled integral error
/*
 * 函数名：IMUupdate
 * 描述  ：四元素解算欧拉角
 * 输入  ：陀螺仪 加速度计
 * 输出  ：无
 * 调用  ：内部调用
 */
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if (ax*ay*az == 0)
    {
        return;
    }

    norm = sqrt(ax*ax + ay*ay + az*az); // acc数据归一化
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    // estimated direction of gravity and flux (v and w)    估计重力方向和流量/变迁
    vx = 2*(q1q3 - q0q2);                                   // 四元素中xyz的表示
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) ;      // 向量外积在相减得到差分就是误差
    ey = (az*vx - ax*vz) ;
    ez = (ax*vy - ay*vx) ;

    exInt = exInt + ex * Ki;    // 对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;    // 将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;    // 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise  // 四元素的微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    Attitude_Angle.x = asin(-2*q1*q3 + 2*q0*q2) * 57.3; // pitch
    Attitude_Angle.x = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3; // roll
    Attitude_Angle.z = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)*57.3; // yaw
//    Attitude_Angle.z = 0;
}


/*
 * 函数名：icm20602_data_update
 * 描述  ：用于定时更新传感器数据
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void icm20602_data_update(void)
{
    static float acc_angle;
    if(gyro_accumulate_status)
    {
        yaw += icm_gyro.x * dt;   // 航向角
        acc_angle = atan2(-icm_acc.y, icm_acc.z) * 180 / PI;   // y  z
        yijielvbo(&hubu_angle, &acc_angle, &icm_gyro.x);

    }
    else
        yaw = 0, hubu_angle = 0, last_angle = 0;
}


// 坡道检测, 每隔5ms调用一次
extern uint16 ramp_encoder_count, ramp_encoder_count_flag;

void ramp_detect(void)
{
    static uint8 ramp_status;
    static uint8 start_pitch;
    static uint16 encoder_count;
    static uint16 start_encoder;
    if(start_pitch)
    {
        pitch += icm_gyro.y * dt;     // 俯仰角
    }
    if(start_encoder)
    {
        encoder_count += encoder;
        if(encoder_count > 1200 * 6)
        {
            ramp_status = 0;
            encoder_count = 0;
            start_encoder = 0;

        }
    }
    switch(ramp_status)
    {
        case 0:       /*准备上坡*/
            flag.ramp_flag = 0;
            if(adc_original[4] > 200 && LAD + RAD < 150)
            {
                ramp_status++;
                pitch = 0;
                start_pitch = 1;

                encoder_count = 0;
                start_encoder = 1;
                flag.ramp_flag = 1;
//                flag.buzzer_flag = 1;
            }
            break;
        case 1:       /*上坡*/
//            flag.ramp_flag = 1;
            if(pitch > 10)
            {
                ramp_status++;
            }
            break;
        case 2:      /*坡顶*/
//            flag.ramp_flag = 1;
            if(get_absf(pitch - 0.0) < 2)
            {
                ramp_status++;
//                flag.buzzer_flag = 1;
            }
            break;
        case 3:
//            flag.ramp_flag = 1;
            if(pitch < -10)
            {
                ramp_status++;
//                flag.buzzer_flag = 1;
            }
            break;
        case 4:
//            flag.ramp_flag = 1;
            if(get_absf(pitch - 0.0) < 2)
            {
                ramp_status = 0;
                start_pitch = 0;
                start_encoder = 0;
                flag.ramp_flag = 0;
//                flag.buzzer_flag = 1;
            }
    }





}


float hubu_angle;
float acc_ratio = 0.05;      //加速度计比例
float gyro_ratio = 0.95;    //陀螺仪比例

float last_angle;
//
//
//
//----------------------------------------------------------------
//  @brief      一阶互补滤波
//  @param      angle_m     加速度计数据
//  @param      gyro_m      陀螺仪数据
//  @return     float       数据融合后的角度
//----------------------------------------------------------------
float angle_calc(float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;
    static uint8 first_angle;

    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio;

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle)*acc_ratio;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now)*dt;

    //保存当前角度值
    last_angle = temp_angle;

    return temp_angle;
}


float K1 = 0.02f;
void yijielvbo(float *angle_now, float *angle_m, float *gyro_m)
{
    static float angle_last;     // 加速度算出的上次的角度
    *angle_now +=  K1 * (*angle_m - angle_last) + (1 - K1) * (*gyro_m) * dt;
    angle_last = *angle_m;
}
