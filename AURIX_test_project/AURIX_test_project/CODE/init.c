#include "init.h"


extern  _pid  servo_pid;
extern  _pid  motor_pid;


//extern kalman_struct kalman2;
extern RINGQ *que, _que;                           // 环形队列
void BSP_Init(void)
{
    disableInterrupts();
    get_clk();                              //获取时钟频率  务必保留

    gpio_init(P20_9, GPO, 0, PULLUP);       // 输出高电平， 默认上拉， 用核心板上LED 常亮表示核心板正常
    gpio_init(P20_8, GPO, 0, PULLUP);       // 输出高电平， 默认上拉， 用核心板上LED 常亮表示核心板正常
    gtm_pwm_init(S_MOTOR_PIN, 200, duty);        // 舵机居中

    mt9v03x_init(); //初始化摄像头
    tft_config();                               // tft屏幕显示初始化

    Motor_init();                               // 电机初始化
#if 0                                           // 电机测试
    pwm_duty(MOTOR2_A, 2000);
    pwm_duty(MOTOR2_B, 0);
    systick_delay_ms(STM0, 10000);
#endif

    gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
    /*************电机 PID*****************/
    PID_param_init(&motor_pid);
#if MOTOR_USE_POSITION_PID
    set_pid(&motor_pid, 150, 12, 0);                 // 位置 pid 速度环
    set_pid(&motor_pid, 60, 7, 0);                 // 位置 pid 速度环
#elif 1
//    set_pid(&motor_pid, 40, 4, 0);               // 增量 pid 速度环
    set_pid(&motor_pid, 100, 10, 0);               // 增量 pid 速度环
#endif
    set_pid_target(&motor_pid, motor_target * 1.0f);

    /*************舵机 PID*****************/
    PID_param_init(&servo_pid);
//#if 0
//    set_pid(&servo_pid, 0.032, 0, 3.1);                 // 增量 pid 位置环
//#elif 1
//    set_pid(&servo_pid, pid.B, 0, 30);               // 位置 pid 位置环
//#endif
    set_pid_target(&servo_pid, 0);

    // 无线串口初始化
    seekfree_wireless_init();

    // 拨码开关初始化
    Switch_Init();

    /* 五向按键初始化 */
    FiveButton_Init();

    // 矩阵键盘初始化
    keyboard_init();

#if USING_ICM20602
//    // 陀螺仪初始化
    icm20602_init_spi();
    systick_delay_ms(STM0, 10);  //上电延时
    get_icm_offset();
#endif

    // kalman_init(&kalman1, 1, 1);
//    kalman_init(&kalman2, 1, 1);

    //初始化蜂鸣器引脚
    gpio_init(BEEP_PIN, GPO, 0, PUSHPULL);

    /***************** ADC *****************/
    adc_config();

    que = &_que;
    ringq_init(que);


    // 干簧管引脚初始化
    gpio_init(REED_PIN, GPI, 0, PUSHPULL);


    pit_interrupt_ms(CCU6_0, PIT_CH1, 5);        // 5ms定时中断
    pit_interrupt_ms(CCU6_0, PIT_CH0, 5);       // 10ms定时中断
    systick_delay_ms(STM0, 1000);                // 延时 1 秒

    enableInterrupts();
}


void Parameter_init(void)
{
    uint32 read_buf;
    motor_start = 1;
    duty = SERVO_MID;

    ringq_length = 50;
    dt = 0.005;
    servo_limit = 650;

    /**********标志参数初始化***********/
    memset(&flag, 0, sizeof(flag));

    flag.tft_flag = 1;
    tri_road_dir =  1;    // 第一次往右拐

    A = 2.5;
    B = 2.0;
    pid.straightKd = 100;

    // 长直道入弯PD值
    bigEncoderValue = 1350;
    smallEncoderValue = 1600;
    tri_road_mad_value = 70;     // 三叉阈值
    out_tri_road_value = 200;


    garage.flag = 1;
    garage.angle = 75;
    garage.dir = 0;               // 左出库， 左入库
    round_mad = 180;
    ramp_angle = 10;

    /**********环岛参数初始化***********/
    round_num = 2;   // 2个环岛
    round_status[0] = 1;   // 右大环
    round_status[1] = 1;   // 右大环
    count_round = 0;

    Lostlinelimit = 40;

    in_tri_road_angle  = 30;
    out_tri_road_angle = 30;


    if(gpio_get(sw2))
    {
        round_status[0] = 1;   // 右大环
        round_status[1] = 1;   // 右大环
        garage.dir = 0;               // 左出库， 左入库
    }
    else
    {
        round_status[0] = 0;   // 右大环
        round_status[1] = 0;   // 右大环
        garage.dir = 1;               // 左出库， 左入库
    }
    if(gpio_get(sw3))                    // 从 flash 里读参数
    {
        /***********************从flash中读取三叉路口参数***********************/
        read_buf = flash_read(2, 2, uint32);
        pid.triRoadA = read_buf / 10.0f;

        read_buf = flash_read(2, 3, uint32);
        pid.triToadB = read_buf / 10.0f;

        read_buf = flash_read(2, 4, uint32);
        pid.triRoadKd = read_buf / 10.0f;

        /***********************从flash中读取大环岛 小环岛 PD值***********************/

        read_buf = flash_read(2, 10, uint32);
        pid.bigRingKp = read_buf / 10.0f;

        read_buf = flash_read(2, 11, uint32);
        pid.bigRingKd = read_buf / 10.0f;

        read_buf = flash_read(2, 12, uint32);
        pid.smallRingKp = read_buf / 10.0f;

        read_buf = flash_read(2, 13, uint32);
        pid.smallRingKd = read_buf / 10.0f;

        read_buf = flash_read(2, 14, uint32);
        pid.curve_kp = read_buf / 10.0f;

        read_buf = flash_read(2, 15, uint32);
        pid.curve_kd = read_buf / 10.0f;

        read_buf = flash_read(2, 16, uint32);
        pid.basic_kp = read_buf / 10.0f;

        read_buf = flash_read(2, 17, uint32);
        pid.basic_kd = read_buf / 10.0f;
        /***********************从flash中读取各个档位速度***********************/

        speed.closed_speed[0] = flash_read(2, 5, uint32);
        speed.closed_speed[1] = flash_read(2, 6, uint32);
        speed.closed_speed[2] = flash_read(2, 7, uint32);
        speed.closed_speed[3] = flash_read(2, 8, uint32);
        speed.closed_speed[4] = flash_read(2, 9, uint32);

        speed.curve_low_speed  = flash_read(2, 19, uint32);
        speed.curve_high_speed = flash_read(2, 20, uint32);

        read_buf  = flash_read(2, 21, uint32);
        A = read_buf / 10.0f;

        read_buf  = flash_read(2, 22, uint32);
        B = read_buf / 10.0f;

        read_buf  = flash_read(2, 23, uint32);
        pid.straightKd = read_buf / 10.0f;

        read_buf = flash_read(2, 24, uint32);
        pid.LRTC_kp = read_buf / 10.0f;

        read_buf = flash_read(2, 25, uint32);
        pid.LRTC_kd = read_buf / 10.0f;
    }
    else // 程序直接写参数
    {

        A = 3.2;
        B = 2.9;
        pid.straightKd = 92;



        pid.smallRingKp = 7.0;
        pid.smallRingKd = 180;
        pid.bigRingKp = 7.0;
        pid.bigRingKd = 101;

        pid.triRoadA = 2.6;
        pid.triToadB = 2;
        pid.triRoadKd = 45;

        pid.basic_kp = 8.5;     // 弯内
        pid.basic_kd = 85;


        pid.curve_kp = 18;    // 丢线
        pid.curve_kd = 200;

        pid.LRTC_kp = 11;     // 入弯
        pid.LRTC_kd = 150;


        speed.closed_speed[0] = 15;
        speed.closed_speed[1] = 40;
        speed.closed_speed[2] = 55;
        speed.closed_speed[3] = 60;
        speed.closed_speed[4] = 90;
        speed.curve_low_speed  = 55;
        speed.curve_high_speed = 70;

        round_mad = 180;
    }
}
