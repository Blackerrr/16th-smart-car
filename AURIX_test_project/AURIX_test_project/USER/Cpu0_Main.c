#include "headfile.h"
#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

_pid  servo_pid, motor_pid;       // 舵机 pid 参数, 电机 pid 参数


FLAG   flag;
ERROR  error;
SPEED  speed;
PID    pid;
GARAGE garage;

// volatile可以保证对特殊地址的稳定访问，不会出错。
// 告诉编译器compiler不能做任何优化
// 表示用volatile定义的变量会在程序外被改变,每次都必须从内存中读取，而不能把他放在cache或寄存器中重复使用。
volatile int16 servo_limit;
volatile int16 duty;
volatile int32 moto;                                  // 电机PWM
volatile uint8 motor_start;                           // 电机停止标志 为 0 时电机停止转动
volatile uint32 motor_target;                         // 编码器目标值
volatile uint8 motor_reverse_level;
volatile int16 encoder;                               // 采集到的编码器当前值
volatile int16 EncoderData[EncoderFilterLength];      // 编码器递推滤波数组
volatile uint16 delay_20, delay_5000, delay_100;      // 20ms, 5000ms 100ms 延时标志位
volatile uint16  ad_value, voltage;        // 电池电压
volatile uint16 adc[10], adc_original[10], adc_max[10], adc_min[10], changqiang;



volatile RINGQ *que, _que;                       // 环形队列
volatile uint16 ringq_length;                    // 环形队列长度


// 0 电感  1 电机舵机
volatile kalman_struct kalman1, kalman2;


volatile uint32 timevar;                     // 计时时间
volatile uint8  sw_code;                     // 四位编码器编码，对应16个状态 0 - 15


volatile uint8 tri_road_status;             // 车在三叉的状态
volatile uint8 tri_road_dir;                // 三叉方向

volatile float A, B;                         // 动态PID系数
volatile float yaw, pitch;                   // 航向角，俯仰角
volatile float dt;                           // 0.005

//volatile uint8 lost_line_dir;                // 丢线方向   1: 左丢线   2 : 右丢线
volatile uint8 reed_value;                   // 干簧管
uint32 current_time;

uint32 temp;
uint8 last_dir;

uint8 curve_status;

uint16 encoder_circle_count;

/* 拨码开关左 1 右 0 */
/*************************************电感归一化参数******************************************/
volatile uint16 adcSingleLineMax[10];        // 三叉单线最大值
volatile uint16 adcSingleLineMin[10];       // 三叉单线最小值
volatile uint16 adc_max[10], adc_min[10];

/*************************************环岛参数******************************************/
volatile uint8 round_status[RoundMaxNum];    // 0:左大环  1：右大环  2 左小环  // 右小环
volatile uint8 round_num;                    // 当前赛道环岛数量
volatile uint8 count_round;                  // 下一个
volatile uint8 round_flag;                   // 环岛状态量标志位
volatile uint16 bigEncoderValue, smallEncoderValue;      // 大小环岛编码器计步值
volatile uint16 round_angle_value;                       // 进环角度积分值
volatile uint16 outround_encoder_value;                  // 出环编码器积分值
volatile uint16 round_mad;                               // 环岛中间电感标定值



/*************************************三叉参数******************************************/
volatile uint8 tri_road_mad_value;                      // 入三叉中间电感值
volatile uint8 out_tri_road_value;                      // 出三叉电感阈值
uint8 in_tri_road_angle;                                   // 入三叉陀螺仪角度
uint8 out_tri_road_angle;                                   // 入三叉陀螺仪角度


/*************************************坡道参数******************************************/
uint8 ramp_angle;                                       // 坡道识别角度

uint16 Lostlinelimit;





//  sw4  屏蔽环岛和三叉
//  sw3  决定参数从flash 中写入还是从程序里直接赋值
//  sw2  决定发车方向
//  sw1  1: 双线归一化     0 单线归一化

int core0_main(void)
{

    BSP_Init();
    Parameter_init();
    systick_start(STM0);

	while (TRUE)
	{
	    if(delay_100)
	    {
	        /**************** 54ms ****************/
	        delay_100 = 0;
	        Switch_Scan();
            flag.cpu0_using_tft = 1;
            if(flag.tft_clear_flag && flag.cpu1_using_tft == 0)
            {
                flag.tft_clear_flag = 0;
                lcd_clear(WHITE);
            }
            if(flag.tft_flag == 1 && flag.cpu1_using_tft == 0)
            {
                tft_show2();   // 显示电感
            }
            if(flag.tft_flag == 2 && flag.cpu1_using_tft == 0)
            {
                tft_show1();  // 显示速度
            }
            if(flag.tft_flag == 5 && flag.cpu1_using_tft == 0)
            {
                tft_show3();  // 显示电感归一化的最大值和最小值
            }
            flag.cpu0_using_tft = 0;
	    }
	    if(delay_20)
	    {
	        delay_20 = 0;
	        FiveButton_Scan();        /* 五向按键检测 */
            /**************** 5ms ****************/
//	        niming_report();
//	        niming_report_voltage();
//	        temp = systick_getval_us(STM0);
//	        vcan_report();                             // 1.5ms
//	        timevar = systick_getval_us(STM0) - temp;

	    }
	    if(flag.two_buzzer_flag)
	    {
            flag.two_buzzer_flag = 0;
            gpio_set(BEEP_PIN,1);
            systick_delay_ms(STM0, 500);
            gpio_set(BEEP_PIN,0);

	    }
	    if(flag.buzzer_flag)
	    {
	        flag.buzzer_flag = 0;
	        gpio_set(BEEP_PIN,1);
	        systick_delay_ms(STM0, 200);
	        gpio_set(BEEP_PIN,0);
	    }
	}

}

#pragma section all restore


