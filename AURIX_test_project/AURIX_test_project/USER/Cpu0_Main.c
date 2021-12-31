#include "headfile.h"
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

_pid  servo_pid, motor_pid;       // ��� pid ����, ��� pid ����


FLAG   flag;
ERROR  error;
SPEED  speed;
PID    pid;
GARAGE garage;

// volatile���Ա�֤�������ַ���ȶ����ʣ��������
// ���߱�����compiler�������κ��Ż�
// ��ʾ��volatile����ı������ڳ����ⱻ�ı�,ÿ�ζ�������ڴ��ж�ȡ�������ܰ�������cache��Ĵ������ظ�ʹ�á�
volatile int16 servo_limit;
volatile int16 duty;
volatile int32 moto;                                  // ���PWM
volatile uint8 motor_start;                           // ���ֹͣ��־ Ϊ 0 ʱ���ֹͣת��
volatile uint32 motor_target;                         // ������Ŀ��ֵ
volatile uint8 motor_reverse_level;
volatile int16 encoder;                               // �ɼ����ı�������ǰֵ
volatile int16 EncoderData[EncoderFilterLength];      // �����������˲�����
volatile uint16 delay_20, delay_5000, delay_100;      // 20ms, 5000ms 100ms ��ʱ��־λ
volatile uint16  ad_value, voltage;        // ��ص�ѹ
volatile uint16 adc[10], adc_original[10], adc_max[10], adc_min[10], changqiang;



volatile RINGQ *que, _que;                       // ���ζ���
volatile uint16 ringq_length;                    // ���ζ��г���


// 0 ���  1 ������
volatile kalman_struct kalman1, kalman2;


volatile uint32 timevar;                     // ��ʱʱ��
volatile uint8  sw_code;                     // ��λ���������룬��Ӧ16��״̬ 0 - 15


volatile uint8 tri_road_status;             // ���������״̬
volatile uint8 tri_road_dir;                // ���淽��

volatile float A, B;                         // ��̬PIDϵ��
volatile float yaw, pitch;                   // ����ǣ�������
volatile float dt;                           // 0.005

//volatile uint8 lost_line_dir;                // ���߷���   1: ����   2 : �Ҷ���
volatile uint8 reed_value;                   // �ɻɹ�
uint32 current_time;

uint32 temp;
uint8 last_dir;

uint8 curve_status;

uint16 encoder_circle_count;

/* ���뿪���� 1 �� 0 */
/*************************************��й�һ������******************************************/
volatile uint16 adcSingleLineMax[10];        // ���浥�����ֵ
volatile uint16 adcSingleLineMin[10];       // ���浥����Сֵ
volatile uint16 adc_max[10], adc_min[10];

/*************************************��������******************************************/
volatile uint8 round_status[RoundMaxNum];    // 0:���  1���Ҵ�  2 ��С��  // ��С��
volatile uint8 round_num;                    // ��ǰ������������
volatile uint8 count_round;                  // ��һ��
volatile uint8 round_flag;                   // ����״̬����־λ
volatile uint16 bigEncoderValue, smallEncoderValue;      // ��С�����������Ʋ�ֵ
volatile uint16 round_angle_value;                       // �����ǶȻ���ֵ
volatile uint16 outround_encoder_value;                  // ��������������ֵ
volatile uint16 round_mad;                               // �����м��б궨ֵ



/*************************************�������******************************************/
volatile uint8 tri_road_mad_value;                      // �������м���ֵ
volatile uint8 out_tri_road_value;                      // ����������ֵ
uint8 in_tri_road_angle;                                   // �����������ǽǶ�
uint8 out_tri_road_angle;                                   // �����������ǽǶ�


/*************************************�µ�����******************************************/
uint8 ramp_angle;                                       // �µ�ʶ��Ƕ�

uint16 Lostlinelimit;





//  sw4  ���λ���������
//  sw3  ����������flash ��д�뻹�Ǵӳ�����ֱ�Ӹ�ֵ
//  sw2  ������������
//  sw1  1: ˫�߹�һ��     0 ���߹�һ��

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
                tft_show2();   // ��ʾ���
            }
            if(flag.tft_flag == 2 && flag.cpu1_using_tft == 0)
            {
                tft_show1();  // ��ʾ�ٶ�
            }
            if(flag.tft_flag == 5 && flag.cpu1_using_tft == 0)
            {
                tft_show3();  // ��ʾ��й�һ�������ֵ����Сֵ
            }
            flag.cpu0_using_tft = 0;
	    }
	    if(delay_20)
	    {
	        delay_20 = 0;
	        FiveButton_Scan();        /* ���򰴼���� */
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


