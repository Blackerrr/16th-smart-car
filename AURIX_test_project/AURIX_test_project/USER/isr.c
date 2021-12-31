#include "isr_config.h"
#include "isr.h"
#include "headfile.h"
//��isr.c���жϺ�������������ĵڶ��������̶�Ϊ0���벻Ҫ���ģ���ʹ����CPU1�����ж�Ҳ��Ҫ���ģ���ҪCPU1�����ж�ֻ��Ҫ��isr_config.h���޸Ķ�Ӧ�ĺ궨�弴��

extern _pid motor_pid;
extern uint8 core1_count;
uint16 delay_count1, delay_count2, delay_count3, delay_count4, delay_100_count;
uint8 test_speed[7] = {40, 90, 150, 120, 60, 100, 70};   // ���Ե����



// 10ms��ʱ�ж�, ���ڵ�����ƣ����ȼ�30
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint8 count = 0;
	enableInterrupts();                   //�����ж�Ƕ��
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
#if MOTOR_DEBUG
    static uint8 i = 0;
    if(++delay_count2 > 500)
    {
        gpio_toggle(P20_9);
        delay_count2 = 0;
        set_pid_target(&motor_pid, 2 * test_speed[i++]);
        i %= 7;
    }
#endif
}

// 5 ms ��ʱ�жϣ����ڶ�����ƣ����ȼ�31
// 750us
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    static uint8 count;
    static uint32 temp;
    static uint8 garage_count = 0;
	enableInterrupts();//�����ж�Ƕ��
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);


//    temp = systick_getval_us(STM0);
	current_time++;              // ��¼��ǰʱ��
    if(++delay_count1 > 4)
    {
        delay_count1 = 0;
        delay_20     = 1;                   // 50 ms��ʱ��־λ
    }
    if(++delay_count3 > 200)                // 1000 ms
    {
        gpio_toggle(P20_9);                 // LED ��˸��ʾϵͳ��������
        delay_count3 = 0;
    }
    if(++delay_100_count > 19)
    {
        delay_100_count = 0;
        delay_100 = 1;
    }


    /**********��ȡ����������***********/
    encoder = gpt12_get(GPT12_T5);
    encoder = RecurrenceFilter(encoder);     /*�����������˲�*/
    if(encoder < 0)
        encoder = 0;
    gpt12_clear(GPT12_T5);


    /***********�������****************/
    KeyScan();
    Check_BottonPress();     /*15us*/

    /********** ������ݲɼ�***************/
    adc_caiji();

    /********** �ɻɹܼ��***************/
//    reed_value = gpio_get(REED_PIN);
    if(flag.already_in_tri_road && road_status == 3 && !flag.enable_enter_garage)
    {
        flag.buzzer_flag = 1;
        tri_road_dir = 1 - tri_road_dir;
        flag.already_in_tri_road = 0;
        flag.enable_enter_garage = 1;
    }


    /********** ���������ݴ���*************/
#if USING_ICM20602
    Icm_Data_Filter();
    icm20602_data_update();
#endif

    ramp_detect();

    if(flag.enable_enter_garage && flag.already_in_tri_road)
    {
        enter_garage();                   /*�����*/
        if(flag.enter_garage)
        {
//            Motor_control(encoder);
//            switch()
//            {
//                case 0:
//                case 1:
//            }
            return;
        }
    }

    /**************��������***************/
    if(sw4_status)
    {
        circle_handle_new_3();             /* ����״̬�� */
    }
    tri_road_handle();

    road_detect();                    /* ·����Ⲣ��ֵ��ͬ���ٶ� */
    /*���⴦��*/
    garage_handle();
#if 1
    /***************�������***************/
    if(garage.flag)
    {
        switch(garage.dir)
        {
            case 0:                /*�����*/
                duty = SERVO_MID - servo_limit;
                break;
            case 1:                /*�ҳ���*/
                duty = SERVO_MID + servo_limit;
        }
        pwm_duty(S_MOTOR_PIN, duty);
    }
    else
        Servo_control();
#endif

    /***************�������***************/
     encoder_handle();
#if STAP_CAR_DETECT
     stop_car_detect();       /* ͣ����� */
#endif
     Motor_control(encoder);

//    timevar = systick_getval_us(STM0) - temp;
}



// 5ms ��ʱ
extern uint8 cpu1_delay_5;
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{

	enableInterrupts();//�����ж�Ƕ��
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);
	cpu1_delay_5 = 1;

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
	enableInterrupts();//�����ж�Ƕ��
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
	if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//ͨ��0�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
	}

	if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//ͨ��4�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
	}
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
	if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//ͨ��1�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
	}

	if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//ͨ��5�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
	}
}

//��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//	enableInterrupts();//�����ж�Ƕ��
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//ͨ��2�ж�
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//ͨ��6�ж�
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}



// �����
IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
	if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//ͨ��3�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
		if		(1 == camera_type)	mt9v03x_vsync();    // �����
		else if	(3 == camera_type)	ov7725_vsync();

	}
	if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//ͨ��7�ж�
	{
		CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

	}
}



IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��

	if		(1 == camera_type)	mt9v03x_dma();
	else if	(3 == camera_type)	ov7725_dma();
}


//�����жϺ���  ʾ��
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
	enableInterrupts();//�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
