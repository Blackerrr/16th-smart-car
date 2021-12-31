#include "isr_config.h"
#include "isr.h"
#include "headfile.h"
//在isr.c的中断函数，函数定义的第二个参数固定为0，请不要更改，即使你用CPU1处理中断也不要更改，需要CPU1处理中断只需要在isr_config.h内修改对应的宏定义即可

extern _pid motor_pid;
extern uint8 core1_count;
uint16 delay_count1, delay_count2, delay_count3, delay_count4, delay_100_count;
uint8 test_speed[7] = {40, 90, 150, 120, 60, 100, 70};   // 测试电机用



// 10ms定时中断, 用于电机控制，优先级30
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint8 count = 0;
	enableInterrupts();                   //开启中断嵌套
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

// 5 ms 定时中断，用于舵机控制，优先级31
// 750us
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    static uint8 count;
    static uint32 temp;
    static uint8 garage_count = 0;
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);


//    temp = systick_getval_us(STM0);
	current_time++;              // 记录当前时间
    if(++delay_count1 > 4)
    {
        delay_count1 = 0;
        delay_20     = 1;                   // 50 ms延时标志位
    }
    if(++delay_count3 > 200)                // 1000 ms
    {
        gpio_toggle(P20_9);                 // LED 闪烁表示系统正在运行
        delay_count3 = 0;
    }
    if(++delay_100_count > 19)
    {
        delay_100_count = 0;
        delay_100 = 1;
    }


    /**********读取编码器数据***********/
    encoder = gpt12_get(GPT12_T5);
    encoder = RecurrenceFilter(encoder);     /*编码器递推滤波*/
    if(encoder < 0)
        encoder = 0;
    gpt12_clear(GPT12_T5);


    /***********按键检测****************/
    KeyScan();
    Check_BottonPress();     /*15us*/

    /********** 电感数据采集***************/
    adc_caiji();

    /********** 干簧管检测***************/
//    reed_value = gpio_get(REED_PIN);
    if(flag.already_in_tri_road && road_status == 3 && !flag.enable_enter_garage)
    {
        flag.buzzer_flag = 1;
        tri_road_dir = 1 - tri_road_dir;
        flag.already_in_tri_road = 0;
        flag.enable_enter_garage = 1;
    }


    /********** 陀螺仪数据处理*************/
#if USING_ICM20602
    Icm_Data_Filter();
    icm20602_data_update();
#endif

    ramp_detect();

    if(flag.enable_enter_garage && flag.already_in_tri_road)
    {
        enter_garage();                   /*入库检测*/
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

    /**************环岛处理***************/
    if(sw4_status)
    {
        circle_handle_new_3();             /* 环岛状态机 */
    }
    tri_road_handle();

    road_detect();                    /* 路况检测并赋值不同的速度 */
    /*车库处理*/
    garage_handle();
#if 1
    /***************舵机控制***************/
    if(garage.flag)
    {
        switch(garage.dir)
        {
            case 0:                /*左出库*/
                duty = SERVO_MID - servo_limit;
                break;
            case 1:                /*右出库*/
                duty = SERVO_MID + servo_limit;
        }
        pwm_duty(S_MOTOR_PIN, duty);
    }
    else
        Servo_control();
#endif

    /***************电机控制***************/
     encoder_handle();
#if STAP_CAR_DETECT
     stop_car_detect();       /* 停车检测 */
#endif
     Motor_control(encoder);

//    timevar = systick_getval_us(STM0) - temp;
}



// 5ms 定时
extern uint8 cpu1_delay_5;
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{

	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);
	cpu1_delay_5 = 1;

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//通道0中断
	{
		CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
	}

	if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//通道4中断
	{
		CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
	}
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//通道1中断
	{
		CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
	}

	if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//通道5中断
	{
		CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
	}
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//	enableInterrupts();//开启中断嵌套
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}



// 总钻风
IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
	{
		CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
		if		(1 == camera_type)	mt9v03x_vsync();    // 总钻风
		else if	(3 == camera_type)	ov7725_vsync();

	}
	if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
	{
		CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

	}
}



IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套

	if		(1 == camera_type)	mt9v03x_dma();
	else if	(3 == camera_type)	ov7725_dma();
}


//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
