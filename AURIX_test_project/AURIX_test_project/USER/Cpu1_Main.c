#include "headfile.h"
#pragma section all "cpu1_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


uint8 cpu1_delay_5;
uint8 core1_count;
void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    //用户在此处调用各种初始化函数等
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);         // 5ms定时中断
    enableInterrupts();
    while (TRUE)
    {
        if(cpu1_delay_5)
        {
            cpu1_delay_5 = 0;     /*5ms定时*/

#if USING_NIMING_REPORT
            niming_report();
#endif
//
#if USING_VCAN_REPORT
            vcan_report();
#endif
        }

#if 1
//        用户在此处编写任务代码
        if(mt9v03x_finish_flag)
        {

            if(flag.tft_flag != 3)
                Image_Handle();
            flag.cpu1_using_tft = 1;
            if(flag.tft_clear_flag && !flag.cpu0_using_tft)
            {
                flag.tft_clear_flag = 0;
                lcd_clear(WHITE);
            }

            if(!flag.cpu0_using_tft)
            {
                if(flag.tft_flag == 3)
                {
                    lcd_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//                    Image_Handle();

                }
                else if(flag.tft_flag == 4)
                {
                    lcd_debug();
                }
            }
            flag.cpu1_using_tft = 0;
            mt9v03x_finish_flag = 0;//在图像使用完毕后  务必清除标志位，否则不会开始采集下一幅图像
//            注意：一定要在图像使用完毕后在清除此标志位
        }
#endif

    }
}



#pragma section all restore
