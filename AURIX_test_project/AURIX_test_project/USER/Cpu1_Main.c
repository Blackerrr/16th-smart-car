#include "headfile.h"
#pragma section all "cpu1_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��


uint8 cpu1_delay_5;
uint8 core1_count;
void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    //�û��ڴ˴����ø��ֳ�ʼ��������
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);         // 5ms��ʱ�ж�
    enableInterrupts();
    while (TRUE)
    {
        if(cpu1_delay_5)
        {
            cpu1_delay_5 = 0;     /*5ms��ʱ*/

#if USING_NIMING_REPORT
            niming_report();
#endif
//
#if USING_VCAN_REPORT
            vcan_report();
#endif
        }

#if 1
//        �û��ڴ˴���д�������
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
            mt9v03x_finish_flag = 0;//��ͼ��ʹ����Ϻ�  ��������־λ�����򲻻Ὺʼ�ɼ���һ��ͼ��
//            ע�⣺һ��Ҫ��ͼ��ʹ����Ϻ�������˱�־λ
        }
#endif

    }
}



#pragma section all restore
