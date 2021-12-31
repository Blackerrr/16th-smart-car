#include "adc.h"

extern uint8 return_to_one_flag, flash_update_flag;                   // 归一化标志，

uint16 queue[10][10];
uint8 adc_caiji(void)
{
    static uint8 k = 0, flag = 0;
    for(uint8 i = 1; i <= 8; i++)
    {
        /************* 算术平均滤波 *****************/
        adc[i] = adc_mean_filter(ADC_0, i - 1, ADC_10BIT, 10);

        /************* 滑动平均滤波 *****************/
        queue[i][k++] = adc[i];
        if(k == NM)
        {
            flag = 1;
        }
        k %= NM;
        if(flag)
        {
            adc[i] = 0;
            for(uint8 j = 0; j < NM; j++)
            {
                adc[i] += queue[i][j];
            }
            adc[i] /= NM;
        }

    }
    for(uint8 i = 1; i <= 8; i++)
    {
        if(gpio_get(sw1))
        {
            return_to_one(i);
            adc_original[i] = adc[i];
        }
        else
        {
            adc_original[i] = adc[i];
            return_to_one(i);
        }
    }
    return_to_one_xianfu(4);
    return_to_one_xianfu(2);
    return_to_one_xianfu(6);

//    return_to_one_xianfu(6);
//    return_to_one_xianfu(7);
//    return_to_one_xianfu(8);



    return flag;      // 0: 未滤波  1 : 已经滤波
}


void return_to_one(uint8 i)
{
    uint32 write_buf;
    if(flag.return_to_one_flag)
    {

        if(gpio_get(sw1) == 1)
        {
            if(adc[i] > adc_max[i])
            {
                adc_max[i] = adc[i];                     // 记录最大值
            }
            if(adc[i] < adc_min[i])
            {
                adc_min[i] = adc[i];
            }
        }
        else
        {
            if(adc[i] > adcSingleLineMax[i])
            {
                adcSingleLineMax[i] = adc[i];
            }
            if(adc[i] < adcSingleLineMin[i])
            {
                adcSingleLineMin[i] = adc[i];
            }
        }
    }
    if(flag.flash_update_flag)
    {
        flag.flash_update_flag = 0;
        // eeprom 写之前先擦除
        // 将最新的最大值，最小值写入flash
        flag.buzzer_flag = 1;
        if(gpio_get(sw1) == 1)
        {
            eeprom_erase_sector(3);
            eeprom_erase_sector(4);
            for(int i = 1; i <= 8; i++)
            {
                write_buf = adc_min[i];
                //将最小值写入扇区0的1-7页
                eeprom_page_program(3, i, &write_buf);

                write_buf = adc_max[i];
                eeprom_page_program(4, i, &write_buf);
            }
        }
        else
        {
            eeprom_erase_sector(5);
            eeprom_erase_sector(6);
            for(int i = 1; i <= 8; i++)
            {
                write_buf = adcSingleLineMin[i];
                //将最小值写入扇区0的1-7页
                eeprom_page_program(5, i, &write_buf);

                write_buf = adcSingleLineMax[i];
                eeprom_page_program(6, i, &write_buf);
            }
        }
    }
    /************* 归一化 *****************/
#if 1
    if(flag.single_line_Flag)
    {
        // 单线归一化
        if(adc[i] < adcSingleLineMin[i])
            adc[i] = adcSingleLineMin[i];
        adc[i] = (adc[i] - adcSingleLineMin[i]) * 100 / (adcSingleLineMax[i] - adcSingleLineMin[i]);
    }
    else
    {
        // 双线归一化
        if(adc[i] < adc_min[i])
            adc[i] = adc_min[i];
        adc[i] = (adc[i] - adc_min[i]) * 100 / (adc_max[i] - adc_min[i]);          // 将值转换成100以内
    }
    if(adc[i] < 1)
        adc[i] = 1;
#endif

}

void return_to_one_xianfu(uint8 i)
{
    if(adc[i] > 100)
        adc[i] = 100;
}

void adc_config(void)
{
    // 电池电压监测初始化
    adc_init(ADC_0, ADC0_CH8_A8);

    // 电感采集初始化
    adc_init(ADC_0, ADC0_CH0_A0); // 1
    adc_init(ADC_0, ADC0_CH1_A1); // 2
    adc_init(ADC_0, ADC0_CH2_A2); // 3
    adc_init(ADC_0, ADC0_CH3_A3); // 4
    adc_init(ADC_0, ADC0_CH4_A4); // 5
    adc_init(ADC_0, ADC0_CH5_A5); // 6
    adc_init(ADC_0, ADC0_CH6_A6); // 7
    adc_init(ADC_0, ADC0_CH7_A7); // 8

    flag.return_to_one_flag = 0;          // 默认不更新最大值，最小值 可以通过按键改变其状态
    for(int i = 1; i <= 8; i++)
    {
        // 正常双线
        // 从flash中读取最大值和最小值
        adc_min[i] = flash_read(3, i, uint16);
        adc_max[i] = flash_read(4, i, uint16);
    }
    for(int i = 1; i <= 8; i++)
    {
        adcSingleLineMin[i] = flash_read(5, i, uint16);
        adcSingleLineMax[i] = flash_read(6, i, uint16);
    }
//    adc_max[1] = 800;
//    adc_max[7] = 550;
//    adc_max[4] = 260;
//////    adc_min[3] = 10;
//    adc_max[3] = 700;
//////    adc_min[5] = 3;
//    adc_max[5] = 700;
//////    adc_min[2] = 30;
//////    adc_min[6] = 5;
//    adc_max[2] = 650;
//    adc_max[6] = 650;
//    adc_max[8] = 270;
}
