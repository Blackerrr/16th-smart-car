
#include "my_DP.h"
#include "stdlib.h"
#include "math.h"

uint8 send_buffer[42] = {0}; //串口发送缓冲区

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//附加说明：用户无需直接操作此函数
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回
static void Float2Byte(float *target, uint8 *buf, uint8 beg)
{
    uint8 *point;
    point = (uint8 *)target; //得到float的地址
    buf[beg] = point[0];
    buf[beg + 1] = point[1];
    buf[beg + 2] = point[2];
    buf[beg + 3] = point[3];
}

//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）

//函数无返回
static void DataScope_Get_Channel_Data(float Data, uint8 Channel)
{
    if ((Channel > 10) || (Channel == 0))
        return; //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        switch (Channel)
        {
        case 1:
            Float2Byte(&Data, send_buffer, 1);
            break;
        case 2:
            Float2Byte(&Data, send_buffer, 5);
            break;
        case 3:
            Float2Byte(&Data, send_buffer, 9);
            break;
        case 4:
            Float2Byte(&Data, send_buffer, 13);
            break;
        case 5:
            Float2Byte(&Data, send_buffer, 17);
            break;
        case 6:
            Float2Byte(&Data, send_buffer, 21);
            break;
        case 7:
            Float2Byte(&Data, send_buffer, 25);
            break;
        case 8:
            Float2Byte(&Data, send_buffer, 29);
            break;
        case 9:
            Float2Byte(&Data, send_buffer, 33);
            break;
        case 10:
            Float2Byte(&Data, send_buffer, 37);
            break;
        }
    }
}

//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败
static uint8 DataScope_Data_Generate(uint8 Channel_Number)
{
    if ((Channel_Number > 10) || (Channel_Number == 0))
    {
        return 0;
    } //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        send_buffer[0] = '$'; //帧头

        switch (Channel_Number)
        {
        case 1:
            send_buffer[5] = 5;
            return 6;
        case 2:
            send_buffer[9] = 9;
            return 10;
        case 3:
            send_buffer[13] = 13;
            return 14;
        case 4:
            send_buffer[17] = 17;
            return 18;
        case 5:
            send_buffer[21] = 21;
            return 22;
        case 6:
            send_buffer[25] = 25;
            return 26;
        case 7:
            send_buffer[29] = 29;
            return 30;
        case 8:
            send_buffer[33] = 33;
            return 34;
        case 9:
            send_buffer[37] = 37;
            return 38;
        case 10:
            send_buffer[41] = 41;
            return 42;
        }
    }
    return 0;
}

void DataScope(void)
{
    uint8 Send_Count, temp;
    int i;
    DataScope_Get_Channel_Data(1.0, 1);   //将数据 1.0  写入通道 1
    DataScope_Get_Channel_Data(encoder, 2);   //将数据 2.0  写入通道 2
    DataScope_Get_Channel_Data(3.0, 3);   //将数据 3.0  写入通道 3
    DataScope_Get_Channel_Data(4.0, 4);   //将数据 4.0  写入通道 4
    DataScope_Get_Channel_Data(5.0, 5);   //将数据 5.0  写入通道 5
    DataScope_Get_Channel_Data(0, 6);   //将数据 6.0  写入通道 6
    // DataScope_Get_Channel_Data(circle_status, 7);   //将数据 7.0  写入通道 7
    // DataScope_Get_Channel_Data(8.0, 8);   //将数据 8.0  写入通道 8
    // DataScope_Get_Channel_Data(9.0, 9);   //将数据 9.0  写入通道 9
    // DataScope_Get_Channel_Data(10.0, 10); //将数据 10.0 写入通道 10

    Send_Count = DataScope_Data_Generate(6); //生成10个通道的 格式化帧数据，返回帧数据长度

    // send_buffer
    for (i = 0; i < Send_Count; i++) //循环发送,直到发送完毕
    {
        temp = send_buffer[i];
//        while(gpio_get(RTS_PIN));
//        uart_putbuff(WIRELESS_UART, &temp,1);
//        seekfree_wireless_send_buff(&temp, sizeof(temp));
    }
}
