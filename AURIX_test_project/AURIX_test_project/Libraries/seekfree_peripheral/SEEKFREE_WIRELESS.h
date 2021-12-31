#ifndef _SEEKFREE_WIRELESS
#define _SEEKFREE_WIRELESS


#include "common.h"
#include "headfile.h"


#define WIRELESS_UART        UART_2         //无线转串口模块 所使用到的串口
#define WIRELESS_UART_TX     UART2_TX_P10_5
#define WIRELESS_UART_RX     UART2_RX_P10_6
#define WIRELESS_UART_BAUD   115200

#define RTS_PIN P10_2 //定义流控位引脚  指示当前模块是否可以接受数据  0可以继续接收  1不可以继续接收


void 	wireless_uart_callback(void);
void    seekfree_wireless_init(void);
uint32  seekfree_wireless_send_buff(uint8 *buff, uint32 len);
void Change(void);

#endif 
