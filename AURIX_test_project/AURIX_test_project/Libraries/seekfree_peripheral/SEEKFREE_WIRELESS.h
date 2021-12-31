#ifndef _SEEKFREE_WIRELESS
#define _SEEKFREE_WIRELESS


#include "common.h"
#include "headfile.h"


#define WIRELESS_UART        UART_2         //����ת����ģ�� ��ʹ�õ��Ĵ���
#define WIRELESS_UART_TX     UART2_TX_P10_5
#define WIRELESS_UART_RX     UART2_RX_P10_6
#define WIRELESS_UART_BAUD   115200

#define RTS_PIN P10_2 //��������λ����  ָʾ��ǰģ���Ƿ���Խ�������  0���Լ�������  1�����Լ�������


void 	wireless_uart_callback(void);
void    seekfree_wireless_init(void);
uint32  seekfree_wireless_send_buff(uint8 *buff, uint32 len);
void Change(void);

#endif 
