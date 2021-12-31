#ifndef  __CHANGE_H
#define  __CHANGE_H
#include "headfile.h"
#include "SEEKFREE_WIRELESS.h"

extern unsigned char DataScope_OutPut_Buffer[100];                             //待发送帧数据缓存区
extern void DataScope_Get_Channel_Data(float Data,unsigned char Channel, uint8);    // 写通道数据至 待发送帧数据缓存区
extern unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数
extern void Send_data(void);
void niming_report(void);
void niming_report_voltage(void);

void wireless_vcan_report(uint8 *data, uint8 len);
void vcan_report(void);


#endif
