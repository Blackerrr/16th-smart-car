#ifndef  __CHANGE_H
#define  __CHANGE_H
#include "headfile.h"
#include "SEEKFREE_WIRELESS.h"

extern unsigned char DataScope_OutPut_Buffer[100];                             //������֡���ݻ�����
extern void DataScope_Get_Channel_Data(float Data,unsigned char Channel, uint8);    // дͨ�������� ������֡���ݻ�����
extern unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ���
extern void Send_data(void);
void niming_report(void);
void niming_report_voltage(void);

void wireless_vcan_report(uint8 *data, uint8 len);
void vcan_report(void);


#endif
