#include "change.h"
#include "stdlib.h"
#include "math.h"

uint8 DataScope_OutPut_Buffer[100] = {0}; //���ڷ��ͻ�����


// ����ת��
void Float2Byte(float *target, unsigned char *buf, unsigned char beg, uint8 flag)
{
    unsigned char *point;
    point = (unsigned char *)target; //�õ�float�ĵ�ַ
    if(flag)                    // ���ģʽ
    {
        buf[beg] = point[3];
        buf[beg + 1] = point[2];
        buf[beg + 2] = point[1];
        buf[beg + 3] = point[0];
    }
    else                        // С��ģʽ
    {
        buf[beg] = point[0];
        buf[beg + 1] = point[1];
        buf[beg + 2] = point[2];
        buf[beg + 3] = point[3];
    }
}

//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷���
void DataScope_Get_Channel_Data(float Data, unsigned char Channel, uint8 flag)
{
    if ((Channel > 20) || (Channel == 0))
        return; //ͨ����������10�����0��ֱ����������ִ�к���
    else
    {
        switch (Channel)
        {
        case 1:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 1, flag);
            break;
        case 2:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 5, flag);
            break;
        case 3:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 9, flag);
            break;
        case 4:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 13, flag);
            break;
        case 5:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 17, flag);
            break;
        case 6:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 21, flag);
            break;
        case 7:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 25, flag);
            break;
        case 8:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 29, flag);
            break;
        case 9:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 33, flag);
            break;
        case 10:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 37, flag);
            break;
        case 11:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 41, flag);
            break;
        case 12:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 45, flag);
            break;
        case 13:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 49, flag);
            break;
        case 14:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 53, flag);
            break;
        case 15:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 57, flag);
            break;
        case 16:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 61, flag);
            break;
        case 17:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 65, flag);
            break;
        case 18:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 69, flag);
            break;
        case 19:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 73, flag);
            break;
        case 20:
            Float2Byte(&Data, DataScope_OutPut_Buffer, 77, flag);
            break;

        }
    }
}

/*******************����������λ����ֲ********************/

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void wireless_niming_report(uint8 fun, uint8* data, uint8 len)
{
    uint8 send_buf[100];
    uint8 i;
    // if(len>28)
    //    return;   //���28�ֽ�����
    send_buf[len+3]=0;  //У��������
    send_buf[0] = 0X88;   //֡ͷ
    send_buf[1] = fun;    //������
    send_buf[2] = len;    //���ݳ���
    for(i = 0; i < len; i++)
        send_buf[3+i] = data[i];            //��������
    for(i = 0; i < len+3; i++)
        send_buf[len+3] += send_buf[i];     //����У���
    seekfree_wireless_send_buff(send_buf, len+4);
}

// ������ͨ������
void niming_report(void)
{
    uint8 channel_number = 0;
    /*****************************����ͨ������************************************/
    DataScope_Get_Channel_Data(icm_acc.x, 1, 1);
    DataScope_Get_Channel_Data(icm_acc.y, 2, 1);
    DataScope_Get_Channel_Data(icm_acc.z, 3, 1);
    DataScope_Get_Channel_Data(icm_gyro.x, 4, 1);
    DataScope_Get_Channel_Data(icm_gyro.y, 5, 1);
    DataScope_Get_Channel_Data(icm_gyro.z, 6, 1);
    DataScope_Get_Channel_Data(hubu_angle, 7, 1);
    DataScope_Get_Channel_Data(pitch, 8, 1);
    DataScope_Get_Channel_Data(round_flag, 9, 1);
    DataScope_Get_Channel_Data(pitch, 10, 1);
    DataScope_Get_Channel_Data(tri_road_status, 11, 1);
    DataScope_Get_Channel_Data(timevar, 12, 1);
    DataScope_Get_Channel_Data(0, 13, 1);
    DataScope_Get_Channel_Data(0, 14, 1);
    DataScope_Get_Channel_Data(0, 15, 1);
    DataScope_Get_Channel_Data(0, 16, 1);
    DataScope_Get_Channel_Data(0, 17, 1);
    DataScope_Get_Channel_Data(0, 18, 1);
    DataScope_Get_Channel_Data(0, 19, 1);
    DataScope_Get_Channel_Data(0, 20, 1);
    channel_number = 12;

    wireless_niming_report(0xA1, DataScope_OutPut_Buffer + 1, channel_number * 4);
}

// ����λ�����͵������
// ����Ϊuint16��ʽ����28���ֽ�
// ������Ϊ0xAE
void niming_report_voltage(void)
{
    uint8 tbuf[28];
    voltage /= 10;             // ԭ�ȵ�����Ϊmv, ���͵�����Ҫ��Ϊʵ�ʵĵ�ѹֵ����100
    tbuf[26] = voltage >> 8;   // ��8λ
    tbuf[27] = voltage & 0xff; // ��8λ
    wireless_niming_report(0xAE, tbuf, 28);    // ���͵�ѹ����
}


void wireless_vcan_report(uint8 *data, uint8 len)
{
    uint8 send_buf[50];
    send_buf[0] = 0x03;
    send_buf[1] = 0xfc;

    for(int i = 0; i < len; i++)
        send_buf[i + 2] = data[i];
    send_buf[2 + len]     = 0xfc;
    send_buf[2 + len + 1] = 0x03;
    seekfree_wireless_send_buff(send_buf, len + 4);


}

void vcan_report(void)
{
    uint8 channel_number;
    /*****************************����ͨ������************************************/
#if 1                               /*���͵������*/
    DataScope_Get_Channel_Data(adc_original[1], 1, 0);   // �����
    DataScope_Get_Channel_Data(adc_original[7], 2, 0);   // �Һ���
    DataScope_Get_Channel_Data(adc_original[4], 3, 0);
    DataScope_Get_Channel_Data(adc_original[3], 4, 0);
    DataScope_Get_Channel_Data(adc_original[5], 5, 0);
    DataScope_Get_Channel_Data(adc_original[8], 6, 0);
    DataScope_Get_Channel_Data(adc_original[6], 7, 0);
    DataScope_Get_Channel_Data(adc_original[2], 8, 0);
#elif 1                             /*��������������*/
    DataScope_Get_Channel_Data(icm_acc_x, 1, 0);
    DataScope_Get_Channel_Data(icm_acc_y, 2, 0);
    DataScope_Get_Channel_Data(icm_acc_z, 3, 0);
    DataScope_Get_Channel_Data(icm_gyro_x, 4, 0);
    DataScope_Get_Channel_Data(icm_gyro_y, 5, 0);
    DataScope_Get_Channel_Data(icm_gyro_z, 6, 0);
    DataScope_Get_Channel_Data(0, 7, 0);
    DataScope_Get_Channel_Data(0, 8, 0);
#endif
    channel_number = 8;
    wireless_vcan_report(DataScope_OutPut_Buffer + 1, channel_number * 4);


}
