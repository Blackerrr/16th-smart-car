#ifndef __IMAGE_H
#define __IMAGE_H

#include"headfile.h"

#define MISS 255
#define MISSL 0
#define MISSR 255
#define NEAR_LINE 85
#define FAR_LINE 10
#define LEFT_LINE 5
#define RIGHT_LINE 183
#define REDUCE 1.2
#define left_max 101
#define right_max 71
#define TOPleft 30
#define TOPright 158
#define red 120
#define DV  120
#define white 255
#define black 0
#define TOP_NEAR 80
#define TOP_FAR 10
extern uint32 YZ;    // ��ֵ
//#define YZ 88
#define repair_line 40
typedef struct{
    int x;
    int y;
}axis;
extern uint8 left_line[MT9V03X_H],right_line[MT9V03X_H];
extern float mid_line[MT9V03X_H];
extern uint8 road_status;
extern int value_num;
extern int tile;
void Image_Handle(void);//ͼ�����ܺ���
double get_K(int y1, int y2, uint8 *x2);//kֵ��ȡ
int mmin(int a, int b);//��Сֵ��ȡ
int mmax(int a,int b);//���ֵ��ȡ
void mmax_min(axis *max,axis *min, int x, int y);//������������С������
void edge_detection(void);//��Ե��ȡ����
void edge_detec(void);//��Ե��ȡ�Զ���
void middle_detection(void);//���ߺϳ�
void crossroad(void);//ʮ���б�
void get_dif(void);//ƫ���ȡ
void get_two(uint8 Value);//ͼ���ֵ��
void get_image(void);//ͼ���ȡ
void sc_judge(void);//�ж�����·��
void status_judge(void);//�ж�����״̬
void get_top(void);//��ȡ��ͷ��
void lcd_debug(void);
float regression(int startline,int endline);
void value_judge(int value_num);
void car_judge(void);
#endif
