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
extern uint32 YZ;    // 阈值
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
void Image_Handle(void);//图像处理总函数
double get_K(int y1, int y2, uint8 *x2);//k值获取
int mmin(int a, int b);//最小值获取
int mmax(int a,int b);//最大值获取
void mmax_min(axis *max,axis *min, int x, int y);//求行数最大和最小的列数
void edge_detection(void);//边缘获取备份
void edge_detec(void);//边缘获取自定义
void middle_detection(void);//中线合成
void crossroad(void);//十字判别
void get_dif(void);//偏差获取
void get_two(uint8 Value);//图像二值化
void get_image(void);//图像获取
void sc_judge(void);//判断三叉路口
void status_judge(void);//判断赛道状态
void get_top(void);//获取盖头线
void lcd_debug(void);
float regression(int startline,int endline);
void value_judge(int value_num);
void car_judge(void);
#endif
