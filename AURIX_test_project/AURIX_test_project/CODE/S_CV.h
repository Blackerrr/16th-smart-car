#ifndef _S_CV_H
#define _S_CV_H
#include "headfile.h"
typedef struct {
    uint8 L[64];
    uint8 R[64];
    uint8 M[64];
} RunWay_;
//图像大小定义
typedef struct {
    float p;
    int d;
} Get_01;
extern Get_01 average;
#define PixMini_H     120
#define PixMini_W     188
#define S_img_H     60
#define S_img_W     94
#define cnt_num 5640
#define S_S_Custom  0xAA
extern int S_nums;
void S_BinaryImage(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W], uint8 ThresholdV);
void S_SE_Operation(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
void S_SE_OperationBIG(uint8 in_IMG[120][188], uint8 out_IMG[120][188]);

/**********************************形态学处理**************************************/
//腐蚀
void S_Erode(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//膨胀
void S_Dilate(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//开操作
void S_Open(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//关操作
void S_Close(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);


/**********************************图像显示**************************************/
#define S_MINI  0
#define S_BIG       1
//显示小图像到TFT模块
void S_Draw_TFT_MINI(uint8 in_IMG[S_img_H][S_img_W]);
//60*94图像显示成188*120
void S_DisplayMINI_Big(uint8 in_IMG[PixMini_H][PixMini_W]);
//显示大图像到TFT模块
void S_Draw_TFT_BIG(uint8 in_IMG[MT9V03X_H][MT9V03X_W]);
//显示灰度图像60*90   可选大小
void S_Gray_DisplayMINI(uint8 in_IMG[PixMini_H][PixMini_W], uint8 size);
//显示灰度图像120*188
void S_Gray_Display(uint8 in_IMG[MT9V03X_H][MT9V03X_W]);
//三线图显示
void S_DisplayRunWay(RunWay_* in_Array);
//三线图显示 大图
void S_DRW_Big(RunWay_* in_Array);



/**********************************其他操作**************************************/
// 图像  抽取压缩188X120 to 94X60
void S_Image_zip(uint8 in_IMG[MT9V03X_H][MT9V03X_W], uint8 out_IMG[S_img_H][S_img_W]);
// 图像 填充放大 94X60  to  188X120
void S_Image_larger(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[MT9V03X_H][MT9V03X_W]);
//三线图检测
void DetectionLine(uint8 in_IMG[PixMini_H][PixMini_W], RunWay_ * out_Array, uint8 LMP, uint8 Mod);
//三线图定制
void DetectionLine_Custom(uint8 in_IMG[PixMini_H][PixMini_W], RunWay_* out_Array, uint8 LMP);



/*********************************求阈值***************************************/
//大津法求阈值
uint8 S_GetOSTU(uint8 tmImage[S_img_H][S_img_W]);       //修改
uint8 S_Other_OSTU(int width, int height, uint8* Image);//特殊大津法
uint8 S_Get_01_Value(uint8 tmImage[S_img_H][S_img_W]);//均值比例法
uint8 S_GetThresh(uint8 tmImage[S_img_H][S_img_W]);//自写迭代法
uint8 S_GetThresh2(uint8 tmImage[MT9V03X_H][MT9V03X_W]);   // 逐飞迭代法
uint8 Itera_Threshold(uint8 tmImage[S_img_H][S_img_W]);//新一代迭代法
uint8 IsDimodal(double gray_itera_threshold[256]);//判断是否为双峰
uint8 GetMinimumThreshold(uint8 tmImage[S_img_H][S_img_W]);//双峰法确定阈值
uint8 S_GetOSTUS(uint8 tmImage[S_img_H][S_img_W]);//模糊大津法
uint8 GetOSTU(uint8 *image, uint16 col, uint16 row);//大津法，不确定范围的大津法
uint8 S_Get_double(uint8 tmImage[S_img_H][S_img_W]);//双峰均值，限制峰的宽度为左右三十
uint8 S_Get_double2(uint8 tmImage[S_img_H][S_img_W]);//双峰均值，先求图像灰度均值，后向两边寻最高值
uint8 S_Get_double3(uint8 tmImage[S_img_H][S_img_W]);//双峰谷底，求灰度均值，然后两边，最后寻谷底
uint8 percent(uint8 tmImage[S_img_H][S_img_W], int Tile);
/*********************************滤波***************************************/
//加权滤波
void S_WeightedFiltering(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//注入填充算法（Flood Fill Algorithm）
void S_FloodSeedFill(uint8 in_IMG[S_img_H][S_img_W], uint8 x, uint8 y, uint8 old_color, uint8 new_color);
//修改注入填充算法      使用for循环  非递归  不溢出
void SX_FloodSeedFill(uint8 in_IMG[40][63], uint8 x, uint8 y, uint8 old_color, uint8 new_color);
//在一幅图像中判定  横穿图像的白块         未实现
uint8 S_Linear(uint8 height, uint8 width, uint8* in_IMG);

/**********************************图像边缘检测算法**************************************/
//Robert算子
void S_Robert(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W]);
//S_Sobel算子
void S_Sobel(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold);
//sobel 定制
void S_Sobel_Custom(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold);



#endif
