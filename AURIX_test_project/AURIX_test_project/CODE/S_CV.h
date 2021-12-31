#ifndef _S_CV_H
#define _S_CV_H
#include "headfile.h"
typedef struct {
    uint8 L[64];
    uint8 R[64];
    uint8 M[64];
} RunWay_;
//ͼ���С����
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

/**********************************��̬ѧ����**************************************/
//��ʴ
void S_Erode(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//����
void S_Dilate(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//������
void S_Open(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//�ز���
void S_Close(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);


/**********************************ͼ����ʾ**************************************/
#define S_MINI  0
#define S_BIG       1
//��ʾСͼ��TFTģ��
void S_Draw_TFT_MINI(uint8 in_IMG[S_img_H][S_img_W]);
//60*94ͼ����ʾ��188*120
void S_DisplayMINI_Big(uint8 in_IMG[PixMini_H][PixMini_W]);
//��ʾ��ͼ��TFTģ��
void S_Draw_TFT_BIG(uint8 in_IMG[MT9V03X_H][MT9V03X_W]);
//��ʾ�Ҷ�ͼ��60*90   ��ѡ��С
void S_Gray_DisplayMINI(uint8 in_IMG[PixMini_H][PixMini_W], uint8 size);
//��ʾ�Ҷ�ͼ��120*188
void S_Gray_Display(uint8 in_IMG[MT9V03X_H][MT9V03X_W]);
//����ͼ��ʾ
void S_DisplayRunWay(RunWay_* in_Array);
//����ͼ��ʾ ��ͼ
void S_DRW_Big(RunWay_* in_Array);



/**********************************��������**************************************/
// ͼ��  ��ȡѹ��188X120 to 94X60
void S_Image_zip(uint8 in_IMG[MT9V03X_H][MT9V03X_W], uint8 out_IMG[S_img_H][S_img_W]);
// ͼ�� ���Ŵ� 94X60  to  188X120
void S_Image_larger(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[MT9V03X_H][MT9V03X_W]);
//����ͼ���
void DetectionLine(uint8 in_IMG[PixMini_H][PixMini_W], RunWay_ * out_Array, uint8 LMP, uint8 Mod);
//����ͼ����
void DetectionLine_Custom(uint8 in_IMG[PixMini_H][PixMini_W], RunWay_* out_Array, uint8 LMP);



/*********************************����ֵ***************************************/
//�������ֵ
uint8 S_GetOSTU(uint8 tmImage[S_img_H][S_img_W]);       //�޸�
uint8 S_Other_OSTU(int width, int height, uint8* Image);//������
uint8 S_Get_01_Value(uint8 tmImage[S_img_H][S_img_W]);//��ֵ������
uint8 S_GetThresh(uint8 tmImage[S_img_H][S_img_W]);//��д������
uint8 S_GetThresh2(uint8 tmImage[MT9V03X_H][MT9V03X_W]);   // ��ɵ�����
uint8 Itera_Threshold(uint8 tmImage[S_img_H][S_img_W]);//��һ��������
uint8 IsDimodal(double gray_itera_threshold[256]);//�ж��Ƿ�Ϊ˫��
uint8 GetMinimumThreshold(uint8 tmImage[S_img_H][S_img_W]);//˫�巨ȷ����ֵ
uint8 S_GetOSTUS(uint8 tmImage[S_img_H][S_img_W]);//ģ�����
uint8 GetOSTU(uint8 *image, uint16 col, uint16 row);//��򷨣���ȷ����Χ�Ĵ��
uint8 S_Get_double(uint8 tmImage[S_img_H][S_img_W]);//˫���ֵ�����Ʒ�Ŀ��Ϊ������ʮ
uint8 S_Get_double2(uint8 tmImage[S_img_H][S_img_W]);//˫���ֵ������ͼ��ҶȾ�ֵ����������Ѱ���ֵ
uint8 S_Get_double3(uint8 tmImage[S_img_H][S_img_W]);//˫��ȵף���ҶȾ�ֵ��Ȼ�����ߣ����Ѱ�ȵ�
uint8 percent(uint8 tmImage[S_img_H][S_img_W], int Tile);
/*********************************�˲�***************************************/
//��Ȩ�˲�
void S_WeightedFiltering(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W]);
//ע������㷨��Flood Fill Algorithm��
void S_FloodSeedFill(uint8 in_IMG[S_img_H][S_img_W], uint8 x, uint8 y, uint8 old_color, uint8 new_color);
//�޸�ע������㷨      ʹ��forѭ��  �ǵݹ�  �����
void SX_FloodSeedFill(uint8 in_IMG[40][63], uint8 x, uint8 y, uint8 old_color, uint8 new_color);
//��һ��ͼ�����ж�  �ᴩͼ��İ׿�         δʵ��
uint8 S_Linear(uint8 height, uint8 width, uint8* in_IMG);

/**********************************ͼ���Ե����㷨**************************************/
//Robert����
void S_Robert(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W]);
//S_Sobel����
void S_Sobel(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold);
//sobel ����
void S_Sobel_Custom(uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold);



#endif
