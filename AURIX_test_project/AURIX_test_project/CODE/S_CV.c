#include "S_CV.h"
#include "string.h"


uint8 temp_IMG[S_img_H][S_img_W];  //����ͼ��
int gray_itera_threshold[256];

/***************************************************************
 *
 * �������ƣ�void S_BinaryImage(uint8 tmImage[MT9V03X_H][MT9V03X_W], uint8 ThresholdV)
 * ����˵����ͼ�����ݶ�ֵ��
 * ����˵����tmImage ��ֵ�����ݴ洢�� ThresholdV ��ֵ
 * �������أ�void
 * �޸�ʱ�䣺2019��4��6��
 * �� ע��
 ***************************************************************/
void S_BinaryImage (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W], uint8 ThresholdV)
{
    int i = 0, j = 0;
    for (i = 0; i < S_img_H; i++)
        for (j = 0; j < S_img_W; j++)
        {
            if (in_IMG[i][j] > ThresholdV)
                out_IMG[i][j] = 1;
            else
                out_IMG[i][j] = 0;
        }
}

//SE    ����
//          1
//      1   X   1
//          1
void S_SE_Operation (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    //���Ƴ���������     ʹ��ͼ��СһȦ
    uint8 img_H = S_img_H - 1;
    uint8 img_W = S_img_W - 1;
    uint8 S_UP, S_DN, S_LL, S_RR;
    //������ݳ�ʼ��
    for (i = 0; i < S_img_H; i++)
        for (j = 0; j < S_img_W; j++)
            out_IMG[i][j] = 0;
    for (i = 1; i < img_H; i++)
    {
        S_UP = i - 1;
        S_DN = i + 1;
        for (j = 1; j < img_W; j++)
        {
            S_LL = j - 1;
            S_RR = j + 1;
            if (in_IMG[i][j])
            {
                out_IMG[i][j]++;
                out_IMG[S_UP][j]++;     //UP
                out_IMG[S_DN][j]++;     //DN
                out_IMG[i][S_LL]++;     //LL
                out_IMG[i][S_RR]++;     //RR
            }
        }
    }
}

void S_SE_OperationBIG (uint8 in_IMG[120][188], uint8 out_IMG[120][188])
{
    uint8 i, j;
    //���Ƴ���������     ʹ��ͼ��СһȦ
    uint8 img_H = 120 - 1;
    uint8 img_W = 188 - 1;
    uint8 S_UP, S_DN, S_LL, S_RR;
    //������ݳ�ʼ��
    for (i = 0; i < 120; i++)
        for (j = 0; j < 188; j++)
            out_IMG[i][j] = 0;
    for (i = 1; i < img_H; i++)
    {
        S_UP = i - 1;
        S_DN = i + 1;
        for (j = 1; j < img_W; j++)
        {
            S_LL = j - 1;
            S_RR = j + 1;
            if (in_IMG[i][j])
            {
                out_IMG[i][j]++;
                out_IMG[S_UP][j]++;     //UP
                out_IMG[S_DN][j]++;     //DN
                out_IMG[i][S_LL]++;     //LL
                out_IMG[i][S_RR]++;     //RR
            }
        }
    }
}

//��ʴ
void S_Erode (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_SE_Operation(in_IMG, out_IMG);
    S_BinaryImage(out_IMG, out_IMG, 3);             //����
}

//����
void S_Dilate (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_SE_Operation(in_IMG, out_IMG);
    S_BinaryImage(out_IMG, out_IMG, 0);             //����
}

//������
void S_Open (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_Erode(in_IMG, temp_IMG);              //��ʴ
    S_Dilate(temp_IMG, out_IMG);            //����
}

//�ز���
void S_Close (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_Dilate(in_IMG, temp_IMG);             //����
    S_Erode(temp_IMG, out_IMG);             //��ʴ
}

//��ʾ��ͼ��TFTģ��
void S_Draw_TFT_BIG (uint8 in_IMG[MT9V03X_H][MT9V03X_W])
{
    uint8 i = 0, j = 0;
    //��ֵ��ͼ����ʾ
    //TFTSPI_Set_Pos(0, 0, 160 - 1, 120 - 1);          //��λ�ַ���ʾ����
    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 160; j++)
        {
            if (in_IMG[i][j])
                lcd_clear(0xffff);
            else
                lcd_clear(0x0000);
        }
    }
}

//��ʾСͼ��TFTģ��
void S_Draw_TFT_MINI (uint8 in_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    //��ֵ��ͼ����ʾ
    //TFTSPI_Set_Pos(0, 0, S_img_W - 1, S_img_H - 1);          //��λ�ַ���ʾ����
    for (i = 0; i < S_img_H; i++)
    {
        for (j = 0; j < S_img_W; j++)
        {
            if (in_IMG[i][j])
                lcd_clear(0xffff);
            else
                lcd_clear(0x0000);
        }
    }
}

// ͼ�� ��ȡѹ��188X120 to 94X60
void S_Image_zip (uint8 in_IMG[MT9V03X_H][MT9V03X_W], uint8 out_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    for (i = 0; i < S_img_H; i++)  //120�У�ÿ2�вɼ�һ�У�
        for (j = 0; j < S_img_W; j++)  //188��
            out_IMG[i][j] = in_IMG[i * 2][j * 2];
}

// ͼ�� ���Ŵ� 94X60  to  188X120
void S_Image_larger (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[MT9V03X_H][MT9V03X_W])
{
    uint8 i, j, iq, jq;
    for (i = 0; i < S_img_H; i++)
    {
        iq = i * 2;
        for (j = 0; j < S_img_W; j++)
        {
            if (in_IMG[i][j])
            {
                in_IMG[i][j] = DV;
            }
            jq = j * 2;
            out_IMG[iq][jq] = out_IMG[iq + 1][jq + 1] = out_IMG[iq][jq + 1] = out_IMG[iq + 1][jq] = in_IMG[i][j];
        }
    }
}

/***************************************************************
 *
 * �������ƣ�uint8 GetOSTU(uint8 tmImage[S_img_H][S_img_W])
 * ����˵��������ֵ��С
 * ����˵����
 * �������أ���ֵ��С
 * �޸�ʱ�䣺2018��3��27��
 * �� ע��
 �ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
 https://www.cnblogs.com/moon1992/p/5092726.html
 https://www.cnblogs.com/zhonghuasong/p/7250540.html
 Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
 1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
 2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
 3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����
 4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
 5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
 6) i++��ת��4)��ֱ��iΪ256ʱ��������
 7�������g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
 ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
 ***************************************************************/
uint8 S_GetOSTU (uint8 tmImage[S_img_H][S_img_W])
{
    int16 i, j;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    int16 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint16 HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];    //ǰ�����ص���
        PixelFore = Amount - PixelBack;         //�������ص���
        OmegaBack = (double) PixelBack / Amount;         //ǰ�����ذٷֱ�
        OmegaFore = (double) PixelFore / Amount;         //�������ذٷֱ�
        PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;  //�����Ҷ�ֵ
        MicroBack = (double) PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (double) PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}
int S_nums = 0;
//���վ�ֵ�ı������ж�ֵ��
Get_01 average;
uint8 S_Get_01_Value (uint8 tmImage[S_img_H][S_img_W])
{
    S_nums = 0;
    int i = 0, j = 0;
    uint8 GaveValue;
    uint32 tv = 0;
    uint8 S_L = 30, S_R = 64, S_F = 20, S_N = 50;
    uint16 S_NUM = (S_R - S_L) * (S_N - S_F);
    //�ۼ�
    for (i = S_F; i < S_N; i++)
    {
        for (j = S_L; j < S_R; j++)
        {
            tv += tmImage[i][j];   //�ۼ�
        }
    }
    GaveValue = tv / S_NUM;     //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
    //���վ�ֵ�ı������ж�ֵ��
    S_nums = GaveValue;
    GaveValue = GaveValue * average.p + average.d;        //�˴���ֵ���ã����ݻ����Ĺ������趨
    return GaveValue;
}

void S_WeightedFiltering (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    int8 i, j;
    int8 UP, DN, L, R;
    for (i = 0; i < S_img_H; i++)
    {
        //�����
        UP = i - 1;
        if (UP < 0)
            UP = 0;
        DN = i + 1;
        if (DN > S_img_H)
            DN = S_img_H;
        for (j = 0; j < S_img_W; j++)
        {
            //�����
            L = j - 1;
            if (L < 0)
                UP = 0;
            R = j + 1;
            if (R > S_img_W)
                DN = S_img_W;

            if ((in_IMG[i][j])
                    && (in_IMG[i][L] + in_IMG[i][R] + in_IMG[UP][j] + in_IMG[DN][j] + in_IMG[UP][L] + in_IMG[UP][R]
                            + in_IMG[DN][L] + in_IMG[DN][R] > 4))
                out_IMG[i][j] = 1;
            else
                out_IMG[i][j] = 0;
        }
    }
}

uint8 S_GetThresh (uint8 tmImage[S_img_H][S_img_W])
{
    uint8 i, j;
    uint8 kl = 0;
    uint8 kr = 0;
    uint8 *map;
    map = &tmImage[0][0];
    int16 MinValue, MaxValue;
    uint8 HistoGram[256];
    uint8 newthreshold = 0, oldthreshold = 0;
    memset(HistoGram, 0, sizeof(HistoGram)); //��ʼ���Ҷ�ֱ��ͼ
    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //��ȡ���Ҷȵ�ֵ

    uint8 Thresh_Value = (MinValue + MaxValue) / 2;
    int avgl = 0, avgr = 0;
    int numl = 0, numr = 0;

    for (i = MinValue; i < Thresh_Value; i++)
    {
        kl = kl + HistoGram[i];
        numl = numl + HistoGram[i] * i;
    }
    for (i = Thresh_Value; i < MaxValue; i++)
    {
        kr = kr + HistoGram[i];
        numr = numr + HistoGram[i] * i;
    }
    avgl = numl / kl;
    avgr = numr / kr;
    oldthreshold = (avgl + avgr) / 2;
    int floot_flag = 0;
    while (abs(newthreshold - oldthreshold) >= 2)
    {
        oldthreshold = newthreshold;
        kl = kl + HistoGram[Thresh_Value];
        numl = numl + HistoGram[Thresh_Value] * Thresh_Value;
        avgl = numl / kl;
        kr = kr - HistoGram[Thresh_Value];
        numr = numr - HistoGram[Thresh_Value] * Thresh_Value;
        avgr = numr / kr;
        newthreshold = (avgl + avgr) / 2;
        Thresh_Value++;
        if (Thresh_Value == MaxValue)
        {
            floot_flag = 1;
            break;
        }
    }
    if (floot_flag)
    {
        Thresh_Value--;
        while (abs(newthreshold - oldthreshold) >= 2)
        {
            oldthreshold = newthreshold;
            kl = kl - HistoGram[Thresh_Value];
            numl = numl - HistoGram[Thresh_Value] * Thresh_Value;
            avgl = numl / kl;
            kr = kr + HistoGram[Thresh_Value];
            numr = numr + HistoGram[Thresh_Value] * Thresh_Value;
            avgr = numr / kr;
            newthreshold = (avgl + avgr) / 2;
            Thresh_Value--;
            if (Thresh_Value == MinValue)
            {
                Thresh_Value = (MinValue + MaxValue) / 2;
                break;
            }
        }
    }
    return Thresh_Value;
}

int Max_images = MT9V03X_W * MT9V03X_H;

uint8 S_GetThresh2(uint8 tmImage[MT9V03X_H][MT9V03X_W])
{
    int HistGram[256];
    int X, Iter = 0;
    int MeanValueOne, MeanValueTwo, SumOne, SumTwo, SumIntegralOne, SumIntegralTwo;
    int MinValue, MaxValue;
    int pxnum = 0;
    uint8 Threshold, NewThreshold;
    memset(HistGram, 0, sizeof(HistGram)); //��ʼ���Ҷ�ֱ��ͼ
    for (int j = 0; j < MT9V03X_H; j++)
    {
        for (int i = 0; i < MT9V03X_W; i++)
        {
            HistGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
            pxnum += tmImage[j][i];
        }
    }
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;

    if (MaxValue == MinValue) return MaxValue;          // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue) return MinValue;      // ͼ����ֻ�ж�����ɫ

    Threshold = MinValue;
    NewThreshold = (MaxValue + MinValue) >> 1;
    while (Threshold != NewThreshold)    // ��ǰ�����ε����Ļ����ֵ��ͬʱ����������
    {
        SumOne = 0; SumIntegralOne = 0;
        SumTwo = 0; SumIntegralTwo = 0;
        Threshold = NewThreshold;
        for (X = MinValue; X <= Threshold; X++)         //������ֵ��ͼ��ָ��Ŀ��ͱ��������֣���������ֵ�ƽ���Ҷ�ֵ
        {
            SumIntegralOne += HistGram[X] * X;
            SumOne += HistGram[X];
        }
        MeanValueOne = SumIntegralOne / SumOne;

        SumIntegralTwo = pxnum - SumIntegralOne;
        SumTwo = Max_images - SumOne;

        MeanValueTwo = SumIntegralTwo / SumTwo;
        NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //����µ���ֵ
        Iter++;
        if (Iter >= 15) return 0;
    }
    return Threshold;
}

uint8 Itera_Threshold (uint8 tmImage[S_img_H][S_img_W])
{
    uint16 i = 0, j = 0, k = 0;
    uint16 cnt = 0;
    uint16 mux = 0; //Camera_Data=0;
    uint8 newthreshold = 0;
    uint16 Pmax = 0, Pmin = 0;
    uint32 sum_h1 = 0, sum_h2 = 0;
    uint32 foresum = 0, backsum = 0;
    uint16 HistoGram[256];
    uint8 threshold_h[256];
    //�������
    memset(HistoGram, 0, sizeof(HistoGram)); //�Ҷ�ֵ�������

    for (i = 0; i < S_img_H; i++)
    {
        for (j = 0; j < S_img_W; j++)
        {
            mux = tmImage[i][j]; //��ȡ�Ҷ�ͼ�����ݣ����pixel�����������ص�Ҷȵģ�
            //���ص�������
            gray_itera_threshold[mux]++; //��ͬ�Ҷ�ֵ��Ӧ�����ص���
        }
    }

    for (Pmin = 0; Pmin < 256 && gray_itera_threshold[Pmin] == 0; Pmin++)
        ; //��ȡ��С�Ҷȵ�ֵ
    for (Pmax = 255; Pmax > Pmin && gray_itera_threshold[Pmax] == 0; Pmax--)
        ; //��ȡ���Ҷȵ�ֵ

    if (Pmax == Pmin)
        return Pmax; // ͼ����ֻ��һ����ɫ
    if (Pmin + 1 == Pmax)
        return Pmin; // ͼ����ֻ�ж�����ɫ
    //��ʼ��ֵ
    threshold_h[0] = (Pmax + Pmin) / 2;

    //Ѱ�������ֵ
    for (k = 0; k < 256; k++)
    {
        //�ָ�ǰ���ͱ���
        for (cnt = 0; cnt < threshold_h[k]; cnt++)
        {
            foresum -= gray_itera_threshold[cnt];
            sum_h1 -= gray_itera_threshold[cnt] * cnt;
        }
        for (cnt = threshold_h[k]; cnt < 256; cnt++)
        {
            backsum += gray_itera_threshold[cnt];
            sum_h2 += gray_itera_threshold[cnt] * cnt;
        }
        sum_h1 /= foresum;    //ǰ����ƽ��ֵ��ֵ
        sum_h2 /= backsum;    //������ƽ��ֵ��ֵ

        //������µ���ֵ
        threshold_h[k + 1] = (sum_h1 + sum_h2) / 2;

        if ( fabs(threshold_h[k] - threshold_h[k+1]) <= 200)    //�������������������ֵ��Χ����Ҫ�Լ���
        {
            newthreshold = threshold_h[k + 1];
            break;
        }
        sum_h1 = 0;
        sum_h2 = 0;
    }
    return newthreshold;

}

uint8 GetMinimumThreshold(uint8 tmImage[S_img_H][S_img_W])
{
    double Peakfound = 0;
    int i, j, Y, Iter = 0;
    double gray_itera_thresholdC[256]; // ���ھ������⣬һ��Ҫ�ø���������������ò�����ȷ�Ľ��
    double gray_itera_thresholdCC[256]; // ���ֵ�Ĺ��̻��ƻ�ǰ������ݣ������Ҫ��������
    for (i = 0; i < S_img_H; i++)
    {
        for (j = 0; j < S_img_W; j++)
        {
            //���ص�������
            gray_itera_threshold[tmImage[i][j]]++; //��ͬ�Ҷ�ֵ��Ӧ�����ص���
        }
    }
    for (Y = 0; Y < 256; Y++)
    {
        gray_itera_thresholdC[Y] = gray_itera_threshold[Y];
        gray_itera_thresholdCC[Y] = gray_itera_threshold[Y];
    }
    // ͨ���������ֵ��ƽ��ֱ��ͼ
    while (IsDimodal(gray_itera_thresholdCC) == 0)                                        // �ж��Ƿ��Ѿ���˫���ͼ����
    {
        gray_itera_thresholdCC[0] = (gray_itera_thresholdC[0] + gray_itera_thresholdC[0] + gray_itera_thresholdC[1])
                / 3;                 // ��һ��

        for (Y = 1; Y < 255; Y++)
            gray_itera_thresholdCC[Y] = (gray_itera_thresholdC[Y - 1] + gray_itera_thresholdC[Y]
                    + gray_itera_thresholdC[Y + 1]) / 3;     // �м�ĵ�

        gray_itera_thresholdCC[255] = (gray_itera_thresholdC[254] + gray_itera_thresholdC[255]
                + gray_itera_thresholdC[255]) / 3;         // ���һ��

        Iter++;
        if (Iter >= 1000)
            return -1;                                                   // ֱ��ͼ�޷�ƽ��Ϊ˫��ģ����ش������
    }
    // ��ֵ��Ϊ����֮�����Сֵ

    for (Y = 1; Y < 255; Y++)
    {
        if (gray_itera_thresholdCC[Y - 1] < gray_itera_thresholdCC[Y]
                && gray_itera_thresholdCC[Y + 1] < gray_itera_thresholdCC[Y])
            Peakfound = 1;
        if (Peakfound = 1 && gray_itera_thresholdCC[Y - 1] >= gray_itera_thresholdCC[Y]
                && gray_itera_thresholdCC[Y + 1] >= gray_itera_thresholdCC[Y])
            return Y - 1;
    }
    return -1;
}
//1�ǿ�����˼

uint8 IsDimodal (double gray_itera_threshold[256]) // ���ֱ��ͼ�Ƿ�Ϊ˫���
{
    // ��ֱ��ͼ�ķ���м�����ֻ�з���λ2��Ϊ˫��
    int Count = 0;
    for (int Y = 1; Y < 255; Y++)
    {
        if (gray_itera_threshold[Y - 1] < gray_itera_threshold[Y]
                && gray_itera_threshold[Y + 1] < gray_itera_threshold[Y])
        {
            Count++;
            if (Count > 2)
                return 0;
        }
    }
    if (Count == 2)
        return 1;
    else
        return 0;
}

uint8 S_GetOSTUS (uint8 tmImage[S_img_H][S_img_W])
{
    uint8 Thresh_Value = 0;
    uint8 L1 = 0, L2 = 0, L3 = 0, L4 = 0;
    uint8 i, j;
    uint8 HistoGram[256];
    uint8 MinValue, MaxValue;
    uint32 Amount[3] = {0, 0, 0};
    uint32 PixelIntegral[3] = {0, 0, 0};
    uint8 gudian;
    uint8 gary1, gary2;
    double Pa, Pb, Pc;
    double Ma = 0, Mb = 0, Mc = 0;
    double S = 0, S_num;
    memset(HistoGram, 0, sizeof(HistoGram));//��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    //Ѱ������ֵ
    gary1 = MinValue;
    gary2 = MaxValue;
    for (i = HistoGram[MinValue]; i < 75; i++)
    {
        if (HistoGram[gary1] < HistoGram[i])
        {
            gary1 = i;
        }
    }
    for (i = 75; i < HistoGram[MaxValue]; i++)
    {
        if (HistoGram[gary2] < HistoGram[i])
        {
            gary2 = i;
        }
    }
    //Ѱ�һҶ�ֱ��ͼ�Ĺȵ�;
    gudian = gary1;
    for (i = gary1; i < gary2; i++)
    {
        if (HistoGram[gudian] > HistoGram[i])
        {
            gudian = i;
        }
    }
    L1 = gary1;
    L2 = gary1 + 20;
    L4 = gary2;

    for (j = MinValue; j <= L2; j++)
    {
        PixelIntegral[0] += HistoGram[j] * j;        //�Ҷ�ֵ����
        Amount[0] += HistoGram[j];              //������ֵ
    }
    for (j = L1; j <= L4; j++)
    {
        PixelIntegral[1] += HistoGram[j] * j;              //�Ҷ�ֵ����
        Amount[1] += HistoGram[j];              //������ֵ
    }
    Pa = Amount[0] * (L2 - MinValue) / cnt_num;
    Pb = Amount[1] * (L4 - L2) / cnt_num;
    Ma = (double) PixelIntegral[0] / Amount[0];
    Mb = (double) PixelIntegral[1] / Amount[1];

    for (i = L2; i < MaxValue; i++)
    {
        for (j = L3; j <= MaxValue; j++)
        {
            PixelIntegral[2] += HistoGram[j] * j;              //�Ҷ�ֵ����
            Amount[2] += HistoGram[j];              //������ֵ
        }
        Pc = Amount[2] * (MaxValue - L2) / cnt_num;
        Mc = (double) PixelIntegral[2] / Amount[2];

        S_num = abs(Pa * Pb * Pc * (Mb - Ma) * (Mc - Mb) * (Mc - Ma));

        if (S_num > S)
        {
            S = S_num;
            Thresh_Value = L3;
        }
    }
    return Thresh_Value;

}

uint8 GetOSTU(uint8 *image, uint16 col, uint16 row)
{
    uint8 j;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    uint32 PixelIntegralFore = 0;
    uint32 PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    uint8 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 HistoGram[256];              //
    int pixelSum=S_img_H * S_img_W;

    memset(HistoGram, 0, sizeof(HistoGram)); //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < pixelSum; j++)
    {
            HistoGram[image[j]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)      return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)  return MinValue;         // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];   //ǰ�����ص���
        PixelFore = Amount - PixelBack;         //�������ص���
        OmegaBack = (float)PixelBack / Amount; //ǰ�����ذٷֱ�
        OmegaFore = (float)PixelFore / Amount; //�������ذٷֱ�
        PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
        MicroBack = (float)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}

uint8 S_Get_double(uint8 tmImage[S_img_H][S_img_W]){
    uint8 i,j;
    uint8 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 HistoGram[256];              //
    uint8 top1 = 0,top2 = 0;
    uint8 num = 0;
    uint8 *p ;
    p = &tmImage[0][0];
    memset(HistoGram, 0, sizeof(HistoGram));

    for(i = 0; i < S_img_H; i++)
        for(j = 0;j < S_img_W; j++){
            HistoGram[tmImage[i][j]]++;
        }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ


    if (MaxValue == MinValue)      return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)  return MinValue;         // ͼ����ֻ�ж�����ɫ

    for(i = MinValue; i < MaxValue; i++){
        if(HistoGram[top1] < HistoGram[i]){
            top1 = i;
        }
    }

    if(top1 - MinValue < 30){
        num = top1 + 30;
        for(i = num; i < MaxValue; i++){
            if(HistoGram[top2]< HistoGram[i]){
                top2 = i;
            }
        }
    }
    else if(MaxValue - top1 < 30){
        num = top1 - 30;
        for(i = MinValue; i < num; i++){
            if(HistoGram[top2]< HistoGram[i]){
                top2 = i;
            }
        }
    }
    else{
        num = top1 - 30;
        for(i = MinValue; i < num; i++){
            if(HistoGram[top2] < HistoGram[i]){
                top2 = i;
            }
        }
        num = top1 + 30;
        for(i = num; i < MaxValue; i++){
            if(HistoGram[top2] < HistoGram[i]){
                top2 = i;
            }
        }
    }
    Threshold = (top1 + top2)/2;
    return Threshold;
}

uint8 S_Get_double2(uint8 tmImage[S_img_H][S_img_W]){
    uint8 i,j;
    uint8 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 HistoGram[256];              //
    uint8 top1 = 0,top2 = 0;
    uint8 *p ;
    int Amount = 0;
    int avg;
    int pixnum = S_img_H * S_img_W;
    p = &tmImage[0][0];
//    gpio_set(BEEP_PIN,1);
//    systick_delay_ms(STM0, 200);
//    gpio_set(BEEP_PIN,0);
    memset(HistoGram, 0, sizeof(HistoGram));

    for(i = 0; i < S_img_H; i++)
        for(j = 0;j < S_img_W; j++){
            HistoGram[tmImage[i][j]]++;
            Amount = Amount + tmImage[i][j];
        }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ


    if (MaxValue == MinValue)      return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)  return MinValue;         // ͼ����ֻ�ж�����ɫ
    avg = Amount / pixnum;
    top1 = top2 = avg;
    for(i = MinValue; i < MaxValue; i++){
        if(i < avg){
            if(HistoGram[top1] < HistoGram[i]){
                top1 = i;
            }
        }
        else{
            if(HistoGram[top2] < HistoGram[i]){
                top2 = i;
            }
        }
    }
    Threshold = (top1 + top2) / 2;
    return Threshold;
}

uint8 S_Get_double3(uint8 tmImage[S_img_H][S_img_W]){
    uint8 i,j;
    uint8 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 top1 = 0,top2 = 0;
    uint8 *p ;
    uint8 HistoGram[256];
    int Amount = 0;
    int avg;
    int pixnum = S_img_H * S_img_W;
    p = &tmImage[0][0];
//    gpio_set(BEEP_PIN,1);
//    systick_delay_ms(STM0, 200);
//    gpio_set(BEEP_PIN,0);
    memset(HistoGram, 0, sizeof(HistoGram));

    for(i = 0; i < S_img_H; i++)
        for(j = 0;j < S_img_W; j++){
            HistoGram[tmImage[i][j]]++;
            Amount = Amount + tmImage[i][j];
        }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ


    if (MaxValue == MinValue)      return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)  return MinValue;         // ͼ����ֻ�ж�����ɫ
    avg = Amount / pixnum;
    top1 = top2 = avg;
    for(i = MinValue; i < MaxValue; i++){
        if(i < avg){
            if(HistoGram[top1] < HistoGram[i]){
                top1 = i;
            }
        }
        else{
            if(HistoGram[top2] < HistoGram[i]){
                top2 = i;
            }
        }
    }
    Threshold = top2;
    for(i = top1; i < top2; i++){
        if(HistoGram[Threshold] > HistoGram[i]){
            Threshold  = i;
        }
    }
    return Threshold;
}

uint8 percent(uint8 tmImage[S_img_H][S_img_W], int Tile){
    int i,j;
    uint8 HistGram[256];
    int Amount = 0;    int Sum = 0;
    memset(HistGram, 0, sizeof(HistGram));

    for(i = 0; i < S_H_mid; i++)
        for(j = 0;j < S_img_W; j++){
            HistGram[tmImage[i][j]]++;
        }
    for (i = 0; i < 256; i++){Amount = Amount + HistGram[i];}
    for (i = 0; i < 256; i++)
    {
        Sum = Sum + HistGram[i];
        if (Sum >= Amount * Tile / 100) return i;
    }
    return -1;
}
typedef struct tagDIRECTION
{
    int x_offset;
    int y_offset;
} DIRECTION;

DIRECTION direction_8[] = {{-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
//DIRECTION direction_4[] = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };
DIRECTION direction_4[] = {{-1, 0}, {1, 0}, {0, 1}, {0, -1}};
#define directionXX direction_4
//ע������㷨��Flood Fill Algorithm��
void S_FloodSeedFill (uint8 in_IMG[S_img_H][S_img_W], uint8 x, uint8 y, uint8 old_color, uint8 new_color)
{
    if (x > 58 || y > 92 || x < 1 || y < 1)
        return;         //�������
    if (in_IMG[x][y] == old_color)
    {
        in_IMG[x][y] = new_color;
        //ImgZip_FLU[x][y] = 1;
        for (int i = 0; i < 4; i++)         // i <COUNT_OF(directionXX)
        {
            S_FloodSeedFill(in_IMG, x + directionXX[i].x_offset, y + directionXX[i].y_offset, old_color, new_color);
        }
    }
}

//extern uint8 SS_ZipF[40][63];                              //ѹ��ͼ������        �˲�ʹ��
////��ʹ�õݹ�  ��������汾
//void SX_FloodSeedFill(uint8 in_IMG[40][63], uint8 x, uint8 y, uint8 old_color, uint8 new_color)
//{
//    uint8 i, j, k;
//    uint8 return_sign;
//    in_IMG[x][y] = 255;     //������ɫ
//    for (;;)
//    {
//        return_sign = 0;
//        for (i=0;i<40;i++)
//            for (j = 0; j < 63; j++)
//            {
//                if (in_IMG[i][j] == 255)        //������ɫ
//                {
//                    in_IMG[i][j] = new_color;       //ʵ�ʸ�ֵ
//                    SS_ZipF[i][j] = 1;
//                    for (k = 0; k < 4; k++)
//                    {
//                        x = i + directionXX[k].x_offset;        y = j + directionXX[k].y_offset;
//                        if (x < 40 && y < 63 && x >= 0 && y >= 0)       //�������
//                            if (in_IMG[x][y] == old_color)
//                            {
//                                in_IMG[x][y] = 255;     //������ɫ
//                                return_sign = 1;
//                            }
//                    }
//                }
//            }
//
//        if (return_sign == 0)       return;
//    }
//}

//��һ��ͼ�����ж�  �ᴩͼ��İ׿�         δʵ��
uint8 S_Linear (uint8 height, uint8 width, uint8* in_IMG)
{
    uint8 i, j, m, n, up, dn;
    //   uint8 getONE = 0;

    uint8 LL_h, LL_w;
    uint8 RR_h, RR_w;

    for (i = 20; i < height - 20; i++)
    {
        j = 0;
        while (j < 5)     //��5����Ѱ�ð׵�
        {
            if (in_IMG[i * width + j++])
            {
                LL_h = i;
                LL_w = j - 1;       //��¼�������
                up = i - 7;
                dn = i + 7;
                for (m = up; m < dn; m++)
                {
                    n = width - 1;
                    while (n > width + 5)       //��5����Ѱ�ð׵�
                    {
                        if (in_IMG[m * width + n--])
                        {
                            RR_h = m;
                            RR_w = n + 1;       //��¼�ҵ�����

                            float kk, bb;
                            kk = (float) (LL_h - RR_h) / (LL_w - RR_w);
                            bb = (float) (LL_h - kk * LL_w);

                            uint8 ww, ww_COUNT = 0;
                            for (ww = LL_w + 10; ww < RR_w - 10; ww += 10)
                            {
                                if (!in_IMG[(uint8) (kk * ww + bb) * width + ww])           //�����
                                    ww_COUNT = 1;
                            }
                            if (ww_COUNT == 0)
                                return 1;

                            break;
                        }
                    }
                }
                break;
            }
        }
    }
    return 0;
}

//�������ͣ��� �� ͼ��ָ�� ��ʼ��  ��ʼ�� �����д�С  �����д�С
#define GrayScale 256   //frame�Ҷȼ�
uint8 pixel[GrayScale];
uint8 S_Other_OSTU (int width, int height, uint8* Image)
{
    int threshold = 0;
    int32 sum_gray = 0;
    int32 sum_pix_num = 0;
    int32 pl_pix_num = 0;
    int32 p2_pix_mum = 0;
    int32 p1_sum_gray = 0;
    float m1 = 0;
    float m2 = 0;
    float V = 0;
    float variance = 0;
    int i, j, k = 0;

    for (i = 0; i < 256; i++)
        pixel[i] = 0;
    //ͳ��ÿ���Ҷȼ������صĸ���
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixel[(int) Image[i * width + j]]++;
        }
    }

    for (k = 0; k < GrayScale; k++)
    {
        sum_gray += k * pixel[k];           //�Ҷ�ֱ��ͼ������
        sum_pix_num += pixel[k];           //�����ظ���
    }

    for (k = 0; k < GrayScale - 1; k++)
    {
        pl_pix_num += pixel[k];           //��һ�������ظ���
        p2_pix_mum = sum_pix_num - pl_pix_num;           //�ڶ��������ظ���
        p1_sum_gray += k * pixel[k];   //��һ����������
        m1 = (float) p1_sum_gray / pl_pix_num;   //��һ���ֻҶȾ�ֵ
        m2 = (float) (sum_gray - p1_sum_gray) / p2_pix_mum;   //�ڶ����ֻҶȾ�ֵ

        V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

        if (V > variance)   //����䷽��ϴ�ʱ�ĻҶ�ֵ��Ϊ��ֵ
        {
            variance = V;
            threshold = k;
        }
    }
    return threshold;
}

/************************************Robert����************************************
 **          Gx={    {1,  0},                       Gy={ {  0,  1},
 **                  {0,-1}}                             {-1,  0}}
 **      ���ҡ���ױ�Ե��ȥ
 **      G(x,y)=abs(f(x,y)-f(x+1,y+1))+abs(f(x,y+1)-f(x+1,y))
 ***********************************************************************************/
#define Robert_G(addr, y, x)    (abs(addr[y][x] - addr[DN][RR]) + abs(addr[y][RR] - addr[DN][x]))
void S_Robert (uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W])
{
    uint8 i, j;
    uint8 DN, RR;
    for (i = 0; i < 59; i++)
    {
        DN = i + 1;
        for (j = 0; j < 93; j++)
        {
            RR = j + 1;
            out_IMG[i][j] = Robert_G(in_IMG, i, j);
        }
    }
}

/************************************Sobel����************************************
 **          Gx={    {-1,  0,  1},                   Gy={    {  1,  2,  1},
 **                  {-2,  0,  2},                           {  0,  0,  0},
 **                  {-1,  0,  1}}                           { -1, -2, -1}}
 **      ���������ұ�Ե��ȥ
 **
 **      Gx = (-1)*f(x-1, y-1) + 0*f(x,y-1) + 1*f(x+1,y-1)
 **            +(-2)*f(x-1,y) + 0*f(x,y)+2*f(x+1,y)
 **            +(-1)*f(x-1,y+1) + 0*f(x,y+1) + 1*f(x+1,y+1)
 **      = [f(x+1,y-1)+2*f(x+1,y)+f(x+1,y+1)]-[f(x-1,y-1)+2*f(x-1,y)+f(x-1,y+1)]
 **
 **      Gy =1* f(x-1, y-1) + 2*f(x,y-1)+ 1*f(x+1,y-1)
 **            +0*f(x-1,y) 0*f(x,y) + 0*f(x+1,y)
 **            +(-1)*f(x-1,y+1) + (-2)*f(x,y+1) + (-1)*f(x+1, y+1)
 **      = [f(x-1,y-1) + 2f(x,y-1) + f(x+1,y-1)]-[f(x-1, y+1) + 2*f(x,y+1)+f(x+1,y+1)]
 **      ��Threshold=0������Ҷ�ͼ      �������ֵ��
 ***********************************************************************************/
#define Sobel_Gx(addr, y, x)    (addr[UP][RR]+2*addr[y][RR]+addr[DN][RR]-(addr[UP][LL]+2*addr[y][LL]+addr[DN][LL]))
#define Sobel_Gy(addr, y, x)    (addr[UP][LL]+2*addr[UP][x]+addr[UP][RR]-(addr[DN][LL]+2*addr[DN][x]+addr[DN][RR]))
#define Sobel_G(addr, y, x)     (abs(Sobel_Gx(addr, y, x)) + abs(Sobel_Gy(addr, y, x)))
void S_Sobel (uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold)
{
    uint8 i, j;
    uint8 UP, DN, LL, RR;
    if (Threshold == 0)
    {
        for (i = 1; i < 59; i++)
        {
            DN = i + 1;
            UP = i - 1;
            for (j = 1; j < 93; j++)
            {
                RR = j + 1;
                LL = j - 1;
                out_IMG[i][j] = Sobel_G(in_IMG, i, j);
            }
        }
    }
    else
    {
        for (i = 1; i < 59; i++)
        {
            DN = i + 1;
            UP = i - 1;
            for (j = 1; j < 93; j++)
            {
                RR = j + 1;
                LL = j - 1;
                out_IMG[i][j] = (Sobel_G(in_IMG, i, j) >= Threshold ? 1 : 0);
            }
        }
    }

}
//sobel ����
//�����ж�ֵ����ͼ���б�Ǳ�Ե
void S_Sobel_Custom (uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold)
{
    uint8 i, j;
    uint8 UP, DN, LL, RR;
    uint16 TempVal;
    for (i = 1; i < 58; i++)        //HIGH  ֻ����ǰ50��
    {
        DN = i + 1;
        UP = i - 1;
        for (j = 1; j < 93; j++)    //WIDE
        {
            RR = j + 1;
            LL = j - 1;
            TempVal = Sobel_G(in_IMG, i, j);
            if (TempVal >= Threshold)
            {
                out_IMG[i][j] = S_S_Custom;
            }
        }
    }

}

/******************************60*94ͼ����ʾ��188*120***********************************
 **
 *******************************************************************************************/
void S_DisplayMINI_Big (uint8 in_IMG[PixMini_H][PixMini_W])
{
    uint8 i, j, k;
    uint16 color;

    //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //��λ�ַ���ʾ����
    for (i = 0; i < PixMini_H; i++)
    {
        for (k = 0; k < 2; k++)
            for (j = 0; j < PixMini_W; j++)
            {
                if (in_IMG[i][j] == 1)
                    color = WHITE;
                else if (in_IMG[i][j] == S_S_Custom)
                    color = PURPLE;
                else
                    color = BLACK;

                lcd_clear(color);
                lcd_clear(color);
            }
    }
}

/******************************60*94ͼ����ʾ�Ҷ�ͼ***********************************
 **      size==S_BIG     ��ʾ��ͼ                                                                                            **
 **      size==S_MINI    ��ʾСͼ                                                                                            **
 ***************************************************************************************/
void S_Gray_DisplayMINI (uint8 in_IMG[PixMini_H][PixMini_W], uint8 size)
{
    uint8 i, j, k;
    uint16 color, temp;

    if (size == S_MINI)          //��ʾСͼ
    {
        //TFTSPI_Set_Pos(0, 0, 94 - 1, 60 - 1);          //��λ�ַ���ʾ����
        for (i = 0; i < 60; i++)
        {
            for (j = 0; j < 94; j++)
            {
                temp = in_IMG[i][j];
                color = (0x001f & ((temp) >> 3)) << 11;
                color = color | (((0x003f) & ((temp) >> 2)) << 5);
                color = color | (0x001f & ((temp) >> 3));
                lcd_clear(color);
            }
        }
    }
    else                                //��ʾ��ͼ
    {
        //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //��λ�ַ���ʾ����
        for (i = 0; i < 60; i++)
        {
            for (k = 0; k < 2; k++)
                for (j = 0; j < 94; j++)
                {
                    temp = in_IMG[i][j];
                    color = (0x001f & ((temp) >> 3)) << 11;
                    color = color | (((0x003f) & ((temp) >> 2)) << 5);
                    color = color | (0x001f & ((temp) >> 3));

                    lcd_clear(color);
                    lcd_clear(color);
                }
        }
    }
}

/******************************120*188ͼ����ʾ�Ҷ�ͼ************************************
 **                                                                                  **
 **************************************************************************************/
void S_Gray_Display (uint8 in_IMG[MT9V03X_H][MT9V03X_W])
{
    uint8 i, j;
    uint16 color, temp;
    //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //��λ�ַ���ʾ����
    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 188; j++)
        {
            temp = in_IMG[i][j];
            color = (0x001f & ((temp) >> 3)) << 11;
            color = color | (((0x003f) & ((temp) >> 2)) << 5);
            color = color | (0x001f & ((temp) >> 3));
            lcd_clear(color);
        }
    }
}

/*********************************����ͼ���ʶ��**************************************
 **                              Mod=0   Ѱ��ֵ��    Mod=2   Ѱsobel                                                      **
 ***************************************************************************************/
void DetectionLine (uint8 in_IMG[PixMini_H][PixMini_W],       //����ͼ��
        RunWay_* out_Array,             //�������
        uint8 LMP,                         //�����е�
        uint8 Mod)                         //���ģʽ
{
    uint8 i, j;
    uint8 M_Point, L_Point, R_Point;
//    uint8 get_Point;

    M_Point = LMP;                         //ȡ����ĵײ��е�

    for (i = 58; i > 0; i--)                    //����
    {
        /**/
        //Ѱ���
        j = M_Point + 5;
        if (j < 99 && j > 92)
            j = 92;
        while (!((!in_IMG[i][j + Mod]) && in_IMG[i][j + 1]) && (j > 1))
        {
            j--;
        }
        L_Point = j;
        //Ѱ�ҵ�
        j = M_Point - 5;
        if (j > 99)
            j = 1;
        while (!((!in_IMG[i][j - Mod]) && in_IMG[i][j - 1]) && (j < 92))
        {
            j++;
        }
        R_Point = j;

        /*
         //Ѱ���
         get_Point = 0;
         for (j = M_Point + 5; j > 0; j--)
         {
         //�����
         //if (j < 0)         j = 0;
         //if (j > 187)        j = 187;
         if (in_IMG[i][j]== Mod)
         {
         L_Point = j;
         get_Point = 1;      //ȡ�õ�
         break;
         }
         }
         if (get_Point==0)                    //ȡ����ߵĺڵ�
         {
         L_Point = 0;
         }

         //Ѱ�ҵ�
         get_Point = 0;
         for (j = M_Point - 5; j < 59; j++)
         {
         //�����
         //if (j < 0)        j = 0;
         //if (j > 187)       j = 187;
         if (in_IMG[i][j] == Mod)
         {
         R_Point = j;
         get_Point = 1;      //ȡ�õ�
         break;
         }
         }
         if (get_Point == 0)                     //ȡ���ұߵĺڵ�
         {
         R_Point = 93;
         }
         */

        M_Point = (L_Point + R_Point) / 2;                         //ȡ��һ���е�
        out_Array->M[i] = M_Point;
        out_Array->L[i] = L_Point;
        out_Array->R[i] = R_Point;

        /**/
        //����
        if (i <= 55)
        {
            //�Ͽ�����x
            uint8 fg_no_c = 0;
            if (i <= 30)
                if (abs(M_Point - out_Array->M[i + 1]) >= 5)            //���������е����5������
                {
                    if (M_Point >= 50)
                        fg_no_c = 1;
                    else if (M_Point <= 45)
                        fg_no_c = 2;
                }

            if ((M_Point < 25) || (fg_no_c == 2))
            {
                for (; i > 0; i--)
                {
                    out_Array->M[i] = 3;
                    out_Array->L[i] = 3;
                    out_Array->R[i] = 3;
                }
                break;
            }
            else if ((M_Point > 69) || (fg_no_c == 1))
            {
                for (; i > 0; i--)
                {
                    out_Array->M[i] = 90;
                    out_Array->L[i] = 90;
                    out_Array->R[i] = 90;
                }
                break;
            }
        }
    }
    //TFTSPI_P6X8Int(20, 5, L_Point, WHITE, BLACK);
    //TFTSPI_P6X8Int(20, 6, R_Point, WHITE, BLACK);
    //S_DisplayRunWay(out_Array);
}

/******************************����ͼ���ʶ����sobel����********************************
 **                                                                                  **
 ***************************************************************************************/
void DetectionLine_Custom (uint8 in_IMG[PixMini_H][PixMini_W],       //����ͼ��
        RunWay_* out_Array,             //�������
        uint8 LMP)                         //�����е�
{
    uint8 i, j;
    uint8 M_Point, L_Point, R_Point;
    //   uint8 get_Point;

    M_Point = LMP;                         //ȡ����ĵײ��е�

    for (i = 57; i > 0; i--)                    //����
    {
        /**/
        //Ѱ���
        j = M_Point + 2;
        if (j < 99 && j > 90)
            j = 90;       //ͼ���Ҳ�2�����ز�����
        while (!(in_IMG[i][j] == S_S_Custom) && (j > 1))
        {
            j--;
        }
        L_Point = j;
        //Ѱ�ҵ�
        j = M_Point - 2;
        if (j > 99)
            j = 1;
        while (!(in_IMG[i][j] == S_S_Custom) && (j < 92))
        {
            j++;
        }
        R_Point = j;

        /*
         //Ѱ���
         get_Point = 0;
         for (j = M_Point + 5; j > 0; j--)
         {
         //�����
         //if (j < 0)         j = 0;
         //if (j > 187)        j = 187;
         if (in_IMG[i][j]== Mod)
         {
         L_Point = j;
         get_Point = 1;      //ȡ�õ�
         break;
         }
         }
         if (get_Point==0)                    //ȡ����ߵĺڵ�
         {
         L_Point = 0;
         }

         //Ѱ�ҵ�
         get_Point = 0;
         for (j = M_Point - 5; j < 59; j++)
         {
         //�����
         //if (j < 0)        j = 0;
         //if (j > 187)       j = 187;
         if (in_IMG[i][j] == Mod)
         {
         R_Point = j;
         get_Point = 1;      //ȡ�õ�
         break;
         }
         }
         if (get_Point == 0)                     //ȡ���ұߵĺڵ�
         {
         R_Point = 93;
         }
         */

        M_Point = (L_Point + R_Point) / 2;                         //ȡ��һ���е�
        out_Array->M[i] = M_Point;
        out_Array->L[i] = L_Point;
        out_Array->R[i] = R_Point;

        /**/
        //����
        if (i <= 55)
        {
            //�Ͽ�����x
            uint8 fg_no_c = 0;
            if (i <= 30)
                if (abs(M_Point - out_Array->M[i + 1]) >= 5)            //���������е����5������
                {
                    if (M_Point >= 50)
                        fg_no_c = 1;
                    else if (M_Point <= 45)
                        fg_no_c = 2;
                }

            if ((M_Point < 25) || (fg_no_c == 2))
            {
                for (; i > 0; i--)
                {
                    out_Array->M[i] = 3;
                    out_Array->L[i] = 3;
                    out_Array->R[i] = 3;
                }
                break;
            }
            else if ((M_Point > 69) || (fg_no_c == 1))
            {
                for (; i > 0; i--)
                {
                    out_Array->M[i] = 90;
                    out_Array->L[i] = 90;
                    out_Array->R[i] = 90;
                }
                break;
            }
        }
    }
    //TFTSPI_P6X8Int(20, 5, L_Point, WHITE, BLACK);
    //TFTSPI_P6X8Int(20, 6, R_Point, WHITE, BLACK);
    //S_DisplayRunWay(out_Array);
}

RunWay_ TempArray;
void S_DisplayRunWay (RunWay_* in_Array)
{
    uint8 i;

    for (i = 0; i < 60; i++)
    {
        if (i % 10 == 0)        //���߱��  �̶�
        {
            lcd_drawpoint(0, i, RED);
            lcd_drawpoint(1, i, RED);
            lcd_drawpoint(2, i, RED);
        }

        //Ĩ�ϴε�
        lcd_drawpoint(TempArray.M[i], i, 0x0000);
        lcd_drawpoint(TempArray.L[i], i, 0x0000);
        lcd_drawpoint(TempArray.R[i], i, 0x0000);
        //��¼�ɵ�
        TempArray.L[i] = in_Array->L[i];
        TempArray.M[i] = in_Array->M[i];
        TempArray.R[i] = in_Array->R[i];
        //��ʾ�µ�
        lcd_drawpoint(in_Array->M[i], i, YELLOW);
        lcd_drawpoint(in_Array->L[i], i, YELLOW);
        lcd_drawpoint(in_Array->R[i], i, YELLOW);
    }
}

void S_DRW_Big (RunWay_* in_Array)
{
    uint8 i, k, kk;

    for (i = 0; i < 60; i++)
    {
        for (k = 0; k < 2; k++)
        {
            kk = 2 * i + k;
            if (kk % 10 == 0)       //���߱��  �̶�
            {
                lcd_drawpoint(0, kk, RED);
                lcd_drawpoint(1, kk, RED);
                lcd_drawpoint(2, kk, RED);
            }

            //Ĩ�ϴε�
            lcd_drawpoint(TempArray.M[i] * 2, kk, 0x0000);
            lcd_drawpoint(TempArray.L[i] * 2, kk, 0x0000);
            lcd_drawpoint(TempArray.R[i] * 2, kk, 0x0000);
            //��ʾ�µ�
            lcd_drawpoint(in_Array->M[i] * 2, kk, PURPLE);
            lcd_drawpoint(in_Array->L[i] * 2, kk, BROWN);
            lcd_drawpoint(in_Array->R[i] * 2, kk, GREEN);
        }
        //��¼�ɵ�
        TempArray.L[i] = in_Array->L[i];
        TempArray.M[i] = in_Array->M[i];
        TempArray.R[i] = in_Array->R[i];
    }
}

