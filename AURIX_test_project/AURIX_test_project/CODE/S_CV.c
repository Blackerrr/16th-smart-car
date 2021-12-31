#include "S_CV.h"
#include "string.h"


uint8 temp_IMG[S_img_H][S_img_W];  //过渡图像
int gray_itera_threshold[256];

/***************************************************************
 *
 * 函数名称：void S_BinaryImage(uint8 tmImage[MT9V03X_H][MT9V03X_W], uint8 ThresholdV)
 * 功能说明：图像数据二值化
 * 参数说明：tmImage 二值化数据存储、 ThresholdV 阈值
 * 函数返回：void
 * 修改时间：2019年4月6日
 * 备 注：
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

//SE    操作
//          1
//      1   X   1
//          1
void S_SE_Operation (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    //限制长宽避免溢出     使用图像小一圈
    uint8 img_H = S_img_H - 1;
    uint8 img_W = S_img_W - 1;
    uint8 S_UP, S_DN, S_LL, S_RR;
    //输出数据初始化
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
    //限制长宽避免溢出     使用图像小一圈
    uint8 img_H = 120 - 1;
    uint8 img_W = 188 - 1;
    uint8 S_UP, S_DN, S_LL, S_RR;
    //输出数据初始化
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

//腐蚀
void S_Erode (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_SE_Operation(in_IMG, out_IMG);
    S_BinaryImage(out_IMG, out_IMG, 3);             //交集
}

//膨胀
void S_Dilate (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_SE_Operation(in_IMG, out_IMG);
    S_BinaryImage(out_IMG, out_IMG, 0);             //并集
}

//开操作
void S_Open (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_Erode(in_IMG, temp_IMG);              //腐蚀
    S_Dilate(temp_IMG, out_IMG);            //膨胀
}

//关操作
void S_Close (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    S_Dilate(in_IMG, temp_IMG);             //膨胀
    S_Erode(temp_IMG, out_IMG);             //腐蚀
}

//显示大图像到TFT模块
void S_Draw_TFT_BIG (uint8 in_IMG[MT9V03X_H][MT9V03X_W])
{
    uint8 i = 0, j = 0;
    //二值化图像显示
    //TFTSPI_Set_Pos(0, 0, 160 - 1, 120 - 1);          //定位字符显示区域
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

//显示小图像到TFT模块
void S_Draw_TFT_MINI (uint8 in_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    //二值化图像显示
    //TFTSPI_Set_Pos(0, 0, S_img_W - 1, S_img_H - 1);          //定位字符显示区域
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

// 图像 抽取压缩188X120 to 94X60
void S_Image_zip (uint8 in_IMG[MT9V03X_H][MT9V03X_W], uint8 out_IMG[S_img_H][S_img_W])
{
    uint8 i, j;
    for (i = 0; i < S_img_H; i++)  //120行，每2行采集一行，
        for (j = 0; j < S_img_W; j++)  //188，
            out_IMG[i][j] = in_IMG[i * 2][j * 2];
}

// 图像 填充放大 94X60  to  188X120
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
 * 函数名称：uint8 GetOSTU(uint8 tmImage[S_img_H][S_img_W])
 * 功能说明：求阈值大小
 * 参数说明：
 * 函数返回：阈值大小
 * 修改时间：2018年3月27日
 * 备 注：
 参考：https://blog.csdn.net/zyzhangyue/article/details/45841255
 https://www.cnblogs.com/moon1992/p/5092726.html
 https://www.cnblogs.com/zhonghuasong/p/7250540.html
 Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
 1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
 2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
 3) i表示分类的阈值，也即一个灰度级，从0开始迭代
 4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
 5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
 6) i++；转到4)，直到i为256时结束迭代
 7）将最大g相应的i值作为图像的全局阈值
 缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
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
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int16 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint16 HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];    //前景像素点数
        PixelFore = Amount - PixelBack;         //背景像素点数
        OmegaBack = (double) PixelBack / Amount;         //前景像素百分比
        OmegaFore = (double) PixelFore / Amount;         //背景像素百分比
        PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;  //背景灰度值
        MicroBack = (double) PixelIntegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (double) PixelIntegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}
int S_nums = 0;
//按照均值的比例进行二值化
Get_01 average;
uint8 S_Get_01_Value (uint8 tmImage[S_img_H][S_img_W])
{
    S_nums = 0;
    int i = 0, j = 0;
    uint8 GaveValue;
    uint32 tv = 0;
    uint8 S_L = 30, S_R = 64, S_F = 20, S_N = 50;
    uint16 S_NUM = (S_R - S_L) * (S_N - S_F);
    //累加
    for (i = S_F; i < S_N; i++)
    {
        for (j = S_L; j < S_R; j++)
        {
            tv += tmImage[i][j];   //累加
        }
    }
    GaveValue = tv / S_NUM;     //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    //按照均值的比例进行二值化
    S_nums = GaveValue;
    GaveValue = GaveValue * average.p + average.d;        //此处阈值设置，根据环境的光线来设定
    return GaveValue;
}

void S_WeightedFiltering (uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W])
{
    int8 i, j;
    int8 UP, DN, L, R;
    for (i = 0; i < S_img_H; i++)
    {
        //防溢出
        UP = i - 1;
        if (UP < 0)
            UP = 0;
        DN = i + 1;
        if (DN > S_img_H)
            DN = S_img_H;
        for (j = 0; j < S_img_W; j++)
        {
            //防溢出
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
    memset(HistoGram, 0, sizeof(HistoGram)); //初始化灰度直方图
    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //获取最大灰度的值

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
    memset(HistGram, 0, sizeof(HistGram)); //初始化灰度直方图
    for (int j = 0; j < MT9V03X_H; j++)
    {
        for (int i = 0; i < MT9V03X_W; i++)
        {
            HistGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
            pxnum += tmImage[j][i];
        }
    }
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;

    if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

    Threshold = MinValue;
    NewThreshold = (MaxValue + MinValue) >> 1;
    while (Threshold != NewThreshold)    // 当前后两次迭代的获得阈值相同时，结束迭代
    {
        SumOne = 0; SumIntegralOne = 0;
        SumTwo = 0; SumIntegralTwo = 0;
        Threshold = NewThreshold;
        for (X = MinValue; X <= Threshold; X++)         //根据阈值将图像分割成目标和背景两部分，求出两部分的平均灰度值
        {
            SumIntegralOne += HistGram[X] * X;
            SumOne += HistGram[X];
        }
        MeanValueOne = SumIntegralOne / SumOne;

        SumIntegralTwo = pxnum - SumIntegralOne;
        SumTwo = Max_images - SumOne;

        MeanValueTwo = SumIntegralTwo / SumTwo;
        NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //求出新的阈值
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
    //数据清空
    memset(HistoGram, 0, sizeof(HistoGram)); //灰度值数据清空

    for (i = 0; i < S_img_H; i++)
    {
        for (j = 0; j < S_img_W; j++)
        {
            mux = tmImage[i][j]; //获取灰度图的数据，这个pixel数组用来像素点灰度的；
            //像素点数自增
            gray_itera_threshold[mux]++; //不同灰度值对应的像素点数
        }
    }

    for (Pmin = 0; Pmin < 256 && gray_itera_threshold[Pmin] == 0; Pmin++)
        ; //获取最小灰度的值
    for (Pmax = 255; Pmax > Pmin && gray_itera_threshold[Pmax] == 0; Pmax--)
        ; //获取最大灰度的值

    if (Pmax == Pmin)
        return Pmax; // 图像中只有一个颜色
    if (Pmin + 1 == Pmax)
        return Pmin; // 图像中只有二个颜色
    //初始阈值
    threshold_h[0] = (Pmax + Pmin) / 2;

    //寻找最佳阈值
    for (k = 0; k < 256; k++)
    {
        //分割前景和背景
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
        sum_h1 /= foresum;    //前景求平均值阈值
        sum_h2 /= backsum;    //背景求平均值阈值

        //计算出新的阈值
        threshold_h[k + 1] = (sum_h1 + sum_h2) / 2;

        if ( fabs(threshold_h[k] - threshold_h[k+1]) <= 200)    //这里的数是用来定义阈值范围，需要自己改
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
    double gray_itera_thresholdC[256]; // 基于精度问题，一定要用浮点数来处理，否则得不到正确的结果
    double gray_itera_thresholdCC[256]; // 求均值的过程会破坏前面的数据，因此需要两份数据
    for (i = 0; i < S_img_H; i++)
    {
        for (j = 0; j < S_img_W; j++)
        {
            //像素点数自增
            gray_itera_threshold[tmImage[i][j]]++; //不同灰度值对应的像素点数
        }
    }
    for (Y = 0; Y < 256; Y++)
    {
        gray_itera_thresholdC[Y] = gray_itera_threshold[Y];
        gray_itera_thresholdCC[Y] = gray_itera_threshold[Y];
    }
    // 通过三点求均值来平滑直方图
    while (IsDimodal(gray_itera_thresholdCC) == 0)                                        // 判断是否已经是双峰的图像了
    {
        gray_itera_thresholdCC[0] = (gray_itera_thresholdC[0] + gray_itera_thresholdC[0] + gray_itera_thresholdC[1])
                / 3;                 // 第一点

        for (Y = 1; Y < 255; Y++)
            gray_itera_thresholdCC[Y] = (gray_itera_thresholdC[Y - 1] + gray_itera_thresholdC[Y]
                    + gray_itera_thresholdC[Y + 1]) / 3;     // 中间的点

        gray_itera_thresholdCC[255] = (gray_itera_thresholdC[254] + gray_itera_thresholdC[255]
                + gray_itera_thresholdC[255]) / 3;         // 最后一点

        Iter++;
        if (Iter >= 1000)
            return -1;                                                   // 直方图无法平滑为双峰的，返回错误代码
    }
    // 阈值极为两峰之间的最小值

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
//1是可以意思

uint8 IsDimodal (double gray_itera_threshold[256]) // 检测直方图是否为双峰的
{
    // 对直方图的峰进行计数，只有峰数位2才为双峰
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
    memset(HistoGram, 0, sizeof(HistoGram));//初始化灰度直方图

    for (j = 0; j < S_img_H; j++)
    {
        for (i = 0; i < S_img_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    //寻找两峰值
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
    //寻找灰度直方图的谷点;
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
        PixelIntegral[0] += HistoGram[j] * j;        //灰度值总数
        Amount[0] += HistoGram[j];              //像素总值
    }
    for (j = L1; j <= L4; j++)
    {
        PixelIntegral[1] += HistoGram[j] * j;              //灰度值总数
        Amount[1] += HistoGram[j];              //像素总值
    }
    Pa = Amount[0] * (L2 - MinValue) / cnt_num;
    Pb = Amount[1] * (L4 - L2) / cnt_num;
    Ma = (double) PixelIntegral[0] / Amount[0];
    Mb = (double) PixelIntegral[1] / Amount[1];

    for (i = L2; i < MaxValue; i++)
    {
        for (j = L3; j <= MaxValue; j++)
        {
            PixelIntegral[2] += HistoGram[j] * j;              //灰度值总数
            Amount[2] += HistoGram[j];              //像素总值
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
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    uint8 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 HistoGram[256];              //
    int pixelSum=S_img_H * S_img_W;

    memset(HistoGram, 0, sizeof(HistoGram)); //初始化灰度直方图

    for (j = 0; j < pixelSum; j++)
    {
            HistoGram[image[j]]++; //统计灰度级中每个像素在整幅图像中的个数
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)      return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  return MinValue;         // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;//灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];   //前景像素点数
        PixelFore = Amount - PixelBack;         //背景像素点数
        OmegaBack = (float)PixelBack / Amount; //前景像素百分比
        OmegaFore = (float)PixelFore / Amount; //背景像素百分比
        PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
        MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
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

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值


    if (MaxValue == MinValue)      return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  return MinValue;         // 图像中只有二个颜色

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

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值


    if (MaxValue == MinValue)      return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  return MinValue;         // 图像中只有二个颜色
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

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值


    if (MaxValue == MinValue)      return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  return MinValue;         // 图像中只有二个颜色
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
//注入填充算法（Flood Fill Algorithm）
void S_FloodSeedFill (uint8 in_IMG[S_img_H][S_img_W], uint8 x, uint8 y, uint8 old_color, uint8 new_color)
{
    if (x > 58 || y > 92 || x < 1 || y < 1)
        return;         //避免溢出
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

//extern uint8 SS_ZipF[40][63];                              //压缩图像数组        滤波使用
////不使用递归  不会溢出版本
//void SX_FloodSeedFill(uint8 in_IMG[40][63], uint8 x, uint8 y, uint8 old_color, uint8 new_color)
//{
//    uint8 i, j, k;
//    uint8 return_sign;
//    in_IMG[x][y] = 255;     //过渡颜色
//    for (;;)
//    {
//        return_sign = 0;
//        for (i=0;i<40;i++)
//            for (j = 0; j < 63; j++)
//            {
//                if (in_IMG[i][j] == 255)        //过渡颜色
//                {
//                    in_IMG[i][j] = new_color;       //实际赋值
//                    SS_ZipF[i][j] = 1;
//                    for (k = 0; k < 4; k++)
//                    {
//                        x = i + directionXX[k].x_offset;        y = j + directionXX[k].y_offset;
//                        if (x < 40 && y < 63 && x >= 0 && y >= 0)       //避免溢出
//                            if (in_IMG[x][y] == old_color)
//                            {
//                                in_IMG[x][y] = 255;     //过渡颜色
//                                return_sign = 1;
//                            }
//                    }
//                }
//            }
//
//        if (return_sign == 0)       return;
//    }
//}

//在一幅图像中判定  横穿图像的白块         未实现
uint8 S_Linear (uint8 height, uint8 width, uint8* in_IMG)
{
    uint8 i, j, m, n, up, dn;
    //   uint8 getONE = 0;

    uint8 LL_h, LL_w;
    uint8 RR_h, RR_w;

    for (i = 20; i < height - 20; i++)
    {
        j = 0;
        while (j < 5)     //在5点内寻得白点
        {
            if (in_IMG[i * width + j++])
            {
                LL_h = i;
                LL_w = j - 1;       //记录左点坐标
                up = i - 7;
                dn = i + 7;
                for (m = up; m < dn; m++)
                {
                    n = width - 1;
                    while (n > width + 5)       //在5点内寻得白点
                    {
                        if (in_IMG[m * width + n--])
                        {
                            RR_h = m;
                            RR_w = n + 1;       //记录右点坐标

                            float kk, bb;
                            kk = (float) (LL_h - RR_h) / (LL_w - RR_w);
                            bb = (float) (LL_h - kk * LL_w);

                            uint8 ww, ww_COUNT = 0;
                            for (ww = LL_w + 10; ww < RR_w - 10; ww += 10)
                            {
                                if (!in_IMG[(uint8) (kk * ww + bb) * width + ww])           //如果黑
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

//参数解释：宽 高 图像指针 起始行  起始列 处理行大小  处理列大小
#define GrayScale 256   //frame灰度级
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
    //统计每个灰度级中像素的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixel[(int) Image[i * width + j]]++;
        }
    }

    for (k = 0; k < GrayScale; k++)
    {
        sum_gray += k * pixel[k];           //灰度直方图质量矩
        sum_pix_num += pixel[k];           //总像素个数
    }

    for (k = 0; k < GrayScale - 1; k++)
    {
        pl_pix_num += pixel[k];           //第一部分像素个数
        p2_pix_mum = sum_pix_num - pl_pix_num;           //第二部分像素个数
        p1_sum_gray += k * pixel[k];   //第一部分质量矩
        m1 = (float) p1_sum_gray / pl_pix_num;   //第一部分灰度均值
        m2 = (float) (sum_gray - p1_sum_gray) / p2_pix_mum;   //第二部分灰度均值

        V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

        if (V > variance)   //将类间方差较大时的灰度值作为阈值
        {
            variance = V;
            threshold = k;
        }
    }
    return threshold;
}

/************************************Robert算子************************************
 **          Gx={    {1,  0},                       Gy={ {  0,  1},
 **                  {0,-1}}                             {-1,  0}}
 **      最右、最底边缘舍去
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

/************************************Sobel算子************************************
 **          Gx={    {-1,  0,  1},                   Gy={    {  1,  2,  1},
 **                  {-2,  0,  2},                           {  0,  0,  0},
 **                  {-1,  0,  1}}                           { -1, -2, -1}}
 **      最上下左右边缘舍去
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
 **      若Threshold=0则输出灰度图      其他则二值化
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
//sobel 定制
//在已有二值化的图像中标记边缘
void S_Sobel_Custom (uint8 in_IMG[PixMini_H][PixMini_W], uint8 out_IMG[PixMini_H][PixMini_W], uint8 Threshold)
{
    uint8 i, j;
    uint8 UP, DN, LL, RR;
    uint16 TempVal;
    for (i = 1; i < 58; i++)        //HIGH  只计算前50行
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

/******************************60*94图像显示成188*120***********************************
 **
 *******************************************************************************************/
void S_DisplayMINI_Big (uint8 in_IMG[PixMini_H][PixMini_W])
{
    uint8 i, j, k;
    uint16 color;

    //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //定位字符显示区域
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

/******************************60*94图像显示灰度图***********************************
 **      size==S_BIG     显示大图                                                                                            **
 **      size==S_MINI    显示小图                                                                                            **
 ***************************************************************************************/
void S_Gray_DisplayMINI (uint8 in_IMG[PixMini_H][PixMini_W], uint8 size)
{
    uint8 i, j, k;
    uint16 color, temp;

    if (size == S_MINI)          //显示小图
    {
        //TFTSPI_Set_Pos(0, 0, 94 - 1, 60 - 1);          //定位字符显示区域
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
    else                                //显示大图
    {
        //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //定位字符显示区域
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

/******************************120*188图像显示灰度图************************************
 **                                                                                  **
 **************************************************************************************/
void S_Gray_Display (uint8 in_IMG[MT9V03X_H][MT9V03X_W])
{
    uint8 i, j;
    uint16 color, temp;
    //TFTSPI_Set_Pos(0, 0, 188 - 1, 120 - 1);          //定位字符显示区域
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

/*********************************三线图检测识别**************************************
 **                              Mod=0   寻二值化    Mod=2   寻sobel                                                      **
 ***************************************************************************************/
void DetectionLine (uint8 in_IMG[PixMini_H][PixMini_W],       //输入图像
        RunWay_* out_Array,             //输出数组
        uint8 LMP,                         //输入中点
        uint8 Mod)                         //检测模式
{
    uint8 i, j;
    uint8 M_Point, L_Point, R_Point;
//    uint8 get_Point;

    M_Point = LMP;                         //取输入的底部中点

    for (i = 58; i > 0; i--)                    //行数
    {
        /**/
        //寻左点
        j = M_Point + 5;
        if (j < 99 && j > 92)
            j = 92;
        while (!((!in_IMG[i][j + Mod]) && in_IMG[i][j + 1]) && (j > 1))
        {
            j--;
        }
        L_Point = j;
        //寻右点
        j = M_Point - 5;
        if (j > 99)
            j = 1;
        while (!((!in_IMG[i][j - Mod]) && in_IMG[i][j - 1]) && (j < 92))
        {
            j++;
        }
        R_Point = j;

        /*
         //寻左点
         get_Point = 0;
         for (j = M_Point + 5; j > 0; j--)
         {
         //防溢出
         //if (j < 0)         j = 0;
         //if (j > 187)        j = 187;
         if (in_IMG[i][j]== Mod)
         {
         L_Point = j;
         get_Point = 1;      //取得点
         break;
         }
         }
         if (get_Point==0)                    //取最左边的黑点
         {
         L_Point = 0;
         }

         //寻右点
         get_Point = 0;
         for (j = M_Point - 5; j < 59; j++)
         {
         //防溢出
         //if (j < 0)        j = 0;
         //if (j > 187)       j = 187;
         if (in_IMG[i][j] == Mod)
         {
         R_Point = j;
         get_Point = 1;      //取得点
         break;
         }
         }
         if (get_Point == 0)                     //取最右边的黑点
         {
         R_Point = 93;
         }
         */

        M_Point = (L_Point + R_Point) / 2;                         //取上一行中点
        out_Array->M[i] = M_Point;
        out_Array->L[i] = L_Point;
        out_Array->R[i] = R_Point;

        /**/
        //归线
        if (i <= 55)
        {
            //断开丢线x
            uint8 fg_no_c = 0;
            if (i <= 30)
                if (abs(M_Point - out_Array->M[i + 1]) >= 5)            //连续两行中点相差5点以上
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

/******************************三线图检测识别结合sobel定制********************************
 **                                                                                  **
 ***************************************************************************************/
void DetectionLine_Custom (uint8 in_IMG[PixMini_H][PixMini_W],       //输入图像
        RunWay_* out_Array,             //输出数组
        uint8 LMP)                         //输入中点
{
    uint8 i, j;
    uint8 M_Point, L_Point, R_Point;
    //   uint8 get_Point;

    M_Point = LMP;                         //取输入的底部中点

    for (i = 57; i > 0; i--)                    //行数
    {
        /**/
        //寻左点
        j = M_Point + 2;
        if (j < 99 && j > 90)
            j = 90;       //图像右侧2点像素不可用
        while (!(in_IMG[i][j] == S_S_Custom) && (j > 1))
        {
            j--;
        }
        L_Point = j;
        //寻右点
        j = M_Point - 2;
        if (j > 99)
            j = 1;
        while (!(in_IMG[i][j] == S_S_Custom) && (j < 92))
        {
            j++;
        }
        R_Point = j;

        /*
         //寻左点
         get_Point = 0;
         for (j = M_Point + 5; j > 0; j--)
         {
         //防溢出
         //if (j < 0)         j = 0;
         //if (j > 187)        j = 187;
         if (in_IMG[i][j]== Mod)
         {
         L_Point = j;
         get_Point = 1;      //取得点
         break;
         }
         }
         if (get_Point==0)                    //取最左边的黑点
         {
         L_Point = 0;
         }

         //寻右点
         get_Point = 0;
         for (j = M_Point - 5; j < 59; j++)
         {
         //防溢出
         //if (j < 0)        j = 0;
         //if (j > 187)       j = 187;
         if (in_IMG[i][j] == Mod)
         {
         R_Point = j;
         get_Point = 1;      //取得点
         break;
         }
         }
         if (get_Point == 0)                     //取最右边的黑点
         {
         R_Point = 93;
         }
         */

        M_Point = (L_Point + R_Point) / 2;                         //取上一行中点
        out_Array->M[i] = M_Point;
        out_Array->L[i] = L_Point;
        out_Array->R[i] = R_Point;

        /**/
        //归线
        if (i <= 55)
        {
            //断开丢线x
            uint8 fg_no_c = 0;
            if (i <= 30)
                if (abs(M_Point - out_Array->M[i + 1]) >= 5)            //连续两行中点相差5点以上
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
        if (i % 10 == 0)        //画线标记  刻度
        {
            lcd_drawpoint(0, i, RED);
            lcd_drawpoint(1, i, RED);
            lcd_drawpoint(2, i, RED);
        }

        //抹上次点
        lcd_drawpoint(TempArray.M[i], i, 0x0000);
        lcd_drawpoint(TempArray.L[i], i, 0x0000);
        lcd_drawpoint(TempArray.R[i], i, 0x0000);
        //记录旧点
        TempArray.L[i] = in_Array->L[i];
        TempArray.M[i] = in_Array->M[i];
        TempArray.R[i] = in_Array->R[i];
        //显示新点
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
            if (kk % 10 == 0)       //画线标记  刻度
            {
                lcd_drawpoint(0, kk, RED);
                lcd_drawpoint(1, kk, RED);
                lcd_drawpoint(2, kk, RED);
            }

            //抹上次点
            lcd_drawpoint(TempArray.M[i] * 2, kk, 0x0000);
            lcd_drawpoint(TempArray.L[i] * 2, kk, 0x0000);
            lcd_drawpoint(TempArray.R[i] * 2, kk, 0x0000);
            //显示新点
            lcd_drawpoint(in_Array->M[i] * 2, kk, PURPLE);
            lcd_drawpoint(in_Array->L[i] * 2, kk, BROWN);
            lcd_drawpoint(in_Array->R[i] * 2, kk, GREEN);
        }
        //记录旧点
        TempArray.L[i] = in_Array->L[i];
        TempArray.M[i] = in_Array->M[i];
        TempArray.R[i] = in_Array->R[i];
    }
}

