/*
 * S_CVS.c
 *
 *  Created on: 2021年6月1日
 *      Author: Dark stars
 */
#include"S_CVS.h"

ValueS Value_test;
int S_H_mid = S_img_H / 2;

ValueS S_Get_double3S(uint8 tmImage[S_img_H][S_img_W]){
    uint8 i,j;
    uint8 MinValue, MaxValue;
    uint8 HistoGram[256];              //
    uint8 top1 = 0,top2 = 0;
    uint8 *p ;
    uint8 avg;
    uint8 num;
    int Amount = 0;

    int pixnums = S_img_H * S_img_W / 2;
    p = &tmImage[0][0];

    //上半图像
    memset(HistoGram, 0, sizeof(HistoGram));

    for(i = 0; i < S_H_mid; i++)
        for(j = 0;j < S_img_W; j++){
            HistoGram[tmImage[i][j]]++;
            Amount = Amount + tmImage[i][j];
        }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值


    if (MaxValue == MinValue)      {Value_test.up = MaxValue; Value_test.down = MaxValue; return Value_test;}         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  {Value_test.up = MinValue; Value_test.down = MinValue; return Value_test;}         // 图像中只有二个颜色
    avg = Amount / pixnums;
    top1 = avg;
    top2 = avg;
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
    Value_test.up = top2;
    for(i = top1; i < top2; i++){
        if(HistoGram[Value_test.up] > HistoGram[i]){
            Value_test.up = i;
        }
    }

    //下半图像因为下半图像的多为赛道，所以用全图的谷底作为阈值

    for(i = S_H_mid; i < S_img_H; i++)
        for(j = 0;j < S_img_W; j++){
            HistoGram[tmImage[i][j]]++;
            Amount = Amount + tmImage[i][j];
        }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

//    avg = Amount / pixnums / 2;
//    top1 = avg;
//    top2 = avg;
//    for(i = MinValue; i < MaxValue; i++){
//        if(i < avg){
//            if(HistoGram[top1] < HistoGram[i]){
//                top1 = i;
//            }
//        }
//        else{
//            if(HistoGram[top2] < HistoGram[i]){
//                top2 = i;
//            }
//        }
//    }
//    Value_test.down = top2;
//    for(i = top1; i < top2; i++){
//        if(HistoGram[Value_test.down] > HistoGram[i]){
//            Value_test.down = i;
//        }
//    }
    /***************均值***********/
//    for(i = MinValue; i < MaxValue; i++){
//        if(HistoGram[top1] < HistoGram[i]){
//            top1 = i;
//        }
//    }
//
//    if(top1 - MinValue < 30){
//        num = top1 + 30;
//        for(i = num; i < MaxValue; i++){
//            if(HistoGram[top2]< HistoGram[i]){
//                top2 = i;
//            }
//        }
//    }
//    else if(MaxValue - top1 < 30){
//        num = top1 - 30;
//        for(i = MinValue; i < num; i++){
//            if(HistoGram[top2]< HistoGram[i]){
//                top2 = i;
//            }
//        }
//    }
//    else{
//        num = top1 - 30;
//        for(i = MinValue; i < num; i++){
//            if(HistoGram[top2] < HistoGram[i]){
//                top2 = i;
//            }
//        }
//        num = top1 + 30;
//        for(i = num; i < MaxValue; i++){
//            if(HistoGram[top2] < HistoGram[i]){
//                top2 = i;
//            }
//        }
//    }
//    Value_test.down = (top1 + top2)/2;
    Value_test.down = Value_test.up;
    return Value_test;
}

void S_BinaryImageS(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W], ValueS ThresholdV){
    int i = 0, j = 0;
    for (i = 0; i < S_H_mid; i++)
        for (j = 0; j < S_img_W; j++)
        {
            if (in_IMG[i][j] > ThresholdV.up)
                out_IMG[i][j] = 1;
            else
                out_IMG[i][j] = 0;
        }
    for (i = S_H_mid; i < S_img_H; i++)
        for (j = 0; j < S_img_W; j++)
        {
            if (in_IMG[i][j] > ThresholdV.down)
                out_IMG[i][j] = 1;
            else
                out_IMG[i][j] = 0;
        }
}
