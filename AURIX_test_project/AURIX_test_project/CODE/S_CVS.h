/*
 * S_CVS.h
 *
 *  Created on: 2021��6��1��
 *      Author: Dark stars
 */

#ifndef CODE_S_CVS_H_
#define CODE_S_CVS_H_
#include"headfile.h"
typedef struct{
        uint8 up;
        uint8 down;
}ValueS;

ValueS S_Get_double3S(uint8 tmImage[S_img_H][S_img_W]);//˫��ȵף���ҶȾ�ֵ��Ȼ�����ߣ����Ѱ�ȵ�
void S_BinaryImageS(uint8 in_IMG[S_img_H][S_img_W], uint8 out_IMG[S_img_H][S_img_W], ValueS ThresholdV);
extern int S_H_mid;

#endif /* CODE_S_CVS_H_ */
