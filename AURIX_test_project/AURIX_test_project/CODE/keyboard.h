#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "headfile.h"

#define KEY_IN_1 P13_0
#define KEY_IN_2 P13_1
#define KEY_IN_3 P13_2
#define KEY_IN_4 P13_3


#define KEY_OUT_1 P14_6
#define KEY_OUT_2 P14_5
#define KEY_OUT_3 P14_4
#define KEY_OUT_4 P14_3

void keyboard_init(void);
void KeyScan(void);
void Check_BottonPress(void);

#endif
