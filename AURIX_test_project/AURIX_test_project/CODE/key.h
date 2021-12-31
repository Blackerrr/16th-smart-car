#ifndef __KEY_H
#define __KEY_H
#include "headfile.h"



/**拨码开关**/
extern uint8 sw1_status;
extern uint8 sw2_status;
extern uint8 sw3_status;
extern uint8 sw4_status;


/********************五向按键引脚定义 ******************/
#define PIN_UP       P11_9    // up
#define PIN_LEFT     P11_3    // left
#define PIN_DOWN     P11_6    // down
#define PIN_MIDDLE   P11_2    // mid
#define PIN_RIGHT    P11_10   // right

void Switch_Init(void);
void FiveButton_Init(void);
void FiveButton_Scan(void);
void KEY_Scan(void);


#endif
