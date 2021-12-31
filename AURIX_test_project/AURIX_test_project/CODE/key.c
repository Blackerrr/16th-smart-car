#include"key.h"

/*拨码开关状态变量*/
uint8 sw1_status;
uint8 sw2_status;
uint8 sw3_status;
uint8 sw4_status;     /* 为 1 时检测环岛*/
extern uint8  sw_code;                            // 对编码器进行编码

/*五向按键状态变量*/
uint8 PIN_up_status;
uint8 PIN_left_status;
uint8 PIN_down_status;
uint8 PIN_mid_status;
uint8 PIN_right_status;

uint8 last_PIN_up_status;
uint8 last_PIN_left_status;
uint8 last_PIN_down_status;
uint8 last_PIN_mid_status;
uint8 last_PIN_right_status;

uint8 PIN_up_flag;
uint8 PIN_left_flag;
uint8 PIN_down_flag;
uint8 PIN_mid_flag;
uint8 PIN_right_flag;

/*拨码开关引脚初始化*/
void Switch_Init(void)
{
   //拨码开关初始化
    gpio_init(sw1,GPI,0,PULLUP);
    gpio_init(sw2,GPI,0,PULLUP);
    gpio_init(sw3,GPI,0,PULLUP);
    gpio_init(sw4,GPI,0,PULLUP);
}

/*五向按键引脚初始化*/
void FiveButton_Init(void)
{
    gpio_init(PIN_UP,GPI,1,PULLUP);
    gpio_init(PIN_LEFT,GPI,1,PULLUP);
    gpio_init(PIN_DOWN,GPI,1,PULLUP);
    gpio_init(PIN_MIDDLE,GPI,1,PULLUP);
    gpio_init(PIN_RIGHT,GPI,1,PULLUP);

    last_PIN_up_status    = 1;
    last_PIN_left_status  = 1;
    last_PIN_down_status  = 1;
    last_PIN_mid_status   = 1;
    last_PIN_right_status = 1;

}


void FiveButton_Scan(void)
{

    uint32 write_buf;
    PIN_up_status    = gpio_get(PIN_UP);
    PIN_left_status  = gpio_get(PIN_LEFT);
    PIN_down_status  = gpio_get(PIN_DOWN);
    PIN_mid_status   = gpio_get(PIN_MIDDLE);
    PIN_right_status = gpio_get(PIN_RIGHT);


    PIN_up_flag    = last_PIN_up_status    && !PIN_up_status    ? 1 : 0;
    PIN_left_flag  = last_PIN_left_status  && !PIN_left_status  ? 1 : 0;
    PIN_down_flag  = last_PIN_down_status  && !PIN_down_status  ? 1 : 0;
    PIN_mid_flag   = last_PIN_mid_status  && !PIN_mid_status   ? 1 : 0;
    PIN_right_flag = last_PIN_right_status && !PIN_right_status ? 1 : 0;

    last_PIN_up_status    = PIN_up_status;
    last_PIN_left_status  = PIN_left_status;
    last_PIN_down_status  = PIN_down_status;
    last_PIN_mid_status  = PIN_mid_status;
    last_PIN_right_status = PIN_right_status;


    if(PIN_up_flag)
    {
        gpio_toggle(P20_8);
//        pid.basic_kp += 0.1;
//        servo_limit += 5;
        round_mad++;
    }
    if(PIN_left_flag)
    {
        gpio_toggle(P20_8);
//        duty += 3;
//        pwm_duty(S_MOTOR_PIN, duty);
//        count_round++;
//        count_round %= round_num;

    }
    if(PIN_down_flag)
    {
        gpio_toggle(P20_8);
//        pid.basic_kp -= 0.1;
//        servo_limit -= 3;
        round_mad--;
    }
    if(PIN_right_flag)
    {
        gpio_toggle(P20_8);
//        duty -= 2;
//        pwm_duty(S_MOTOR_PIN, duty);
    }
    if(PIN_mid_flag)
    {
//        gyro_accumulate_status = 1 - gyro_accumulate_status;
//        garage.flag = 1 - garage.flag;
        // garage.dir = 0;      /*左出库*/
//        gpio_toggle(P20_8);
//        count_round++;
//        count_round %= round_num;
//        flag.buzzer_flag = 1;
        eeprom_erase_sector(2);
//      eeprom_page_program(2, 1, &YZ);         //将阈值写入扇区2的第一页

        write_buf = (uint32)(pid.triRoadA * 10);
        eeprom_page_program(2, 2, &write_buf);

        write_buf = (uint32)(pid.triToadB * 10);
        eeprom_page_program(2, 3, &write_buf);

        write_buf = (uint32)(pid.triRoadKd * 10);
        eeprom_page_program(2, 4, &write_buf);



        // 将五个档位速度写到flash中
        write_buf = (uint32)(speed.closed_speed[0]);
        eeprom_page_program(2, 5, &write_buf);

        write_buf = (uint32)(speed.closed_speed[1]);
        eeprom_page_program(2, 6, &write_buf);

        write_buf = (uint32)(speed.closed_speed[2]);
        eeprom_page_program(2, 7, &write_buf);

        write_buf = (uint32)(speed.closed_speed[3]);
        eeprom_page_program(2, 8, &write_buf);

        write_buf = (uint32)(speed.closed_speed[4]);
        eeprom_page_program(2, 9, &write_buf);

        // 将大环岛pd值写入flash

        write_buf = (uint32)(pid.bigRingKp * 10);
        eeprom_page_program(2, 10, &write_buf);

        write_buf = (uint32)(pid.bigRingKd * 10);
        eeprom_page_program(2, 11, &write_buf);

        // 将小环岛pd值写入flash

        write_buf = (uint32)(pid.smallRingKp * 10);
        eeprom_page_program(2, 12, &write_buf);

        write_buf = (uint32)(pid.smallRingKd * 10);
        eeprom_page_program(2, 13, &write_buf);

        // 弯道pd值写入flash
        write_buf = (uint32)(pid.curve_kp * 10);
        eeprom_page_program(2, 14, &write_buf);

        write_buf = (uint32)(pid.curve_kd * 10);
        eeprom_page_program(2, 15, &write_buf);

        write_buf = (uint32)(pid.basic_kp * 10);
        eeprom_page_program(2, 16, &write_buf);

        write_buf = (uint32)(pid.basic_kd * 10);
        eeprom_page_program(2, 17, &write_buf);

        write_buf = (uint32)(servo_limit * 10);
        eeprom_page_program(2, 18, &write_buf);

        /*写入弯道最低速度和弯道最高速度*/
        write_buf = (uint32)(speed.curve_low_speed);
        eeprom_page_program(2, 19, &write_buf);

        write_buf = (uint32)(speed.curve_high_speed);
        eeprom_page_program(2, 20, &write_buf);


        /*写入直道参数*/
        write_buf = (uint32)(A * 10);
        eeprom_page_program(2, 21, &write_buf);


        write_buf = (uint32)(B * 10);
        eeprom_page_program(2, 22, &write_buf);

        write_buf = (uint32)(pid.straightKd * 10);
        eeprom_page_program(2, 23, &write_buf);

        write_buf = (uint32)(pid.LRTC_kp * 10);
        eeprom_page_program(2, 24, &write_buf);

        write_buf = (uint32)(pid.LRTC_kd * 10);
        eeprom_page_program(2, 25, &write_buf);
    }



}

/*
 *  0 :
 * */
void Switch_Scan(void)
{

    //获取拨码开关状态
    sw1_status = gpio_get(sw1);
    sw2_status = gpio_get(sw2);
    sw3_status = gpio_get(sw3);
    sw4_status = gpio_get(sw4);



    sw_code = sw_code | sw1_status << 4;
    sw_code = sw_code | sw2_status << 3;
    sw_code = sw_code | sw3_status << 2;
    sw_code = sw_code | sw4_status;
}
