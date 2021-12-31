#include "show.h"

//extern uint16 circle_flag;

extern  _pid  servo_pid;
extern  _pid  motor_pid;


uint8 last_tft_status;


//extern uint8 curve;
extern uint8 road_status;          // 0 直道  1 十字 2 环岛 3 普通弯道
void tft_config(void)
{
    lcd_init();                                 // TFT 屏幕初始化

}

/*显示当前各参数状态*/
void tft_show1(void)
{
    lcd_showstr(0, 0, "Servo PD:");
    lcd_showfloat(0, 1, servo_pid.Kp, 2, 1);
    lcd_showfloat(64, 1, servo_pid.Kd, 3, 1);

#ifdef MOTOR_USE_POSITION_PID
        lcd_showstr(0, 2, "Pos Moto PI");
#elif
        lcd_showstr(0, 2, "Inc Moto PI");
#endif
    lcd_showfloat(0, 3, motor_pid.Kp, 2, 1);
    lcd_showfloat(64, 3, motor_pid.Ki, 2, 1);

    lcd_showstr(0, 4, "speed:");
    lcd_showint32 (64 , 4,  flag.speed_debug, 1);
    lcd_showint32 (0 , 5,  speed.closed_speed[0], 3);
    lcd_showint32 (32 , 5,  speed.closed_speed[1], 3);
    lcd_showint32 (64 , 5,  speed.closed_speed[2], 3);
    lcd_showint32 (96 , 5,  speed.closed_speed[3], 3);
    lcd_showint32 (0 , 6,  speed.closed_speed[4], 3);
    lcd_showint32 (32 , 6,  speed.curve_low_speed, 3);
    lcd_showint32 (64 , 6,  speed.curve_high_speed, 3);
    lcd_showstr(0, 7, "error: ");
    lcd_showfloat(64 , 7,  error.error, 3, 1);
}


/*显示电感及主要参数*/
void tft_show2(void)
{
//     lcd_showstr(0, 0, "guiyi: ");
//     lcd_showint32 (50,  0, flag.return_to_one_flag, 1);
     lcd_showstr(0, 1, "L1:");
     lcd_showint32 (25, 1, adc_original[1], 3);
     lcd_showstr(64, 1, "L5:");
     lcd_showint32 (89, 1, adc_original[7], 3);
     lcd_showstr(0, 2, "L2:");
     lcd_showint32 (25, 2, adc_original[2], 3);
     lcd_showstr(64, 2, "L4:");
     lcd_showint32 (89, 2, adc_original[6], 3);
     lcd_showstr(0, 3, "L3:");
     lcd_showint32 (25, 3,  adc_original[4], 3);
     lcd_showint32 (89, 3, adc_original[8], 3);
     lcd_showint32 (24, 4, adc_original[3], 3);
     lcd_showint32 (89, 4, adc_original[5], 3);

     lcd_showstr(0, 5, "encoder:");
     lcd_showint32 (64 , 5,  encoder, 3);

     switch(flag.PID_parameter_debug)
     {
         case 0:
             //显示直道参数
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "normal");
             lcd_showfloat(0, 7, A, 2, 1);
             lcd_showfloat(64, 7, B , 2, 1);
             lcd_showfloat(0, 8,  pid.straightKd, 4, 1);
             break;
         case 1:
             // 显示丢线参数
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "lost line  ");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             lcd_showfloat(0, 7, pid.curve_kp, 2, 1);
             lcd_showfloat(64, 7, pid.curve_kd , 4, 1);
             lcd_showuint16_new (0,  8,    Lostlinelimit, 3);

             break;
         case 2:
             // 显示小环岛参数
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "smallRing");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
             lcd_showfloat(0, 7, pid.smallRingKp, 2, 1);
             lcd_showfloat(64, 7, pid.smallRingKd , 4, 1);
//             lcd_showstr  (0, 8, "      ");
             lcd_showint32(0, 8, smallEncoderValue, 4);
             break;
         case 3:
             // 显示大环岛参数
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "bigRing");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
             lcd_showfloat(0, 7, pid.bigRingKp, 2, 1);
             lcd_showfloat(64, 7, pid.bigRingKd , 4, 1);
//             lcd_showstr  (0, 8, "      ");
             lcd_showint32(0, 8, bigEncoderValue, 4);
             break;
         case 4:
             // 显示三叉参数
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "triRoad");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             lcd_showfloat(0, 7, pid.triRoadA, 2, 1);
             lcd_showfloat(64, 7, pid.triToadB , 2, 1);
             lcd_showfloat(0, 8,  pid.triRoadKd, 4, 1);
             lcd_showint32 (40 , 8, tri_road_mad_value, 2);
             lcd_showfloat(40 + 24 + 4 , 8, hubu_angle, 2,  1);
             lcd_showint32 (40 + 24 + 4 + 24 + 8 , 8, tri_road_status, 1);
             break;
         case 5:
             // 显示电机参数
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "moto");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             lcd_showfloat(0, 7, motor_pid.Kp, 3, 1);
             lcd_showfloat(64, 7, motor_pid.Ki , 2, 1);
             break;

         case 6:
             /*显示新动态P值*/
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "in curve ");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             lcd_showfloat(0,  7, pid.basic_kp,  4, 1);
             lcd_showfloat(64, 7, pid.basic_kd , 4, 1);
             break;
         case 7:
             /*显示入弯PD*/
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "enter curve ");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             lcd_showfloat(0,  7, pid.LRTC_kp,  4, 1);
             lcd_showfloat(64, 7, pid.LRTC_kd , 4, 1);
             break;
         case 8:
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "speed debug ");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
//             lcd_showstr(0, 7, "                ");
//             lcd_showstr(0, 8, "                ");
             break;
         case 9:
//             lcd_showstr(32, 6, "            ");
             lcd_showstr(32, 6, "garage ");
             lcd_showint32 (0 , 6,  flag.PID_parameter_debug, 1);
             lcd_showstr(0, 7, "dir ");   lcd_showint32 (32 , 7, garage.dir, 1);
             lcd_showstr(64,7, "flag ");  lcd_showint32 (64 + 40 + 4 , 7, flag.enter_garage, 1);
             lcd_showstr(0, 8, "angle");  lcd_showint32 (40 + 4 , 8, garage.angle, 3);
             lcd_showint32 (70 , 8, garage.status, 1);
             break;
         default: ;

     }

//     lcd_showfloat(64 , 9,  pitch, 3, 1);
//     lcd_showfloat(64 , 9,  error.lean_error, 3, 1);
//     lcd_showint32 (64, 8, flag.ramp_flag, 1);
//     lcd_showint32 (96, 9, curve_status, 1);
//     lcd_showint32 (0, 9, flag.front_lost_line, 1);
//     lcd_showint32 (32, 9, flag.back_lost_line, 1);
//     lcd_showint32 (64, 9, servo_deviation_flag, 1);
       lcd_showint32 (0, 9, round_flag, 1);
       lcd_showint32(24 , 9,  encoder_circle_count, 4);
}


void tft_show3(void)
{
    lcd_showstr(0,  0, "double");
    lcd_showstr(64, 0, "single");
    for(int i = 1; i <= 8; i++)
    {
        switch(i)
        {
            case 1:
                lcd_showstr(0,  i, "1:");
                break;
            case 2:
                lcd_showstr(0,  i, "2:");
                break;
            case 3:
                lcd_showstr(0,  i, "3:");
                break;
            case 4:
                lcd_showstr(0,  i, "4:");
                break;
            case 5:
                lcd_showstr(0,  i, "5:");
                break;
            case 6:
                lcd_showstr(0,  i, "6:");
                break;
            case 7:
                lcd_showstr(0,  i, "7:");
                break;
            case 8:
                lcd_showstr(0,  i, "8:");
                break;
        }
        lcd_showuint16_new (12,  i,    adc_min[i], 2);         lcd_showuint16_new (36, i,          adc_max[i], 3);
        lcd_showuint16_new (64,  i, adcSingleLineMin[i], 3);   lcd_showuint16_new (96, i, adcSingleLineMax[i], 3);

    }

}
