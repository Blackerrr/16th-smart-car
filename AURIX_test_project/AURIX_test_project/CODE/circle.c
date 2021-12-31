#include "circle.h"

extern RINGQ *que;                      // ���ζ���

//uint16 encoder_count;
//extern int16 encoder;
uint16 circle_detect_sleep;



// �жϴ�ʱ���ζ����ǲ��ǵݼ���
uint16 circle_judge(RINGQ *p_ringq)
{
    if(!ringq_is_full(p_ringq))           // ������δ����ֱ�ӷ���
        return 0;
    for(int i = p_ringq->head, j = 0; j < p_ringq->size - 1; j++)
    {
        if(p_ringq->space[i] < p_ringq->space[(i + 1) % p_ringq->size] )
            return 0;
        i = (i + 1) % p_ringq->size;
    }
    return 1;
}


// ��ֱ����뻷
void circle_handle_new_3(void)
{
    static uint16 last_mid = 0;
    static uint16 count = 0;
    static uint16 encoder_count;
    switch(round_flag)
    {
        case 0:
            if(!flag.circle && !flag.circle_left && !flag.circle_right && adc_original[4] > round_mad && LAD > 80 && RAD > 80
               && !flag.tri_road_in && !flag.ramp_flag && !garage.flag && !tri_road_status)
            {
                round_flag++;     /*��ǰհ���뻷������*/
            }
//            if(!flag.circle && !flag.circle_left && !flag.circle_right && adc_original[8] > round_mad && lean_AD1 > 80 && lean_AD2 > 80
//               && !flag.tri_road_in && !flag.ramp_flag && !garage.flag && !tri_road_status)

            break;
        case 1:
            if(LMAD < 10 && RMAD < 10)
            {
                round_flag++;     /*��ǰհ���ﻷ�е�*/
                encoder_count = 0;
            }
            break;
        case 2:
            encoder_count += encoder;
            if(round_status[count_round] > 1)    // С��
            {

                if(encoder_count > smallEncoderValue)
                {
                    encoder_count = 0;
                    round_flag++;    // �����뻷
                    if(round_status[count_round] == 0 || round_status[count_round] == 2)//
                    {
                        flag.circle_left = 1;    /* ��������� */
                    }
                    else
                    {
                        flag.circle_right = 1;   /* �������ұ� */
                    }
                    flag.buzzer_flag = 1;
                    START_ANGLE_ACCUMULATE;      /* ��ʼ������Ƕ� */
                }
            }
            else                                     //��
            {
                if(encoder_count > bigEncoderValue)
                {
                    encoder_count = 0;
                    round_flag++;    // �����뻷
                    if(round_status[count_round] == 0 || round_status[count_round] == 2)//
                    {
                        flag.circle_left = 1;    /* ��������� */
                    }
                    else
                    {
                        flag.circle_right = 1;   /* �������ұ� */
                    }
                    flag.buzzer_flag = 1;
                    START_ANGLE_ACCUMULATE;      /* ��ʼ������Ƕ� */
                }
            }
            break;
        case 3:
            /* ��ֱ����뻷 */
//            encoder_count += encoder;


           if(get_absf(hubu_angle) > 60)
           {
               round_flag++;                /* �ɹ��뻷 */
               flag.buzzer_flag = 1;
               encoder_count = 0;
           }
//           if(encoder_count > 1200 * 8)
//           {
//               /*˵���뻷ʧ�ܣ� ��������*/
//               encoder_count = 0;
//               round_flag = 0;        /*���ص���⻷��״̬*/
//           }
            break;
        case 4:
            /* ���� */
            if(get_absf(hubu_angle) > 300)   /* ���Գ��� */
            {
                round_flag++;                /* ���Գ����� */
                encoder_count = 0;
                STOP_ANGLE_ACCUMULATE;
                flag.buzzer_flag = 1;
//                flag.circle_left = 0;
//                flag.circle_right = 0;
            }
            break;
        case 5:
            /* �����������Ʋ� */
            encoder_count += encoder;
            if(encoder_count > 1200 * 2)
            {
                flag.circle_left = 0;
                flag.circle_right = 0;
            }
            if(encoder_count > 1200 * 20)
            {
                encoder_count = 0;
                round_flag = 0;     /*  �ص�0 ���¿�ʼ��⻷��*/
                count_round++;
                count_round %= round_num;
            }
    }
    last_mid = adc_original[8];
    encoder_circle_count = encoder_count;
}



//extern uint8 road_status;          // 0 ֱ��  1 ʮ�� 2 ���� 3 ��ͨ���

/*·�����*/
void road_detect(void)
{
    servo_deviation_flag = 0;
    if(garage.flag)           /*����*/
    {
        motor_target = speed.closed_speed[2];   // ����
        return;
    }
    if(flag.ramp_flag)           /*�µ�*/
    {
        flag.curve_flag = 0;
        motor_target = speed.closed_speed[2];   // ����
        return;
    }
    if(tri_road_status)           /*����*/
    {
        switch(tri_road_status)
        {
            case 1:    // ׼����������
                motor_target = speed.closed_speed[0];  // ����
                flag.curve_flag = 0;
                break;
            case 2:   // ���ڽ�������
                motor_target = speed.closed_speed[1];
                flag.curve_flag = 1;
                break;
            case 3:   // ������
                motor_target = speed.closed_speed[2];
//                in_curve_speed_control();
                if(LMAD > 10 || RMAD > 10)
                {
                    flag.curve_flag = 1;
                }
                else
                    flag.curve_flag = 0;
//                curve_handle();
                break;
            case 4:
                motor_target = speed.closed_speed[1];
                flag.curve_flag = 1;
                servo_deviation_flag = 6;    /*����ǰհ������*/
                break;
            case 5:
                motor_target = speed.closed_speed[2];
                flag.curve_flag = 1;
                servo_deviation_flag = 2;
        }
        return;
    }
    if(flag.circle_left || flag.circle_right)
    {
        switch(round_flag)
        {
            case 1:
                /*��ǰհ���뻷��*/
                servo_deviation_flag = 2;                 /*����ǰ��ˮƽ��м�Ȩ����ƫ��*/
                motor_target = speed.closed_speed[2];    // ����
                flag.curve_flag = 1;
                break;
            case 2:
                servo_deviation_flag = 2;                 /*����ǰ��ˮƽ��м�Ȩ����ƫ��*/
                motor_target = speed.closed_speed[2];     // ����
                flag.curve_flag = 1;
                break;
            case 3:
                /* ��ֱ����뻷 */
                servo_deviation_flag = 3;
                motor_target = speed.curve_low_speed;     // ����
                flag.curve_flag = 0;
                break;
            case 4:
                /* ���� */
                servo_deviation_flag = 2;
                flag.curve_flag = 0;
//                in_curve_speed_control();
                motor_target = speed.closed_speed[3];
                break;
            case 5:
                /* �����������Ʋ�  */
                flag.curve_flag = 1;
                motor_target = speed.curve_low_speed;     // ����
                servo_deviation_flag = 2;      /*����ǰ��ˮƽ��м�Ȩ����ƫ��*/
                break;
        }
        return;
    }
    curve_handle();
    if(flag.front_lost_line || flag.back_lost_line)
    {
        motor_target = speed.closed_speed[2];   // ����
    }
}

void in_curve_speed_control(void)
{
    int delt = (speed.curve_high_speed - speed.curve_low_speed) / 4;

     /*���ݳ���е�ƫ������ٶ�*/
     if(get_absf(error.H_error) < 25) {
         motor_target = speed.curve_high_speed;
     }
     else if(get_absf(error.H_error) < 50) {
         motor_target = speed.curve_high_speed - delt;
     }
     else if(get_absf(error.H_error) < 75) {
         motor_target = speed.curve_high_speed - delt * 2;
     }
     else {
         motor_target = speed.curve_high_speed - delt * 3;
     }
}


/*�������*/
void curve_handle(void)
{
    static uint16 encoder_count;
    static uint8 count0, count1, count2 ;
    if(flag.circle_left || flag.circle_right)
    {
        curve_status = 0;
        return;
    }

    if(LMAD > 10 || RMAD > 10)
    {
        flag.curve_flag = 1;
        motor_target = speed.closed_speed[2];      /*�������*/
    }
    else
    {
        flag.curve_flag = 0;
    }
//    switch(curve_status)
//    {
//        case 0:                    /*ֱ���������*/
//            flag.curve_flag = 0;
//            motor_target = speed.closed_speed[4];
//            if((LMAD > 10 || RMAD > 10) && !flag.front_lost_line && !flag.back_lost_line)
//            {
//                    curve_status += 1;
//                    encoder_count = 0;
//                    START_ANGLE_ACCUMULATE;
//            }
//            else
//                count0 = 0;
//            break;
//        case 1:
//            flag.curve_flag = 1;
//            servo_deviation_flag = 1;
//            encoder_count += encoder;
//            motor_target = speed.curve_low_speed;      /*�������*/
//            if(LMAD < 10 && RMAD < 10 && !flag.front_lost_line)
//            {
//                count1++;
//                if(count1 > 9)
//                {
//                    curve_status = 0;
//                }
//            }
//            else
//                count1 = 0;
//
//#if 1
//            if(get_absf(hubu_angle) > 30)
//            {
//                STOP_ANGLE_ACCUMULATE;
//                curve_status++;
//            }
//#endif
//#if 1
//            encoder_count += encoder;
//            if(encoder_count > 2400 && get_absf(hubu_angle) <= 30)
//            {
//                curve_status++;
//                encoder_count = 0;
//                STOP_ANGLE_ACCUMULATE;
//            }
//#endif
//            break;
//        case 2:
//            flag.curve_flag = 1;
//            servo_deviation_flag = 2;
//            in_curve_speed_control();
////            if(LMAD < 10 && RMAD < 10 && !flag.front_lost_line)
////            {
////                count2++;
////                if(count2 > 9)
////                {
////                    curve_status = 0;
////                }
////            }
////            else
////                count2 = 0;
//            if(LMAD < 10 && RMAD < 10 && !flag.front_lost_line)
//            {
//                curve_status++;
//                encoder_count = 0;
//            }
//            break;
//        case 3:
//            encoder_count += encoder;
//            if(encoder_count > 1800)
//            {
//                encoder_count = 0;
//                curve_status = 0;
//            }
//            break;
//
//    }
}

/*�������*/
void curve_handle_new(void)
{
    static uint16 encoder_count;
    static uint8 count0, count1, count2 ;

    switch(curve_status)
    {
        case 0:                    /*ֱ���������*/
            flag.curve_flag = 0;
            motor_target = speed.closed_speed[4];
            if((LMAD > 10 || RMAD > 10) && !flag.front_lost_line && !flag.back_lost_line)
            {
//                count0++;
//                if(count0 > 4)
//                {
                    curve_status++;
                    encoder_count = 0;
                    START_ANGLE_ACCUMULATE;
//                }
            }
            else
                count0 = 0;
            break;
        case 1:
            flag.curve_flag = 1;
            servo_deviation_flag = 1;
            motor_target = speed.curve_low_speed;      /*�������*/
            if(LMAD < 10 && RMAD < 10 && !flag.front_lost_line && MAD > 60)
            {
                count1++;
                if(count1 > 9)
                {
                    curve_status = 0;
                }
            }
            else
                count1 = 0;

#if 1
            if(get_absf(hubu_angle) > 30)
            {
                STOP_ANGLE_ACCUMULATE;
                curve_status++;
            }
#endif
#if 1
            encoder_count += encoder;
            if(encoder_count > 2400 && get_absf(hubu_angle) <= 30)
            {
                curve_status++;
                encoder_count = 0;
                STOP_ANGLE_ACCUMULATE;
            }
#endif
            break;
        case 2:
            flag.curve_flag = 1;
            servo_deviation_flag = 2;
            in_curve_speed_control();
            if(LMAD < 10 && RMAD < 10 && !flag.front_lost_line  && MAD > 60)
            {
                count2++;
                if(count2 > 9)
                {
                    curve_status = 0;
                }
            }
            else
                count2 = 0;
            break;
    }
}


/*���⴦��*/

void garage_handle(void)
{
    switch(garage.status)
    {
        case 0:
            if(garage.flag)
            {
                garage.status++;
                START_ANGLE_ACCUMULATE;
            }
            break;
        case 1:
            motor_target = 30;
            if(get_absf(yaw) > 45)
            {
                garage.status++;
                garage.status %= 2;
                garage.flag = 0;
                STOP_ANGLE_ACCUMULATE;
            }
    }
}


void enter_garage(void)
{
    static uint8 time;
    switch(flag.enter_garage)
    {
        case 0:
            if(road_status == 3 && !flag.ramp_flag)
            {
                flag.enter_garage++;
                START_ANGLE_ACCUMULATE;
                flag.buzzer_flag = 1;
                time = 0;
            }
            break;
        case 1:
            time++;
            pwm_duty(MOTOR2_PWM, 1000);
            gpio_set(MOTOR2_DIR, 1);
            switch(garage.dir)
            {
              case 0:
                  duty = SERVO_MID - servo_limit;
                  pwm_duty(S_MOTOR_PIN, duty);
                  /*���������*/
                  break;
              case 1:
                  duty = SERVO_MID + servo_limit;
                  pwm_duty(S_MOTOR_PIN, duty);
                  /*���Ҵ�����*/
                  break;
            }
            if(time > 19)
            {
                flag.enter_garage++;
                time = 0;
            }
            break;
        case 2:
            pwm_duty(MOTOR2_PWM, 0);
            gpio_set(MOTOR2_DIR, 1);
            if(get_absf(hubu_angle) > garage.angle)
            {
                pwm_duty(S_MOTOR_PIN, SERVO_MID);
                motor_start = 0;
            }
            break;
    }
}






