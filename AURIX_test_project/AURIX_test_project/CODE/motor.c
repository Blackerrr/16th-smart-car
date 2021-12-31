#include "motor.h"
#include "stdlib.h"
#include "math.h"


extern volatile _pid  servo_pid;
extern volatile _pid  motor_pid;

uint8  servo_deviation_flag;    // 0 ˮƽ���ѭ��  1 ��ֱ���ѭ��

extern uint8 road_status;
extern RINGQ *que;                           // ���ζ���
uint8 dir_last;

void Motor_init(void)
{
#if 0     /*��PWM��������*/
	gtm_pwm_init(MOTOR2_A, 13000, 0);
	gtm_pwm_init(MOTOR2_B, 13000, 0);
	pwm_duty(MOTOR2_A, 0);
	pwm_duty(MOTOR2_B, 0);
#elif 1   /*��PWM��������*/
    gtm_pwm_init(MOTOR2_PWM, 13000, 0);
    gpio_init(MOTOR2_DIR, GPO, 0, PUSHPULL);
    pwm_duty(MOTOR2_PWM, 0);
    gpio_set(MOTOR2_DIR, 0);
#endif
}

// ���� 10000
static void pwm_xianfu(void)
{
	if(moto >= 9000)
		moto = 9000;
	else if(moto < 0)
		moto = 0;
	motor_pid.actual_val = (float)moto;
}

static int16 Servo_xianfu(int16 servo)
{
	if(servo < -servo_limit)
		servo = -servo_limit;
	else if(servo > servo_limit)
		servo = servo_limit;
	return servo;
}

// �ǿ⺯������
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

float get_absf(float value)
{
	return value > 0 ? value : -value;
}

/* �����������˲� */
int16 RecurrenceFilter (int16 DATA)
{
	static uint8 first = 1;
	uint8 i;
	int16 sum = 0;
	uint8 i_val = 5;
	if(first) {
		first = 0;
		for (i = 1; i < i_val; i++)
			EncoderData[i - 1] = DATA;
	} else {
		for (i = 1; i < i_val; i++) {
			EncoderData[i - 1] = EncoderData[i];
		}
	}
	EncoderData[i_val - 1] = DATA;
	sum = 0;
	//����Ȩ����ӣ�����ȥ��ͻ��㡣
	sum = EncoderData[0] * 0.05 + EncoderData[1] * 0.1 + EncoderData[2] * 0.15 + EncoderData[3] * 0.3 + EncoderData[4] * 0.4;

	return sum;

}

void Motor_control(int32 encoder)
{
	static int32 last_moto;
	float moto_error;
	if(!motor_start) {
		pwm_duty(MOTOR2_A, 0);
		pwm_duty(MOTOR2_B, 0);
		return;
	}
#if MOTOR_DEBUG

#else
//    motor_target = closed_lower_limit_speed + (100 - get_absf(error)) * (100 - get_absf(error))  * (closed_up_limit_speed - closed_lower_limit_speed) / 10000;
	//motor_target = 150;
	set_pid_target(&motor_pid, motor_target);
#endif
//    moto_error = get_absf(motor_pid.target_val - encoder);

	/************* PID�õ�ƫ�� *****************/
#if MOTOR_USE_POSITION_PID
	moto = (int32)PosPID_realize(&motor_pid, (float)encoder);               // λ�� PID
#elif 1
	moto = (int32)IncPID_realize(&motor_pid, (float)encoder);                // ���� PID
#endif

////	if(!flag.curve_flag)
////	{
////	    if(!flag.curve_flag) {     /*������ٲ��ð���*/
//	        if(motor_pid.target_val - encoder > 10)
//	            moto = 9000;
////	    }
//	    if(encoder - motor_pid.target_val > 5)
//	        moto = last_moto * 0 / 10;
////	}
//    /*************³��*****************/
	pwm_xianfu();
	last_moto = moto;

	if(!flag.motor_reverse)
	{
//	    pwm_duty(MOTOR2_A, moto);
//	    pwm_duty(MOTOR2_B, 0);

	    pwm_duty(MOTOR2_PWM, moto);
	    gpio_set(MOTOR2_DIR, 0);
	}
	else
	{
	    switch(motor_reverse_level)
	    {
	        case 0:
//	            pwm_duty(MOTOR2_A, 0);
//	            pwm_duty(MOTOR2_B, 500);

	            pwm_duty(MOTOR2_PWM, 500);
	            gpio_set(MOTOR2_DIR, 1);

	            break;
	        case 1:
//                pwm_duty(MOTOR2_A, 0);
//                pwm_duty(MOTOR2_B, 1000);
                pwm_duty(MOTOR2_PWM, 1000);
                gpio_set(MOTOR2_DIR, 1);
	            break;
	    }
	}
}



// ����ˮƽ���ƫ��
void update_H_error(void)
{
//	static float last_error = 0;
    static float temp = 5;     /*��������*/
    if(LAD + RAD > 30) {
        error.H_error = 100 * (my_sqrt(RAD) - my_sqrt(LAD)) / (my_sqrt(RAD) + my_sqrt(LAD));
        temp = my_sqrt(RAD) - my_sqrt(LAD);
    } else {
        error.H_error = 100 * temp / (my_sqrt(RAD) + my_sqrt(LAD));
    }
}

void update_MAD_error(void)
{
	static float last_error = 0;
	float L, R;
	L = LMAD + LAD;
	R = RMAD + RAD;
//    if(!flag.front_lost_line)
//    {
//        error.mid_error = 0;
//    }
//    else
//    {
	float e = get_absf(100 - MAD);
	error.mid_error = lean_AD2 > lean_AD1 ? e : -e;
//    }
}

void update_V_error(void)
{
	error.V_error = 100 * (my_sqrt(RMAD) - my_sqrt(LMAD)) / (my_sqrt(LMAD) + my_sqrt(RMAD));
}


// ���°��ֵ��ƫ��
void update_lean_error(void)
{
	static float temp = 5;     /*��������*/
	if(lean_AD1 < 30 || lean_AD2 < 30) {
		error.lean_error = 100 * (my_sqrt(lean_AD2) - my_sqrt(lean_AD1)) / (my_sqrt(lean_AD2) + my_sqrt(lean_AD1));
//		temp = get_absf(my_sqrt(lean_AD2) - my_sqrt(lean_AD1));
        temp = my_sqrt(lean_AD2) - my_sqrt(lean_AD1);
	} else {
		error.lean_error = 100 * temp / (my_sqrt(lean_AD2) + my_sqrt(lean_AD1));
//		error.lean_error = lean_AD2 > lean_AD1 ? error.lean_error : -1 * error.lean_error;
	}
}


void lost_line(void)
{
    if(tri_road_status == 1)
    {
        flag.front_lost_line = 0;
        flag.back_lost_line = 0;
        return;
    }
	switch(flag.back_lost_line) {
	case 0:
		if(lean_AD1 + lean_AD2 < Lostlinelimit)
		{
            last_dir = lean_AD2 > lean_AD1 ? 1 : 0;
			flag.back_lost_line = 1 - flag.back_lost_line;   // ��ǰհ����
		}
		break;
	case 1:
		if(lean_AD1 + lean_AD2 > Lostlinelimit) {
			flag.back_lost_line = 1 - flag.back_lost_line;
		}
		break;

	}
}


// ������kp
void CalculateServoKp(void)
{

	float t = get_absf(error.error / 100.0f);
	float e6 = t * 6;
	// ֱ��.
	servo_pid.Kp = A * (t * t) + B;
	servo_pid.Kd = pid.straightKd;
	if(flag.tri_road_in) {
		servo_pid.Kp = pid.triRoadA * (t * t) + pid.triToadB;
		servo_pid.Kd = pid.triRoadKd;
	}
	if((flag.circle_left || flag.circle_right)) {
	    /* �����뻷 */
	    if(round_flag == 3)
	    {
	        if(round_status[count_round] < 2) {
	            // �󻷵�
	            servo_pid.Kp = pid.bigRingKp;
	            servo_pid.Kd = pid.bigRingKd;
	        } else {
	            // С����
	            servo_pid.Kp = pid.smallRingKp;
	            servo_pid.Kd = pid.smallRingKd;
	        }
	    }
	    else
	    {
	        if(round_status[count_round] < 2) {
	            // �󻷵�
	            servo_pid.Kp = pid.bigRingKp * (0.1 + 0.9 * get_absf((expf(e6) - 1) / (expf(e6) + 1)));
	            servo_pid.Kd = pid.bigRingKd;
	        } else {
	            // С����
	            servo_pid.Kp = pid.smallRingKp * (0.4 + 0.6 * get_absf((expf(e6) - 1) / (expf(e6) + 1)));
	            servo_pid.Kd = pid.smallRingKd;
	        }
	    }

	}
	if(flag.curve_flag) {
	        servo_pid.Kp = pid.basic_kp * (0.2 + 0.8 * get_absf((expf(e6) - 1) / (expf(e6) + 1)));
	        servo_pid.Kd = pid.basic_kd;
	}
	if(flag.back_lost_line) {
		servo_pid.Kp = pid.curve_kp;
		servo_pid.Kd = pid.curve_kd;
	}
}


static void error_filter(void)
{
	static float a[3];
	static uint8 i;
	static uint8 first = 1;
	if(first) {
		first = 0;
		a[0] = a[1] = a[2] = error.error;
	}
	a[i] = error.error;
	error.error = a[i] * 1 / 4 + a[(i + 1) % 3] * 1 / 2 + a[(i + 2) % 3] * 1 / 4;
	i++;
}

float get_max(float t1, float t2)
{
    return get_absf(t1) > get_absf(t2) ? t1 : t2;
}
void CalculateError(void)
{
    static uint16 t;
    float t_e;
    t = encoder;
    if(t > 90)
        t = 90;
    if(t < 50) {
        t = 50;
    }
    t -= 50;
	switch(servo_deviation_flag) {
	case 0:       /*����ѭ��*/
		error.error = error.lean_error;
		break;
	case 1:      /*������*/
	    error.error = error.lean_error;
	    break;
	case 2:
	    /*����*/
        error.error = error.lean_error;
	    break;
	case 3:
	     error.error = error.V_error;
	     break;


	}



}

void Servo_control(void)
{
	if(!motor_start)
		return;
	static uint16 last_duty = SERVO_MID;
	update_lean_error(); // ���°��ֵ��ƫ��
	update_H_error();    // ����ˮƽ������
	update_V_error();    // ���´�ֱ������
	update_MAD_error();  // �����м��б궨ƫ��

    lost_line();                   // ��鶪��

	CalculateError();    // ���㵱ǰƫ��
	error.error_delta = error.error - error.last_error;
	error.last_error = error.error;
	CalculateServoKp();  // ���PDֵ����
#if USING_ERROR_FILTER
	error_filter();      // ���ƽ���˲�
#endif

//    /*************��ƫ����������*****************/
#if 0
	if(ringq_is_full(que))
		ringq_poll(que);
	ringq_push(que, error.error);
#endif

#if 0
	if(get_absf(error.error_delta) > 10)
		error.error_delta = error.error_delta > 0.0f ? 10.0f : -10.0f;
	error.error = error.last_error + error.error_delta;
#endif

	int16 servo = (int16)PosPID_realize(&servo_pid, error.error);      // λ��PD
	servo = Servo_xianfu(servo);                                       // ����޷�
	last_duty = duty;
#if 1
	if(flag.front_lost_line && flag.back_lost_line) {
		if(!last_dir)   // �������
			duty = SERVO_MID - servo_limit;
		else            // ���Ҵ���
			duty = SERVO_MID + servo_limit;

	} else
		duty = SERVO_MID - servo;
#endif
	pwm_duty(S_MOTOR_PIN, duty);
}


void stop_car_detect(void)
{
	static uint8 stop_count = 0;
	if(LMAD < 5 && RMAD < 5 && LAD < 5 && RAD < 5 && _MAD < 5 && lean_AD1 < 5 && lean_AD2 < 5) {                  // ˵�����Ѿ�Զ����������Ҫ�ѳ�ͣס
		stop_count++;
		if(stop_count == 10) {
			motor_start = 0;
			stop_count = 0;
		}
	} else {
		stop_count = 0;
		motor_start = 1;
	}

}
void encoder_handle(void)
{
	// ����תһȦ���20cm��������ת2��3��֮һȦ����Լ1200��

}

uint8 tri_road_delay_count, tri_road_delay_flag;    // ����·�ڴ�����ʱ

void tri_road_handle(void)
{
    static uint16 encoder_count = 0;
	static uint8 count = 0;
	switch(tri_road_status) {
	case 0:
		/***********��⵽ǰ����������*************/
	    if(road_status == 2 &&!flag.circle_left && !flag.circle_right && LMAD  < 50 && RMAD < 50 && !garage.flag) {
			flag.two_buzzer_flag = 1;
			tri_road_status++;
		}
		break;
	case 1:
		/***********׼����������*************/
//            if(flag.camera_tri_road_flag)
		if(_MAD < tri_road_mad_value) {
			tri_road_status++;
			START_ANGLE_ACCUMULATE;
		}
		break;
	case 2:
		/***********���ڽ�������*************/
		if(tri_road_dir == 0) {
//               // ����
			LAD = 2 * LAD;
			RAD = 0;
		    lean_AD1 = 2 * lean_AD1;
		    lean_AD2 = 0;

		} else {
			// ����
			RAD = 2 * RAD;
			LAD = 0;
		    lean_AD2 = 2 * lean_AD2;
		    lean_AD1 = 0;
		}
		if(get_absf(hubu_angle) > in_tri_road_angle) {
			flag.buzzer_flag = 1;
			tri_road_status++;
			flag.tri_road_in = 1;
			flag.already_in_tri_road = 1;       // ��ʾ�Ѿ���������
			flag.single_line_Flag = 1;          // ���뵥��ģʽ
			STOP_ANGLE_ACCUMULATE;
		}
		break;
	case 3:
		/***********������*************/

		if(adc_original[3] > out_tri_road_value || adc_original[5] > out_tri_road_value || adc_original[8] > out_tri_road_value) {
//			flag.two_buzzer_flag = 1;
			flag.tri_road_in = 0;
			flag.single_line_Flag = 0;
			tri_road_status++;

            START_ANGLE_ACCUMULATE;   /*�����ǳ���*/
		}
		break;
	case 4:
	    /*����*/
        if(tri_road_dir == 0) {
//               // ����
            LAD = 2 * LAD;
            RAD = 0;
            lean_AD1 = 2 * lean_AD1;
            lean_AD2 = 0;

        } else {
            // ����
            RAD = 2 * RAD;
            LAD = 0;
            lean_AD2 = 2 * lean_AD2;
            lean_AD1 = 0;
        }
        if(get_absf(hubu_angle) > out_tri_road_angle)
        {
            STOP_ANGLE_ACCUMULATE;
            tri_road_status++;
        }
	    break;
	case 5:
	    encoder_count += encoder;
	    if(encoder_count > 1200 * 3)
	    {
	        tri_road_status = 0;
	    }
	    break;
	default:
		;
	}

}

