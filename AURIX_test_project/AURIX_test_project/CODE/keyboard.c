#include "keyboard.h"



uint8 press_key;
uint8 last_show;


extern _pid  servo_pid;
extern _pid  motor_pid;


// 当前的按键状态
uint8 KeySta[4][4] = {
	{1, 1, 1, 1},
	{1, 1, 1, 1},
	{1, 1, 1, 1},
	{1, 1, 1, 1}
};

// 上一时刻的按键状态
uint8 backup[4][4] = {
	{1, 1, 1, 1},
	{1, 1, 1, 1},
	{1, 1, 1, 1},
	{1, 1, 1, 1}
};

void keyboard_init(void)
{
	gpio_init(KEY_IN_1, GPI, 0, PULLDOWN);   // 输入下拉
	gpio_init(KEY_IN_2, GPI, 0, PULLDOWN);
	gpio_init(KEY_IN_3, GPI, 0, PULLDOWN);
	gpio_init(KEY_IN_4, GPI, 0, PULLDOWN);


	gpio_init(KEY_OUT_1, GPO, 0, PUSHPULL);
	gpio_init(KEY_OUT_2, GPO, 0, PUSHPULL);
	gpio_init(KEY_OUT_3, GPO, 0, PUSHPULL);
	gpio_init(KEY_OUT_4, GPO, 0, PUSHPULL);  // 推挽输出
}
void KeyScan(void)
{
	unsigned char i;
	static unsigned char keyout = 0;
	static unsigned char keybuf[4][4] = {
		{0xFF, 0xFF, 0xFF, 0xFF},
		{0xFF, 0xFF, 0xFF, 0xFF},
		{0xFF, 0xFF, 0xFF, 0xFF},
		{0xFF, 0xFF, 0xFF, 0xFF}
	};

	keybuf[keyout][0] = (keybuf[keyout][0] << 1) | gpio_get(KEY_IN_1);
	keybuf[keyout][1] = (keybuf[keyout][1] << 1) | gpio_get(KEY_IN_2);
	keybuf[keyout][2] = (keybuf[keyout][2] << 1) | gpio_get(KEY_IN_3);
	keybuf[keyout][3] = (keybuf[keyout][3] << 1) | gpio_get(KEY_IN_4);

	for (i = 0; i < 4; i++) {

		if ((keybuf[keyout][i] & 0x0F) == 0x00) {
			KeySta[keyout][i] = 0;
		} else if ((keybuf[keyout][i] & 0x0F) == 0x0F) {
			KeySta[keyout][i] = 1;   // 按键按下, 此时按键状态为1时
		}
	}

	keyout++;
	keyout = keyout & 0x03;  // 当 keyout为 4 的时候回到 0
	switch (keyout) {
	case 0:
		gpio_set(KEY_OUT_1, 1);
		gpio_set(KEY_OUT_4, 0);
		break;
	case 1:
		gpio_set(KEY_OUT_2, 1);
		gpio_set(KEY_OUT_1, 0);
		break;
	case 2:
		gpio_set(KEY_OUT_3, 1);
		gpio_set(KEY_OUT_2, 0);
		break;
	case 3:
		gpio_set(KEY_OUT_4, 1);
		gpio_set(KEY_OUT_3, 0);
		break;
	default:
		break;
	}
}

/*
 * 检测按键是否按下
 */
void Check_BottonPress(void)
{

//    static uint8 key_up = 1; //按键松开标志
	uint32 write_buf;
	uint8 i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (backup[i][j] != KeySta[i][j]) {
				if (backup[i][j] != 0) {
					press_key = i * 4 + j + 1;
				}
				backup[i][j] = KeySta[i][j];
			}
		}
	}


	switch (press_key) {
	case 1:
		last_show = 1;
//        lcd_showint32 (0 , 9,  last_show, 1);
#if RETURN_TO_ONE_DEBUG
		flag.buzzer_flag = 1;
		if(flag.return_to_one_flag) {
			// 更新最大值，最小值结束
			flag.flash_update_flag = 1;
		} else {
			// 开始更新
		    if(gpio_get(sw1))
		    {
		        for(int i = 1; i <= 8; i++)
		        {
	                adc_min[i] = 100;
	                adc_max[i] = 0;
		        }
		    }
		    else
		    {
		        for(int i = 1; i <= 8; i++)
		        {
	                adcSingleLineMin[i] = 100;
	                adcSingleLineMax[i] = 0;
		        }
		    }

		}
		flag.return_to_one_flag = 1 - flag.return_to_one_flag;
#endif
		    // 写入阈值
		break;
	case 2:
		last_show = 2;
		flag.buzzer_flag = 1;
		flag.tft_clear_flag = 1;
		flag.tft_flag++;
		if(flag.tft_flag == 6)
			flag.tft_flag = 1;
		break;
	case 3:
		last_show = 3;
		flag.buzzer_flag = 1;
        flag.tft_clear_flag = 1;
        flag.tft_flag--;
        if(!flag.tft_flag)
            flag.tft_flag = 5;

		break;
	case 4:
		last_show = 4;
//        pid.curve_kp -= 0.1;
//        pid.triRoadKp -= 0.1;

		flag.speed_debug++;
		flag.speed_debug %= 7;

		break;
	case 5:
		last_show = 5;
		switch(flag.PID_parameter_debug) {
		case 0:
			A += 0.1;
			break;
		case 1:
			pid.curve_kp += 0.1;
			break;
		case 2:
			pid.smallRingKp += 0.1;
			break;
		case 3:
			pid.bigRingKp += 0.1;
			break;
		case 4:
			pid.triRoadA += 0.1;
			break;
		case 5:
			motor_pid.Kp += 3;
			break;
		case 6:
			pid.basic_kp += 0.1;
			break;
		case 7:
		    pid.LRTC_kp += 0.1;
		    break;
		case 8:     // 调速度
			switch(flag.speed_debug) {
			case 0:
				speed.closed_speed[0]++;
				break;
			case 1:
				speed.closed_speed[1]++;
				break;
			case 2:
				speed.closed_speed[2]++;
				break;
			case 3:
				speed.closed_speed[3]++;
				break;
			case 4:
				speed.closed_speed[4]++;
				break;
			case 5:
			    speed.curve_low_speed++;
				break;
			case 6:
			    speed.curve_high_speed++;
			    break;
			}
			break;
		case 9:
		    garage.angle += 1;
		    break;


		default:
			flag.buzzer_flag = 1;

		}
//        pid.triRoadA += 0.1;
//        motor_pid.Kp += 1;
//		A += 0.1;
		break;
	case 6:
		last_show = 6;
//        pid.triRoadA -= 0.1;
//        pid.A -= 0.1;
//        A -= 0.1;
		switch(flag.PID_parameter_debug) {
		case 0:
			A -= 0.1;
			break;
		case 1:
			pid.curve_kp -= 0.1;
			break;
		case 2:
			pid.smallRingKp -= 0.1;
			break;
		case 3:
			pid.bigRingKp -= 0.1;
			break;
		case 4:
			pid.triRoadA -= 0.1;
			break;
		case 5:
			motor_pid.Kp -= 2;
			break;
		case 6:
			pid.basic_kp -= 0.1;
			break;
		case 7:
		    pid.LRTC_kp -= 0.1;
		    break;
		case 8:
			switch(flag.speed_debug) {
			case 0:
				speed.closed_speed[0]--;
				break;
			case 1:
				speed.closed_speed[1]--;
				break;
			case 2:
				speed.closed_speed[2]--;
				break;
			case 3:
				speed.closed_speed[3]--;
				break;
			case 4:
				speed.closed_speed[4]--;
				break;
			case 5:
			    speed.curve_low_speed--;
			    break;
			case 6:
			    speed.curve_high_speed--;
			    break;
			}
			break;
        case 9:
            garage.angle -= 1;
            break;
		default:
			flag.buzzer_flag = 1;

		}
		break;
	case 7:
		last_show = 7;
//        motor_pid.Ki += 0.1;
//        pid.triToadB += 0.1;
//        B += 0.1;
		switch(flag.PID_parameter_debug) {
		case 0:
			B += 0.1;
			break;
		case 1:
			pid.curve_kd += 3;
			break;
		case 2:
			pid.smallRingKd += 3;
			break;
		case 3:
			pid.bigRingKd += 3;
			break;
		case 4:
			pid.triToadB += 0.1;
			break;
		case 5:
			motor_pid.Ki += 0.3;
			break;
		case 6:
			pid.basic_kd += 3;
			break;
		case 7:
		    pid.LRTC_kd += 3;
		    break;
		case 8:
		    break;
        case 9:
//            garage.dir = 1 - garage.dir;
//            round_status[0] = 1 - round_status[0];   // 右大环
//            round_status[1] = 1 - round_status[1];   // 右大环
            break;
		default:
			;

		}
		break;
	case 8:
		last_show = 8;
//        motor_pid.Ki -= 0.1;
//        pid.triToadB -= 0.1;
//        B -= 0.1;
		switch(flag.PID_parameter_debug) {
		case 0:
			B -= 0.1;
			break;
		case 1:
			pid.curve_kd -= 2;
			break;
		case 2:
			pid.smallRingKd -= 2;
			break;
		case 3:
			pid.bigRingKd -= 2;
			break;
		case 4:
			pid.triToadB -= 0.1;
			break;
		case 5:
			motor_pid.Ki -= 0.2;
			break;
		case 6:
			pid.basic_kd -= 2;
			break;
		case 7:
		    pid.LRTC_kd -= 2;
		    break;
        case 8:
            break;
        case 9:
            break;
		default:
			flag.buzzer_flag = 1;

		}
		break;
	case 9:
		last_show = 9;
//        pid.triRoadKd += 3;
//        Cp += 0.001;
//        basic_C += 10;
//        pid.straightKd += 3;
		switch(flag.PID_parameter_debug) {
		case 0:
			pid.straightKd += 3;
			break;
		case 1:
//                pid.curve_kd += 0.1;
			break;
		case 2:
            smallEncoderValue += 50;
			break;
		case 3:
		    bigEncoderValue += 50;
			break;
		case 4:
			pid.triRoadKd += 3;
			break;
		default:
			flag.buzzer_flag = 1;

		}
		break;
	case 10:
		last_show = 10;
//        pid.triRoadKd -= 2;
//        Cp -= 0.001;
//        basic_C -= 10;
//        pid.straightKd -= 2;
		switch(flag.PID_parameter_debug) {
		case 0:
			pid.straightKd -= 2;
			break;
		case 1:
//                pid.curve_kd += 0.1;
			break;
		case 2:
//                pid.smallRingKd += 0.1;
            smallEncoderValue -= 50;
			break;
		case 3:
//                pid.bigRingKd += 0.1;
            bigEncoderValue -= 50;
			break;
		case 4:
//                pid.triToadB += 0.1;
			pid.triRoadKd -= 2;
			break;
		default:
			flag.buzzer_flag = 1;

		}
		break;
	case 11:
		last_show = 11;
        flag.PID_parameter_debug++;
        flag.tft_clear_flag = 1;
        flag.PID_parameter_debug %= 10;
        flag.buzzer_flag = 1;
		break;
	case 12:
		last_show = 12;
        flag.PID_parameter_debug--;
        flag.tft_clear_flag = 1;
        if(flag.PID_parameter_debug < 0)
            flag.PID_parameter_debug = 9;
        flag.buzzer_flag = 1;
		break;
	}

	press_key = 0;
}
