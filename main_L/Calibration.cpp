
//******
// Calibration.c
// ���́F�Ȃ�
// �o�́Fint min,int max
//******

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "Calibration.h"

using namespace ev3api;

void Calibration(int* min, int* max, ev3api::ColorSensor* color,ev3api::Motor* left, ev3api::Motor* right, ev3api::GyroSensor* gyroSen, ev3api::Motor* tail, ev3api::TouchSensor* touch, ev3api::Clock* clock){
	int8_t cur_brightness;	/* ���o�������Z���T�l */
	int8_t pwm_L, pwm_R; /* ���E���[�^PWM�o�� */
	bool ret = false;
	
	/* �L�����u���[�V�����ҋ@ */
	while(1)
	{
		if(!ret){
			ret = tail_control_cal(
				TAIL_ANGLE_STAND_UP,
				tail,
				eSlow); /* ���S��~�p�p�x�ɐ��� */
		}
		if (touch->isPressed())
		{
			break; /* �^�b�`�Z���T�������ꂽ */
		}
		if(ev3_button_is_pressed(BACK_BUTTON)){
			//�L�����u���[�V�������s��Ȃ�
			*max = 70;
			*min = 2;
			return;
		}
		
		clock->sleep(10);
		
	}
    
    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    left->reset();
    right->reset();
    
    /* �W���C���Z���T�[���Z�b�g */
    gyroSen->reset();
    balance_init(); /* �|���U�qAPI������ */
	
    ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */

	/* ���s�̂̏�Ԃ��N���� */
	while(1)
	{
		float pwm = (float)(TAIL_ANGLE_START - tail->getCount()); // ��ᐧ��
		if (pwm > 0)
		{
			tail->setPWM(20);
		}
		else if (pwm < 0)
		{
			break;
		}
		clock->sleep(4);
		
	}
	ret = false;

    /**
    * Main loop for the self-balance control algorithm
    */
	//clock_t start = clock();    // �X�^�[�g����
	int forward = 23; /* �O�i���� */
	int turn = 0;
	int count=0, count2=0;

    while(1)
    {
    	int32_t motor_ang_l, motor_ang_r;
		int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        if(!ret){
			ret = tail_control_cal(TAIL_ANGLE_DRIVE,tail, eFast); /* �o�����X���s�p�p�x�ɐ��� */
		}


        cur_brightness = color->getBrightness();
			
		if(cur_brightness>=*max){
			*max = cur_brightness;
		}
		if(cur_brightness<=*min){
			*min = cur_brightness;
		}
			//fprintf(bt, "max = %d, min = %d\n", *max, *min);
		//}

        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = left->getCount();
        motor_ang_r = right->getCount();
        gyro = gyroSen->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();


        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET_CALIBRATION,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R);

        left->setPWM(pwm_L);
        right->setPWM(pwm_R);
    	
        clock->sleep(4); /* 4msec�����N�� */
        if(count>=250){
			forward=forward*(-1);
			count2++;
			if(count2>=4){
					break;
			}
			count=0;
		}
        count++;
    }
    left->reset();
    right->reset();
}

//*****************************************************************************
// �֐��� : tail_control_cal
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
bool tail_control_cal(int32_t angle, Motor* tail, tailSpeed sp)
{
	float pwm_max;
	float pwm = (float)(angle - tail->getCount()) * P_GAIN; // ��ᐧ��
	if (pwm<0.1 && pwm >-0.1){
		tail->setBrake(true);
		tail->setPWM(0);
		return true;
	}else{
		tail->setBrake(false);
		if (sp == eFast){
			pwm_max = PWM_ABS_MAX_FAST;
		}else if (sp == eSlow){
			pwm_max = PWM_ABS_MAX_SLOW;
		}else{
			pwm_max = 45;
		}
		
		// PWM�o�͖O�a����
		if (pwm > pwm_max)
		{
			pwm = pwm_max;
		}
		else if (pwm < -pwm_max)
		{
			pwm = -pwm_max;
		}
		tail->setPWM(pwm);
		return false;
	}
}
