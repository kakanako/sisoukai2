//******
// Calibration.h
// 
// 
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

using namespace ev3api;

#define TAIL_ANGLE_STAND_UP  80  /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_START	90  /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3  /* �o�����X���s���̊p�x[�x] */
#define TAIL_ANGLE_INIT       0  /* 0�x */
#define P_GAIN             2.5F  /* ���S��~�p���[�^������W�� */
//#define PWM_ABS_MAX          60  /* ���S��~�p���[�^����PWM��΍ő�l */
#define PWM_ABS_MAX_FAST	60  /* ���S��~�p���[�^����PWM��΍ő�l */
#define PWM_ABS_MAX_SLOW	30  /* ���S��~�p���[�^����PWM��΍ő�l */
#define GYRO_OFFSET_CALIBRATION   0  /*�L�����u���[�V�������̃W���C���I�t�Z�b�g�l*/
enum tailSpeed {eFast, eSlow};

void Calibration(int* min, int* max, ev3api::ColorSensor* colorSensor, ev3api::Motor* left, ev3api::Motor* right, ev3api::GyroSensor* gyroSen, ev3api::Motor* tail, ev3api::TouchSensor* touch, ev3api::Clock* clock);
bool tail_control_cal(int32_t angle, ev3api::Motor* tail, tailSpeed sp);


