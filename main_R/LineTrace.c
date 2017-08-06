/**
 * @file LineTrace.c
 * @brief ���C���g���[�X�iPID����j
 * @author Keishi MURAI
 * @date 2017/06/19
 */
#include "LineTrace.h"

/**
 * @brief ���C���g���[�X����
 * @param [in] int status ���C���̏��(����/�J�[�u)
 * @param [in] int targetVal �ڕW�̌��Z���T�l
 * @param [in] int currentVal ���݂̌��Z���T�l
 * @param [in] float opePeriod ��������[s]
 * @param [in,out] int* lastErr �O��΍�
 * @param [in, out] int forward
 * @return ����p�x[deg.] 100(�E����ő�l)�`-100(������ő�l)
 * @detail ���Z���T�l�����PID������s��
 */
float LineTrace(int status, int targetVal, int currentVal, float opePeriod, int* lastErr, int8_t* forward) {
	int integral=0;	//�΍��ϕ�
	float turn;	//����p�x
	
	LineTrace_param(status,forward,&kp,&ki,&kd);

	// P����
	errParam = currentVal - targetVal;	// �����̍������g���[�X

	// D����
	diffParam = (errParam - *lastErr) / opePeriod;
	*lastErr = errParam;

	// PD����Ɋ�Â�������l���Z�o
	// I����̌W���A�y�сA�ϐ��͕s�v�����A�R�[�X�}�b�v�ɍ��킹�Ċ܂񂾌`�ɂ���B
	turn = kp * (float)errParam + ki * (float)integral + kd * diffParam;

	// ����p�x���͈͓��Ɏ��܂��Ă��邩�m�F
	if (turn > MAX_TURN_RIGHT) {
		turn = MAX_TURN_RIGHT;
	}
	else if (turn < MAX_TURN_LEFT) {
		turn = MAX_TURN_LEFT;
	}
	return turn;
}

// PID����W���擾�i�f�o�b�O�p�j
void GetPID(float* kkp,float* kki, float* kkd){
	*kkp = kp;
	*kki = ki;
	*kkd = kd;
}

// PID����ϐ��擾�i�f�o�b�O�p�j
void GetVar(int* err, float* diff){
	*err = errParam;
	*diff = diffParam;
}
