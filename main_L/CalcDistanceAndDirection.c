/**
* @file CalcDistanceAndDirection.c
* @brief ���s�����ƌ����̎Z�o
* @author Keishi MURAI
* @date 2017/06/20
*/

#include "CalcDistanceAndDirection.h"

/**
* @brief ���s�����ƌ����̎Z�o
* @param [in] int leftCnt ���ԗփ��[�^�[�̃G���R�[�_�[�l
* @param [in] int rightCnt �E�ԗփ��[�^�[�̃G���R�[�_�[�l
* @param [out] int* distance ���s����
* @param [out] int* direction ����
* @return ����
* @detail ���E���[�^�[�̃G���R�[�_�[�̕��ϒl���狗���ƌ������Z�o
*/
void CalcDistanceAndDirection (int leftCnt, int rightCnt, int* distance, int* direction) {
	// ���[�^�[�̌��o�p�x�i�ݐϒl�j
	int angle;
	int dir;
	angle = (leftCnt + rightCnt) / 2;

	*distance = angle * PI * DIAMETER / 360;		// ����
	
	// �����i�X�^�[�g���̌�����0�x�Ƃ��āA���v���̊p�x�j
	dir = (leftCnt % (360 * 4) - rightCnt % (360 * 4)) / 4;
	if (dir < 0) {
		dir = dir + 360;
	}
	*direction = dir;
	
	return;
}
