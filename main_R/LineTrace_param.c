/**
 * @file LineTrace_param.c
 * @brief ���C���g���[�X�iPID����j�̃p�����[�^�ǂݍ���
 * @author kanako baba ww
 * @date 2017/07/23
 */
 
 #include "LineTrace_param.h"
 
/**
 * LineTrace_param�֐�
 * input�F���݂̏�ԁi�����E�J�[�u�E��蓙�j
 * output�Fforward,kp,ki,kd
 * �@���ꂼ��̃p�����[�^�́uparam.txt�v�Őݒ肷��
 */
 
void LineTrace_param(int status, int8_t* forward, float* kp, float* ki, float* kd){
	
	/* for���[�v�p�ϐ� */
	int i=0;
	int j=0;
	
	/* �z��̗v�f���v�Z */
	//int buf_size_c = sizeof(buf[0])/sizeof(buf[0][0]);/*4*/
	//int buf_size_l = sizeof(buf)/sizeof(buf[0]);/*9*/
	
	switch(status){
	
	case 1:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 1){//1�s��
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 2:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 2){//2�s��
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 3:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 3){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 4:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 4){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 5:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 5){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 6:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 6){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 7:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 7){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 8:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 8){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	case 9:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 9){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	default:
		for(j=0; j<getBufColumnSize(); j++){
			if(buf[0][j] == 0){
				for(i=0; i<getBufLineSize(); i++){
					param[i] = buf[i][j];
				}
			}
		}
		break;
	}

	/* �l���� */
	*forward = param[1];
	*kp = (float)param[2]/100;
	*ki = (float)param[3]/100;
	*kd = (float)param[4]/100;
}


