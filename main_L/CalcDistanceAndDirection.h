/**
* @file CalcDistanceAndDirection.h
* @brief ���s�����ƌ����̎Z�o
* @author Keishi MURAI
* @date 2017/06/20
*/
#ifndef _CALCDISTANCEANDDIRECTION_H_INCLUDED
#define _CALCDISTANCEANDDIRECTION_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define PI					3.14	//�~����
#define DIAMETER			80	//�ԗւ̒��a(80mm)

extern void CalcDistanceAndDirection(int leftCnt, int rightCnt, int* distance, int* direction);

#ifdef __cplusplus
}
#endif

#endif /* ! _CALCDISTANCEANDDIRECTION_H_INCLUDED */
