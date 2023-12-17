#ifndef SETUP_H_
#define SETUP_H_
//======================================//
// �C���N���[�h
//======================================//
#include "io.h"
#include "mtu.h"
#include "ADconverter.h"
#include "control.h"
#include "E2dataFlash.h"
#include "AQM0802A.h"
#include "MicroSD.h"
#include "sci.h"
#include "ICM20648.h"
#include "MemorryTrace.h"
#include <stdio.h>
//======================================//
// �V���{����`
//======================================//
#define UD	0
#define LR	1

#define START_COUNT	    1
#define START_GATE		2

#define B	0
#define F	1

//======================================//
// �O���[�o���ϐ��̐錾
//======================================//
// �p�^�[���֘A
extern char		start;

// �^�C�}�֘A
extern unsigned short 	cntSetup1;
extern unsigned short 	cntSetup2;
extern unsigned short 	cntSetup3;	
extern unsigned short	cntCalibration; //10ms���ƂɃJ�E���g
extern short		cntSwitchUD;	// �X�C�b�`����������p�E
extern short		cntSwitchLR;	// �X�C�b�`����������p��

// �p�����[�^�֘A
extern char fixSpeed;
extern char sensAutoCalibration_flg;
extern char sensAutoCalibration_flg2;
extern char sensAutoCalibration_flg3;
extern char sensAutoCalibration_flg4;
extern char sensAutoCalibration_flg5;
extern int Calibration_j;
//======================================//
// �v���g�^�C�v�錾
//======================================//
void setup(void);
char fixSpeedSetting ( void );

#endif /* SCI_H_ */