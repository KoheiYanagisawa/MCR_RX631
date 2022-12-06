#ifndef ADCONVERTER_H_
#define ADCONVERTER_H_
//====================================//
// �C���N���[�h
//====================================//
#include "R_PG_IGC-P8080_v1.h"
//====================================//
// �V���{����`
//====================================//
#define GATE_VAL			300		    // �Q�[�g�Z���T�������l
#define DEG2AD              41          // 1�x�������AD�l �T�[�{�ő�؂�p����AD�l[]/�T�[�{�ő�؂�p[��]
#define AD2DEG              0.0247F     // 1AD������̊p�x �T�[�{�ő�؂�p[��]/�T�[�{�ő�؂�p����AD�l[]
#define LOWVOLTAGE          10.5        // �Œᓮ��d��
/*************************************** ���������֐� *************************************/
// AD�R���o�[�^
#define SET_ADC		R_PG_ADC_12_Set_S12AD0(); 				// 12�r�b�gA/D�R���o�[�^(S12AD0)��ݒ�
#define START_ADC	R_PG_ADC_12_StartConversionSW_S12AD0();	// A/D�ϊ��J�n
#define GET_ADC		R_PG_ADC_12_GetResult_S12AD0( result );	// AD�l���擾
/******************************************************************************************/

//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
// �Z���T�֘A
extern short		sensorR;		// �E�A�i���O�Z���T
extern short		sensorL;		// ���A�i���O�Z���T
extern short		sensorG;		// �Q�[�g�Z���T
extern short		sensorG_th;	// �Q�[�g�Z���T
extern short		sensorC;		// ���S�A�i���O�Z���T
extern short		sensorLL;		// �ō��[�A�i���O�Z���T
extern short		sensorRR;		// �ŉE�[�A�i���O�Z���T

extern short		L_sencnt;

extern double		Voltage;        //�d���`�F�b�J�[

extern short		Angle0;		    // �T�[�{�Z���^�[�l
//====================================//
// �v���g�^�C�v�錾
//====================================//
// �Z���T�֘A
short getServoAngle(void);
short getAnalogSensor( void );
unsigned char sensor_inp( void );
unsigned char startbar_get( void );

#endif // ADCONVERTER_H_