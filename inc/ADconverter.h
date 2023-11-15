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
#define AD2DEG              0.082F     // 1AD������̊p�x �T�[�{�ő�؂�p[��]/�T�[�{�ő�؂�p����AD�l[]
#define LOWVOLTAGE          10.5        // �Œᓮ��d��
#define DsensRR		100
#define DsensR		100
#define DsensC		100
#define DsensL		100
#define DsensLL		100
#define sensorRR_reg	0
#define sensorRR_max	1
#define sensorRR_min	2
#define sensorC_reg	3
#define sensorC_max	4
#define sensorC_min	5
#define sensorLL_reg	6
#define sensorLL_max	7
#define sensorLL_min	8


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
extern short				sensorR_reg;
extern short				sensorR_max;	// ���A�i���O�Z���TMAXAD�l
extern short				sensorR_min;	// ���A�i���O�Z���TMINAD�l
extern short		sensorL;		// ���A�i���O�Z���T
extern short				sensorL_reg;
extern short				sensorL_max;	// ���A�i���O�Z���TMAXAD�l
extern short				sensorL_min;	// ���A�i���O�Z���TMINAD�l
extern short		sensor_calc[9];
extern short		sensorG;		// �Q�[�g�Z���T
extern short		sensorG_th;	// �Q�[�g�Z���T
extern short		sensorC;		// ���S�A�i���O�Z���T
extern short		sensorLL;		// �ō��[�A�i���O�Z���T
extern short		sensorRR;		// �ŉE�[�A�i���O�Z���T
extern short		sensor_saka;

extern float				RR_tatio;
extern float				R_tatio;
extern float				C_tatio;
extern float				L_tatio;
extern float				LL_tatio;

extern volatile unsigned char RR_data;
extern volatile unsigned char R_data;
extern volatile unsigned char C_data;
extern volatile unsigned char L_data;
extern volatile unsigned char LL_data;

extern short		L_sencnt;

extern double		Voltage;        //�d���`�F�b�J�[

extern short		Angle0;		    // �T�[�{�Z���^�[�l
extern short		Angle_buff;
extern short		iAngle;
//====================================//
// �v���g�^�C�v�錾
//====================================//
// �Z���T�֘A
short getServoAngle(void);
short getAnalogSensor( void );

unsigned char RRget( void );
unsigned char Rget( void );
unsigned char Cget( void );
unsigned char Lget( void );
unsigned char LLget( void );
unsigned char sensor_inp( void );
unsigned char startbar_get( void );

#endif // ADCONVERTER_H_