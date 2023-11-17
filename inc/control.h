#ifndef LINECHASE_H_
#define LINECHASE_H_
//====================================//
// �C���N���[�h
//====================================//
#include "io.h"
#include "mtu.h"
#include "ADconverter.h"
#include "control.h"
#include "ICM20648.h"
#include "AQM0802A.h"
#include <math.h>
#include <stdint.h>
//====================================//
// �V���{����`
//====================================//

/* �}�X�N�l�ݒ� �~�F�}�X�N����(����)�@���F�}�X�N����(�L��) */
#define         MASK11111		0x1f  	/* �����������i�S���j */
#define         MASK11011		0x1b  	/* �����~�����i�S���j */
#define         MASK01010		0x0a  	/* �~���~���~�i�S���j */
#define         MASK10101		0x15  	/* ���~���~���i�S���j */
#define         MASK01110		0x07  	/* �~�������~�i�S���j */
#define         MASK10000		0x10  	/* ���~�~�~�~�i���O�j */
#define         MASK10001		0x11  	/* ���~�~�~���i���E�j */
#define         MASK11000       	0x18    /* �����~�~�~�i���S�j */
#define         MASK10011       	0x13    /* ���~�~�����i���S�j */
#define         MASK11001      		0x19    /* �����~�~���i���S�j */
#define         MASK01000		0x08    /* �~���~�~�~�i�����j */
#define         MASK01100		0x0c    /* �~�����~�~�i�����j */
#define         MASK10100       	0x14    /* ���~���~�~�i�����j */
#define         MASK11100      		0x1c    /* �������~�~�i�����j */
#define         MASK11110       	0x1e    /* ���������~�i�����j */
#define         MASK00100       	0x04    /* �~�~���~�~�i�����j */
#define         MASK01111       	0x0f    /* �~���������i�E���j */
#define         MASK00111       	0x07    /* �~�~�������i�E���j */
#define         MASK00101       	0x05    /* �~�~���~���i�E���j */
#define		MASK00110		0x06	/* �~�~�����~�i�E���j	*/
#define		MASK00010		0x02	/* �~�~�~���~�i�E���j	*/
#define		MASK00011		0x03	/* �~�~�~�����i�E�S�j	*/
#define		MASK00001		0x01	/* �~�~�~�~���i�E�O�j	*/

/*************************************** ���������֐� *************************************/
// �^�C�}���荞��
#define SET_CMT_C0		R_PG_Timer_Set_CMT_U0_C0();			// �R���y�A�}�b�`�^�C�}������(ch0)
#define START_CMT_C0	R_PG_Timer_StartCount_CMT_U0_C0();	// �J�E���g�X�^�[�g(ch0)
#define STOP_CMT_C0	    R_PG_Timer_HaltCount_CMT_U0_C0();	// �J�E���g�ꎞ��~(ch0)
/******************************************************************************************/
// �@�̏���
#define WHELLBASE   149
#define TREAD       143
#define SENSORBAR   307
#define MAXDEG      42

#define M_PI        3.141592
// �ً}��~
#define	STOPPING_METER		   3		// ��~����

// �e�Z�N�V�����ł̖ڕW���x�@x/10[m/s]
#define SPEED_STRAIGHT			42	// �ʏ�g���[�X
#define SPEED_CURVE_BRAKE		40	// �J�[�u�u���[�L
#define SPEED_CURVE_R600		45	// R600�J�[�u���x
#define SPEED_CURVE_R450		38	// R450�J�[�u���x
#define SPEED_CURVE_STRAIGHT	42	// S���J�[�u�������x

#define SPEED_CROSSLINE			25	// �N���X���C���i�����x
#define SPEED_CLANK_TRACE		20	// �N�����N�i�����x
#define SPEED_RIGHTCLANK_CURVE		20	// �E�N�����N���񑬓x
#define SPEED_RIGHTCLANK_ESCAPE		30	// �E�N�����N���A���x
#define SPEED_LEFTCLANK_CURVE		20	// ���N�����N���񑬓x
#define SPEED_LEFTCLANK_ESCAPE		30	// ���N�����N���A���x

#define SPEED_HALFLINE				40	// �n�[�t���C���i�����x
#define SPEED_RIGHTCHANGE_TRACE	    40	// �E���[���`�F���W�i�����x
#define SPEED_RIGHTCHANGE_CURVE	    40	// �E���[���`�F���W���񑬓x
#define SPEED_RIGHTCHANGE_ESCAPE	40	// �E���[���`�F���W���A���x
#define SPEED_LEFTCHANGE_TRACE		40	// �����[���`�F���W�i�����x
#define SPEED_LEFTCHANGE_CURVE		40	// �����[���`�F���W���񑬓x
#define SPEED_LEFTCHANGE_ESCAPE		40	// �����[���`�F���W���A���x

#define SPEED_SLOPE_BRAKE		25	    // �����I�_���x
#define SPEED_SLOPE_TRACE		30	    // ��ǂݔ�΂����x
// �p�x
#define ANGLE_RIGHTCLANK		550	// �E�N�����N����p�x480
#define ANGLE_LEFTCLANK		        -510	// ���N�����N����p�x480
#define ANGLE_RIGHTCHANGE		230	// �E���[���`�F���W����p�x
#define ANGLE_LEFTCHANGE	        -230	    // �����[���`�F���W����p�x



// �J�[�u�֘A
#define CURVE_R600_START		160		// R600�J�nAD�l
#define CURVE_R600_START_i		20		// R600�J�niAngle
#define CURVE_R450_START		250		// R450�J�nAD�l

// �W���C���֘A
#define SLOPE_UPPERLINE_IMU		-5		// ���⌟�o�p�x
#define SLOPE_LOWERLINE_IMU	    	5		// ����⌟�o�p�x
#define INTEGRAL_LIMIT			200		// �p���x�ώZ����

// PID�Q�C���֘A
#define VOLTAGELIM 10.5 // �o�͍ő�d��
#define VOLTAGELIMTRACE 7.0 // �o�͍ő�d��
//�����g���[�X
#define KP		25
#define KI		10
#define KD		10

// �p�x����
#define KP2		70
#define KI2		0
#define KD2		80

// ���x����
#define KP3		60
#define KI3		10
#define KD3		0

//basicRunPower
#define R450FIN		0.6
#define R450FOUT	0.8	
#define R450RIN		0
#define R450ROUT	0.8

#define R600FIN		1.0
#define R600FOUT	1.0	
#define R600RIN		0.9	
#define R600ROUT	1.0

// �ً}��~�֘A
#define STOP_SENSOR1	60		// �Z���T�S��
#define STOP_SENSOR2	800		// �Z���T�S����
#define STOP_ENCODER	100		// �G���R�i�X�̉����x���m(�R�[�X���痎�����H)
#define STOP_COUNT		1000	// ���Ԓ�~
//===================================�[�_��~(�Ђ�����Ԃ����H)
#define STOP_GYRO		100		// �}�C=//
// �O���[�o���ϐ��̐錾
//====================================//
// �p�^�[���A���[�h�֘A
extern char pattern;		// �p�^�[���ԍ�
extern char modeLCD;		// LCD�\���I��
extern char modeSlope;		// ��`�F�b�N		0:����n��	1:����I���	2:�����n��	3:�����I���
extern char	modeAngle;		// �T�[�{PWM�ύX	0:�����g���[�X	1:�p�x����
extern char	modePushcart;	// �艟�����[�h��	0:�������s	1:�艟��
extern char	msdset;			// MicroSD�����������ꂽ��	0:���������s	1:����������
extern char	IMUSet;			// IMU�����������ꂽ��	0: ���������s	1:����������

// �p�����[�^�֘A
// ����
extern short	stopping_meter;			    // ��~����
// ���x
extern short	speed_straight;			    // �ʏ�g���[�X
extern short	speed_curve_brake;		    // �J�[�u�u���[�L
extern short	speed_curve_r600;		    // R600�J�[�u���x
extern short	speed_curve_r450;		    // R450�J�[�u���x
extern short	speed_curve_straight;	    // S���J�[�u�������x

extern short	speed_crossline;			// �N���X���C���i�����x
extern short	speed_ckank_trace;		    // �N�����N�i�����x
extern short	speed_rightclank_curve;	    // �E�N�����N���񑬓x
extern short	speed_rightclank_escape;	// �E�N�����N���A���x
extern short	speed_leftclank_curve;	    // ���N�����N���񑬓x
extern short	speed_leftclank_escape;	    // ���N�����N���A���x

extern short	speed_halfine;			    // �n�[�t���C���i�����x
extern short	speed_rightchange_trace;	// �E���[���`�F���W�i�����x
extern short	speed_rightchange_curve;	// �E���[���`�F���W���񑬓x
extern short	speed_rightchange_escape;   // �E���[���`�F���W���A���x

extern short	speed_leftchange_trace;	    // �����[���`�F���W�i�����x
extern short	speed_leftchange_curve;	    // �����[���`�F���W���񑬓x
extern short	speed_leftchange_escape;	// �����[���`�F���W���񑬓x

extern short	speed_slope_brake;		    // �����I�_���x
extern short	speed_slope_trace;		    // ��ǂݔ�΂����x

// �T�[�{�p�x
extern short	angle_rightclank;		    // �E�N�����N����p�x
extern short	angle_leftclank;			// ���N�����N����p�x
extern short	angle_rightchange;		    // �E���[���`�F���W����p�x
extern short	angle_leftchange;		    // �E���[���`�F���W����p�x

// �^�C�}�֘A
extern short	cntGyro;			// �p�x�v�Z�p�J�E���^

// �p�x�֘A
extern double 	TurningAngleEnc;	// �G���R�[�_���狁�߂�����p�x
extern double	PichAngleAD;		// �A�i���O�W���C�����狁�߂��s�b�`�p�x

// ���[�^�[�֘A
extern signed char 	motorPwm;	    // ���[�^�[����PWM
extern short		targetSpeed;	// �ڕW���x
extern unsigned int encStable;
extern short        cntStable;

// �Q�C���֘A
extern char	    kp_buff, ki_buff, kd_buff;
extern char	    kp2_buff, ki2_buff, kd2_buff;
extern char 	kp3_buff, ki3_buff, kd3_buff;
extern char 	kp4_buff, ki4_buff, kd4_buff;
extern char 	kp3, ki3, kd3;

//basicRunPower_buff
extern float   r450fin_buff,r450fout_buff;
extern float   r450rin_buff,r450rout_buff;
extern float   r600fin_buff,r600fout_buff;
extern float   r600rin_buff,r600rout_buff;

// �f���֘A
extern char demo;

// �T�[�{�֘A
extern double		Int;		// I�����ώZ�l(�����g���[�X)
extern short 		SetAngle;	// �ڕW�p�x
extern signed char 	ServoPwm;	// �����g���[�X�T�[�{PWM
extern signed char 	ServoPwm2;	// �p�x�T�[�{PWM

// ���C�����m
extern volatile unsigned char cnt_crossline;
extern volatile unsigned char cnt_rightline;
extern volatile unsigned char cnt_leftline;
extern volatile int line;
//====================================//
// �v���g�^�C�v�錾
//====================================//
// �}�[�J�[�֘A
int checkLine( void );
//int checkCrossLine( void );
//int checkRightLine( void );
//int checkLeftLine( void );
bool check_crossline( void );
bool check_rightline( void );
bool check_leftline( void );
signed char checkSlope( void );

// �G���R�[�_�֘A
unsigned int encMM( short mm );
unsigned int stableSpeedDistance( void );

// ���[�^�[�֘A
void motorControl( void );
int acceleControl( int targetAccele);

// �@�̐���֘A
void diff ( signed char pwm );
double getLinePositionNow( short angleAD, double angleIMU );
double getLinePositionAfter ( short angle, double angleIMU );
short getReturnAngle( double angleIMU, double y1);

// �T�[�{�֘A
void servoControlTrace( void );
void servoControlAngle( void );

#endif // LINECHASE_H_