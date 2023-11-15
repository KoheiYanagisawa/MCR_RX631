//====================================//
// �C���N���[�h
//====================================//
#include "io.h"
#include "mtu.h"
#include "ADconverter.h"

#include "control.h"
#include "setup.h"
#include "sci.h"
#include "E2dataFlash.h"
#include "AQM0802A.h"
#include "microSD.h"
#include "ICM20648.h"
#include "MemorryTrace.h"
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
//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
// ���s�p�^�[���֘A
char		pattern = 0;	// �p�^�[���ԍ�

// ���[�h�֘A
char		modeCurve;		// �J�[�u����	0:�J�[�u�ȊO	1:�J�[�u���s��
char		modeError;		// 0: �ʏ푖�s 1:������~ 2:�Z���T�S�� 3:�Z���T�S���� 4:�G���R�[�_��~ 5:�W���C������
char 		modeAutoMotor;	// 0: switch���ŃT�[�{�A�쓮���[�^��PWM�o�͂��w�肷�� 1: Timer�֐����Ŏ����I��PWM�o�͂����s
char      	modeMotor = 1; //1�ʏ푖�s�A0���[�^�[�t���[
volatile unsigned char check_line;
// �^�C�}�֘A
// 1ms�^�C�}
unsigned int 		cnt1;		// ���s�p�^�C�}�J�E���g
unsigned short	 	cntOut1;	// �R�[�X�A�E�g����p�^�C�}
unsigned short	 	cntOut2;	// �R�[�X�A�E�g����p�^�C�}2
unsigned short	 	cntOut3;	// �R�[�X�A�E�g����p�^�C�}3
unsigned short	 	cntOut4;	// �R�[�X�A�E�g����p�^�C�}4
static char			Timer10;	// 1ms�J�E���g�p

int correct_angle;
int Angle_fixed;			//�}�[�J�[�ǂݔ�΂��p�Œ�p�x
//====================================//
// �v���g�^�C�v�錾
//====================================//
void initParameter ( bool lcd );
//====================================//
// ���C���v���O����
//====================================//
void main(void){
	char		countdown = 0x0;
	short i, j, angleAfter, angleCenter;
	double y1;
	unsigned int ui;
	//=================================//
	// ������
	//=================================//
	ledOut(0); 		// LED����
	L_Sen_ON;		// �Z���T�_��
	intiLcd();		// LCD������
	
	motorPwmOut(0, 0, 0, 0);	// ���[�^�[��~
	servoPwmOut( 0 );
	SetAngle = 0;
	
	modePushcart = 0;		// �艟�����[�hoff 
	modeSlope = 0;			// ����`�F�b�N
	modeAngle = 0;			// �����g���[�X
	modeAutoMotor = 0;		// ����PWM�o�͒�~
	start = 0;				// �Q�[�g�X�^�[�g
	
	//IMU������
	initIMU();
	// �t���b�V��������
	// �f�[�^�t���b�V������O��p�����[�^��ǂݍ���
	if( !initFlash() ) readFlashSetup( 1, 1, 1 ,1 ,1 ,1 ,1);
	// MicroSD�J�[�h������
	if( !initMicroSD() ) msdset = 1;
	else msdset = 0;

	// �d���d���̊m�F
	if (Voltage < LOWVOLTAGE ) {
		cnt1=0;
		while( cnt1 < 1500){
			lcdRowPrintf(UPROW, "LOW BAT ");
			lcdRowPrintf(LOWROW, "  %05.2fV",Voltage);
			ledOut(LED_R);
			// while(1);
		}
	} else {
		cnt1=0;
		while( cnt1 < 1500){
			lcdRowPrintf(UPROW, "Voltage ");
			lcdRowPrintf(LOWROW, "  %05.2fV",Voltage);
		}
	}

	cnt1=0;
	while(msdset == 1 && cnt1 < 1500){
		lcdRowPrintf(UPROW, " SYSTEM ");
		lcdRowPrintf(LOWROW, "ALLGREEN");
		ledOut(LED_G);
	}
	
	while(1){
		__setpsw_i();
		if( pattern >= 11 && pattern <= 99 ) {
			if( !modePushcart ) {		
				// �艟�����[�hOFF
				if( cnt1 >= 100 ) {		// �����o���Ă���
					if ( EncoderTotal >= ( PALSE_METER * stopping_meter ) ) { // �������߂̏ꍇ
						modeError = 1;
					} else if ( cntOut1 >= STOP_SENSOR1 ) {	// �Z���T�S��
						modeError = 2;
					} else if ( cntOut2 >= STOP_SENSOR2 ) {	// �Z���T�S����
						modeError = 3;
					} else if ( cntOut3 >= STOP_ENCODER ) {	// �G���R�[�_��~(�Ђ�����Ԃ����H)
						modeError = 4;
					} else if( cntOut4 >= STOP_GYRO ) {	// �}�C�i�X�̉����x���m(�R�[�X���痎�����H)
						modeError = 5;	
					}
					// // Buletooth�ŊO�������~
					// if ( stopWord == 1 ) {
					// 	modeError = 6;
					// }
					// ��莞�ԂŒ�~
					// if( cntStable >= STOP_COUNT ) {
					// 	modeError = 7;
					// }
					if (modeError > 0) {
						pattern = 101;
						modeAutoMotor = 0;
						ui = cnt1;	// ���s���Ԏ擾
						LEDR_OFF;
						LEDG_OFF;
						LEDB_OFF;
					}
				}
			} else {			
				// �艟�����[�hON

				// ��������
				// lcdRowPrintf(UPROW, "mm%4.1f", (double)EncoderTotal/PALSE_MILLIMETER);
				// lcdRowPrintf(LOWROW, " %7d", EncoderTotal);

				lcdRowPrintf(UPROW, "now  %3d", pattern);
				lcdRowPrintf(LOWROW, "D   0x%2x", sensor_inp(MASK11111));
			}
			// �X�C�b�`�Œ�~
			if ( cnt1 >= 1000 && taswGet() == SW_PUSH ) {
				modeError = 6;
				pattern = 101;
			}
		} else if ( pattern >= 100 ) {
			modeLCD = 1;
			// ���x�Q�C�������p
			// lcdRowPrintf(UPROW, "TIME%4d", cntStable);
			// lcdRowPrintf(LOWROW, "%6.1f", (double)encStable/PALSE_MILLIMETER);

			// �G���[���[�h�m�F
			lcdRowPrintf(UPROW, "MODE   %1d", modeError);
			lcdRowPrintf(LOWROW, "    %4d",angleAfter);
		}
	if( check_line ){	
																		
		switch(	checkLine() ){											
			case 1:					/* �N���X���C�����o���̕��� */
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //���O�̐i���p�x���L�^
				pattern = 21;											
				break;												
																		
			case 2:					/* �E�n�[�t���C�����o���̕��� */	
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //���O�̐i���p�x���L�^
				pattern = 51;											
				break;												
																	
			case 3:					/* ���n�[�t���C�����o���̕��� */
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //���O�̐i���p�x���L�^
				pattern = 61;										
				break;												
																	
			default:												
				break;												
		}																
	}
		
	switch( pattern ) {
		//-------------------------------------------------------------------
		// �y000�z�X�^�[�g����
		//-------------------------------------------------------------------
		case 0:
			// �X�^�[�g�O�ݒ�
			setup();
			if ( start && !modePushcart ) {
				demo = 0;		// �f�����[�h����
				modeAngle = 0;	// �����g���[�X
				Int = 0;			// �ϕ����Z�b�g
				txt= txtData;		// ��M�z�񃊃Z�b�g
				cntByte = 0;		// ��M�J�E���g���Z�b�g
				
				if ( msdset ) init_log();	// ���O�L�^����
				
				if ( !fixSpeed ) writeFlashBeforeStart(1, 0, 1, 1, 1, 1);	// ���x�p�����[�^���f�[�^�t���b�V���ɕۑ�
				else writeFlashBeforeStart(0, 0, 1, 1, 1, 1);		// ���x�p�����[�^�ȊO��ۑ�
				
				//if (IMUSet) caribrateIMU();
				
				waitLcd(500);		// 500ms�҂�
				cnt1 = 0;
				pattern = 1;
				break;
			} else if ( start && modePushcart ) {
				// �艟�����[�h�̏ꍇ�����ɒʏ�g���[�X
				if ( msdset ) init_log();	// ���O�L�^����
				
				// �����g���[�X�pPID�Q�C���ۑ�
				// �p�x����pPID�Q�C���ۑ�
				writeFlashBeforeStart(0, 0, 1, 1, 0, 0);
				// �ϐ�������
				initParameter( 1 );
				break;
			}
			break;
			
		case 1:
			servoPwmOut( ServoPwm );
			if ( start == START_COUNT ) {
				// �J�E���g�_�E���X�^�[�g
				if ( cnt1 >= 3000 ) {	
					// �ϐ�������
					initParameter( 0 );
					break;
				} else if ( !(cnt1 % 1000) ) {
					ledOut( countdown );
					countdown = countdown << 1;
					break;
				}
			} else if ( start == START_GATE ) {
				// �X�^�[�g�Q�[�g�J���X�^�[�g
				pattern = 2;
				break;
			}
			break;
			
		case 2:
			servoPwmOut( ServoPwm );
			// �X�^�[�g�o�[�J�҂�
			if ( !startbar_get() ) {
				// �ϐ�������
				initParameter( 0 );
				break;
			}
			// LED�_�ŏ���
			if ( cnt1 >= 2000 ) cnt1 = 0;
			if ( cnt1 < 1000 ) {
				ledOut( LED_R );
			} else {
				ledOut( LED_B  );
			}
			break;
		//-------------------------------------------------------------------
		// �y010�z�g���[�X����
		//-------------------------------------------------------------------
		case 11:
			targetSpeed = speed_straight * SPEED_CURRENT;
			i = getServoAngle();
			ledOut( 0x00 );
			
			check_line  = 1;
			
			// �⓹�`�F�b�N
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			
			// �J�[�u�`�F�b�N
			if ( i >=  CURVE_R600_START || i <= -CURVE_R600_START ) {
				enc1 = 0;
				modeCurve = 1;
				pattern = 12;
				break;
			}
			break;
			
		case 12:
			// �J�[�u�u���[�L
			targetSpeed = speed_curve_brake * SPEED_CURRENT;
			ledOut( LED_R );
			i = getServoAngle();
			
			if ( enc1 > encMM( 60 ) ) {		// 60mm�i��
				enc1 = 0;
				TurningAngleEnc = 0;
				TurningAngleIMU = 0;
				pattern = 13;
				break;
			}
			
			check_line  = 1;
			
			// �⓹�`�F�b�N
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			if ( memory_mode ) {
				enc1 = 0;
				pattern = 16;
				break;
			}
			// �����`�F�b�N
			if ( i < CURVE_R600_START && i > -CURVE_R600_START ) {
				enc1 = 0;
				pattern = 11;
				break;
			}
			break;
			
		case 13:
			// R600�J�[�u���s
			targetSpeed = speed_curve_r600 * SPEED_CURRENT;
			i = getServoAngle();
			
			check_line  = 1;
			
			// �⓹�`�F�b�N
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			// R450�`�F�b�N
			if ( i >= CURVE_R450_START || i <= -CURVE_R450_START ) {
				enc1 = 0;
				pattern = 14;
				break;
			}
			// �J�[�u�p���ڃ`�F�b�N
			if ( i <  CURVE_R600_START && i > -CURVE_R600_START ) {
				enc1 = 0;
				pattern = 15;
				break;
			}
			break;
			
		case 14:
			// R450�J�[�u���s
			targetSpeed = speed_curve_r450 * SPEED_CURRENT;
			i = getServoAngle();
			
			check_line  = 1;
			
			// R600�`�F�b�N
			if ( i < CURVE_R450_START && i > -CURVE_R450_START ) {
				enc1 = 0;
				pattern = 13;
				break;
			}
			break;
		
		case 15:
			// �J�[�u�p���ڑ��s
			targetSpeed = speed_curve_straight * SPEED_CURRENT;
			i = getServoAngle();
			
			if ( enc1 >= encMM( 300 ) ) {		// 300mm�i��
				enc1 = 0;
				modeCurve = 0;
				pattern = 11;
				break;
			}
			
			check_line  = 1;
			
			// �⓹�`�F�b�N
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			// �J�[�u�`�F�b�N
			if( i >=  CURVE_R600_START || i <= - CURVE_R600_START ) {
				enc1 = 0;
				pattern = 13;
				break;
			}
			break;
			
		case 16:
			targetSpeed = speed_straight * SPEED_CURRENT;
			
			break;
		//-------------------------------------------------------------------
		//�y020�z�N�����N���o����
		//-------------------------------------------------------------------
		case 21:
			check_line  = 0;
			
			targetSpeed = speed_crossline* SPEED_CURRENT;
			
			ledOut( LED_G );
			
			if( enc1 > encMM( 90 ) ) {		// 60mm�i��
				enc1 = 0;
				pattern = 22;
				break;
			}
			break;
			
		case 22:
			targetSpeed = speed_ckank_trace * SPEED_CURRENT;
			
			// �E�N�����N�`�F�b�N
			if( sensor_inp(MASK00001) ==  0x01 ) {
				enc1 = 0;
				modeAngle = 1;
				TurningAngleEnc = 0;
				TurningAngleIMU = 0;
				pattern = 30;
				break;
			}
			// ���N�����N�`�F�b�N
			if( sensor_inp(MASK11100) ==  0x1c) {
				enc1 = 0;
				modeAngle = 1;
				TurningAngleEnc = 0;
				TurningAngleIMU = 0;
				pattern = 40;
				break;
			}
			
	        break;
		//-------------------------------------------------------------------
		//�y030�z�E�N�����N����
		//-------------------------------------------------------------------
		case 30:// �ݒ�p�x�܂Ő؂菭���҂�
			SetAngle = angle_rightclank;
			if( getServoAngle() > (ANGLE_RIGHTCLANK) ){
				y1 = getLinePositionNow( getServoAngle(), TurningAngleIMU);
				enc1 = 0;
				Int = 0;			// �ϕ����Z�b�g
				if (sensor_inp() == 0x2) {
					pattern = 36;	
					break;
				}
				if(sensor_inp() == 0x10) {
					pattern = 34;
					break;
				}
				pattern = 31;
				break;

			}
			break;
		case 31://�n���h���L�[�v
			SetAngle = angle_rightclank;
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT;
			
			if (sensor_inp() == 0x2) {
				pattern = 36;	
				break;
			}
			if(sensor_inp() == 0x1) {
				pattern = 34;
				break;
			}
				
			break;
		case 32://�O�g�����o����
			SetAngle = angle_rightclank;
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT;
			
			if(sensor_inp() == 0x10){
				pattern = 34;
				break;
			}
			break;
		case 33:
			break;
		case 34://�ō��[�Z���T���o����
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT ;
			correct_angle = 230;
			if(sensor_inp() == 0x2){
				SetAngle = correct_angle;
				if(getServoAngle() > correct_angle){
					modeAngle = 0;
					pattern = 36;
					break;
					
				}
			}
			break;
		case 35://�����Z���T���o����
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT;
			enc1 = 0;
			
			Int = 0;			// �ϕ����Z�b�g
			pattern = 36;
			break;

		case 36://���A
			modeAngle = 0;
			targetSpeed = speed_rightclank_escape * SPEED_CURRENT;
			if( enc1 >= encMM( 600 ) ) {		// ���肷��܂ő҂�(600mm)
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		//-------------------------------------------------------------------
		//�y040�z���N�����N����
		//-------------------------------------------------------------------
		case 40:// �ݒ�p�x�܂Ő؂菭���҂�
			SetAngle = angle_leftclank;
			if( getServoAngle() < ANGLE_LEFTCLANK ){
				y1 = getLinePositionNow( getServoAngle(), TurningAngleIMU);
				enc1 = 0;
				Int = 0;			// �ϕ����Z�b�g
				if (sensor_inp() == 0x10) {
					pattern = 44;	
					break;
				}
				if(sensor_inp() == 0x8) {
					pattern = 46;
					break;
				}
				pattern = 41;
				break;
			}
			break;
		case 41://�n���h���L�[�v
			SetAngle = angle_leftclank;
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			
			if (sensor_inp() == 0x8) {
				pattern = 46;	
				break;
			}
			if(sensor_inp() == 0x10) {
				pattern = 44;
				break;
			}
				
			break;
		case 42://�O�g�����o����
			SetAngle = angle_leftclank;
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			
			if(sensor_inp() == 0x1){
				pattern = 44;
				break;
			}
			break;
		case 43:
			break;
		case 44://�ō��[�Z���T���o����
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			correct_angle = -230;
			if(sensor_inp() == 0x8){
				SetAngle = correct_angle;
				if(getServoAngle() > correct_angle){
					modeAngle = 0;
					pattern = 46;
					break;
					
				}
			}
			break;
		case 45://�����Z���T���o����
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			enc1 = 0;
			
			Int = 0;			// �ϕ����Z�b�g
			pattern = 46;
			break;

		case 46://���A
			modeAngle = 0;
			targetSpeed = speed_leftclank_escape * SPEED_CURRENT;
			if( enc1 >= encMM( 600 ) ) {		// ���肷��܂ő҂�(600mm)
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		
		//-------------------------------------------------------------------
		//�y050�z�E���[���`�F���W����
		//-------------------------------------------------------------------
		case 51://�}�[�J�[��g���[�X����
			check_line  = 0;
			targetSpeed = speed_halfine * SPEED_CURRENT;
			modeAngle = 1;
			SetAngle = Angle_fixed; //���O�i���p�x�ŌŒ�
			if( enc1 > encMM( 20 ) ) {
				enc1 = 0;
				modeAngle = 0;
				Int = 0;			// �ϕ����Z�b�g
				pattern = 52;
				break;
			}
			
			if( check_crossline() ) {		// �N���X���C���`�F�b�N
				enc1 = 0;
				pattern = 21;
				break;
			}
			break;
			
		case 52://���؂�ǂݔ�΂�����(300mm)
			targetSpeed = speed_rightchange_trace * SPEED_CURRENT;
			
			if (enc1 > encMM( 100 ) ){
				enc1 = 0;
				pattern = 53;
				break;
			}
			break;
			
		case 53:
			targetSpeed = speed_rightchange_trace * SPEED_CURRENT;
			if( sensor_inp(MASK11111) == 0x0 ) {
				enc1 = 0;
				modeAngle = 1;
				modeMotor = 0;
				pattern = 54;
				break;
			}
			break;
			
		case 54:
			SetAngle = angle_rightchange;
			//targetSpeed = speed_rightchange_curve * SPEED_CURRENT;
			
			if( sensor_inp(MASK00001) == 0x1 ) {
				enc1 = 0;
				pattern = 55;
				break;
			}
			break;
			
		case 55:
			SetAngle = 0;
			//targetSpeed = speed_rightchange_curve * SPEED_CURRENT;

			if(sensor_inp(MASK10000) == 0x10) {
				enc1 = 0;
				Int = 0;			// �ϕ����Z�b�g
				
				pattern = 56;
				break;
			}
			break;
			
		case 56:
			//targetSpeed = speed_rightchange_curve * SPEED_CURRENT;

			
			if(  sensor_inp(MASK10000) == 0x00 ) {
				enc1 = 0;
				SetAngle = -(angle_rightchange);
				pattern = 57;
				break;
			}
			break;
		case 57:
			
			//targetSpeed = speed_rightchange_curve * SPEED_CURRENT;
			if(  sensor_inp(MASK00100) == 0x04 && 10 >= abs(getAnalogSensor()) ) {
				modeAngle = 0;
				enc1 = 0;
				modeMotor = 1;
				pattern = 58;
				break;
				

			}
			break;
			
		case 58:
			targetSpeed = speed_rightchange_escape * SPEED_CURRENT;

			if( enc1 >= encMM( 50 ) ) {
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		//-------------------------------------------------------------------
		//�y060�z�����[���`�F���W����
		//-------------------------------------------------------------------
		case 61:
			check_line  = 0;
			targetSpeed = speed_halfine * SPEED_CURRENT;
			modeAngle = 1;
			SetAngle = Angle_fixed; //���O�i���p�x�ŌŒ�
			if( enc1 > encMM( 20 ) ) {
				enc1 = 0;
				modeAngle = 0;
				Int = 0;			// �ϕ����Z�b�g
				pattern = 62;
				break;
			}
			if( check_crossline() ) {		// �N���X���C���`�F�b�N
				enc1 = 0;
				pattern = 21;
				break;
			}
			break;
			
		case 62://���؂�ǂݔ�΂�����(300mm)
			targetSpeed = speed_leftchange_trace * SPEED_CURRENT;
			if (enc1 > encMM( 100 ) ){
				enc1 = 0;
				pattern = 63;
				break;
			}
			break;
		case 63:
			targetSpeed = speed_leftchange_trace * SPEED_CURRENT;
			
			if( sensor_inp(MASK11111) == 0x0 ) {
				enc1 = 0;
				modeAngle = 1;
				modeMotor = 0;
				pattern = 64;
				break;
			}
			break;
			
		case 64:
			SetAngle = angle_leftchange;
			//targetSpeed = speed_leftchange_curve * SPEED_CURRENT;
			
			if( sensor_inp(MASK10000) == 0x10 ) {
				enc1 = 0;
				pattern = 65;
				break;
			}
			break;
			
		case 65:
			SetAngle = 0;
			//targetSpeed = speed_leftchange_curve * SPEED_CURRENT;

			if( sensor_inp(MASK00001) == 0x1 ) {
				enc1 = 0;
				Int = 0;			// �ϕ����Z�b�g
				
				pattern = 66;
				break;
			}
			break;
		case 66:
			//targetSpeed = speed_leftchange_curve * SPEED_CURRENT;
			
			if(  sensor_inp(MASK00001) == 0x00 ) {
				enc1 = 0;
				SetAngle = -(angle_leftchange/2);
				pattern = 67;
				break;
			}
			break;
		case 67:
			
			//targetSpeed = speed_leftchange_curve * SPEED_CURRENT;
			
			if(  sensor_inp(MASK00100) == 0x4 && 30 >= abs(getAnalogSensor()) ) {
				enc1 = 0;
				modeAngle = 0;
				modeMotor = 1;
				pattern = 68;
				break;
			}
			break;
			
			
		case 68:
			targetSpeed = speed_leftchange_escape * SPEED_CURRENT;

			if( enc1 >= encMM( 50 ) ) {
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		//-------------------------------------------------------------------
		//�y070�z�⓹����
		//-------------------------------------------------------------------
		case 71:
			// �댟�m���f
			// �ڕW���x�ς��Ȃ�
			if( checkSlope() == 1 ) {
				if( modeSlope == 0 ) {
					
					// ���n��
					modeSlope = 1;
					enc1 = 0;
					ledOut( LED_R | LED_B );
					pattern = 72;
					break;
				} else if ( modeSlope == 2 && enc_slope >= encMM( 600 ) ) {
					// ����I���
					modeSlope = 3;
					pattern = 74;
					break;
				} else {
					enc1 = 0;
					pattern = 11;
					break;
				}
			} else if ( checkSlope() == -1 ) {
				if( modeSlope == 1 && enc_slope >= encMM( 1000 ) ) {
					// ���I���A����n��
					modeSlope = 2;
					enc1 = 0;
					pattern = 75;
					break;
				} else {
					enc1 = 0;
					pattern = 11;
					break;
				}
				break;
			} else {
				enc1 = 0;
				pattern = 11;
				break;
			}
			break;
			
		case 72:
			// �Ⓒ�_�܂ő��s
			targetSpeed = speed_slope_trace * SPEED_CURRENT;
			
			if( enc1 >= encMM( 1200 ) ) {
				enc1 = 0;
				pattern = 73;
				break;
			}
			break;
			
		case 73:
			// ����I�_�u���[�L �S�̓u���[�L
			targetSpeed = 0;

			if( enc1 >= encMM( 50 ) ) {
				enc1 = 0;
				pattern = 75;
				break;
			}
			break;
			
		case 74:
			// �����I�_�u���[�L
			targetSpeed = speed_slope_brake * SPEED_CURRENT;
			if( enc1 >= encMM( 40 ) ) {
				enc1 = 0;
				pattern = 75;
				break;
			}
			break;
			
		case 75:
			// �W���C���Z���T�����肷��܂œǂݔ�΂�
			targetSpeed = speed_slope_trace * SPEED_CURRENT;
			//motorPwmOut(0, 0,
				   // 0, 0);
			if( enc1 >= encMM( 150 ) ) {
				enc1 = 0;
				pattern = 76;
				break;
			}
			break;
			
		case 76:
			// �W���C���Z���T�����肷��܂œǂݔ�΂�
			if ( modeSlope == 3 ) {
				targetSpeed = speed_straight * SPEED_CURRENT;
			} else {
				targetSpeed = speed_slope_trace * SPEED_CURRENT;
			}
			
			// �N���X���C���`�F�b�N
			if( check_crossline() ) {
				enc1 = 0;
				pattern = 21;
				break;
      			}
			// �E�n�[�t���C���`�F�b�N
   			if( check_rightline() ) {
				enc1 = 0;
				pattern = 51;
				break;
			}
			// ���n�[�t���C���`�F�b�N
   			if( check_leftline() ) {
				enc1 = 0;
				pattern = 61;
				break;
			}
			if( modeSlope  == 0 ) {
				if( enc1 >= encMM( 1000 ) ) {
					enc1 = 0;
					enc_slope = 0;
					pattern = 11;
					break;
				}
			} else if ( modeSlope == 3 ) {
				if( enc1 >= encMM( 500 ) ) {
					enc1 = 0;
					enc_slope = 0;
					pattern = 11;
					break;
				}
			} else {
				if( enc1 >= encMM( 400 ) ) {
					enc1 = 0;
					enc_slope = 0;
					pattern = 11;
					break;
				}
			}
			break;
		//-------------------------------------------------------------------
		//�y100�z��~����
		//-------------------------------------------------------------------
		case 101:
			// ��������
			modeAngle = 0;
			targetSpeed = 10;
			motor_mode_f( F, F );
			motor_mode_r( F, F );
			if( enc1 >= encMM( 100 ) ) {
				enc1 = 0;
				pattern = 102;
				break;
			}
			break;
			
		case 102:
			// �ԑ̒�~����
			modeAngle = 0;
			motor_mode_f( B, B );
			motor_mode_r( B, B );
			
			if( Encoder <= 5 && Encoder >= -1 ) {
				//servoPwmOut( 0 );
				//R_PG_IO_PORT_Write_PE6( 0 );	//�T�[�{���[�^ free���[�h
				if( msdset == 1 ) {
					// microSD�̓��삪�L���ȂƂ�
					pattern = 103;
					cnt1 = 0;
					break;
				} else {
					// microSD�̓��삪�����ȂƂ�
					pattern = 106;
					break;
				}
			}
			break;
			
		case 103:
			servoPwmOut( ServoPwm );
			// �Ō�̃f�[�^���������܂��܂ő҂�
			if ( cnt1 <= 1000 ) {	// 1000ms�҂�
				if( checkMicroSDProcess() == 11 ) {
					msdFlag = 0;			// ���O�L�^�I��
					microSDProcessEnd();    // microSDProcess�I������
					pattern = 104;
					break;
				}
			} else {			// 1000ms�ȏ�o�߂����珑�����݂������I��
				pattern = 106;
				break;
			}
			break;
			
		case 104:
			servoPwmOut( ServoPwm );
			// microSD�̏������ݏI���������I���܂ő҂�
			if( checkMicroSDProcess() == 0 ) {
				// MicroSD�ŏI�������݃A�h���X�ۑ�
				flashDataBuff[ 0 ] = msdWorkaddress >> 16;
				flashDataBuff[ 1 ] = msdWorkaddress & 0xffff;	// �I���A�h���X
				writeFlashData( MSD_STARTAREA, MSD_ENDAREA, MSD_DATA, 2 );
				pattern = 105;
				break;
			}
			break;
			
		case 105:
			servoPwmOut( ServoPwm );
			// microSD�������ݐ���
			// LED�_�ŏ���
			if( cnt1 >= 200 ) cnt1 = 0;
			if( cnt1 < 100 ) {
				LEDB_ON;
			} else {
				LEDB_OFF;
			}
			break;
			
		case 106:
			servoPwmOut( ServoPwm );
			// mMicroSD�������ݎ��s
			// LED�_�ŏ���
			if( cnt1 >= 200 ) cnt1 = 0;
			if( cnt1 < 100 ) {
				LEDR_ON;
			} else {
				LEDR_OFF;
			}
			break;
			
		default:
			pattern = 101;
			break;
			
	} // end of "switch ( pattern )"
	} // end of "while ( 1 )"
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� Timer
// �����T�v     1ms���ƂɃ^�C�}���荞��
// ����         �Ȃ�
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void Timer (void) {
	short n;
	__setpsw_i();	// ���d���荞�݋���
	
	//�@�^�C�}�J�E���g
	if ( pattern >= 11 ) {
		if ( pattern <= 99 ) {
			if ( pattern != 21 ) {				// �N���X���C���ʉߎ��͖���
				if ( sensor_inp(MASK11111) == 0x1f || sensor_inp(MASK11111) == 0x5 ) {	// �Z���T�S��
					cntOut1++;	
				} else {
					cntOut1 = 0;
				}
			}
			if ( sensor_inp(MASK11111) == 0x0 && pattern != 53 && pattern != 63 ) cntOut2++;	// �Z���T�S����
			else cntOut2 = 0;
			if ( Encoder <= 1 && Encoder >= -1 ) cntOut3++;		// �G���R�[�_��~(�Ђ�����Ԃ����H)
			else cntOut3 = 0;
			if ( (short)RollAngleIMU >= 20 || (short)RollAngleIMU <= -20 ) cntOut4++;
			else	cntOut4 = 0;
		}
	} else if ( pattern < 11 ) {
		cntSetup1++;
		cntSetup2++;
		cntSetup3++;
		cnt_flash++;
	}
	cnt1++;
	cntGyro++;
			
	// LCD�\��
	if ( modeLCD ) lcdShowProcess();
	// �G���R�[�_�J�E���g�擾
	getEncoder();
	// �A�i���O�Z���T���K��
	getsens_ratio();

	// PID����l�Z�o
	if ( modeAngle ) servoControlAngle();	// �p�x
	else servoControlTrace();		// ����
	if ( modeMotor )motorControl();		// ���[�^
	else {
		motor_mode_f( F, F );
		motor_mode_r( F, F );
	}

	// ���s����PWM�o��
	if ( modeAutoMotor ) {
		if ( modeAngle ) servoPwmOut( ServoPwm2 );	// �p�x
		else servoPwmOut( ServoPwm );	// ����
		if (!modePushcart) {
			diff( motorPwm ); // �쓮�փ��[�^PWM�o��
		}
	}
	

	
	// MicroSD��������
	microSDProcess();
	if ( msdFlag ) sendLog( 12,6, 1
					// char
					, (char)pattern
					, (char)motorPwm
					, (char)accele_fL
					, (char)accele_fR
					, (char)accele_rL
					, (char)accele_rR
					, (char)sensor_inp()
					, (char)modeSlope
					, (char)Encoder
					, (char)sPwm
					, (char)(PichAngleIMU*10)
					, (char)(RollAngleIMU*10)
					//short
					//, (short)(TurningAngleIMU*10)
					//, xg
					//, yg
					//, zg
					, (short)getServoAngle()
					, (short)SetAngle
					, (short)getAnalogSensor()
					//, (short)(Voltage*100)
					, (short)(targetSpeed/SPEED_CURRENT)
					, (short)currentSpeed
					, (short)iAngle
					// unsigned int
					, (unsigned int)(EncoderTotal / 6.150)
					//, encStable
					//, cnt_log
					);
	Timer10++;
	
	// �ʐM
	// �����x�y�ъp���x���擾
	
		
	// 10�����ƂɎ��s
	switch ( Timer10 ) {	
	case 1:
		getSwitch();		// �X�C�b�`�ǂݍ���
		getVoltage();		// �d���d���擾
		
		n = getServoAngle();
		if(n < 0) n = -n;
		iAngle = n - Angle_buff;
		Angle_buff = n;
		
		break;
	case 2:
		readGyroData();
		readAccelData();
		getTurningAngleIMU();
		getPichAngleIMU();
		getRollAngleIMU();
		if (cntGyro > 200) {
			RollAngleIMU = 0;
			PichAngleIMU = 0;
			cntGyro  = 0;
		}
		break;
	case 3:

		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		Timer10 = 0;
		break;
	default:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////
// ���W���[���� initParameter
// �����T�v     �ϐ��̏�����
// ����         lcd: 1 lcd�\��  0 lcd��\��
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void initParameter ( bool lcd ) {
	cntmpattern2 = 0;	// �L�^���s�J�E���g���Z�b�g
	EncoderTotal = 10;	// �����s����
	cnt1 = 0;			// �^�C�}���Z�b�g
	enc1 = 0;			// ��ԋ������Z�b�g
	modeLCD = lcd;		// LCD�\��OFF
	modeAutoMotor = 1; // PWM�o��
	msdFlag = 1;		// �f�[�^�L�^�J�n
	targetSpeed = speed_straight * SPEED_CURRENT; // �ڕW���x�ݒ�
	
	// �p�x�ώZ�l���Z�b�g
	TurningAngleIMU = 0;
	RollAngleIMU = 0;
	PichAngleIMU = 0;
	pattern = 11;		// �ʏ푖�s
}