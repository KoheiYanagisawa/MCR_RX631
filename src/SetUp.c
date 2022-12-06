//====================================//
// �C���N���[�h
//====================================//
#include "setup.h"
//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
char 	start;		// 0:�Z�b�g�A�b�v��	1:�Z�b�g�A�b�v����

// �^�C�}�֘A
unsigned short 		cntSetup1;		// �Z�b�g�A�b�v�Ŏg�p
unsigned short 		cntSetup2;		// �Z�b�g�A�b�v�Ŏg�p
unsigned short 		cntSetup3;		// �Z�b�g�A�b�v�Ŏg�p
short				cntSwitchUD;	// �X�C�b�`����������p�E
short				cntSwitchLR;	// �X�C�b�`����������p��

// �X�C�b�`�֘A
signed char pushLR = 0;
signed char pushUD = 0;

// �p�^�[���֘A
char push = 0;
char push1 = 0;
char pattern_sensor = 1;
char pattern_parameter = 1;
char pattern_parameter2 = 1;
char pattern_parameter3 = 1;
char pattern_parameter4 = 1;
char pattern_gain = 1;
char pattern_gain2 = 1;
char pattern_gain3 = 1;
char pattern_speedseting = 1;
char pattern_msd = 1;
char pattern_flash = 1;
char pattern_scanf = 0;

// �t���O�֘A
char setting_1meter;
char setting_2meter;
char setting_3meter;
char motor_test = 0;
char servo_test = 0;
char servo_test2 = 0;
char fixSpeed = 0;
char str[8];

// �p�����[�^�֘A
char motorTestPwm = 10;

//====================================//
// �v���g�^�C�v�錾
//====================================//
void data_select ( char *data , char button );
void data_tuningUD ( void *data, char add );
void data_tuningLR ( void *data, char add );
void wait2 ( int waittime );

short cnttest = 0;
///////////////////////////////////////////////////////////////
// ���W���[���� setup
// �����T�v     ���s�O�ݒ�
// ����         �Ȃ�
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////
void setup( void )
{
	char cnt_led;
	short i, j, k;
	uint8_t sd_sw;
	
	// �f�B�b�v�X�C�b�`�ō��ڑI��
	switch ( dipswGet() ) {
		//------------------------------------------------------------------
		// �X�^�[�g�҂�
		//------------------------------------------------------------------
		case 0x0:
			lcdRowPrintf(UPROW, "START   ");
			lcdRowPrintf(LOWROW, "STOP %2dm", stopping_meter );
			
			// �Q�[�g���o
			if ( startbar_get() ) ledOut(LED_B);
			else 	ledOut(LED_R);
			// ��~��������
			data_tuningUD ( &stopping_meter, 1 );
			
			modeAngle = 0;
			data_select( &servo_test, SW_RIGHT );
			if ( servo_test == 1 ) servoPwmOut( ServoPwm );
			else servoPwmOut( 0 );
			
			// �v�b�V���X�C�b�`�����҂�
			if ( taswGet() == SW_PUSH ) {
				start = START_COUNT;
			} else if ( taswGet() == SW_LEFT ) {
				start = START_GATE;
			} else if ( taswGet() == SW_RIGHT ) {
				modePushcart = 1;
				start = START_COUNT;
			}
			
			break;
		//------------------------------------------------------------------
		// �p�����[�^����(�ʏ�g���[�X)
		//------------------------------------------------------------------
		case 0x1:
			data_tuningLR( &pattern_parameter, 1 );
			
			if ( pattern_parameter == 6 ) pattern_parameter = 1;
			else if ( pattern_parameter == 0 ) pattern_parameter = 5;
			
			switch( pattern_parameter ) {
				case 1:
					// �ʏ푖�s���x
					lcdRowPrintf(UPROW, "STRAIGHT");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_straight / 10 );
					
					data_tuningUD ( &speed_straight, 1 );
					break;
				case 2:
					// �J�[�u�u���[�L
					lcdRowPrintf(UPROW, "BRAKE   ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_curve_brake / 10 );
					
					data_tuningUD ( &speed_curve_brake, 1 );
					break;
				case 3:
					// R600�J�[�u���s���x
					lcdRowPrintf(UPROW, "R600    ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_curve_r600 / 10  );
					
					data_tuningUD ( &speed_curve_r600, 1 );
					break;
				case 4:
					// R450�J�[�u���s���x
					lcdRowPrintf(UPROW, "R450    ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_curve_r450 / 10 );
					
					data_tuningUD ( &speed_curve_r450, 1 );
					break;
				case 5:
					// S���J�[�u�������x
					lcdRowPrintf(UPROW, "CURVE_ST");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_curve_straight / 10 );
					
					data_tuningUD ( &speed_curve_straight, 1 );
					break;
			}
			break;
			
		//------------------------------------------------------------------
		// �p�����[�^����(�N�����N)
		//------------------------------------------------------------------
		case 0x2:
			data_tuningLR( &pattern_parameter2, 1 );
			
			if ( pattern_parameter2 == 7 ) pattern_parameter2 = 1;
			else if ( pattern_parameter2 == 0 ) pattern_parameter2 = 6;
			
			switch( pattern_parameter2 ) {
				case 1:
					// �N���X���C���ʉߎ�
					lcdRowPrintf(UPROW, "CROSLINE");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_crossline / 10 );
					
					data_tuningUD ( &speed_crossline, 1 );
					break;
				case 2:
					// �N�����N�O�g���[�X
					lcdRowPrintf(UPROW, "CL_TRACE");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_ckank_trace / 10 );
					
					data_tuningUD ( &speed_ckank_trace, 1 );
					break;
				case 3:
					// �E�N�����N���񑬓x
					lcdRowPrintf(UPROW, "RCL_CUR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_rightclank_curve / 10 );
					
					data_tuningUD ( &speed_rightclank_curve, 1 );
					break;
				case 4:
					// �E�N�����N���A���x
					lcdRowPrintf(UPROW, "RCL_ESC ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_rightclank_escape / 10 );
					
					data_tuningUD ( &speed_rightclank_escape, 1 );
					break;
					
				case 5:
					// ���N�����N���񑬓x
					lcdRowPrintf(UPROW, "LCL_CUR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_leftclank_curve / 10 );
					
					data_tuningUD ( &speed_leftclank_curve, 1 );
					break;
					
				case 6:
					// ���N�����N���A���x
					lcdRowPrintf(UPROW, "LCL_ESC ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_leftclank_escape / 10 );
					
					data_tuningUD ( &speed_leftclank_escape, 1 );
					break;
			}
			break;
		//------------------------------------------------------------------
		// �p�����[�^����(���[���`�F���W)
		//------------------------------------------------------------------
		case 0x3:
			data_tuningLR( &pattern_parameter3, 1 );
			
			if ( pattern_parameter3 == 8 ) pattern_parameter3 = 1;
			else if ( pattern_parameter3 == 0 ) pattern_parameter3 = 7;
			
			switch( pattern_parameter3 ) {
				case 1:
					// �n�[�t���C���ʉߎ�
					lcdRowPrintf(UPROW, "HALFLINE");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_halfine / 10 );
					
					data_tuningUD ( &speed_halfine, 1 );
					break;
				case 2:
					// �E���[���`�F���W�i�����x
					lcdRowPrintf(UPROW, "RLC_STR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_rightchange_trace / 10 );
					
					data_tuningUD ( &speed_rightchange_trace, 1 );
					break;
				case 3:
					// �E���[���`�F���W���񑬓x
					lcdRowPrintf(UPROW, "RLC_CUR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_rightchange_curve / 10 );
					
					data_tuningUD ( &speed_rightchange_curve, 1 );
					break;
				case 4:
					// �E���[���`�F���W���A���x
					lcdRowPrintf(UPROW, "RLC_ESC ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_rightchange_escape / 10 );
					
					data_tuningUD ( &speed_rightchange_escape, 1 );
					break;
				case 5:
					// �����[���`�F���W�i�����x
					lcdRowPrintf(UPROW, "LLC_STR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_leftchange_trace / 10 );
					
					data_tuningUD ( &speed_leftchange_trace, 1 );
					break;
				case 6:
					// �����[���`�F���W���񑬓x
					lcdRowPrintf(UPROW, "LLC_CUR ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_leftchange_curve / 10 );
					
					data_tuningUD ( &speed_leftchange_curve, 1 );
					break;
				case 7:
					// �����[���`�F���W���񑬓x
					lcdRowPrintf(UPROW, "LLC_ESC ");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_leftchange_escape / 10 );
					
					data_tuningUD ( &speed_leftchange_escape, 1 );
					break;
			}
			break;
		
		//------------------------------------------------------------------
		// �p�����[�^����(�⓹�A�p�x)
		//------------------------------------------------------------------
		case 0x4:
			data_tuningLR( &pattern_parameter4, 1 );
			
			if ( pattern_parameter4 == 7 ) pattern_parameter4 = 1;
			else if ( pattern_parameter4 == 0 ) pattern_parameter4 = 6;
			
			switch( pattern_parameter4 ) {
				case 1:
					// �⓹�u���[�L
					lcdRowPrintf(UPROW, "SL_BRAKE");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_slope_brake / 10 );
					
					data_tuningUD ( &speed_slope_brake, 1 );
					break;
				case 2:
					// ��ǂݔ�΂����x
					lcdRowPrintf(UPROW, "SL_TRACE");
					lcdRowPrintf(LOWROW, "  %3gm/s", (double)speed_slope_trace / 10 );
					
					data_tuningUD ( &speed_slope_trace, 1 );
					break;
				case 3:
					// �E�N�����N����p�x
					lcdRowPrintf(UPROW, "ANG_RCR ");
					lcdRowPrintf(LOWROW, "   %5d", angle_rightclank );
					
					data_tuningUD ( &angle_rightclank, 1 );
					break;
				case 4:
					// ���N�����N����p�x
					lcdRowPrintf(UPROW, "ANG_LCL ");
					lcdRowPrintf(LOWROW, "   %5d", angle_leftclank );
					
					data_tuningUD ( &angle_leftclank, 1 );
					break;
				case 5:
					// �E���[���`�F���W����p�x
					lcdRowPrintf(UPROW, "ANG_RLC ");
					lcdRowPrintf(LOWROW, "   %5d", angle_rightchange );
					
					data_tuningUD ( &angle_rightchange, 1 );
					break;
				case 6:
					// �����[���`�F���W����p�x
					lcdRowPrintf(UPROW, "ANG_LLC ");
					lcdRowPrintf(LOWROW, "   %5d", angle_leftchange );
					
					data_tuningUD ( &angle_leftchange, 1 );
					break;
			}
			break;
		//------------------------------------------------------------------
		// �Q�C������(�T�[�{)
		//------------------------------------------------------------------
		case 0x5:
			lcdRowPrintf(UPROW, "kp ki kd");
			
			data_select( &servo_test, SW_PUSH );
			modeAngle = 0;
			if ( servo_test == 1 ) servoPwmOut( ServoPwm );
			else servoPwmOut( 0 );
			
			data_tuningLR( &pattern_gain, 1 );
			if ( pattern_gain == 4 ) pattern_gain = 1;
			else if ( pattern_gain == 0 ) pattern_gain = 3;
			
			switch( pattern_gain ) {
				case 1:
					// kp
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "   %2d %2d", ki_buff, kd_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp_buff, ki_buff, kd_buff);
					}
					
					data_tuningUD ( &kp_buff, 1 );
					break;
				case 2:
					// ki
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d    %2d", kp_buff, kd_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp_buff, ki_buff, kd_buff);
					}
					
					data_tuningUD ( &ki_buff, 1 );
					break;
				case 3:
					// kd
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d %2d   ", kp_buff, ki_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp_buff, ki_buff, kd_buff);
					}
					
					data_tuningUD ( &kd_buff, 1 );
					break;
			}
			break;
		//------------------------------------------------------------------
		// �Q�C������(�p�x)
		//------------------------------------------------------------------
		case 0x6:
			lcdRowPrintf(UPROW, "kp ki kd");
			
			data_select( &servo_test2, SW_PUSH );
			modeAngle = 1;
			if ( servo_test == 1 ) servoPwmOut( ServoPwm2 );
			else servoPwmOut( 0 );
			if ( servo_test2 == 1 ) SetAngle = 700;
			else SetAngle = -700;
			
			data_tuningLR( &pattern_gain2, 1 );
			if ( pattern_gain2 == 4 ) pattern_gain2 = 1;
			else if ( pattern_gain2 == 0 ) pattern_gain2 = 3;
			
			switch( pattern_gain2 ) {
				case 1:
					// kp2
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "   %2d %2d", ki2_buff, kd2_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp2_buff, ki2_buff, kd2_buff);
					}
					data_tuningUD ( &kp2_buff, 1 );
					break;
				case 2:
					// ki2
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d    %2d", kp2_buff, kd2_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp2_buff, ki2_buff, kd2_buff);
					}
					data_tuningUD ( &ki2_buff, 1 );
					break;
				case 3:
					// kd2
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d %2d   ", kp2_buff, ki2_buff);
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp2_buff, ki2_buff, kd2_buff);
					}
					data_tuningUD ( &kd2_buff, 1 );
					break;
			}
			break;
		//------------------------------------------------------------------
		// �Q�C������(���x)
		//------------------------------------------------------------------
		case 0x7:
			lcdRowPrintf(UPROW, "kp ki kd");
			
			data_tuningLR( &pattern_gain3, 1 );
			if ( pattern_gain3 == 4 ) pattern_gain3 = 1;
			else if ( pattern_gain3 == 0 ) pattern_gain3 = 3;
			
			switch( pattern_gain3 ) {
				case 1:
					// kp3
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "   %2d %2d", ki3_buff, kd3_buff );
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp3_buff, ki3_buff, kd3_buff );
					}
					data_tuningUD ( &kp3_buff, 1 );
					break;
				case 2:
					// ki3
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d    %2d", kp3_buff, kd3_buff );
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp3_buff, ki3_buff, kd3_buff );
					}
					data_tuningUD ( &ki3_buff, 1 );
					break;
					
				case 3:
					// kd3
					//�l��_��
					if ( cntSetup1 >= 500 ) cntSetup1 = 0;
					if ( cntSetup1 < 250 ) {
						lcdRowPrintf(LOWROW, "%2d %2d   ", kp3_buff, ki3_buff );
					} else {
						lcdRowPrintf(LOWROW, "%2d %2d %2d", kp3_buff, ki3_buff, kd3_buff );
					}
					data_tuningUD ( &kd3_buff, 1 );
					break;
			}
			break;
		//------------------------------------------------------------------
		// �v���Z�b�g�p�����[�^
		//------------------------------------------------------------------
		case 0x8:
			lcdRowPrintf(LOWROW, "SETTING ");
			data_tuningLR( &pattern_speedseting, 1 );
				
			if ( pattern_speedseting == 4 ) pattern_speedseting = 1;
			else if ( pattern_speedseting == 0 ) pattern_speedseting = 3;
			
			switch( pattern_speedseting ) {
				case 1:
					lcdRowPrintf(UPROW, "1METER %d", setting_1meter);
					
					data_select( &setting_1meter, SW_PUSH );
					if ( setting_1meter == 1 ) {
						fixSpeedSetting();
						setting_2meter = 0;
						setting_3meter = 0;
					} else {
						fixSpeed = 0;
						setting_2meter = 0;
						setting_3meter = 0;
					}
					break;
					
				case 2:
					lcdRowPrintf(UPROW, "2METER %d", setting_2meter);
					
					data_select( &setting_2meter, SW_PUSH );
					if ( setting_2meter == 1 ) {
						fixSpeedSetting();
						setting_1meter = 0;
						setting_3meter = 0;
					} else {
						fixSpeed = 0;
						setting_1meter = 0;
						setting_3meter = 0;
					}
					break;
					
				case 3:
					lcdRowPrintf(UPROW, "3METER %d", setting_3meter);
					
					data_select( &setting_3meter, SW_PUSH );
					if ( setting_3meter == 1 ) {
						fixSpeedSetting();
						setting_1meter = 0;
						setting_2meter = 0;
					} else {
						fixSpeed = 0;
						setting_1meter = 0;
						setting_2meter = 0;
					}
					break;
			}
			break;
		//------------------------------------------------------------------
		// Motor_test
		//------------------------------------------------------------------
		case 0x9:
			data_tuningLR( &pattern_sensor, 1 );
			modeAngle = 0;
			
			if ( pattern_sensor == 14 ) pattern_sensor = 1;
			else if ( pattern_sensor == 0 ) pattern_sensor = 13;
			
			switch( pattern_sensor ) {
				case 1:
					// LED
					lcdRowPrintf(UPROW, "LED     ");
					lcdRowPrintf(LOWROW, "        ");
					cnt_led = 0x00;
					if ( taswGet() == SW_PUSH) {
						while(1){
						while( cnt_led <= 0x7 ) {
							ledOut( cnt_led );
							cnt_led++;
							waitLcd(200);
 						}
						cnt_led = 0x00;
						}
					}
					break;
					
				case 2:
					// �W���C��
					if ( taswGet() == SW_TOP ) PichAngleIMU = 0;
					if ( taswGet() == SW_DOWN ) RollAngleIMU = 0;
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "Roll%4d", (short)RollAngleIMU);
						lcdRowPrintf(LOWROW, "Pich%4d",(short)PichAngleIMU);
					}
					break;
					
				case 3:
					// �G���R�[�_
					lcdRowPrintf(UPROW, "Encoder ");
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						// lcdRowPrintf(LOWROW, "   %4.1f",(double)EncoderTotal/PALSE_MILLIMETER);
						//lcdRowPrintf(LOWROW, "   %5d",EncoderTotal);
						lcdRowPrintf(LOWROW, "   %5d",Encoder);
					}
					break;
							
				case 4:
					// �|�e���V�������[�^�[
					lcdRowPrintf(UPROW, "Angle   ");
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(LOWROW, "   %5d",getServoAngle());
					}
					break;
					
				case 5:
					// �A�i���O�Z���T
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "R   %4d",sensorR);
						lcdRowPrintf(LOWROW, "L   %4d",sensorL);
					}
					break;
					
				case 6:
					// �f�W�^�����A�i���O�Z���T, �Q�[�g�Z���T
					motor_test = 0;
					data_tuningUD ( &sensorG_th, 1 );
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						startbar_get();
						lcdRowPrintf(UPROW, "G   %4d", sensorG);
						lcdRowPrintf(LOWROW, "D    0x%x", sensor_inp());
					}
					break;
					
				case 7:
					// ���[�^�[�e�X�g
					lcdRowPrintf(UPROW, "Motortes");
					lcdRowPrintf(LOWROW, "    %3d%%",motorTestPwm);
					demo = 0;
					data_tuningUD ( &motorTestPwm, 1 );
					if ( motor_test == 1 ) {
						// if ( cntSetup1 >= 500 ) {
							// cntSetup1 = 0;
							diff( motorTestPwm );
						// }
						// motorPwmOut(motorTestPwm, motorTestPwm, motorTestPwm, motorTestPwm);
					} else {
						motorPwmOut(0, 0, 0, 0);
					}
					
					data_select( &motor_test, SW_PUSH );
					break;
				case 8:
					// �T�[�{�e�X�g
					lcdRowPrintf(UPROW, "Servo   ");
					lcdRowPrintf(LOWROW, "        ");
					if ( motor_test == 1 ) servoPwmOut( 20 );
					else				servoPwmOut( 0 );
					
					data_select( &motor_test, SW_PUSH );
					break;
				case 9:
					// ���C���Z���T����
					lcdRowPrintf(UPROW, "gASensor");
					
					motor_test = 0;
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(LOWROW, "   %5d", getAnalogSensor());
					}
					break;
					
				case 10:
					// ����p�x
					lcdRowPrintf(UPROW, "IMU %4d", (short)TurningAngleIMU);
					lcdRowPrintf(LOWROW, "        ");
					if ( taswGet() == SW_PUSH ) TurningAngleIMU = 0;
					if ( taswGet() == SW_TOP ) caribrateIMU();
					break;
					
				case 11:
					// IMU
					if ( cntSetup1 >= 500 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "xa%2.3f",(double)xa / ACCELLSB);
						lcdRowPrintf(LOWROW, "ya%2.3f",(double)ya / ACCELLSB);
					}
					break;
					
				/*case 12:
					// IMU
					if ( cntSetup1 >= 500 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "za%2.3f",(double)za / ACCELLSB);
						lcdRowPrintf(LOWROW, "xg%2.3f",(double)xg/GYROLSB);
					}
					break;
					
				case 13:
					// IMU
					if ( cntSetup1 >= 500 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "yg%2.3f",(double)yg/GYROLSB);
						lcdRowPrintf(LOWROW, "zg%2.3f",(double)zg/GYROLSB);
					}
					break;*/
				/*case 12:
					// �A�i���O�Z���T
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "RR  %4d",sensorRR);
						lcdRowPrintf(LOWROW, "LL  %4d",sensorLL);
					}
					break;*/
				/*case 13:
					// �A�i���O�Z���T
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						//lcdRowPrintf(UPROW, "C   %4d",sensorC);
						lcdRowPrintf(UPROW, "C   %4x",sensor_inp());
					}
					break;*/	
				case 12:
					// ���C���Z���T16�i���\��
					if ( cntSetup1 >= 100 ) {
						cntSetup1 = 0;
						lcdRowPrintf(UPROW, "0x%d",sensor_inp());
					
					}
					break;			
			}
			break;
		//------------------------------------------------------------------
		// �ʒu�Œ�f��
		//------------------------------------------------------------------
		case 0xa:
			lcdRowPrintf(UPROW, "DEMO%4d",motorPwm);
			targetSpeed  = 0;
			
			data_select ( &demo, 1 );
			if ( demo == 1 ) {
				motorPwmOut(motorPwm, motorPwm, motorPwm, motorPwm);
				lcdRowPrintf(LOWROW, "   Start");
			} else {
				lcdRowPrintf(LOWROW, "    Stop");
			}
			break;
		//------------------------------------------------------------------
		// MicroSD
		//------------------------------------------------------------------
		case 0xb:
			GET_SDSWITCH;
			lcdRowPrintf(UPROW, "MicroSD%d",sd_sw);
			
			servo_test = 0;
			modeAngle = 0;
			data_tuningLR( &pattern_msd, 1 );
			if ( pattern_msd == 13 ) pattern_msd = 1;
			else if ( pattern_msd == 0 ) pattern_msd = 12;
			
			switch ( pattern_msd ) {
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
				case 8:
				case 9:
				case 10:
					msdWorkaddress = msdaddrBuff[pattern_msd*2-1];
					msdWorkaddress2 = msdaddrBuff[pattern_msd*2-2];
					if ( msdWorkaddress == 0 && msdWorkaddress2 == 0 ) {
						lcdRowPrintf(LOWROW, "Nodata%2d",pattern_msd);
					} else {
						lcdRowPrintf(LOWROW, "data%2d  ",pattern_msd);
						if ( taswGet() == SW_PUSH ) {
							ledOut(LED_R);	// ���O���M�I���ʒm(��LED�_��)
							msd_sendToPC();	
							ledOut(LED_G);	// ���O���M�I���ʒm(��LED�_��)
							i = 1;
							cntSetup1 = 0;
						}
						
						if ( i = 1 && cntSetup1 >= 1500 ) {
							ledOut(0);	// 1.5�b�ŏI���ʒm���I���
						}
					}
					
					break;
					
				case 11:
				// ���O�L�^
					if ( msdFlag == 0 ) { 
						lcdRowPrintf(LOWROW, "LogWrite");
					}
					if ( taswGet() == SW_TOP && push1 == 0 && msdFlag == 0) {
						push1 = 1;
						readFlashSetup( 0, 0, 1 ,0 ,0 ,0 ,0);
						init_log();	// ���O�L�^����
						msdFlag = 1;		// �f�[�^�L�^�J�n
						lcdRowPrintf(LOWROW, "Logging ");
					} else if ( taswGet() == SW_DOWN && push1 == 0 && msdFlag == 1) {
						push1 = 1;
						msdEndLog();		// MicroSD�̏I������
					} else if ( taswGet() == 0x0 ) {
						push1 = 0;
					}
					break;
					
				case 12:
				// �R�[�X���
					if ( msdFlag == 0 ) { 
						lcdRowPrintf(LOWROW, "LogRead ");
					}
					if ( taswGet() == SW_PUSH && push1 == 0 && msdFlag == 0) {
						//���O���
						msdgetData () ;
					} else if ( taswGet() == 0x0 ) {
						push1 = 0;
					}
					break;
			}
			break;
		//------------------------------------------------------------------
		// �L�����u���[�V����
		//------------------------------------------------------------------
		case 0xc:
			lcdRowPrintf(UPROW, "Angle0  ");
			lcdRowPrintf(LOWROW, "    %4d", Angle0);
			
			flashDataBuff[0] = Angle0;
			data_tuningUD ( &Angle0, 1 );
			data_select( &servo_test, SW_PUSH );
			modeAngle = 1;
			if ( servo_test == 1 ) {
				SetAngle = 0;
				servoPwmOut( ServoPwm2 );
			} else {
				servoPwmOut( 0 );
			}
			if ( taswGet() == 0x4 && push1 == 0 ) {
				push1 = 1;
				writeFlashData( ANGLE0_STARTAREA, ANGLE0_ENDAREA, ANGLE0_DATA, 1 );
			} else if ( taswGet() == 0x0 ) {
				push1 = 0;
			}
			break;
		//------------------------------------------------------------------
		// �t���b�V��
		//------------------------------------------------------------------
		case 0xd:
			switch( pattern_flash ) {
				case 1:
					lcdRowPrintf(UPROW, "Flash   ");
					lcdRowPrintf(LOWROW, "AllErase");
					
					i = 0;
					data_select( &i, SW_PUSH );
					if ( i ) {
						pattern_flash = 2;
						break;
					}
					break;
				case 2:
					lcdRowPrintf(UPROW, "Really? ");
					lcdRowPrintf(LOWROW, "Y:<  N:>");

					i = 0;
					data_tuningLR( &i, 1 );
					if ( i > 0 ) {
						pattern_flash = 1;
						break;
					} else if ( i < 0 ) {
						pattern_flash = 3;
						break;
					}
					break;
				case 3:
					lcdRowPrintf(UPROW, "AllErase");
					
					i = 0;
					while ( i <= 1023 ) {
						lcdRowPrintf(LOWROW, "    %4d", i);
						eraseE2DataFlash( i );
						i++;
					}
					cntSetup1 = 0;
					pattern_flash = 4;
					break;
				case 4:
					lcdRowPrintf(LOWROW, "complete");
					if ( cntSetup1 >= 1500 ) {
						pattern_flash = 1;
						break;
					}
					break;
			}
			break;

	default:
		lcdRowPrintf(UPROW, "%#x     ", dipswGet());
		lcdRowPrintf(LOWROW, "none    ");
		
		servo_test = 0;
		break;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// ���W���[���� data_select
// �����T�v     �^�N�g�X�C�b�`��0,1�ɕω�������
// ����         data: �ω�������ϐ� button: �ǂ̃X�C�b�`�ŕω������邩
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////////////////
void data_select ( char *data , char button )
{
	if ( taswGet() == button ) {
		if ( *data == 1 && push == 0) {
			push = 1;
			*data = 0;
		} else if ( *data == 0 && push == 0) {
			push = 1;
			*data = 1;
			Int = 0;
		}
	} else {
		push = 0;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// ���W���[���� data_tuningUD
// �����T�v     �^�N�g�X�C�b�`��data����������
// ����         data: ����������ϐ� add: 0: �ω��� dir: 0:�㉺ 1:���E
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////////////////
void data_tuningUD ( void *data, char add )
{
	short *data2 = (short*)data;	// short�^�|�C���^�ɃL���X�g
	
	if ( cntSetup2 >= 50 ) {
		if ( taswGet() == SW_TOP || taswGet() == SW_DOWN ) {
			cntSwitchUD++; // ���������Ԍv��
			if ( taswGet() == SW_TOP  ) {
				// �C���N�������g
				if ( cntSwitchUD >= 20 ) {	// ����������
					*data2 += add;
				} else if (pushUD == 0) {	// 1�񉟂�����
					pushUD = 1;
					*data2 += add;
				}
			} else if ( taswGet() == SW_DOWN  ) {
				// �f�N�������g
				if ( cntSwitchUD >= 20 ) {	// ����������
					*data2 -= add;
				} else if (pushUD == 0) {	// 1�񉟂�����
					pushUD = 1;
					*data2 -= add;
				}
			}
		} else {
			pushUD = 0;
			cntSwitchUD = 0;
		}
		cntSetup2 = 0;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// ���W���[���� data_tuningUD
// �����T�v     �^�N�g�X�C�b�`��data����������
// ����         data: ����������ϐ� add: 0: �ω��� dir: 0:�㉺ 1:���E
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////////////////
void data_tuningLR ( void *data, char add )
{
	short *data2 = (short*)data;	// short�^�|�C���^�ɃL���X�g

	if ( cntSetup3 >= 50 ) {
		if ( taswGet() == SW_LEFT || taswGet() == SW_RIGHT ) {
			cntSwitchLR++; // ���������Ԍv��
			if ( taswGet() == SW_RIGHT  ) {
				// �C���N�������g
				if ( cntSwitchLR >= 20 ) {	// ����������
					*data2 += add;
				} else if (pushLR == 0) {	// 1�񉟂�����
					pushLR = 1;
					*data2 += add;
				}
			} else if ( taswGet() == SW_LEFT  ) {
				// �f�N�������g
				if ( cntSwitchLR >= 20 ) {	// ����������
					*data2 -= add;
				} else if (pushLR == 0) {	// 1�񉟂�����
					pushLR = 1;
					*data2 -= add;
				}
			}
		} else {
			pushLR = 0;
			cntSwitchLR = 0;
		}
		cntSetup3 = 0;
	}
}
/////////////////////////////////////////////////////////////////////////////////
// ���W���[���� fixSpeedSetting
// �����T�v     ���x�p�����[�^���Œ�l�ɂ���
// ����         �Ȃ�
// �߂�l       0: ���x���ɂ��Ȃ��@1: ���x���ɂ���
/////////////////////////////////////////////////////////////////////////////////
char fixSpeedSetting ( void )
{
	char ret = 0;
	
	if ( setting_1meter == 1 ) {
		speed_straight		= 10;
		speed_curve_brake	= 10;
		speed_curve_r600	= 10;
		speed_curve_r450	= 10;
		speed_curve_straight	= 10;
		
		speed_crossline		= 10;
		speed_ckank_trace	= 10;
		speed_rightclank_curve	= 10;
		speed_rightclank_escape	= 10;
		speed_leftclank_curve	= 10;
		speed_leftclank_escape	= 10;
		
		speed_halfine		= 10;
		speed_rightchange_trace = 10;
		speed_rightchange_curve	= 10;
		speed_rightchange_escape= 10;
		speed_leftchange_trace 	= 10;
		speed_leftchange_curve	= 10;
		speed_leftchange_escape	= 10;
		
		speed_slope_brake	= 10;
		speed_slope_trace	= 10;
		
		fixSpeed = 1;
		ret = 1;
	} else if ( setting_2meter == 1 ) {
		speed_straight		= 20;
		speed_curve_brake	= 20;
		speed_curve_r600	= 20;
		speed_curve_r450	= 20;
		speed_curve_straight	= 20;
		
		speed_crossline		= 20;
		speed_ckank_trace	= 16;
		speed_rightclank_curve	= 16;
		speed_rightclank_escape	= 20;
		speed_leftclank_curve	= 16;
		speed_leftclank_escape	= 20;
		
		speed_halfine		= 20;
		speed_rightchange_trace = 20;
		speed_rightchange_curve	= 20;
		speed_rightchange_escape= 20;
		speed_leftchange_trace 	= 20;
		speed_leftchange_curve	= 20;
		speed_leftchange_escape	= 20;
		
		speed_slope_brake	= 20;
		speed_slope_trace	= 20;
		
		fixSpeed = 1;
		ret = 1;
	} else if ( setting_3meter == 1 ) {
		speed_straight		= 30;
		speed_curve_brake	= 30;
		speed_curve_r600	= 30;
		speed_curve_r450	= 30;
		speed_curve_straight	= 30;
		
		speed_crossline		= 25;
		speed_ckank_trace	= 20;
		speed_rightclank_curve	= 20;
		speed_rightclank_escape	= 30;
		speed_leftclank_curve	= 20;
		speed_leftclank_escape	= 30;
		
		speed_halfine		= 30;
		speed_rightchange_trace = 30;
		speed_rightchange_curve	= 30;
		speed_rightchange_escape= 30;
		speed_leftchange_trace 	= 30;
		speed_leftchange_curve	= 30;
		speed_leftchange_escape	= 30;
		
		speed_slope_brake	= 20;
		speed_slope_trace	= 30;
		
		fixSpeed = 1;
		ret = 1;
	}
	
	return ret;
}