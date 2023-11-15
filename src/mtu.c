//====================================//
// �C���N���[�h
//====================================//
#include "mtu.h"
//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
// �G���R�[�_�֘A
static unsigned short 	cnt_Encoder;	// �G���R�[�_�l�̊i�[��
static unsigned short	encbuff;		// �O��̃G���R�[�_�l
short					Encoder;			// 1ms���Ƃ̃p���X
short					speed_enc;
short					currentSpeed;
unsigned int			EncoderTotal;		// �����s����
unsigned int			enc1;				// ���s�p�����J�E���g
unsigned int			enc_slope;			// ��㋗���J�E���g
// ���[�^�[�֘A
signed char		accele_fR;		// �E�O���[�^�[PWM�l
signed char		accele_fL;		// ���O���[�^�[PWM�l
signed char		accele_rR;		// �E�ヂ�[�^�[PWM�l
signed char		accele_rL;		// ���ヂ�[�^�[PWM�l
signed char		sPwm;			// �T�[�{���[�^�[PWM�l
/////////////////////////////////////////////////////////////////////////////////
// ���W���[���� getEncoder
// �����T�v     �G���R�[�_�̃J�E���g���擾���ώZ����(1ms���ƂɎ��s)
// ����         �Ȃ�
// �߂�l       �Ȃ�
/////////////////////////////////////////////////////////////////////////////////
void getEncoder (void)
{

	ENCODER_COUNT			// �G���R�[�_�J�E���g�l�擾
	Encoder = cnt_Encoder - encbuff;// ���ݒn����1ms�O�̒l��������1ms�Ԃ̃J�E���g���v�Z
	currentSpeed = ((Encoder*10000)/PALSE_METER);
	// �ώZ
	
	EncoderTotal += Encoder;
	enc1 += Encoder;
	enc_slope += Encoder;
	
	encbuff = cnt_Encoder;	// ����͂��̒l��1ms�O�̒l�ƂȂ�
}

///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor_f
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motorPwmOut( signed char accelefL, signed char accelefR, signed char accelerL, signed char accelerR )
{
	uint16_t pwmfl, pwmfr, pwmrl, pwmrr;
	
	accele_fR = accelefR;
	accele_fL = accelefL;
	accele_rR = accelerR;
	accele_rL = accelerL;
	
	pwmfl = TGR_MOTOR * accelefL / 100;
	pwmfr = TGR_MOTOR * accelefR / 100;
	pwmrl = TGR_MOTOR * accelerL / 100;
	pwmrr = TGR_MOTOR * accelerR / 100;
	
	// ���O��
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// �E�O��
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	// �����
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// �E���
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� servoPwmOut
// �����T�v     �����g���[�X���T�[�{��PWM�̕ύX
// ����         spwm
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void servoPwmOut( signed char servopwm )
{
	uint16_t pwm;
	short angle;
	
	sPwm = servopwm;		// ���O�p�ϐ��ɑ��
	//servopwm = -servopwm;		// ��]������ς���
	
	// �T�[�{���~�b�g����
	angle = getServoAngle();
	
	// �p�x�ɂ�郊�~�b�g����
	if ( angle >= SERVO_LIMIT ) servopwm = -15;
	if ( angle <= -SERVO_LIMIT ) servopwm = 15;
	
	// �|�e���V�������[�^�[���O��Ă����琧�䂵�Ȃ�
	if ( angle > SERVO_LIMIT + 100 ) servopwm = 0;
	if ( angle < -SERVO_LIMIT - 100 ) servopwm = 0;

	pwm = (uint16_t)TGR_SERVO * servopwm / 100;
	// �T�[�{���[�^����
	if( servopwm > 0) {				
		// ���]
		DIR_SERVO_FOR
	} else {				
		// �t�]
		pwm = -pwm;
		DIR_SERVO_REV
	}
	PWM_SERVO_OUT
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor2_f
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor2_f( signed char accelefL, signed char accelefR )
{
	uint16_t pwmfl, pwmfr;
	
	accele_fR = accelefR;
	accele_fL = accelefL;
		
	pwmfl = TGR_MOTOR * accelefL / 100;
	pwmfr = TGR_MOTOR * accelefR / 100;
	
		
	// ���O��
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// �E�O��
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor2_r
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor2_r( signed char accelerL, signed char accelerR )
{
	uint16_t pwmrl, pwmrr;
	
	accele_rR = accelerR;
	accele_rL = accelerL;
	
	pwmrl = TGR_MOTOR * accelerL / 100;
	pwmrr = TGR_MOTOR * accelerR / 100;
	
	// �����
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// �E���
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor3_f
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor3_f( int setsp, signed char accelefL, signed char accelefR )
{
	uint16_t pwmfl, pwmfr;
	
	if( currentSpeed >= ( setsp + 13 ) ){
		accelefL = -10;
		accelefR = -10;
	}else if(  currentSpeed >= ( setsp + 9 ) ){
		accelefL = -7;
		accelefR = -7;
	}else if(  currentSpeed >= ( setsp + 4 ) ){
		accelefL = -5;
		accelefR = -5;
	}else if(  currentSpeed >= setsp ){
		accelefL = 0;
		accelefR = 0;
	}else{
	}
		
	
	accele_fR = accelefR;
	accele_fL = accelefL;
		
	pwmfl = TGR_MOTOR * accelefL / 100;
	pwmfr = TGR_MOTOR * accelefR / 100;
	
		
	// ���O��
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// �E�O��
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor3_r
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor3_r( int setsp, signed char accelerL, signed char accelerR )
{
	uint16_t pwmrl, pwmrr;
	
	if( currentSpeed >= ( setsp + 13 ) ){
		accelerL = -10;
		accelerR = -10;
	}else if(  currentSpeed >= ( setsp + 9 ) ){
		accelerL = -7;
		accelerR = -7;
	}else if(  currentSpeed >= ( setsp + 4 ) ){
		accelerL = -5;
		accelerR = -5;
	}else if(  currentSpeed >= setsp ){
		accelerL = 0;
		accelerR = 0;
	}else{
	}	
	
	accele_rR = accelerR;
	accele_rL = accelerL;
	
	pwmrl = TGR_MOTOR * accelerL / 100;
	pwmrr = TGR_MOTOR * accelerR / 100;
	
	// �����
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// �E���
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor6_f
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor6_f( int setsp, signed char accelefL, signed char accelefR )
{
	uint16_t pwmfl, pwmfr;
	
	if( currentSpeed >= ( setsp + 5 ) ){
		accelefL = -80;
		accelefR = -80;
	}else if(  currentSpeed >= ( setsp + 3 ) ){
		accelefL = -50;
		accelefR = -50;
	}else if(  currentSpeed >= ( setsp + 1 ) ){
		accelefL = -30;
		accelefR = -30;
	}else if(  currentSpeed >= setsp ){
		accelefL = 30;
		accelefR = 30;
	}else{
	}
		
	
	accele_fR = accelefR;
	accele_fL = accelefL;
		
	pwmfl = TGR_MOTOR * accelefL / 100;
	pwmfr = TGR_MOTOR * accelefR / 100;
	
		
	// ���O��
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// �E�O��
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� motor6_r
// �����T�v     ���[�^�[��PWM�̕ύX
// ����         accelefL, accelefR(PWM��1�`100%�Ŏw��)
// �߂�l       ��
///////////////////////////////////////////////////////////////////////////
void motor6_r( int setsp, signed char accelerL, signed char accelerR )
{
	uint16_t pwmrl, pwmrr;
	
	if( currentSpeed >= ( setsp + 5 ) ){
		accelerL = -80;
		accelerR = -80;
	}else if(  currentSpeed >= ( setsp + 3 ) ){
		accelerL = -50;
		accelerR = -50;
	}else if(  currentSpeed >= ( setsp + 1 ) ){
		accelerL = -30;
		accelerR = -30;
	}else if(  currentSpeed >= setsp ){
		accelerL = 30;
		accelerR = 30;
	}else{
	}	
	
	accele_rR = accelerR;
	accele_rL = accelerL;
	
	pwmrl = TGR_MOTOR * accelerL / 100;
	pwmrr = TGR_MOTOR * accelerR / 100;
	
	// �����
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// �E���
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    uint16_t pwmrl, pwmrr;
    if( mode_l ) {
        pwmrl = 0;
	PWM_RL_OUT
	SR_RL_OFF
    } else {
        pwmrl = 0;
	PWM_RL_OUT
	SR_RL_ON
    }
        if( mode_r ) {
        pwmrr = 0;
	PWM_RR_OUT
	SR_RR_OFF
    } else {
        pwmrr = 0;
	PWM_RR_OUT
	SR_RR_ON
    }
}
/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    uint16_t pwmfl, pwmfr;
    if( mode_l ) {
        pwmfl = 0;
	PWM_FL_OUT
	SR_FL_OFF
    } else {
        pwmfl = 0;
	PWM_FL_OUT
	SR_FL_ON
    }
        if( mode_r ) {
        pwmfr = 0;
	PWM_FR_OUT
	SR_FR_OFF
    } else {
        pwmfr = 0;
	PWM_FR_OUT
	SR_FR_ON
    }
}