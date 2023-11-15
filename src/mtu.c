//====================================//
// インクルード
//====================================//
#include "mtu.h"
//====================================//
// グローバル変数の宣言
//====================================//
// エンコーダ関連
static unsigned short 	cnt_Encoder;	// エンコーダ値の格納先
static unsigned short	encbuff;		// 前回のエンコーダ値
short					Encoder;			// 1msごとのパルス
short					speed_enc;
short					currentSpeed;
unsigned int			EncoderTotal;		// 総走行距離
unsigned int			enc1;				// 走行用距離カウント
unsigned int			enc_slope;			// 坂上距離カウント
// モーター関連
signed char		accele_fR;		// 右前モーターPWM値
signed char		accele_fL;		// 左前モーターPWM値
signed char		accele_rR;		// 右後モーターPWM値
signed char		accele_rL;		// 左後モーターPWM値
signed char		sPwm;			// サーボモーターPWM値
/////////////////////////////////////////////////////////////////////////////////
// モジュール名 getEncoder
// 処理概要     エンコーダのカウントを取得し積算する(1msごとに実行)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////
void getEncoder (void)
{

	ENCODER_COUNT			// エンコーダカウント値取得
	Encoder = cnt_Encoder - encbuff;// 現在地から1ms前の値を引いて1ms間のカウントを計算
	currentSpeed = ((Encoder*10000)/PALSE_METER);
	// 積算
	
	EncoderTotal += Encoder;
	enc1 += Encoder;
	enc_slope += Encoder;
	
	encbuff = cnt_Encoder;	// 次回はこの値が1ms前の値となる
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 motor_f
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
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
	
	// 左前輪
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// 右前輪
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	// 左後輪
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// 右後輪
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 servoPwmOut
// 処理概要     白線トレース時サーボのPWMの変更
// 引数         spwm
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void servoPwmOut( signed char servopwm )
{
	uint16_t pwm;
	short angle;
	
	sPwm = servopwm;		// ログ用変数に代入
	//servopwm = -servopwm;		// 回転方向を変える
	
	// サーボリミット制御
	angle = getServoAngle();
	
	// 角度によるリミット制御
	if ( angle >= SERVO_LIMIT ) servopwm = -15;
	if ( angle <= -SERVO_LIMIT ) servopwm = 15;
	
	// ポテンションメーターが外れていたら制御しない
	if ( angle > SERVO_LIMIT + 100 ) servopwm = 0;
	if ( angle < -SERVO_LIMIT - 100 ) servopwm = 0;

	pwm = (uint16_t)TGR_SERVO * servopwm / 100;
	// サーボモータ制御
	if( servopwm > 0) {				
		// 正転
		DIR_SERVO_FOR
	} else {				
		// 逆転
		pwm = -pwm;
		DIR_SERVO_REV
	}
	PWM_SERVO_OUT
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor2_f
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
///////////////////////////////////////////////////////////////////////////
void motor2_f( signed char accelefL, signed char accelefR )
{
	uint16_t pwmfl, pwmfr;
	
	accele_fR = accelefR;
	accele_fL = accelefL;
		
	pwmfl = TGR_MOTOR * accelefL / 100;
	pwmfr = TGR_MOTOR * accelefR / 100;
	
		
	// 左前輪
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// 右前輪
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor2_r
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
///////////////////////////////////////////////////////////////////////////
void motor2_r( signed char accelerL, signed char accelerR )
{
	uint16_t pwmrl, pwmrr;
	
	accele_rR = accelerR;
	accele_rL = accelerL;
	
	pwmrl = TGR_MOTOR * accelerL / 100;
	pwmrr = TGR_MOTOR * accelerR / 100;
	
	// 左後輪
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// 右後輪
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor3_f
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
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
	
		
	// 左前輪
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// 右前輪
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor3_r
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
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
	
	// 左後輪
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// 右後輪
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor6_f
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
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
	
		
	// 左前輪
	if( accelefL >= 0) DIR_FL_FOR
	else{
		pwmfl = -pwmfl;
		DIR_FL_REV
	}
	if ( accelefL == 100 || accelefL == -100 )pwmfl = TGR_MOTOR + 2;
	PWM_FL_OUT
	
	// 右前輪
	if( accelefR >= 0) DIR_FR_FOR
	else{
		pwmfr = -pwmfr;
		DIR_FR_REV
	}
	if ( accelefR == 100 || accelefR == -100 ) pwmfr = TGR_MOTOR + 2;
	PWM_FR_OUT

	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motor6_r
// 処理概要     モーターのPWMの変更
// 引数         accelefL, accelefR(PWMを1～100%で指定)
// 戻り値       な
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
	
	// 左後輪
	if( accelerL >= 0 ) DIR_RL_FOR
	else{
		pwmrl = -pwmrl;
		DIR_RL_REV
	}
	if ( accelerL == 100 || accelerL == -100 ) pwmrl = TGR_MOTOR + 2;
	PWM_RL_OUT
	
	// 右後輪
	if( accelerR >= 0 ) DIR_RR_FOR
	else{
		pwmrr = -pwmrr;
		DIR_RR_REV
	}
	if ( accelerR == 100 || accelerR == -100 ) pwmrr = TGR_MOTOR + 2;
	PWM_RR_OUT
}
/************************************************************************/
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
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
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
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