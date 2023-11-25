//====================================//
// インクルード
//====================================//
#include "control.h"
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
char 	modeLCD = 1;		// LCD表示可否		1:表示		0:消灯		
char 	modeSlope;			// 坂チェック		0:上り坂始め	1:上り坂終わり	2:下り坂始め
char 	modeAngle;			// サーボPWM変更	0:白線トレース	1:角度制御
char	modePushcart;		// 手押しモード可否	0:自動走行	1:手押し
char	msdset;				// MicroSDが初期化されたか	0:初期化失敗	1:初期化成功
char	IMUSet = 0;			// IMUが初期化されたか		0: 初期化失敗	1:初期化成功

// パラメータ関連
// 距離
short	stopping_meter;			// 停止距離
// 速度
short	speed_straight;			// 通常トレース
short	speed_curve_brake;		// カーブブレーキ
short	speed_curve_r600;		// R600カーブ速度
short	speed_curve_r450;		// R450カーブ速度
short	speed_curve_straight;		// S字カーブ直線速度

short	speed_crossline;			// クロスライン進入速度
short	speed_ckank_trace;			// クランク進入速度
short	speed_rightclank_curve;		// 右クランク旋回速度
short	speed_rightclank_escape;	// 右クランク復帰速度
short	speed_leftclank_curve;		// 左クランク旋回速度
short	speed_leftclank_escape;		// 左クランク復帰速度

short	speed_halfine;				// ハーフライン進入速度
short	speed_rightchange_trace;	// 右レーンチェンジ進入速度
short	speed_rightchange_curve;	// 右レーンチェンジ旋回速度
short	speed_rightchange_escape;	// 右レーンチェンジ復帰速度

short	speed_leftchange_trace;		// 左レーンチェンジ進入速度
short	speed_leftchange_curve;		// 左レーンチェンジ旋回速度
short	speed_leftchange_escape;	// 左レーンチェンジ旋回速度

short	speed_slope_brake;			// 下り坂終点速度
short	speed_slope_trace;			// 坂読み飛ばし速度

// サーボ角度
short	angle_rightclank;		// 右クランク旋回角度
short	angle_leftclank;		// 左クランク旋回角度
short	angle_rightchange;		// 右レーンチェンジ旋回角度
short	angle_leftchange;		// 左レーンチェンジ旋回角度

// タイマ関連
short		cntGyro;			// 角度計算用カウンタ

// 角度関連
double 		TurningAngleEnc;	// エンコーダから求めた旋回角度
double		PichAngleAD;		// アナログジャイロから求めたピッチ角度
double		gyVoltageBefore;

// サーボ関連
// 白線トレース
signed char	ServoPwm;		// 白線トレースサーボPWM
short 		SensorBefore;	// 1ms前のセンサ値
char		DevBefore;		// I成分リセット用
double		Int;			// I成分積算値(白線トレース)
// 角度制御
signed char	ServoPwm2;		// 角度サーボPWM
short 		SetAngle;		// 目標角度
short		SetAngleBefore;	// 1ms前の目標角度
short 		AngleBefore2;	// 1ms前の角度
char		AngleBefore3;	// I成分リセット用
double		Int2;			// I成分積算値(角度制御)
int 		Angle_fixed;	//マーカー読み飛ばし用固定角度

// モーター関連
signed char 	motorPwm;			// モーター制御PWM
short			targetSpeed;		// 目標速度
bool			stableSpeed = false;
unsigned int	encStable = 0;
short			cntStable = 0;
char 			AccelefBefore;		// I成分リセット用
short			EncoderBefore;		// 1ms前の速度
int 			targetSpeedBefore;	// 1ms前の目標速度	
double 			Int3;				// I成分積算値(速度制御)

// デモ関連
char 	demo;

// ゲイン関連
char	kp_buff, ki_buff, kd_buff;
char	kp2_buff, ki2_buff, kd2_buff;
char 	kp3_buff, ki3_buff, kd3_buff;
char 	kp4_buff, ki4_buff, kd4_buff;
char kp3, ki3, kd3;
//basicRunPower
float   r450fin_buff,r450fout_buff;
float   r450rin_buff,r450rout_buff;
float   r600fin_buff,r600fout_buff;
float   r600rin_buff,r600rout_buff;

// ライン検知
volatile unsigned char cnt_crossline;
volatile unsigned char cnt_rightline;
volatile unsigned char cnt_leftline;
volatile int line;
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkLine
// 処理概要     ライン検知
// 引数         なし
// 戻り値       0:ラインなし 1:クロスライン 2:右ハーフライン 3:左ハーフライン
///////////////////////////////////////////////////////////////////////////
int checkLine( void ){
	line = 0;

	if( check_crossline() ) {       	/* クロスラインチェック     */
		cnt_crossline++;
		
		if( cnt_crossline >= 2 ){
			cnt_crossline = 0;
			Angle_fixed = getServoAngle(); //直前の進入角度を記録
			modeAngle = 1;
			SetAngle = Angle_fixed;
			line = 1;
		}
		
	}else{
		cnt_crossline = 0;
	}
		
		
	if( check_rightline() ) {  		    /* 右ハーフラインチェック   */
		cnt_rightline++;
			
		if( cnt_rightline >= 2 ){
			cnt_rightline = 0;
			Angle_fixed = getServoAngle(); //直前の進入角度を記録
			modeAngle = 1;
			SetAngle = Angle_fixed;
			line = 2;
		}
		
	}else{
		cnt_rightline = 0;
	}
		
	if( check_leftline() ) {        	/* 左ハーフラインチェック   */
		cnt_leftline++;
			
		if( cnt_leftline >= 2 ){
			cnt_leftline = 0;
			Angle_fixed = getServoAngle(); //直前の進入角度を記録
			modeAngle = 1;
			SetAngle = Angle_fixed;
			line = 3;
		}
		
	}else{
		cnt_leftline = 0;
	}
	
    return line;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkCrossLine
// 処理概要     クロスライン検知
// 引数         なし
// 戻り値       0:クロスラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
bool check_crossline( void )
{
	if ( sensor_inp(MASK11111) == 0x1f || sensor_inp(MASK11111) == 0x15 || sensor_inp(MASK11111) == 0x17 ){
		return true;
	}else 	return false;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkRightLine
// 処理概要     右ハーフライン検出処理
// 引数         なし
// 戻り値       0:右ハーフラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
bool check_rightline( void )
{
	
	if ( sensor_inp(MASK11111) == 0x7 || sensor_inp(MASK11111) == 0x3 ){
		return true;
	}else 	return false;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkLeftLine
// 処理概要     左ハーフライン検出処理
// 引数         なし
// 戻り値       0:左ハーフラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
bool check_leftline( void )
{	
	if ( sensor_inp(MASK11111) == 0x1C || sensor_inp(MASK11111) == 0x18 ){
		return true;
	}else 	return false;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkCrossLine
// 処理概要     クロスライン検知
// 引数         なし
// 戻り値       0:クロスラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
/*int checkCrossLine( void )
{
    volatile unsigned char b;
    volatile int ret;

    ret = 0;
    b = sensor_inp(MASK10101);
    if( b==0x15 ) {
        ret = 1;
    }
	b = sensor_inp(MASK11111);
    if( b==0x1f ) {
        ret = 1;
    }
    return ret;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkRightLine
// 処理概要     右ハーフライン検出処理
// 引数         なし
// 戻り値       0:右ハーフラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
int checkRightLine( void )
{
    volatile unsigned char b;
    volatile int ret;
    	
    ret = 0;
    b = sensor_inp( MASK00101 );
    if( b == 0x05 ) {
	//servoPwmOut(0);
        ret = 1;
    }
    return ret;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkLeftLine
// 処理概要     左ハーフライン検出処理
// 引数         なし
// 戻り値       0:左ハーフラインなし 1:あり
///////////////////////////////////////////////////////////////////////////
int checkLeftLine( void )
{
    volatile unsigned char b;
    volatile int ret;
    
    ret = 0;
    b = sensor_inp( MASK10100 );
    if( b==0x14 ) {
	//servoPwmOut(0);
        ret = 1;
    }
    return ret;
}*/
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkSlope
// 処理概要     ジャイロセンサの値から坂道検出
// 引数         なし
// 戻り値       0:坂道なし 1:上り坂　-1:下り坂
///////////////////////////////////////////////////////////////////////////
signed char checkSlope( void )
{
	signed char ret = 0;

	if ( PichAngleIMU <= SLOPE_UPPERLINE_IMU ) ret = 1;
	if ( PichAngleIMU >= SLOPE_LOWERLINE_IMU ) ret = -1;
	
	return ret;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 Slope_Observer
// 処理概要     ジャイロセンサの値から坂道検出
// 引数         なし
// 戻り値       0:坂道なし 1:上り坂　-1:下り坂
///////////////////////////////////////////////////////////////////////////
void Slope_Observer ( void )
{

	if ( PichAngleIMU <= SLOPE_UPPERLINE_IMU ) ret = 1;
	if ( PichAngleIMU >= SLOPE_LOWERLINE_IMU ) ret = -1;
	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 encMM
// 処理概要     mmをエンコーダのパルス数に変換して返す
// 引数         mm:変換する長さ[mm]
// 戻り値       変換したパルス数
///////////////////////////////////////////////////////////////////////////
unsigned int encMM( short mm )
{
	return PALSE_MILLIMETER * mm;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 stableSpeedDistance
// 処理概要     目標速度に達してからの時間と距離を計測する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
unsigned int stableSpeedDistance( void )
{
	if ( Encoder*10 >= targetSpeed ) stableSpeed = true;
	if (stableSpeed) {
		encStable += Encoder;
		cntStable++;
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 servoControlTrace
// 処理概要     ライントレース時サーボのPWMの計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void servoControlTrace( void )
{
	int iP, iD, iI, iRet, maxpwm;
	short Dev, Dif;
	
	//サーボモータ用PWM値計算
	Dev = getAnalogSensor();
	// I成分積算
	Int += (double)Dev * 0.001;
	if ( Int > 10000 ) Int = 10000;		// I成分リミット
	else if ( Int < -10000 ) Int = -10000;
	Dif = ( Dev - SensorBefore ) * 1;	// dゲイン1/1000倍

	iP = (int)kp_buff * Dev;		// 比例
	iI = (double)ki_buff * Int;		// 積分
	iD = ((int)kd_buff * 10 ) * Dif;		// 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 10;				// PWMを0～100近傍に収める

	// PWMの上限の設定
	// 出力電圧がVOLTAGELIMとなるDuty比を計算
	maxpwm = (int8_t)(VOLTAGELIMTRACE / Voltage *100);

	if ( iRet > maxpwm ) iRet =  maxpwm;
	if ( iRet < -maxpwm ) iRet = -maxpwm;

	if ( iRet >  100 ) iRet =  100;
	if ( iRet <  -100 ) iRet = -100;
	
	if ( Dev >= 0 )	DevBefore = 0;
	else			DevBefore = 1;
	ServoPwm = iRet;
	SensorBefore = Dev;				// 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 servoControlAngle
// 処理概要     角度制御時サーボのPWMの計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void servoControlAngle( void )
{
	short i, j, Dev, Dif;
	int iP, iD, iI, iRet, maxpwm;
	
	// 目標値、現在値取得
	i = SetAngle;
	j = getServoAngle();
	
	//サーボモータ用PWM値計算
	Dev = i - j;
		
	// 目標値を超えたらI成分リセット
	if ( Dev >= 0 && AngleBefore3 == 1 ) Int2 = 0;
	else if ( Dev < 0 && AngleBefore3 == 0 ) Int2 = 0;
	
	// 目標値を変更したらI成分リセット
	if ( !(i == SetAngleBefore) ) Int2 = 0;
	
	Int2 += (double)Dev * 0.001;
	Dif = ( Dev - AngleBefore2 ) * 1;		// dゲイン1/1000倍

	iP = (int)kp2_buff * Dev;		// 比例
	iI = (double)ki2_buff * Int2;	// 積分
	iD = (int)kd2_buff * Dif;		// 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 4;		// PWMを0～100近傍に収める

	// PWMの上限の設定
	// 出力電圧がVOLTAGELIMとなるDuty比を計算
	maxpwm = (int8_t)(VOLTAGELIMTRACE / Voltage *100);

	if ( iRet > maxpwm ) iRet =  maxpwm;
	if ( iRet < -maxpwm ) iRet = -maxpwm;
	if ( iRet >  100 ) iRet =  100;
	if ( iRet <  -100 ) iRet = -100;

	if ( Dev >= 0 ) 	AngleBefore3 = 0;
	else 				AngleBefore3 = 1;
	SetAngleBefore = i;
	ServoPwm2 = iRet;
	AngleBefore2 = Dev;			// 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 diff
// 処理概要   	R1,R2,R3及びR4の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void diff ( signed char pwm )
{
	const char rev_difference_D[] = {       // 角度から内輪、外輪回転差計算
			100,100,100,
			98,99,100,
			97,97,100,
			95,96,100,
			94,94,100,
			92,93,100,
			91,92,99,
			89,90,99,
			88,89,99,
			86,88,99,
			84,87,99,
			83,85,98,
			81,84,98,
			80,83,98,
			78,82,97,
			76,81,97,
			75,80,97,
			73,79,96,
			72,77,96,
			70,76,95,
			69,75,95,
			67,74,94,
			65,73,94,
			64,72,93,
			62,71,93,
			61,70,92,
			59,69,92,
			58,69,91,
			56,68,90,
			54,67,90,
			53,66,89,
			51,65,88,
			50,64,88,
			48,63,87,
			47,63,86,
			45,62,86,
			44,61,85,
			42,60,84,
			41,60,83,
			39,59,83,
			38,58,82,
			36,57,81,
			35,57,80,
			33,56,79,
			32,56,79,
			30,55,78,
		};

	signed char R1, R2, R3, R4;
	short angle2, r1, r2, r3;
	double angle;
	
	angle = (double)getServoAngle()*AD2DEG;		// AD値を角度に変換
	angle2 = abs((short)angle* 3);
	
	if ( pwm >= 0 ) {
		r1 = rev_difference_D[ angle2 ];
		r2 = rev_difference_D[ angle2 + 1 ];
		r3 = rev_difference_D[ angle2 + 2 ];
		
		R1 = r1 * pwm / 100;
		R2 = r2 * pwm / 100;
		R3 = r3 * pwm / 100;
		R4 = pwm;
		
		if ( angle >= 28/*85*/ && angle <= 45) {
			//motorPwmOut(R1, R3, R2, R4);//左カーブ
			motorPwmOut(R3 * 0.6, R4 * 0.8,
				    0, R2 * 0.8);
			
		}else if ( angle >= 18/*55*/ && angle <= 27) {
			//motorPwmOut(R1, R3, R2, R4);//左カーブ
			motorPwmOut(R3 , R4 ,
				    R1 * 0.9, R2 );
		}else if ( angle >= 5/*15*/ && angle <= 17 ) {
			//motorPwmOut(R1, R3, R2, R4);//左カーブ
			motorPwmOut(R3 , R4,
				    R1, R2);
		}else if( angle >= 0/*15*/ && angle <= 4 ){
			//motorPwmOut(R3, R1, R4, R2);//右カーブ
			motorPwmOut(R4, R4,
				    R4, R4);
		}
		
		if ( angle <= -28 && angle >= -45) {
			//motorPwmOut(R3, R1, R4, R2);//右カーブ
			motorPwmOut(R3 * 0.6, R4 * 0.8,
				    0, R2 * 0.8);
		}else if ( angle <= -18 && angle >= -27) {
			//motorPwmOut(R3, R1, R4, R2);//右カーブ
			motorPwmOut(R3 , R4 ,
				    R1 * 0.9, R2 );
		}else if ( angle <= -5 && angle >= -17) {
			//motorPwmOut(R3, R1, R4, R2);//右カーブ
			motorPwmOut(R4, R3,
				    R2, R1);
		}else if ( angle <= 0 && angle >= -4){
			//motorPwmOut(R3, R1, R4, R2);//右カーブ
			motorPwmOut(R4, R4,
				    R4, R4);
		}
	} else {
		r1 = rev_difference_D[ angle2 ];
		r2 = rev_difference_D[ angle2 + 1 ];
		r3 = rev_difference_D[ angle2 + 2 ];
		
		R1 = r1 * pwm / 100;
		R2 = r2 * pwm / 100;
		R3 = r3 * pwm / 100;
		R4 = pwm;
		
		if ( angle >= 0 ) {
			motorPwmOut(R4, R2,
				    R3, R1);
		} else {
			motorPwmOut(R2, R4, 
				    R1, R3);
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControl
// 処理概要     モーターのPWM決計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControl( void )
{
	int i, j, iRet, Dif, iP, iI, iD, Dev, maxpwm;
	//char kp3, ki3, kd3;
	
	i = targetSpeed;		// 目標値
	j = Encoder *10;		// 現在値 targetSpeedはエンコーダのパルス数*10のため
							// 現在位置も10倍する

	// デモモードのときゲイン変更
	if ( targetSpeed < targetSpeedBefore ) {
		kp3 = 10;
		ki3 = 4;
		kd3 = 0;
	} else {
		kp3 = kp3_buff;
		ki3 = ki3_buff;
		kd3 = kd3_buff;
	}
	
	// 駆動モーター用PWM値計算
	Dev = i - j;	// 偏差
		
	// 目標値を変更したらI成分リセット
	if ( Dev >= 0 && AccelefBefore == 1 ) {
		Int3 = 0;
	} else if ( Dev < 0 && AccelefBefore == 0 ) {
		Int3 = 0;
	}
	//if ( i != targetSpeedBefore ) Int3 = 0;
	
	Int3 += (double)Dev * 0.001;	// 時間積分
	Dif = Dev - EncoderBefore;		// 微分　dゲイン1/1000倍
	
	iP = (int)kp3 * Dev;			// 比例 
	
	iI = (double)ki3 * Int3;		// 積分
	if(iI > 5 ) iI = 5;			// 追加20190319
	if(iI < -5 ) iI = -5;			// 追加20190319
	
	iD = (int)kd3 * Dif;			// 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 4;
	
	// PWMの上限の設定
	// 出力電圧がVOLTAGELIMとなるDuty比を計算
	//maxpwm = (int8_t)(VOLTAGELIM / Voltage *100);

	// if ( iRet > maxpwm ) iRet =  maxpwm;
	// if ( iRet < -maxpwm ) iRet = -maxpwm;
	if ( iRet >  100 ) iRet = 100;
	if ( iRet <  -100 ) iRet = -100;
	
	if ( Dev > 0 )	AccelefBefore = 0;
	else		AccelefBefore = 1;
	
	motorPwm = iRet*0.6;
	EncoderBefore = Dev;
	targetSpeedBefore = i;
	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getLinePositionNow
// 処理概要     中央センサの位置を算出する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
double getLinePositionNow( short angleAD, double angleIMU ) {
	const double turningRadius[] = {       // 角度から内輪、外輪回転差計算
		3637467,8504,4221,2794,2081,1653,1367,1163,1010,890,
		794,716,650,594,546,505,468,436,407,381,
		358,336,317,299,282,267,253,239,227,215,
		204,194,184,174,165,157,148,140,133,125,
		118,110,103,96,
		};
	double thetaM, thetaS, theta1, r1;
	short angle;
	thetaS = (double)abs(angleAD) * AD2DEG;	// [°]に変換
	thetaM = fabs(angleIMU) * M_PI / 180;	// [rad]に変換
	r1 = turningRadius[(short)thetaS];
	theta1 = 90.0 - thetaS - fabs(angleIMU);	// 機体角度
	theta1 = thetaS * M_PI / 180;			// [rad]に変換

	return ( (r1+TREAD/2) * sin(thetaM) ) + ( WHELLBASE * cos(thetaM) ) + ( SENSORBAR * sin(theta1) );
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getLinePositionAfter
// 処理概要     旋回後の中央センサの位置を算出する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
double getLinePositionAfter ( short angle, double angleIMU ) {
	const double turningRadius[] = {       // 角度から内輪、外輪回転差計算
		3637467,8504,4221,2794,2081,1653,1367,1163,1010,890,
		794,716,650,594,546,505,468,436,407,381,
		358,336,317,299,282,267,253,239,227,215,
		204,194,184,174,165,157,148,140,133,125,
		118,110,103,96,
		};
	double thetaS, thetaM, theta1, theta2, theta3, y1, x1, r, r1;
	
	r1 = turningRadius[angle];
	thetaS = (double)angle * M_PI / 180;	// [rad]に変換
	thetaM = fabs(angleIMU) * M_PI / 180;			// [rad]に変換

	// 各パラメータ算出
	theta1 = M_PI/2 - thetaS - thetaM;
	x1 = turningRadius[(short)angle*4] + ( (TREAD/2) - SENSORBAR*cos(thetaS) );
	y1 = ( (r1+TREAD/2) * sin(thetaM) ) + ( WHELLBASE * cos(thetaM) ) + ( SENSORBAR * sin(theta1) );
	theta2 = atan(fabs(x1/y1));
	r = y1/cos(theta2);
	if (x1 > 0) {
		theta3 = theta1 - theta2;
	} else {
		theta3 = theta1 + theta2;
	}

	return r * cos(theta3);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getReturnAngle
// 処理概要     旋回後にセンサーバーと白線が並行になるサーボ角度の算出
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
short getReturnAngle( double angleIMU, double y1) {
	short i;

	for (i=0;i<=MAXDEG;i++) {
		if (y1 >= getLinePositionAfter( i, angleIMU )) {
			return i;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 acceleControl
// 処理概要     指定加速度への追従制御
// 引数         指定加速度
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
/*int  acceleControl( int targetAccele) {

	int target,speedBefor,accceleBefor,iRet,Dev,KP,KI,KD,Int;
	short nowspeed;
	float dt,P,I,D;
	
	target = targetAccele;				// 目標値
	nowspeed = currentSpeed;			// 現在速度
	dt = 0.001;					// 速度取得周期（1ms）
	
	KP = 0;
	KI = 0;
	KD = 0;
	
	nowaccele = (nowspeed - speedBefor) / dt;	//現在加速度
	
	Dev = targetAccele - nowaccele;		//偏差
	Int += ( Dev + accceleBefor ) / dt;		//加速度時間積分値
		
	P = KP * Dev;			 	// 比例 
	I = KI * Int;				// 積分
	D = KD * (Dev - accceleBefor)/dt;	// 微分
	iRet = P + I + D;
	

	speedBefor = nowspeed;
	accceleBefor = nowaccele;
}*/