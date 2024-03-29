#ifndef LINECHASE_H_
#define LINECHASE_H_
//====================================//
// インクルード
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
// シンボル定義
//====================================//

/* マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効) */
#define         MASK11111		0x1f  	/* ○○○○○（全部） */
#define         MASK11011		0x1b  	/* ○○×○○（全部） */
#define         MASK01010		0x0a  	/* ×○×○×（全部） */
#define         MASK10101		0x15  	/* ○×○×○（全部） */
#define         MASK01110		0x07  	/* ×○○○×（全部） */
#define         MASK10000		0x10  	/* ○××××（左外） */
#define         MASK10001		0x11  	/* ○×××○（左右） */
#define         MASK11000       	0x18    /* ○○×××（左全） */
#define         MASK10011       	0x13    /* ○××○○（左全） */
#define         MASK11001      		0x19    /* ○○××○（左全） */
#define         MASK01000		0x08    /* ×○×××（左内） */
#define         MASK01100		0x0c    /* ×○○××（左内） */
#define         MASK10100       	0x14    /* ○×○××（左中） */
#define         MASK11100      		0x1c    /* ○○○××（左中） */
#define         MASK11110       	0x1e    /* ○○○○×（左中） */
#define         MASK00100       	0x04    /* ××○××（中央） */
#define         MASK01111       	0x0f    /* ×○○○○（右中） */
#define         MASK00111       	0x07    /* ××○○○（右中） */
#define         MASK00101       	0x05    /* ××○×○（右中） */
#define		MASK00110		0x06	/* ××○○×（右内）	*/
#define		MASK00010		0x02	/* ×××○×（右内）	*/
#define		MASK00011		0x03	/* ×××○○（右全）	*/
#define		MASK00001		0x01	/* ××××○（右外）	*/

/*************************************** 自動生成関数 *************************************/
// タイマ割り込み
#define SET_CMT_C0		R_PG_Timer_Set_CMT_U0_C0();			// コンペアマッチタイマ初期化(ch0)
#define START_CMT_C0	R_PG_Timer_StartCount_CMT_U0_C0();	// カウントスタート(ch0)
#define STOP_CMT_C0	    R_PG_Timer_HaltCount_CMT_U0_C0();	// カウント一時停止(ch0)
/******************************************************************************************/
// 機体諸元
#define WHELLBASE   149
#define TREAD       143
#define SENSORBAR   307
#define MAXDEG      42

#define M_PI        3.141592
// 緊急停止
#define	STOPPING_METER		   3		// 停止距離

// 各セクションでの目標速度　x/10[m/s]
#define SPEED_STRAIGHT			42	// 通常トレース
#define SPEED_CURVE_BRAKE		40	// カーブブレーキ
#define SPEED_CURVE_R600		45	// R600カーブ速度
#define SPEED_CURVE_R450		38	// R450カーブ速度
#define SPEED_CURVE_STRAIGHT	42	// S字カーブ直線速度

#define SPEED_CROSSLINE			25	// クロスライン進入速度
#define SPEED_CLANK_TRACE		20	// クランク進入速度
#define SPEED_RIGHTCLANK_CURVE		20	// 右クランク旋回速度
#define SPEED_RIGHTCLANK_ESCAPE		30	// 右クランク復帰速度
#define SPEED_LEFTCLANK_CURVE		20	// 左クランク旋回速度
#define SPEED_LEFTCLANK_ESCAPE		30	// 左クランク復帰速度

#define SPEED_HALFLINE				40	// ハーフライン進入速度
#define SPEED_RIGHTCHANGE_TRACE	    40	// 右レーンチェンジ進入速度
#define SPEED_RIGHTCHANGE_CURVE	    40	// 右レーンチェンジ旋回速度
#define SPEED_RIGHTCHANGE_ESCAPE	40	// 右レーンチェンジ復帰速度
#define SPEED_LEFTCHANGE_TRACE		40	// 左レーンチェンジ進入速度
#define SPEED_LEFTCHANGE_CURVE		40	// 左レーンチェンジ旋回速度
#define SPEED_LEFTCHANGE_ESCAPE		40	// 左レーンチェンジ復帰速度

#define SPEED_SLOPE_BRAKE		25	    // 下り坂終点速度
#define SPEED_SLOPE_TRACE		30	    // 坂読み飛ばし速度
// 角度
#define ANGLE_RIGHTCLANK		550	// 右クランク旋回角度480
#define ANGLE_LEFTCLANK		        -510	// 左クランク旋回角度480
#define ANGLE_RIGHTCHANGE		230	// 右レーンチェンジ旋回角度
#define ANGLE_LEFTCHANGE	        -230	    // 左レーンチェンジ旋回角度



// カーブ関連
#define CURVE_R600_START		160		// R600開始AD値
#define CURVE_R600_START_i		20		// R600開始iAngle
#define CURVE_R450_START		250		// R450開始AD値

// ジャイロ関連
#define SLOPE_UPPERLINE_IMU		-12		// 上り坂検出角度
#define SLOPE_LOWERLINE_IMU	    	5		// 下り坂検出角度
#define INTEGRAL_LIMIT			200		// 角速度積算時間

// PIDゲイン関連
#define VOLTAGELIM 10.5 // 出力最大電圧
#define VOLTAGELIMTRACE 7.0 // 出力最大電圧
//白線トレース
#define KP		30
#define KI		10
#define KD		10

// 角度制御
#define KP2		70
#define KI2		0
#define KD2		80

// 速度制御
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

// 緊急停止関連
#define STOP_SENSOR1	60		// センサ全灯
#define STOP_SENSOR2	800		// センサ全消灯
#define STOP_ENCODER	100		// エンコナスの加速度検知(コースから落ちた？)
#define STOP_COUNT		1000	// 時間停止
//===================================ーダ停止(ひっくり返った？)
#define STOP_GYRO		100		// マイ=//
// グローバル変数の宣言
//====================================//
// パターン、モード関連
extern char pattern;		// パターン番号
extern char modeLCD;		// LCD表示選択
extern char modeSlope;		// 坂チェック		0:上り坂始め	1:上り坂終わり	2:下り坂始め	3:下り坂終わり
extern char	modeAngle;		// サーボPWM変更	0:白線トレース	1:角度制御
extern char	modePushcart;	// 手押しモード可否	0:自動走行	1:手押し
extern char	msdset;			// MicroSDが初期化されたか	0:初期化失敗	1:初期化成功
extern char	IMUSet;			// IMUが初期化されたか	0: 初期化失敗	1:初期化成功

extern int pich_flg;

// パラメータ関連
// 距離
extern short	stopping_meter;			    // 停止距離
// 速度
extern short	speed_straight;			    // 通常トレース
extern short	speed_curve_brake;		    // カーブブレーキ
extern short	speed_curve_r600;		    // R600カーブ速度
extern short	speed_curve_r450;		    // R450カーブ速度
extern short	speed_curve_straight;	    // S字カーブ直線速度

extern short	speed_crossline;			// クロスライン進入速度
extern short	speed_ckank_trace;		    // クランク進入速度
extern short	speed_rightclank_curve;	    // 右クランク旋回速度
extern short	speed_rightclank_escape;	// 右クランク復帰速度
extern short	speed_leftclank_curve;	    // 左クランク旋回速度
extern short	speed_leftclank_escape;	    // 左クランク復帰速度

extern short	speed_halfine;			    // ハーフライン進入速度
extern short	speed_rightchange_trace;	// 右レーンチェンジ進入速度
extern short	speed_rightchange_curve;	// 右レーンチェンジ旋回速度
extern short	speed_rightchange_escape;   // 右レーンチェンジ復帰速度

extern short	speed_leftchange_trace;	    // 左レーンチェンジ進入速度
extern short	speed_leftchange_curve;	    // 左レーンチェンジ旋回速度
extern short	speed_leftchange_escape;	// 左レーンチェンジ旋回速度

extern short	speed_slope_brake;		    // 下り坂終点速度
extern short	speed_slope_trace;		    // 坂読み飛ばし速度

// サーボ角度
extern short	angle_rightclank;		    // 右クランク旋回角度
extern short	angle_leftclank;			// 左クランク旋回角度
extern short	angle_rightchange;		    // 右レーンチェンジ旋回角度
extern short	angle_leftchange;		    // 右レーンチェンジ旋回角度

// タイマ関連
extern short	cntGyro;			// 角度計算用カウンタ

// 角度関連
extern double 	TurningAngleEnc;	// エンコーダから求めた旋回角度
extern double	PichAngleAD;		// アナログジャイロから求めたピッチ角度
extern int 		Angle_fixed;

// モーター関連
extern signed char 	motorPwm;	    // モーター制御PWM
extern short		targetSpeed;	// 目標速度
extern unsigned int encStable;
extern short        cntStable;

// ゲイン関連
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

// デモ関連
extern char demo;

// サーボ関連
extern double		Int;		// I成分積算値(白線トレース)
extern short 		SetAngle;	// 目標角度
extern signed char 	ServoPwm;	// 白線トレースサーボPWM
extern signed char 	ServoPwm2;	// 角度サーボPWM

// ライン検知
extern volatile unsigned char cnt_crossline;
extern volatile unsigned char cnt_rightline;
extern volatile unsigned char cnt_leftline;
extern volatile int line;
//====================================//
// プロトタイプ宣言
//====================================//
// マーカー関連
int checkLine( void );
//int checkCrossLine( void );
//int checkRightLine( void );
//int checkLeftLine( void );
bool check_crossline( void );
bool check_rightline( void );
bool check_leftline( void );
signed char checkSlope( void );

// エンコーダ関連
unsigned int encMM( short mm );
unsigned int stableSpeedDistance( void );

// モーター関連
void motorControl( void );
int acceleControl( int targetAccele);

// 機体制御関連
void diff ( signed char pwm );
double getLinePositionNow( short angleAD, double angleIMU );
double getLinePositionAfter ( short angle, double angleIMU );
short getReturnAngle( double angleIMU, double y1);

// サーボ関連
void servoControlTrace( void );
void servoControlAngle( void );

void Slope_Observer ( void );

#endif // LINECHASE_H_