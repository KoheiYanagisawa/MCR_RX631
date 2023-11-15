//====================================//
// インクルード
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
//====================================//
// グローバル変数の宣言
//====================================//
// 走行パターン関連
char		pattern = 0;	// パターン番号

// モード関連
char		modeCurve;		// カーブ判定	0:カーブ以外	1:カーブ走行中
char		modeError;		// 0: 通常走行 1:距離停止 2:センサ全灯 3:センサ全消灯 4:エンコーダ停止 5:ジャイロ反応
char 		modeAutoMotor;	// 0: switch文でサーボ、駆動モータのPWM出力を指定する 1: Timer関数内で自動的にPWM出力を実行
char      	modeMotor = 1; //1通常走行、0モーターフリー
volatile unsigned char check_line;
// タイマ関連
// 1msタイマ
unsigned int 		cnt1;		// 走行用タイマカウント
unsigned short	 	cntOut1;	// コースアウト判定用タイマ
unsigned short	 	cntOut2;	// コースアウト判定用タイマ2
unsigned short	 	cntOut3;	// コースアウト判定用タイマ3
unsigned short	 	cntOut4;	// コースアウト判定用タイマ4
static char			Timer10;	// 1msカウント用

int correct_angle;
int Angle_fixed;			//マーカー読み飛ばし用固定角度
//====================================//
// プロトタイプ宣言
//====================================//
void initParameter ( bool lcd );
//====================================//
// メインプログラム
//====================================//
void main(void){
	char		countdown = 0x0;
	short i, j, angleAfter, angleCenter;
	double y1;
	unsigned int ui;
	//=================================//
	// 初期化
	//=================================//
	ledOut(0); 		// LED消灯
	L_Sen_ON;		// センサ点灯
	intiLcd();		// LCD初期化
	
	motorPwmOut(0, 0, 0, 0);	// モーター停止
	servoPwmOut( 0 );
	SetAngle = 0;
	
	modePushcart = 0;		// 手押しモードoff 
	modeSlope = 0;			// 上り坂チェック
	modeAngle = 0;			// 白線トレース
	modeAutoMotor = 0;		// 自動PWM出力停止
	start = 0;				// ゲートスタート
	
	//IMU初期化
	initIMU();
	// フラッシュ初期化
	// データフラッシュから前回パラメータを読み込む
	if( !initFlash() ) readFlashSetup( 1, 1, 1 ,1 ,1 ,1 ,1);
	// MicroSDカード初期化
	if( !initMicroSD() ) msdset = 1;
	else msdset = 0;

	// 電源電圧の確認
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
				// 手押しモードOFF
				if( cnt1 >= 100 ) {		// 動き出してから
					if ( EncoderTotal >= ( PALSE_METER * stopping_meter ) ) { // 距離超過の場合
						modeError = 1;
					} else if ( cntOut1 >= STOP_SENSOR1 ) {	// センサ全灯
						modeError = 2;
					} else if ( cntOut2 >= STOP_SENSOR2 ) {	// センサ全消灯
						modeError = 3;
					} else if ( cntOut3 >= STOP_ENCODER ) {	// エンコーダ停止(ひっくり返った？)
						modeError = 4;
					} else if( cntOut4 >= STOP_GYRO ) {	// マイナスの加速度検知(コースから落ちた？)
						modeError = 5;	
					}
					// // Buletoothで外部から停止
					// if ( stopWord == 1 ) {
					// 	modeError = 6;
					// }
					// 一定時間で停止
					// if( cntStable >= STOP_COUNT ) {
					// 	modeError = 7;
					// }
					if (modeError > 0) {
						pattern = 101;
						modeAutoMotor = 0;
						ui = cnt1;	// 走行時間取得
						LEDR_OFF;
						LEDG_OFF;
						LEDB_OFF;
					}
				}
			} else {			
				// 手押しモードON

				// 距離測定
				// lcdRowPrintf(UPROW, "mm%4.1f", (double)EncoderTotal/PALSE_MILLIMETER);
				// lcdRowPrintf(LOWROW, " %7d", EncoderTotal);

				lcdRowPrintf(UPROW, "now  %3d", pattern);
				lcdRowPrintf(LOWROW, "D   0x%2x", sensor_inp(MASK11111));
			}
			// スイッチで停止
			if ( cnt1 >= 1000 && taswGet() == SW_PUSH ) {
				modeError = 6;
				pattern = 101;
			}
		} else if ( pattern >= 100 ) {
			modeLCD = 1;
			// 速度ゲイン調整用
			// lcdRowPrintf(UPROW, "TIME%4d", cntStable);
			// lcdRowPrintf(LOWROW, "%6.1f", (double)encStable/PALSE_MILLIMETER);

			// エラーモード確認
			lcdRowPrintf(UPROW, "MODE   %1d", modeError);
			lcdRowPrintf(LOWROW, "    %4d",angleAfter);
		}
	if( check_line ){	
																		
		switch(	checkLine() ){											
			case 1:					/* クロスライン検出時の分岐 */
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //直前の進入角度を記録
				pattern = 21;											
				break;												
																		
			case 2:					/* 右ハーフライン検出時の分岐 */	
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //直前の進入角度を記録
				pattern = 51;											
				break;												
																	
			case 3:					/* 左ハーフライン検出時の分岐 */
				line = 0;
				enc1 = 0;
				Angle_fixed = getServoAngle(); //直前の進入角度を記録
				pattern = 61;										
				break;												
																	
			default:												
				break;												
		}																
	}
		
	switch( pattern ) {
		//-------------------------------------------------------------------
		// 【000】スタート処理
		//-------------------------------------------------------------------
		case 0:
			// スタート前設定
			setup();
			if ( start && !modePushcart ) {
				demo = 0;		// デモモード解除
				modeAngle = 0;	// 白線トレース
				Int = 0;			// 積分リセット
				txt= txtData;		// 受信配列リセット
				cntByte = 0;		// 受信カウントリセット
				
				if ( msdset ) init_log();	// ログ記録準備
				
				if ( !fixSpeed ) writeFlashBeforeStart(1, 0, 1, 1, 1, 1);	// 速度パラメータをデータフラッシュに保存
				else writeFlashBeforeStart(0, 0, 1, 1, 1, 1);		// 速度パラメータ以外を保存
				
				//if (IMUSet) caribrateIMU();
				
				waitLcd(500);		// 500ms待つ
				cnt1 = 0;
				pattern = 1;
				break;
			} else if ( start && modePushcart ) {
				// 手押しモードの場合すぐに通常トレース
				if ( msdset ) init_log();	// ログ記録準備
				
				// 白線トレース用PIDゲイン保存
				// 角度制御用PIDゲイン保存
				writeFlashBeforeStart(0, 0, 1, 1, 0, 0);
				// 変数初期化
				initParameter( 1 );
				break;
			}
			break;
			
		case 1:
			servoPwmOut( ServoPwm );
			if ( start == START_COUNT ) {
				// カウントダウンスタート
				if ( cnt1 >= 3000 ) {	
					// 変数初期化
					initParameter( 0 );
					break;
				} else if ( !(cnt1 % 1000) ) {
					ledOut( countdown );
					countdown = countdown << 1;
					break;
				}
			} else if ( start == START_GATE ) {
				// スタートゲート開放スタート
				pattern = 2;
				break;
			}
			break;
			
		case 2:
			servoPwmOut( ServoPwm );
			// スタートバー開閉待ち
			if ( !startbar_get() ) {
				// 変数初期化
				initParameter( 0 );
				break;
			}
			// LED点滅処理
			if ( cnt1 >= 2000 ) cnt1 = 0;
			if ( cnt1 < 1000 ) {
				ledOut( LED_R );
			} else {
				ledOut( LED_B  );
			}
			break;
		//-------------------------------------------------------------------
		// 【010】トレース処理
		//-------------------------------------------------------------------
		case 11:
			targetSpeed = speed_straight * SPEED_CURRENT;
			i = getServoAngle();
			ledOut( 0x00 );
			
			check_line  = 1;
			
			// 坂道チェック
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			
			// カーブチェック
			if ( i >=  CURVE_R600_START || i <= -CURVE_R600_START ) {
				enc1 = 0;
				modeCurve = 1;
				pattern = 12;
				break;
			}
			break;
			
		case 12:
			// カーブブレーキ
			targetSpeed = speed_curve_brake * SPEED_CURRENT;
			ledOut( LED_R );
			i = getServoAngle();
			
			if ( enc1 > encMM( 60 ) ) {		// 60mm進む
				enc1 = 0;
				TurningAngleEnc = 0;
				TurningAngleIMU = 0;
				pattern = 13;
				break;
			}
			
			check_line  = 1;
			
			// 坂道チェック
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			if ( memory_mode ) {
				enc1 = 0;
				pattern = 16;
				break;
			}
			// 直線チェック
			if ( i < CURVE_R600_START && i > -CURVE_R600_START ) {
				enc1 = 0;
				pattern = 11;
				break;
			}
			break;
			
		case 13:
			// R600カーブ走行
			targetSpeed = speed_curve_r600 * SPEED_CURRENT;
			i = getServoAngle();
			
			check_line  = 1;
			
			// 坂道チェック
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			// R450チェック
			if ( i >= CURVE_R450_START || i <= -CURVE_R450_START ) {
				enc1 = 0;
				pattern = 14;
				break;
			}
			// カーブ継ぎ目チェック
			if ( i <  CURVE_R600_START && i > -CURVE_R600_START ) {
				enc1 = 0;
				pattern = 15;
				break;
			}
			break;
			
		case 14:
			// R450カーブ走行
			targetSpeed = speed_curve_r450 * SPEED_CURRENT;
			i = getServoAngle();
			
			check_line  = 1;
			
			// R600チェック
			if ( i < CURVE_R450_START && i > -CURVE_R450_START ) {
				enc1 = 0;
				pattern = 13;
				break;
			}
			break;
		
		case 15:
			// カーブ継ぎ目走行
			targetSpeed = speed_curve_straight * SPEED_CURRENT;
			i = getServoAngle();
			
			if ( enc1 >= encMM( 300 ) ) {		// 300mm進む
				enc1 = 0;
				modeCurve = 0;
				pattern = 11;
				break;
			}
			
			check_line  = 1;
			
			// 坂道チェック
			if( checkSlope() == 1 || checkSlope() == -1 ) {
				pattern = 71;
				break;
			}
			// カーブチェック
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
		//【020】クランク検出処理
		//-------------------------------------------------------------------
		case 21:
			check_line  = 0;
			
			targetSpeed = speed_crossline* SPEED_CURRENT;
			
			ledOut( LED_G );
			
			if( enc1 > encMM( 90 ) ) {		// 60mm進む
				enc1 = 0;
				pattern = 22;
				break;
			}
			break;
			
		case 22:
			targetSpeed = speed_ckank_trace * SPEED_CURRENT;
			
			// 右クランクチェック
			if( sensor_inp(MASK00001) ==  0x01 ) {
				enc1 = 0;
				modeAngle = 1;
				TurningAngleEnc = 0;
				TurningAngleIMU = 0;
				pattern = 30;
				break;
			}
			// 左クランクチェック
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
		//【030】右クランク処理
		//-------------------------------------------------------------------
		case 30:// 設定角度まで切り少し待つ
			SetAngle = angle_rightclank;
			if( getServoAngle() > (ANGLE_RIGHTCLANK) ){
				y1 = getLinePositionNow( getServoAngle(), TurningAngleIMU);
				enc1 = 0;
				Int = 0;			// 積分リセット
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
		case 31://ハンドルキープ
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
		case 32://外枠線検出処理
			SetAngle = angle_rightclank;
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT;
			
			if(sensor_inp() == 0x10){
				pattern = 34;
				break;
			}
			break;
		case 33:
			break;
		case 34://最左端センサ検出処理
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
		case 35://中央センサ検出処理
			targetSpeed = speed_rightclank_curve * SPEED_CURRENT;
			enc1 = 0;
			
			Int = 0;			// 積分リセット
			pattern = 36;
			break;

		case 36://復帰
			modeAngle = 0;
			targetSpeed = speed_rightclank_escape * SPEED_CURRENT;
			if( enc1 >= encMM( 600 ) ) {		// 安定するまで待つ(600mm)
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		//-------------------------------------------------------------------
		//【040】左クランク処理
		//-------------------------------------------------------------------
		case 40:// 設定角度まで切り少し待つ
			SetAngle = angle_leftclank;
			if( getServoAngle() < ANGLE_LEFTCLANK ){
				y1 = getLinePositionNow( getServoAngle(), TurningAngleIMU);
				enc1 = 0;
				Int = 0;			// 積分リセット
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
		case 41://ハンドルキープ
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
		case 42://外枠線検出処理
			SetAngle = angle_leftclank;
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			
			if(sensor_inp() == 0x1){
				pattern = 44;
				break;
			}
			break;
		case 43:
			break;
		case 44://最左端センサ検出処理
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
		case 45://中央センサ検出処理
			targetSpeed = speed_leftclank_curve * SPEED_CURRENT;
			enc1 = 0;
			
			Int = 0;			// 積分リセット
			pattern = 46;
			break;

		case 46://復帰
			modeAngle = 0;
			targetSpeed = speed_leftclank_escape * SPEED_CURRENT;
			if( enc1 >= encMM( 600 ) ) {		// 安定するまで待つ(600mm)
				enc1 = 0;
				ledOut( 0x0 );
				pattern = 11;
				break;
			}
			break;
		
		//-------------------------------------------------------------------
		//【050】右レーンチェンジ処理
		//-------------------------------------------------------------------
		case 51://マーカー誤トレース処理
			check_line  = 0;
			targetSpeed = speed_halfine * SPEED_CURRENT;
			modeAngle = 1;
			SetAngle = Angle_fixed; //直前進入角度で固定
			if( enc1 > encMM( 20 ) ) {
				enc1 = 0;
				modeAngle = 0;
				Int = 0;			// 積分リセット
				pattern = 52;
				break;
			}
			
			if( check_crossline() ) {		// クロスラインチェック
				enc1 = 0;
				pattern = 21;
				break;
			}
			break;
			
		case 52://線切れ読み飛ばし処理(300mm)
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
				Int = 0;			// 積分リセット
				
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
		//【060】左レーンチェンジ処理
		//-------------------------------------------------------------------
		case 61:
			check_line  = 0;
			targetSpeed = speed_halfine * SPEED_CURRENT;
			modeAngle = 1;
			SetAngle = Angle_fixed; //直前進入角度で固定
			if( enc1 > encMM( 20 ) ) {
				enc1 = 0;
				modeAngle = 0;
				Int = 0;			// 積分リセット
				pattern = 62;
				break;
			}
			if( check_crossline() ) {		// クロスラインチェック
				enc1 = 0;
				pattern = 21;
				break;
			}
			break;
			
		case 62://線切れ読み飛ばし処理(300mm)
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
				Int = 0;			// 積分リセット
				
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
		//【070】坂道処理
		//-------------------------------------------------------------------
		case 71:
			// 誤検知判断
			// 目標速度変えない
			if( checkSlope() == 1 ) {
				if( modeSlope == 0 ) {
					
					// 上り始め
					modeSlope = 1;
					enc1 = 0;
					ledOut( LED_R | LED_B );
					pattern = 72;
					break;
				} else if ( modeSlope == 2 && enc_slope >= encMM( 600 ) ) {
					// 下り終わり
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
					// 上り終わり、下り始め
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
			// 坂頂点まで走行
			targetSpeed = speed_slope_trace * SPEED_CURRENT;
			
			if( enc1 >= encMM( 1200 ) ) {
				enc1 = 0;
				pattern = 73;
				break;
			}
			break;
			
		case 73:
			// 上り坂終点ブレーキ 全力ブレーキ
			targetSpeed = 0;

			if( enc1 >= encMM( 50 ) ) {
				enc1 = 0;
				pattern = 75;
				break;
			}
			break;
			
		case 74:
			// 下り坂終点ブレーキ
			targetSpeed = speed_slope_brake * SPEED_CURRENT;
			if( enc1 >= encMM( 40 ) ) {
				enc1 = 0;
				pattern = 75;
				break;
			}
			break;
			
		case 75:
			// ジャイロセンサが安定するまで読み飛ばし
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
			// ジャイロセンサが安定するまで読み飛ばし
			if ( modeSlope == 3 ) {
				targetSpeed = speed_straight * SPEED_CURRENT;
			} else {
				targetSpeed = speed_slope_trace * SPEED_CURRENT;
			}
			
			// クロスラインチェック
			if( check_crossline() ) {
				enc1 = 0;
				pattern = 21;
				break;
      			}
			// 右ハーフラインチェック
   			if( check_rightline() ) {
				enc1 = 0;
				pattern = 51;
				break;
			}
			// 左ハーフラインチェック
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
		//【100】停止処理
		//-------------------------------------------------------------------
		case 101:
			// 減速処理
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
			// 車体停止処理
			modeAngle = 0;
			motor_mode_f( B, B );
			motor_mode_r( B, B );
			
			if( Encoder <= 5 && Encoder >= -1 ) {
				//servoPwmOut( 0 );
				//R_PG_IO_PORT_Write_PE6( 0 );	//サーボモータ freeモード
				if( msdset == 1 ) {
					// microSDの動作が有効なとき
					pattern = 103;
					cnt1 = 0;
					break;
				} else {
					// microSDの動作が無効なとき
					pattern = 106;
					break;
				}
			}
			break;
			
		case 103:
			servoPwmOut( ServoPwm );
			// 最後のデータが書き込まれるまで待つ
			if ( cnt1 <= 1000 ) {	// 1000ms待つ
				if( checkMicroSDProcess() == 11 ) {
					msdFlag = 0;			// ログ記録終了
					microSDProcessEnd();    // microSDProcess終了処理
					pattern = 104;
					break;
				}
			} else {			// 1000ms以上経過したら書き込みを強制終了
				pattern = 106;
				break;
			}
			break;
			
		case 104:
			servoPwmOut( ServoPwm );
			// microSDの書き込み終了処理が終わるまで待つ
			if( checkMicroSDProcess() == 0 ) {
				// MicroSD最終書き込みアドレス保存
				flashDataBuff[ 0 ] = msdWorkaddress >> 16;
				flashDataBuff[ 1 ] = msdWorkaddress & 0xffff;	// 終了アドレス
				writeFlashData( MSD_STARTAREA, MSD_ENDAREA, MSD_DATA, 2 );
				pattern = 105;
				break;
			}
			break;
			
		case 105:
			servoPwmOut( ServoPwm );
			// microSD書き込み成功
			// LED点滅処理
			if( cnt1 >= 200 ) cnt1 = 0;
			if( cnt1 < 100 ) {
				LEDB_ON;
			} else {
				LEDB_OFF;
			}
			break;
			
		case 106:
			servoPwmOut( ServoPwm );
			// mMicroSD書き込み失敗
			// LED点滅処理
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
// モジュール名 Timer
// 処理概要     1msごとにタイマ割り込み
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void Timer (void) {
	short n;
	__setpsw_i();	// 多重割り込み許可
	
	//　タイマカウント
	if ( pattern >= 11 ) {
		if ( pattern <= 99 ) {
			if ( pattern != 21 ) {				// クロスライン通過時は無視
				if ( sensor_inp(MASK11111) == 0x1f || sensor_inp(MASK11111) == 0x5 ) {	// センサ全灯
					cntOut1++;	
				} else {
					cntOut1 = 0;
				}
			}
			if ( sensor_inp(MASK11111) == 0x0 && pattern != 53 && pattern != 63 ) cntOut2++;	// センサ全消灯
			else cntOut2 = 0;
			if ( Encoder <= 1 && Encoder >= -1 ) cntOut3++;		// エンコーダ停止(ひっくり返った？)
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
			
	// LCD表示
	if ( modeLCD ) lcdShowProcess();
	// エンコーダカウント取得
	getEncoder();
	// アナログセンサ正規化
	getsens_ratio();

	// PID制御値算出
	if ( modeAngle ) servoControlAngle();	// 角度
	else servoControlTrace();		// 白線
	if ( modeMotor )motorControl();		// モータ
	else {
		motor_mode_f( F, F );
		motor_mode_r( F, F );
	}

	// 走行中のPWM出力
	if ( modeAutoMotor ) {
		if ( modeAngle ) servoPwmOut( ServoPwm2 );	// 角度
		else servoPwmOut( ServoPwm );	// 白線
		if (!modePushcart) {
			diff( motorPwm ); // 駆動輪モータPWM出力
		}
	}
	

	
	// MicroSD書き込み
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
	
	// 通信
	// 加速度及び角速度を取得
	
		
	// 10ｍごとに実行
	switch ( Timer10 ) {	
	case 1:
		getSwitch();		// スイッチ読み込み
		getVoltage();		// 電源電圧取得
		
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
// モジュール名 initParameter
// 処理概要     変数の初期化
// 引数         lcd: 1 lcd表示  0 lcd非表示
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void initParameter ( bool lcd ) {
	cntmpattern2 = 0;	// 記録走行カウントリセット
	EncoderTotal = 10;	// 総走行距離
	cnt1 = 0;			// タイマリセット
	enc1 = 0;			// 区間距離リセット
	modeLCD = lcd;		// LCD表示OFF
	modeAutoMotor = 1; // PWM出力
	msdFlag = 1;		// データ記録開始
	targetSpeed = speed_straight * SPEED_CURRENT; // 目標速度設定
	
	// 角度積算値リセット
	TurningAngleIMU = 0;
	RollAngleIMU = 0;
	PichAngleIMU = 0;
	pattern = 11;		// 通常走行
}