#ifndef ADCONVERTER_H_
#define ADCONVERTER_H_
//====================================//
// インクルード
//====================================//
#include "R_PG_IGC-P8080_v1.h"
//====================================//
// シンボル定義
//====================================//
#define GATE_VAL			300		    // ゲートセンサしきい値
#define DEG2AD              41          // 1度あたりのAD値 サーボ最大切れ角時のAD値[]/サーボ最大切れ角[°]
#define AD2DEG              0.082F     // 1ADあたりの角度 サーボ最大切れ角[°]/サーボ最大切れ角時のAD値[]
#define LOWVOLTAGE          10.5        // 最低動作電圧
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


/*************************************** 自動生成関数 *************************************/
// ADコンバータ
#define SET_ADC		R_PG_ADC_12_Set_S12AD0(); 				// 12ビットA/Dコンバータ(S12AD0)を設定
#define START_ADC	R_PG_ADC_12_StartConversionSW_S12AD0();	// A/D変換開始
#define GET_ADC		R_PG_ADC_12_GetResult_S12AD0( result );	// AD値を取得
/******************************************************************************************/

//====================================//
// グローバル変数の宣言
//====================================//
// センサ関連
extern short		sensorR;		// 右アナログセンサ
extern short				sensorR_reg;
extern short				sensorR_max;	// 左アナログセンサMAXAD値
extern short				sensorR_min;	// 左アナログセンサMINAD値
extern short		sensorL;		// 左アナログセンサ
extern short				sensorL_reg;
extern short				sensorL_max;	// 左アナログセンサMAXAD値
extern short				sensorL_min;	// 左アナログセンサMINAD値
extern short		sensor_calc[9];
extern short		sensorG;		// ゲートセンサ
extern short		sensorG_th;	// ゲートセンサ
extern short		sensorC;		// 中心アナログセンサ
extern short		sensorLL;		// 最左端アナログセンサ
extern short		sensorRR;		// 最右端アナログセンサ
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

extern double		Voltage;        //電圧チェッカー

extern short		Angle0;		    // サーボセンター値
extern short		Angle_buff;
extern short		iAngle;
//====================================//
// プロトタイプ宣言
//====================================//
// センサ関連
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