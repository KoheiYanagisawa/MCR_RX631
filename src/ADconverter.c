//====================================//
// インクルード
//====================================//
#include "ADconverter.h"
//====================================//
// グローバル変数の宣言
//====================================//
// タイマ関連
static char				ADTimer10;	// AD変換カウント用



// センサ関連
// AD変換結果格納
static unsigned short 	result[14]; 	// 12bitA/D変換結果の格納先
static int			senR;		// 右アナログセンサ積算AD値
static int			senL;		// 左アナログセンサ積算AD値
static int			senG;		// ゲートセンサ積算AD値
static int			senC;		// 中心アナログセンサ積算AD値
static int			senLL;		// 最左端アナログセンサ積算AD値
static int			senRR;		// 最右端アナログセンサ積算AD値
static int			VolC;		// 電圧チェッカーAD値
static int			pot;		// ポテンションメーター積算AD値
static int			sen_saka;
// AD変換結果平均値
short 				Angle;		// ポテンションメーター平均AD値
short				sensor_saka;
short				sensor_calc[9];
short				sensorR;	// 右アナログセンサ平均AD値
short				sensorR_reg;
short				sensorR_max;	// 右アナログセンサMAXAD値
short				sensorR_min;	// 右アナログセンサMINAD値
short				sensorL;	// 左アナログセンサ平均AD値
short				sensorL_reg;
short				sensorL_max;	// 左アナログセンサMAXAD値
short				sensorL_min;	// 左アナログセンサMINAD値
short				sensorG;	// ゲートセンサ平均AD値
short				sensorC;	// 中心アナログセンサ平均AD値
short				sensorLL;	// 最左端アナログセンサ平均AD値
short				sensorRR;	// 最右端アナログセンサ平均AD値
short				VoltageC;	// 電圧チェッカーAD値平均値
short				Angle0;		// サーボセンター値
short				iAngle;		// 
short				Angle_buff;		//

float				RR_tatio;
float				R_tatio;
float				C_tatio;
float				L_tatio;
float				LL_tatio;

double				Voltage;	// AD値から推定した電源電圧
short				sensorG_th = GATE_VAL;	// ゲート開放しきい値
// デジタルセンサデータ
volatile unsigned char 		RR_data;
volatile unsigned char 		R_data;
volatile unsigned char 		C_data;
volatile unsigned char 		L_data;
volatile unsigned char 		LL_data;
/////////////////////////////////////////////////////////////////////
// モジュール名 ADconverter
// 処理概要     AD変換割り込み
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ADconverter ( void )
{
	__setpsw_i();
	GET_ADC
	sensor_calc[sensorRR_max] =  1715;
	sensor_calc[sensorRR_min] = 180;
	
	sensorR_max = 1473;
	sensorR_min = 169;
	
	sensor_calc[sensorC_max] = 1315;
	sensor_calc[sensorC_min] = 167;
	
	sensorL_max = 1184;
	sensorL_min = 163;
	
	sensor_calc[sensorLL_max] = 1437;
	sensor_calc[sensorLL_min] = 175;
	
	/*ADTimer10++;
	if ( ADTimer10 == 10 ) {
		ADTimer10 = 0;
		
		Angle = pot / 10;
		sensorR = senL / 10;
		sensorL = senR / 10;	
		sensorG = senG / 10;
		sensorC = senC / 10;
		sensorLL = senLL / 10;
		sensorRR = senRR / 10;
		VoltageC = VolC /10;
		sensor_saka = sen_saka/10;
		
		senR = 0;
		senL = 0;
		senG = 0;
		senC = 0;
		senLL = 0;
		senRR = 0;
		sen_saka = 0;
		
		VolC = 0;
		pot = 0;
	}*/
		Angle = pot;
		sensorR = senL;
		sensorL = senR;	
		sensorG = senG;
		sensorC = senC;
		sensorLL = senLL;
		sensorRR = senRR;
		VoltageC = VolC;
		sensor_saka = sen_saka;
		
		senR = 0;
		senL = 0;
		senG = 0;
		senC = 0;
		senLL = 0;
		senRR = 0;
		sen_saka = 0;
		
		VolC = 0;
		pot = 0;
	
	// AD変換値をバッファに格納
	pot += result[3];
	senG += result[4];
	senLL += result[5];
	senL += result[9];//6
	senC += result[7];
	senRR += result[8];
	senR += result[6];//9
	sen_saka += result[10];
	VolC += result[11];
	
	
	
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getVoltage
// 処理概要     電圧の取得
// 引数         なし
// 戻り値       な
/////////////////////////////////////////////////////////////////////
void getVoltage ( void )
{
	// AD値 * R1 * R2 / 分解能
	Voltage = VoltageC * 5.05 * 3.94 / 4096;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getServoAngle
// 処理概要     ポテンションメーターのアナログ値で取得
// 引数         なし
// 戻り値       センサ値
///////////////////////////////////////////////////////////////////////////
short getServoAngle(void) 
{	
	return  ( Angle0 - Angle );
}

/////////////////////////////////////////////////////////////////////
// モジュール名 getsens_ratio
// 処理概要    	センサ値の正規化
// 引数         なし
// 戻り値       な
/////////////////////////////////////////////////////////////////////
void getsens_ratio ( void )
{
	float	RR, R, C, L, LL;
	
	RR = sensorRR;
	R = sensorR;
	C = sensorC;
	L = sensorL;
	LL = sensorLL;
	
	RR_tatio =(RR - sensor_calc[sensorRR_min])/(sensor_calc[sensorRR_max] - sensor_calc[sensorRR_min]);
	if(RR_tatio > 1) RR_tatio = 1;
	if(RR_tatio < 0.01) RR_tatio = 0.01;
	
	R_tatio =(R - sensorR_min)/(sensorR_max - sensorR_min);
	if(R_tatio > 1) R_tatio = 1;
	if(R_tatio < 0.01) R_tatio = 0.01;
	
	C_tatio =(C - sensor_calc[sensorC_min])/(sensor_calc[sensorC_max] - sensor_calc[sensorC_min]);
	if(C_tatio > 1) C_tatio = 1;
	if(C_tatio < 0.01) C_tatio = 0.01;
	
	L_tatio =(L - sensorL_min)/(sensorL_max - sensorL_min);
	if(L_tatio > 1) L_tatio = 1;
	if(L_tatio < 0.01) L_tatio = 0.01;
	
	LL_tatio =(LL - sensor_calc[sensorLL_min])/(sensor_calc[sensorLL_max] - sensor_calc[sensorLL_min]);
	if(LL_tatio > 1) LL_tatio = 1;
	if(LL_tatio < 0.01) LL_tatio = 0.01;
	
	sensor_calc[sensorRR_reg] = RR_tatio * 2000;
	sensorR_reg = R_tatio * 2000;
	sensor_calc[sensorC_reg] = C_tatio * 2000;
	sensorL_reg = L_tatio * 2000;
	sensor_calc[sensorLL_reg] = LL_tatio * 2000;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getAnalogSensor
// 処理概要     アナログセンサのアナログ値で取得
// 引数         なし
// 戻り値       センサ値
///////////////////////////////////////////////////////////////////////////
short getAnalogSensor(void) 
{
	return sensorL_reg - sensorR_reg;
	//return sensorL - sensorR;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 RRget
// 処理概要     デジタルセンサ読み込み
// 引数         なし
// 戻り値       1:反応あり 0:反応なし
///////////////////////////////////////////////////////////////////////////
unsigned char RRget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorRR_reg] < DsensRR) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 Rget
// 処理概要     デジタルセンサ読み込み
// 引数         なし
// 戻り値       1:反応あり 0:反応なし
///////////////////////////////////////////////////////////////////////////
unsigned char Rget( void ){
	volatile unsigned char	s;
	
	if(sensorR_reg < DsensR) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 Cget
// 処理概要     デジタルセンサ読み込み
// 引数         なし
// 戻り値       1:反応あり 0:反応なし
///////////////////////////////////////////////////////////////////////////
unsigned char Cget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorC_reg] < DsensC) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 Lget
// 処理概要     デジタルセンサ読み込み
// 引数         なし
// 戻り値       1:反応あり 0:反応なし
///////////////////////////////////////////////////////////////////////////
unsigned char Lget( void ){
	volatile unsigned char	s;
	
	if(sensorL_reg < DsensL) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 LLget
// 処理概要     デジタルセンサ読み込み
// 引数         なし
// 戻り値       1:反応あり 0:反応なし
///////////////////////////////////////////////////////////////////////////
unsigned char LLget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorLL_reg] < DsensLL) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 sensor_inp
// 処理概要     デジタルセンサの値を16進数で取得
// 引数         なし
// 戻り値       センサ値0～
///////////////////////////////////////////////////////////////////////////
unsigned char sensor_inp( unsigned char mask ) 
{
	volatile unsigned char sensor;
	static volatile unsigned char RR, R, C, L, LL;

	RR_data = RRget();
	R_data  = Rget();
	C_data  = Cget();
	L_data  = Lget();
	LL_data = LLget();
	
	RR = ( RR_data ) & 0x01;
	R  = (R_data << 1) & 0x02;
	C  = (C_data << 2) & 0x04;
  	L  = (L_data << 3) & 0x08;
	LL = (LL_data << 4) & 0x10;
	
	sensor = LL | L | C | R | RR;
	sensor &= mask;
	
	return sensor;
	
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 startbar_get
// 処理概要     スタートゲートの開閉の確認
// 引数         なし
// 戻り値       0; 閉じている 1; 開いてい
///////////////////////////////////////////////////////////////////////////
unsigned char startbar_get(void) 
{
	char ret;
	
	if ( sensorG <= 300 )	ret = 1;
	else			ret = 0;
	
	return ret;
}