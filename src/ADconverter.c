//====================================//
// �C���N���[�h
//====================================//
#include "ADconverter.h"
//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
// �^�C�}�֘A
static char				ADTimer10;	// AD�ϊ��J�E���g�p



// �Z���T�֘A
// AD�ϊ����ʊi�[
static unsigned short 	result[14]; 	// 12bitA/D�ϊ����ʂ̊i�[��
static int			senR;		// �E�A�i���O�Z���T�ώZAD�l
static int			senL;		// ���A�i���O�Z���T�ώZAD�l
static int			senG;		// �Q�[�g�Z���T�ώZAD�l
static int			senC;		// ���S�A�i���O�Z���T�ώZAD�l
static int			senLL;		// �ō��[�A�i���O�Z���T�ώZAD�l
static int			senRR;		// �ŉE�[�A�i���O�Z���T�ώZAD�l
static int			VolC;		// �d���`�F�b�J�[AD�l
static int			pot;		// �|�e���V�������[�^�[�ώZAD�l
static int			sen_saka;
// AD�ϊ����ʕ��ϒl
short 				Angle;		// �|�e���V�������[�^�[����AD�l
short				sensor_saka;
short				sensor_calc[9];
short				sensorR;	// �E�A�i���O�Z���T����AD�l
short				sensorR_reg;
short				sensorR_max;	// �E�A�i���O�Z���TMAXAD�l
short				sensorR_min;	// �E�A�i���O�Z���TMINAD�l
short				sensorL;	// ���A�i���O�Z���T����AD�l
short				sensorL_reg;
short				sensorL_max;	// ���A�i���O�Z���TMAXAD�l
short				sensorL_min;	// ���A�i���O�Z���TMINAD�l
short				sensorG;	// �Q�[�g�Z���T����AD�l
short				sensorC;	// ���S�A�i���O�Z���T����AD�l
short				sensorLL;	// �ō��[�A�i���O�Z���T����AD�l
short				sensorRR;	// �ŉE�[�A�i���O�Z���T����AD�l
short				VoltageC;	// �d���`�F�b�J�[AD�l���ϒl
short				Angle0;		// �T�[�{�Z���^�[�l
short				iAngle;		// 
short				Angle_buff;		//

float				RR_tatio;
float				R_tatio;
float				C_tatio;
float				L_tatio;
float				LL_tatio;

double				Voltage;	// AD�l���琄�肵���d���d��
short				sensorG_th = GATE_VAL;	// �Q�[�g�J���������l
// �f�W�^���Z���T�f�[�^
volatile unsigned char 		RR_data;
volatile unsigned char 		R_data;
volatile unsigned char 		C_data;
volatile unsigned char 		L_data;
volatile unsigned char 		LL_data;
/////////////////////////////////////////////////////////////////////
// ���W���[���� ADconverter
// �����T�v     AD�ϊ����荞��
// ����         �Ȃ�
// �߂�l       �Ȃ�
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
	
	// AD�ϊ��l���o�b�t�@�Ɋi�[
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
// ���W���[���� getVoltage
// �����T�v     �d���̎擾
// ����         �Ȃ�
// �߂�l       ��
/////////////////////////////////////////////////////////////////////
void getVoltage ( void )
{
	// AD�l * R1 * R2 / ����\
	Voltage = VoltageC * 5.05 * 3.94 / 4096;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� getServoAngle
// �����T�v     �|�e���V�������[�^�[�̃A�i���O�l�Ŏ擾
// ����         �Ȃ�
// �߂�l       �Z���T�l
///////////////////////////////////////////////////////////////////////////
short getServoAngle(void) 
{	
	return  ( Angle0 - Angle );
}

/////////////////////////////////////////////////////////////////////
// ���W���[���� getsens_ratio
// �����T�v    	�Z���T�l�̐��K��
// ����         �Ȃ�
// �߂�l       ��
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
// ���W���[���� getAnalogSensor
// �����T�v     �A�i���O�Z���T�̃A�i���O�l�Ŏ擾
// ����         �Ȃ�
// �߂�l       �Z���T�l
///////////////////////////////////////////////////////////////////////////
short getAnalogSensor(void) 
{
	return sensorL_reg - sensorR_reg;
	//return sensorL - sensorR;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� RRget
// �����T�v     �f�W�^���Z���T�ǂݍ���
// ����         �Ȃ�
// �߂�l       1:�������� 0:�����Ȃ�
///////////////////////////////////////////////////////////////////////////
unsigned char RRget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorRR_reg] < DsensRR) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� Rget
// �����T�v     �f�W�^���Z���T�ǂݍ���
// ����         �Ȃ�
// �߂�l       1:�������� 0:�����Ȃ�
///////////////////////////////////////////////////////////////////////////
unsigned char Rget( void ){
	volatile unsigned char	s;
	
	if(sensorR_reg < DsensR) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� Cget
// �����T�v     �f�W�^���Z���T�ǂݍ���
// ����         �Ȃ�
// �߂�l       1:�������� 0:�����Ȃ�
///////////////////////////////////////////////////////////////////////////
unsigned char Cget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorC_reg] < DsensC) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� Lget
// �����T�v     �f�W�^���Z���T�ǂݍ���
// ����         �Ȃ�
// �߂�l       1:�������� 0:�����Ȃ�
///////////////////////////////////////////////////////////////////////////
unsigned char Lget( void ){
	volatile unsigned char	s;
	
	if(sensorL_reg < DsensL) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� LLget
// �����T�v     �f�W�^���Z���T�ǂݍ���
// ����         �Ȃ�
// �߂�l       1:�������� 0:�����Ȃ�
///////////////////////////////////////////////////////////////////////////
unsigned char LLget( void ){
	volatile unsigned char	s;
	
	if(sensor_calc[sensorLL_reg] < DsensLL) s = 1;
	else s = 0;
	
	return s;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� sensor_inp
// �����T�v     �f�W�^���Z���T�̒l��16�i���Ŏ擾
// ����         �Ȃ�
// �߂�l       �Z���T�l0�`
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
// ���W���[���� startbar_get
// �����T�v     �X�^�[�g�Q�[�g�̊J�̊m�F
// ����         �Ȃ�
// �߂�l       0; ���Ă��� 1; �J���Ă�
///////////////////////////////////////////////////////////////////////////
unsigned char startbar_get(void) 
{
	char ret;
	
	if ( sensorG <= 300 )	ret = 1;
	else			ret = 0;
	
	return ret;
}