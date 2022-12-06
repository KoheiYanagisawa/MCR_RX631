//====================================//
// �C���N���[�h
//====================================//
#include "sci.h"

//====================================//
// �O���[�o���ϐ��̐錾
//====================================//
char		revErr = 0;			// �ʐM�G���[�ԍ�
// SCI1�֘A
char		modeSCI1;			// �ʐM����
char		txtCommand[128];	// �R�}���h�i�[
char		txtData[128];		// �f�[�^�i�[
char*		txt = txtData;					// ��M�f�[�^�i�[
char		cmmandMode = 0;		// �R�}���h�I��
char		stopWord = 0;		// 0: ��~���[�h����M 1:��~���[�h��M
short 		cntByte = 0;			// ��M�����o�C�g��
char 		command = 0;		// 0:�R�}���h��M�҂� 1:�R�}���h���͒� 2:�R�}���h���蒆

char		SCI1_Req_mode;		// 0:�X�^�[�g 1:�X�g�b�v 2:�f�[�^����M��
char		SCI1_RW_mode;		// 0:���M 1:��M
char		SCI1_Slaveaddr;		// �X���[�u�A�h���X
char		SCI1_NumData;		// ���M�f�[�^��
char*		SCI1_DataArry;			// ���M�f�[�^�z��
char		SCI1_DataBuff[255];	// ���M�f�[�^�o�b�t�@

// SCI12�֘A
char		SCI12_Req_mode = 0;		// 0:�X�^�[�g 1:�X�g�b�v
char		SCI12_Slaveaddr;		// ���M�f�[�^��
char		SCI12_NumData;			// �f�[�^��
char		SCI1_NumData2;			// ���M�f�[�^��2
char*		SCI12_DataArry;			// �f�[�^�z��
char*		SCI1_DataArry2;			// ���M�f�[�^�z��2
char		SCI12_DataBuff[255];	// ���M�f�[�^�o�b�t�@

#pragma interrupt Excep_SCI6_RXI6 (vect = VECT_SCI6_RXI6, enable)	// RXI1���荞�݊֐���`
#pragma interrupt Excep_SCI6_TXI6 (vect = VECT_SCI6_TXI6, enable)	// TXI1���荞�݊֐���`
#pragma interrupt Excep_SCI6_TEI6 (vect = VECT_SCI6_TEI6, enable)	// TEI1���荞�݊֐���`

///////////////////////////////////////////////////////////////////////////
// ���W���[���� initSCI1
// �����T�v     SCI1�̏�����
// ����         mode: �ʐM���� rate:�{�[���[�g��bps�œ���
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void initSCI1( char rate )
{
	unsigned char brr,abcs;
	
		// �{�[���[�g�I��
		if ( rate == RATE_9600 ) {
		abcs = 0;
		brr = 155;
	} else if ( rate == RATE_14400 ) {
		abcs = 0;
		brr = 103;
	} else if ( rate == RATE_19200 ) {
		abcs = 0;
		brr = 77;
	} else if ( rate == RATE_38400 ) {
		abcs = 0;
		brr = 38;
	} else if ( rate == RATE_57600 ) {
		abcs = 0;
		brr = 25;
	} else if ( rate == RATE_115200 ) {
		abcs = 0;
		brr = 12;
	} else if ( rate == RATE_230400 ) {
		abcs = 1;
		brr = 12;
	} else if ( rate == RATE_500000 ) {
		abcs = 0;
		brr = 2;
	} else if ( rate == RATE_750000 ) {
		abcs = 0;
		brr = 1;
	} else if ( rate == RATE_1000000 ) {
		abcs = 1;
		brr = 2;
	} else if ( rate == RATE_1500000 ) {
		abcs = 0;
		brr = 0;
	} else if ( rate == RATE_3000000 ) {
		abcs = 1;
		brr = 0;
	}
	
	// SCI1
	SYSTEM.PRCR.WORD = 0xA502;		// Release protect
	MSTP(SCI1) = 0;					// Wake up SCI1
	SYSTEM.PRCR.WORD = 0xA500;		// Protect
	
	SCI1.SCR.BYTE = 0;				//Set PFC of external pin used
	
	//__set_fintv((unsigned int*)FastInterupt);
	 
	ICU.IER[IER_SCI1_RXI1].BIT.IEN_SCI1_RXI1 = 1;	// RXI??????J?n
	ICU.IPR[VECT_SCI1_RXI1].BIT.IPR = 15;			// RXI?????????
	
	// Set MPC
	PORT3.PMR.BIT.B0 = 1;			// Disable PB1: peripheral
	PORT2.PMR.BIT.B6 = 1;			// Disable PB0: peripheral
	MPC.PWPR.BIT.B0WI = 0;			// Release protect
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P30PFS.BIT.PSEL = 0x0A;		// Set PB1: TXD1
	MPC.P26PFS.BIT.PSEL = 0x0A;		// Set PB0: RXD1
	MPC.PWPR.BIT.PFSWE = 0;			// Protect
	MPC.PWPR.BIT.B0WI = 1;
	PORT3.PMR.BIT.B0 = 1;			// PB1: peripheral
	PORT2.PMR.BIT.B6 = 1;			// PB0: peripheral
	
	// SCI1 Settings
	SCI1.SCR.BIT.CKE = 0;			// Disable CKE
	SCI1.SMR.BIT.CKS = 0;			// PCLK
	SCI1.SMR.BIT.MP = 0;			// Disable multiple processor
	
	SCI1.SIMR1.BIT.IICM = 0;
	SCI1.SPMR.BIT.CKPH = 0;
	SCI1.SPMR.BIT.CKPOL = 0;
	SCI1.SCMR.BIT.SMIF = 0;
	SCI1.SEMR.BIT.ACS0 = 0;
	SCI1.SEMR.BIT.ABCS = abcs;		// 1?r?b?g?]?????????N???b?N?T?C?N?????@0: 16 1: 8
	
	SCI1.SMR.BIT.STOP = 0;			// 1 stop bit
	SCI1.SMR.BIT.PM = 0;			// none parity
	SCI1.SMR.BIT.CHR = 0;			// 8bit data length
	SCI1.SMR.BIT.CM = 0;			// ??????????
	SCI1.BRR = brr;					// 12: 115200bps 1:750000bps 0:1500000bps
	SCI1.SCR.BIT.RIE = 1;			// RXI??????v??
	SCI1.SCR.BIT.TE = 1;			// Enable TX
	SCI1.SCR.BIT.RE = 1;			// Enable RX
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� initSCI6
// �����T�v     SCI6�̏�����
// ����         mode: �ʐM���� rate:�{�[���[�g��bps�œ���
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void initSCI6( char rate )
{
	unsigned char brr,abcs;
	
		// �{�[���[�g�I��
		if ( rate == RATE_9600 ) {
		abcs = 0;
		brr = 155;
	} else if ( rate == RATE_14400 ) {
		abcs = 0;
		brr = 103;
	} else if ( rate == RATE_19200 ) {
		abcs = 0;
		brr = 77;
	} else if ( rate == RATE_38400 ) {
		abcs = 0;
		brr = 38;
	} else if ( rate == RATE_57600 ) {
		abcs = 0;
		brr = 25;
	} else if ( rate == RATE_115200 ) {
		abcs = 0;
		brr = 12;
	} else if ( rate == RATE_230400 ) {
		abcs = 1;
		brr = 12;
	} else if ( rate == RATE_500000 ) {
		abcs = 0;
		brr = 2;
	} else if ( rate == RATE_750000 ) {
		abcs = 0;
		brr = 1;
	} else if ( rate == RATE_1000000 ) {
		abcs = 1;
		brr = 2;
	} else if ( rate == RATE_1500000 ) {
		abcs = 0;
		brr = 0;
	} else if ( rate == RATE_3000000 ) {
		abcs = 1;
		brr = 0;
	}
	
	// SCI1
	SYSTEM.PRCR.WORD = 0xA502;		// Release protect
	MSTP(SCI6) = 0;				// Wake up SCI6
	SYSTEM.PRCR.WORD = 0xA500;		// Protect
	
	SCI6.SCR.BYTE = 0;			//Set PFC of external pin used
	
	//__set_fintv((unsigned int*)FastInterupt);
	 
	ICU.IER[IER_SCI6_RXI6].BIT.IEN_SCI6_RXI6 = 1;	// RXI??????J?n
	ICU.IPR[VECT_SCI6_RXI6].BIT.IPR = 15;		// RXI?????????
	IEN( SCI6, RXI6 ) = 1;	// RXI���荞�݊J�n
	
	// Set MPC
	PORT3.PMR.BIT.B3 = 1;			// Disable P33: peripheral
	PORT3.PMR.BIT.B2 = 1;			// Disable P32: peripheral
	MPC.PWPR.BIT.B0WI = 0;			// Release protect
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P33PFS.BIT.PSEL = 0x0A;		// Set P33: RXD6
	MPC.P32PFS.BIT.PSEL = 0x0A;		// Set P32: TXD6
	MPC.PWPR.BIT.PFSWE = 0;			// Protect
	MPC.PWPR.BIT.B0WI = 1;
	PORT3.PMR.BIT.B3 = 1;			// P33: peripheral
	PORT2.PMR.BIT.B2 = 1;			// P32: peripheral
	
	// SCI6 Settings
	SCI6.SCR.BIT.CKE = 0;			// Disable CKE
	SCI6.SMR.BIT.CKS = 0;			// PCLK
	SCI6.SMR.BIT.MP = 0;			// Disable multiple processor
	
	SCI6.SIMR1.BIT.IICM = 0;
	SCI6.SPMR.BIT.CKPH = 0;
	SCI6.SPMR.BIT.CKPOL = 0;
	SCI6.SCMR.BIT.SMIF = 0;
	SCI6.SEMR.BIT.ACS0 = 0;
	SCI6.SEMR.BIT.ABCS = abcs;		// 1?r?b?g?]?????????N???b?N?T?C?N?????@0: 16 1: 8
	
	SCI6.SMR.BIT.STOP = 0;			// 1 stop bit
	SCI6.SMR.BIT.PM = 0;			// none parity
	SCI6.SMR.BIT.CHR = 0;			// 8bit data length
	SCI6.SMR.BIT.CM = 0;			// ??????????
	SCI6.BRR = brr;					// 12: 115200bps 1:750000bps 0:1500000bps
	SCI6.SCR.BIT.RIE = 1;			// RXI??????v??
	SCI6.SCR.BIT.TE = 1;			// Enable TX
	SCI6.SCR.BIT.RE = 1;			// Enable RX
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� charput
// �����T�v     printf�̏o��(printf�Ŏg�p����)
// ����         data:�o�͂���ꕶ��
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void charput( uint8_t data )
{
	while(SCI1.SSR.BIT.TEND == 0);
	SCI1.TDR = data;
	SCI1.SSR.BIT.TEND = 0;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� charget
// �����T�v     scanf�̓���(scanf�Ŏg�p����)
// ����         �Ȃ�
// �߂�l       data:���͂����ꕶ��
///////////////////////////////////////////////////////////////////////////
unsigned char charget(void)
{
	uint8_t data;
	data = SCI1.RDR;
	return data;
}
///////////////////////////////////////////////////////////////////////////
// ���W���[���� Excep_SCI1_RXI1
// �����T�v     UART��M���Ɋ��荞�݂Ŏ��s�����
// ����         �Ȃ�
// �߂�l       �Ȃ�
///////////////////////////////////////////////////////////////////////////
void Excep_SCI6_RXI6(void)
{
	char c;
	c = SCI1.RDR;
	
	*txt++ = c;
	if ( c == 0xA) {	// ���s�R�[�h�ŕ����񃊃Z�b�g
		memset(txtData, 0, strlen(txtData));
		txt = txtData;	// �A�h���X���Z�b�g
	}
}