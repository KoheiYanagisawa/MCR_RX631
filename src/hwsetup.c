/************************************************************************
*
* Device     : RX/RX600/RX63N,RX631
*
* File Name  : hwsetup.c
*
* Abstract   : Hardware Setup file.
*
* History    : 0.10  (2011-02-21)  [Hardware Manual Revision : 0.01]
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2011 Renesas Electronics Corporation.
* and Renesas Solutions Corp.
*
************************************************************************/

#include "iodefine.h"
#include "io.h"
#include "mtu.h"
#include "ADconverter.h"
#include "control.h"
#include "microSD.h"
#include "AQM0802A.h"
#include "ICM20648.h"
#include "sci.h"

extern void HardwareSetup(void);

// ID�R�[�h�ݒ�(45ffffffffffffffffffffffffffffff)
#pragma address id_code=0xffffffa0 // ID codes (Default)
const unsigned long id_code[4] = {
        0x45ffffff,
        0xffffffff,
        0xffffffff,
        0xffffffff,
};

void HardwareSetup(void)
{
	R_PG_IO_PORT_SetPortNotAvailable();	// ���݂��Ȃ��|�[�g��ݒ�
	R_PG_Clock_WaitSet(0.01); 		// �N���b�N��ݒ肵0.01�b��ɃN���b�N�\�[�X�؂�ւ�
	
	SET_MTU_C0		// �}���`�t�@���N�V�����^�C�}��ݒ�
	SET_MTU_C1
	SET_MTU_C2
	SET_MTU_C3
	SET_MTU_C4
	
	SET_SCI_C2
	SET_SCI_MSD		// �V���A��I/O�`���l����ݒ�(SPI microSd)
	initSCI1( RATE_230400 );		// �V���A��I/O�`���l����ݒ�(UART)
	SET_SCI_LCD 		// �V���A��I/O�`���l����ݒ�(I2C)
	
	SET_CMT_C0		// �R���y�A�}�b�`�^�C�}��ݒ�(ch0)
	SET_CMT_MSD		// �R���y�A�}�b�`�^�C�}��ݒ�(ch2)
	
	initIO();			// IO�|�[�g�̏�����
	
	SET_ADC			// 12�r�b�gA/D�R���o�[�^(S12AD0)��ݒ�
	
	START_MTU		// MTU0,1,2,3,4�̃J�E���g�J�n
	
	START_ADC		// A/D�ϊ��J�n
	START_CMT_C0 	// �J�E���g�X�^�[�g(ch0)
	START_CMT_MSD 	// �J�E���g�X�^�[�g(ch2)
	
	/*----------0:brake�@1:stop----------*/	
	R_PG_IO_PORT_Write_PE4( 0 );	//���O���[�^
	R_PG_IO_PORT_Write_PA7( 0 );	//�E�O���[�^
	R_PG_IO_PORT_Write_PC5( 0 );	//���ヂ�[�^
	R_PG_IO_PORT_Write_PB7( 0 );	//�E�ヂ�[�^
	R_PG_IO_PORT_Write_PE7( 1 );	//�T�[�{���[�^
	R_PG_IO_PORT_Write_PC3( 1 );	//�����T�[���[�^
}