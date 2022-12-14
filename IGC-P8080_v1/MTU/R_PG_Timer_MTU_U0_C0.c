/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, 
* INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
* PARTICULAR PURPOSE AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY 
* DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES 
* FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS 
* AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this 
* software and to discontinue the availability of this software.  
* By using this software, you agree to the additional terms and 
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
******************************************************************************
* Copyright (C) 2010-2013 Renesas Electronics Corporation.
* and Renesas Solutions Corporation. All rights reserved.
******************************************************************************
* File Name    : R_PG_Timer_MTU_U0_C0.c
* Version      : 1.00
* Device(s)    : 
* Tool-Chain   : 
* H/W Platform : 
* Description  : 
* Limitations  : 
******************************************************************************
* History : 30.05.2022 Version Description
*         :   
******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_pdl_mtu2.h"
#include "r_pdl_intc.h"
#include "r_pdl_definitions.h"
#include "R_PG_IntFuncsExtern.h"


/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_Set_MTU_U0_C0(void)
*
* Function Name: R_PG_Timer_Set_MTU_U0_C0
*
* Description  : MTU??????
*
* Arguments    : ????
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_Create
*                   : R_MTU2_Create_load_defaults
*                   : R_MTU2_Set
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_Set_MTU_U0_C0(void)
{
	bool res;
	R_MTU2_Create_structure parameters;

	R_MTU2_Create_load_defaults( &parameters );

	parameters.channel_mode = PDL_MTU2_MODE_PWM1;
	parameters.counter_operation = PDL_MTU2_CLK_PCLK_DIV_1 | PDL_MTU2_CLK_RISING | PDL_MTU2_CLEAR_TGRA;
	parameters.TGR_A_B_operation = PDL_MTU2_A_OC_LOW_CM_HIGH | PDL_MTU2_B_OC_LOW;
	parameters.TGR_C_D_operation = PDL_MTU2_C_OC_LOW_CM_HIGH | PDL_MTU2_D_OC_LOW;
	parameters.TGRA_TCNTV_value = 1583;
	parameters.TGRB_TCNTW_value = 0;
	parameters.TGRC_TGRU_value = 1583;
	parameters.TGRD_TGRV_value = 0;
	parameters.TGRE_TGRW_value = 0;
	parameters.TGRF_TADCORA_value = 0;

	res = R_MTU2_Set(
		0,
		PDL_MTU2_PIN_0A_PB3 | PDL_MTU2_PIN_0C_PB1,
		PDL_NO_DATA
	);

	if( !res ){
		return res;
	}

	return R_MTU2_Create(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_StartCount_MTU_U0_C0(void)
*
* Function Name: R_PG_Timer_StartCount_MTU_U0_C0
*
* Description  : MTU???J?E???g?????J?n
*
* Arguments    : ????
*
* Return Value : true  : ?J?E???g?????????J?????????s??????????
*              : false : ?J?E???g?????????J?????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_StartCount_MTU_U0_C0(void)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_MTU2_START;
	parameters.register_selection = PDL_NO_DATA;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_HaltCount_MTU_U0_C0(void)
*
* Function Name: R_PG_Timer_HaltCount_MTU_U0_C0
*
* Description  : MTU???J?E???g?????????????~
*
* Arguments    : ????
*
* Return Value : true  : ???~??????????????
*              : false : ???~?????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_HaltCount_MTU_U0_C0(void)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_MTU2_STOP;
	parameters.register_selection = PDL_NO_DATA;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_GetCounterValue_MTU_U0_C0(uint16_t * counter_val)
*
* Function Name: R_PG_Timer_GetCounterValue_MTU_U0_C0
*
* Description  : MTU???J?E???^?l??????
*
* Arguments    : uint16_t * counter_val : ?J?E???^?l???i?[??
*
* Return Value : true  : ??????????????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ReadChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_GetCounterValue_MTU_U0_C0(uint16_t * counter_val)
{
	if( counter_val == 0 ){ return false; }

	return R_MTU2_ReadChannel(
		0,
		PDL_NO_PTR,
		counter_val,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetCounterValue_MTU_U0_C0(uint16_t counter_val)
*
* Function Name: R_PG_Timer_SetCounterValue_MTU_U0_C0
*
* Description  : MTU???J?E???^?l??????
*
* Arguments    : uint16_t counter_val : ?J?E???^???????????l
*
* Return Value : true  : ?J?E???^?l????????????????????
*              : false : ?J?E???^?l???????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetCounterValue_MTU_U0_C0(uint16_t counter_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_COUNTER;
	parameters.TCNT_TCNTU_value = counter_val;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_GetTGR_MTU_U0_C0(uint16_t * tgr_a_val, uint16_t * tgr_b_val, uint16_t * tgr_c_val, uint16_t * tgr_d_val, uint16_t * tgr_e_val, uint16_t * tgr_f_val)
*
* Function Name: R_PG_Timer_GetTGR_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l??????
*
* Arguments    : uint16_t * tgr_a_val : ?W?F?l???????W?X?^A?l???i?[??
*              : uint16_t * tgr_b_val : ?W?F?l???????W?X?^B?l???i?[??
*              : uint16_t * tgr_c_val : ?W?F?l???????W?X?^C?l???i?[??
*              : uint16_t * tgr_d_val : ?W?F?l???????W?X?^D?l???i?[??
*              : uint16_t * tgr_e_val : ?W?F?l???????W?X?^E?l???i?[??
*              : uint16_t * tgr_f_val : ?W?F?l???????W?X?^F?l???i?[??
*
* Return Value : true  : ??????????????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ReadChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_GetTGR_MTU_U0_C0(uint16_t * tgr_a_val, uint16_t * tgr_b_val, uint16_t * tgr_c_val, uint16_t * tgr_d_val, uint16_t * tgr_e_val, uint16_t * tgr_f_val)
{
	uint16_t * local_a = PDL_NO_PTR;
	uint16_t * local_b = PDL_NO_PTR;
	uint16_t * local_c = PDL_NO_PTR;
	uint16_t * local_d = PDL_NO_PTR;
	uint16_t * local_e = PDL_NO_PTR;
	uint16_t * local_f = PDL_NO_PTR;

	if( tgr_a_val ){
		local_a = tgr_a_val;
	}
	if( tgr_b_val ){
		local_b = tgr_b_val;
	}
	if( tgr_c_val ){
		local_c = tgr_c_val;
	}
	if( tgr_d_val ){
		local_d = tgr_d_val;
	}
	if( tgr_e_val ){
		local_e = tgr_e_val;
	}
	if( tgr_f_val ){
		local_f = tgr_f_val;
	}

	return R_MTU2_ReadChannel(
		0,
		PDL_NO_PTR,
		PDL_NO_PTR,
		local_a,
		local_b,
		local_c,
		local_d,
		local_e,
		local_f
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_A_MTU_U0_C0(uint16_t tgr_a_val)
*
* Function Name: R_PG_Timer_SetTGR_A_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRA)
*
* Arguments    : uint16_t tgr_a_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_A_MTU_U0_C0(uint16_t tgr_a_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRA;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = tgr_a_val;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_B_MTU_U0_C0(uint16_t tgr_b_val)
*
* Function Name: R_PG_Timer_SetTGR_B_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRB)
*
* Arguments    : uint16_t tgr_b_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_B_MTU_U0_C0(uint16_t tgr_b_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRB;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = tgr_b_val;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_C_MTU_U0_C0(uint16_t tgr_c_val)
*
* Function Name: R_PG_Timer_SetTGR_C_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRC)
*
* Arguments    : uint16_t tgr_c_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_C_MTU_U0_C0(uint16_t tgr_c_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRC;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = tgr_c_val;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_D_MTU_U0_C0(uint16_t tgr_d_val)
*
* Function Name: R_PG_Timer_SetTGR_D_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRD)
*
* Arguments    : uint16_t tgr_d_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_D_MTU_U0_C0(uint16_t tgr_d_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRD;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = tgr_d_val;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_E_MTU_U0_C0(uint16_t tgr_e_val)
*
* Function Name: R_PG_Timer_SetTGR_E_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRE)
*
* Arguments    : uint16_t tgr_e_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_E_MTU_U0_C0(uint16_t tgr_e_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRE;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = tgr_e_val;
	parameters.TGRF_value = PDL_NO_DATA;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_SetTGR_F_MTU_U0_C0(uint16_t tgr_f_val)
*
* Function Name: R_PG_Timer_SetTGR_F_MTU_U0_C0
*
* Description  : ?W?F?l???????W?X?^???l?????? (TGRF)
*
* Arguments    : uint16_t tgr_f_val : ?W?F?l???????W?X?^???????????l
*
* Return Value : true  : ?????????????s??????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ControlChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_SetTGR_F_MTU_U0_C0(uint16_t tgr_f_val)
{
	R_MTU2_ControlChannel_structure parameters;

	parameters.control_setting = PDL_NO_DATA;
	parameters.register_selection = PDL_MTU2_REGISTER_TGRF;
	parameters.TCNT_TCNTU_value = PDL_NO_DATA;
	parameters.TGRA_TCNTV_value = PDL_NO_DATA;
	parameters.TGRB_TCNTW_value = PDL_NO_DATA;
	parameters.TGRC_TGRU_value = PDL_NO_DATA;
	parameters.TGRD_TGRV_value = PDL_NO_DATA;
	parameters.TGRE_TGRW_value = PDL_NO_DATA;
	parameters.TGRF_value = tgr_f_val;
	parameters.TADCOBRA_value = PDL_NO_DATA;
	parameters.TADCOBRB_value = PDL_NO_DATA;

	return R_MTU2_ControlChannel(
		0,
		&parameters
	);

}

/******************************************************************************
* ID           : 
*
* Include      : 
*
* Declaration  : bool R_PG_Timer_GetRequestFlag_MTU_U0_C0(bool * cm_ic_a, bool * cm_ic_b, bool * cm_ic_c, bool * cm_ic_d, bool * cm_e, bool * cm_f, bool * ov, bool * un)
*
* Function Name: R_PG_Timer_GetRequestFlag_MTU_U0_C0
*
* Description  : MTU???????????v???t???O?????????N???A
*
* Arguments    : bool * cm_ic_a : ?R???y?A?}?b?`/?C???v?b?g?L???v?`??A?t???O???i?[??
*              : bool * cm_ic_b : ?R???y?A?}?b?`/?C???v?b?g?L???v?`??B?t???O???i?[??
*              : bool * cm_ic_c : ?R???y?A?}?b?`/?C???v?b?g?L???v?`??C?t???O???i?[??
*              : bool * cm_ic_d : ?R???y?A?}?b?`/?C???v?b?g?L???v?`??D?t???O???i?[??
*              : bool * cm_e : ?R???y?A?}?b?`E?t???O???i?[??
*              : bool * cm_f : ?R???y?A?}?b?`F?t???O???i?[??
*              : bool * ov : ?I?[?o?t???[?t???O???i?[??
*              : bool * un : ?A???_?t???[?t???O???i?[??
*
* Return Value : true  : ??????????????????
*              : false : ?????????s????????
*
* Calling Functions : R_MTU2_ReadChannel
*
* Details      : ?????????????????t?@?????X?}?j???A?????Q???????????????B
******************************************************************************/
bool R_PG_Timer_GetRequestFlag_MTU_U0_C0(bool * cm_ic_a, bool * cm_ic_b, bool * cm_ic_c, bool * cm_ic_d, bool * cm_e, bool * cm_f, bool * ov, bool * un)
{
	uint8_t data;
	bool res;

	res = R_MTU2_ReadChannel(
		0,
		&data,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR,
		PDL_NO_PTR
	);

	if( cm_ic_a ){
		*cm_ic_a = (data >> 1)& 0x01;
	}
	if( cm_ic_b ){
		*cm_ic_b = (data >> 2) & 0x01;
	}
	if( cm_ic_c ){
		*cm_ic_c = (data >> 3) & 0x01;
	}
	if( cm_ic_d ){
		*cm_ic_d = (data >> 4) & 0x01;
	}
	if( cm_e ){
		*cm_e = (data >> 5) & 0x01;
	}
	if( cm_f ){
		*cm_f = (data >> 6) & 0x01;
	}
	if( ov ){
		*ov = (data >> 7) & 0x01;
	}
	if( un ){
		*un = 0x00;
	}

	return res;
}



