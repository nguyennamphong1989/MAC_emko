/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_MTU3_user.c
* Component Version: 1.12.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_MTU3.
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "Config_MTU3.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
volatile uint8_t l3a_edge_detected =0;
volatile uint16_t l3a_end_value =0;
volatile uint16_t l3a_start_value =0;
volatile float l3a_period =0;

volatile uint8_t l3c_edge_detected =0;
volatile uint16_t l3c_end_value =0;
volatile uint16_t l3c_start_value =0;
volatile float l3c_period =0;
float grid_freq2=0;
float grid_freq1=0;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_Create_UserInit
* Description  : This function adds user code after initializing the MTU3 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU3_tgia3_interrupt
* Description  : This function is TGIA3 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_MTU3_TGIA3
#pragma interrupt r_Config_MTU3_tgia3_interrupt(vect=VECT(MTU3,TGIA3),fint)
#else
#pragma interrupt r_Config_MTU3_tgia3_interrupt(vect=VECT(MTU3,TGIA3))
#endif
static void r_Config_MTU3_tgia3_interrupt(void)
{
    /* Start user code for r_Config_MTU3_tgia3_interrupt. Do not edit comment generated here */
	if(l3a_edge_detected<3)
	{
		l3a_edge_detected++;
		grid_freq1 =0;
	}
	if(l3a_edge_detected==1) //first edge
	{
		l3a_start_value = MTU3.TGRA; // ghi lai gia tri bat dau

	}
	if (l3a_edge_detected==2) //second edge
	{
		l3a_end_value = MTU3.TGRA;
		if(l3a_end_value >l3a_start_value)
		{
			l3a_period = l3a_end_value - l3a_start_value;
			l3a_period = l3a_period/1000000;
			grid_freq1 = 0.125/l3a_period;
			l3a_edge_detected=3; // data ready
		}
		else l3a_edge_detected=0;
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU3_tgic3_interrupt
* Description  : This function is TGIC3 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_MTU3_TGIC3
#pragma interrupt r_Config_MTU3_tgic3_interrupt(vect=VECT(MTU3,TGIC3),fint)
#else
#pragma interrupt r_Config_MTU3_tgic3_interrupt(vect=VECT(MTU3,TGIC3))
#endif
static void r_Config_MTU3_tgic3_interrupt(void)
{
    /* Start user code for r_Config_MTU3_tgic3_interrupt. Do not edit comment generated here */
	if(l3c_edge_detected<3)
	{
		l3c_edge_detected++;
		grid_freq2 = 0;
	}
	if(l3c_edge_detected==1) //first edge
	{
		l3c_start_value = MTU3.TGRC; // ghi lai gia tri bat dau

	}
	if (l3c_edge_detected==2) //second edge
	{
		l3c_end_value = MTU3.TGRC;
		if(l3c_end_value >l3c_start_value)
		{
			l3c_period = l3c_end_value - l3c_start_value;
			l3c_period = l3c_period/1000000;
			grid_freq2 = 0.125/l3c_period;
			l3c_edge_detected=3; // data ready
		}
		else l3c_edge_detected=0;
	}
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
