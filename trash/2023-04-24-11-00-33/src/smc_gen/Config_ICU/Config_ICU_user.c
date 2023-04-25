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
* File Name        : Config_ICU_user.c
* Component Version: 2.3.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_ICU.
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
#include "Config_ICU.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
extern volatile bool overflow;
extern volatile uint32_t tick;
uint32_t timestamp0[2],timestamp1[2],timestamp4[2],timestamp5[2],timestamp6[2],timestamp7[2];
float period0,period1,period4,period5,period6,period7;
uint8_t state0=0;
uint8_t state1=0;
uint8_t state4=0;
uint8_t state5=0;
uint8_t state6=0;
uint8_t state7=0;
#define LED1 (PORT2.PODR.BIT.B7)
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_ICU_Create_UserInit
* Description  : This function adds user code after initializing the ICU module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_ICU_irq0_interrupt
* Description  : This function is IRQ0 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ0
#pragma interrupt r_Config_ICU_irq0_interrupt(vect=VECT(ICU,IRQ0),fint)
#else
#pragma interrupt r_Config_ICU_irq0_interrupt(vect=VECT(ICU,IRQ0))
#endif
static void r_Config_ICU_irq0_interrupt(void)
{
    /* Start user code for r_Config_ICU_irq0_interrupt. Do not edit comment generated here */
	if(state0==0)// wait for 1st edge
	{
		timestamp0[0] = tick;
		state0=1; // recorded 1st edge, wait for second edge
		LED1=0;
	}
	else if(state0==1) // wait for second edge
	{
		timestamp0[1] = tick;
		if(timestamp0[1]>timestamp0[0])
		{
			period0 = timestamp0[1]-timestamp0[0];
		}
		else
		{
			period0 = 0xFFFFFFFF - timestamp0[0] + timestamp0[1];
		}
		state0 = 2;// period ready;
		LED1=1;
	}

    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_ICU_irq1_interrupt
* Description  : This function is IRQ1 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ1
#pragma interrupt r_Config_ICU_irq1_interrupt(vect=VECT(ICU,IRQ1),fint)
#else
#pragma interrupt r_Config_ICU_irq1_interrupt(vect=VECT(ICU,IRQ1))
#endif
static void r_Config_ICU_irq1_interrupt(void)
{
    /* Start user code for r_Config_ICU_irq1_interrupt. Do not edit comment generated here */
	if(state1==0)// wait for 1st edge
	{
		timestamp1[0] = tick;
		state1=1; // recorded 1st edge, wait for second edge
		LED1=0;
	}
	else if(state1==1) // wait for second edge
	{
		timestamp1[1] = tick;
		if(timestamp1[1]>timestamp1[0])
		{
			period1 = timestamp1[1]-timestamp1[0];
		}
		else
		{
			period1 = 0xFFFFFFFF - timestamp1[0] + timestamp1[1];
		}
		state1 = 2;// period ready;
		LED1=1;
	}

    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_ICU_irq4_interrupt
* Description  : This function is IRQ4 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ4
#pragma interrupt r_Config_ICU_irq4_interrupt(vect=VECT(ICU,IRQ4),fint)
#else
#pragma interrupt r_Config_ICU_irq4_interrupt(vect=VECT(ICU,IRQ4))
#endif
static void r_Config_ICU_irq4_interrupt(void)
{
    /* Start user code for r_Config_ICU_irq4_interrupt. Do not edit comment generated here */
	if(state4==0)// wait for 1st edge
	{
		timestamp4[0] = tick;
		state4=1; // recorded 1st edge, wait for second edge
		LED1 =0;
	}
	else if(state4==1) // wait for second edge
	{
		timestamp4[1] = tick;
		if(timestamp4[1]>timestamp4[0])
		{
			period4 = timestamp4[1]-timestamp4[0];
		}
		else
		{
			period4 = 0xFFFFFFFF - timestamp4[0] + timestamp4[1];
		}
		state4 = 2;// period ready;
		LED1 =1;
	}
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

