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
* File Name        : Config_CMT0_user.c
* Component Version: 2.3.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_CMT0.
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
#include "Config_CMT0.h"
/* Start user code for include. Do not edit comment generated here */
#include "Config_S12AD0.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#define WDI (PORT3.PODR.BIT.B2)
#define UART12_timeout 100
volatile uint32_t tick=0;
volatile bool overflow;
uint16_t ADC_sample_count;
#define SAMPLES_NUM  64
volatile bool Sample_done=0;
extern uint16_t ADC_VGen1[SAMPLES_NUM], ADC_VGen2[SAMPLES_NUM], ADC_VGen3[SAMPLES_NUM];
extern uint16_t ADC_VGrid1[SAMPLES_NUM], ADC_VGrid2[SAMPLES_NUM], ADC_VGrid3[SAMPLES_NUM];
extern volatile bool adc_completed;
volatile uint8_t UART12_rxdone=0;
void ADC_Sample(uint16_t count);

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_CMT0_Create_UserInit
* Description  : This function adds user code after initializing the CMT0 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_CMT0_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_CMT0_cmi0_interrupt
* Description  : This function is CMI0 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_CMT0_CMI0
#pragma interrupt r_Config_CMT0_cmi0_interrupt(vect=VECT(CMT0,CMI0),fint)
#else
#pragma interrupt r_Config_CMT0_cmi0_interrupt(vect=VECT(CMT0,CMI0))
#endif
static void r_Config_CMT0_cmi0_interrupt(void)
{
    /* Start user code for r_Config_CMT0_cmi0_interrupt. Do not edit comment generated here */
	//System tick
	tick++;
	if(tick == 0xFFFFFFFF)
	{
		tick=0;
		overflow =1;
	}

	//ADC samples
	if(!Sample_done)
	{
		if (ADC_sample_count<SAMPLES_NUM)
		{
			ADC_Sample(ADC_sample_count);
			ADC_sample_count++;
		}
		if(ADC_sample_count >= SAMPLES_NUM)
		{
			ADC_sample_count=0;
			Sample_done =1; // notice to perform Calculating
		}
	}
	else
	{


	}

	// Watch Dog
	WDI =1;
	R_BSP_SoftwareDelay(5, BSP_DELAY_MICROSECS);
	WDI =0;
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
void ADC_Sample(uint16_t count)
{
	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL0,&ADC_VGrid1[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);

	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL1,&ADC_VGrid2[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);

	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL2,&ADC_VGrid3[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);

	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL3,&ADC_VGen1[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);

	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL4,&ADC_VGen2[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);

	R_Config_S12AD0_Start();
	while((!adc_completed) && (!R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS)));
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL5,&ADC_VGen3[count]);
	R_Config_S12AD0_Stop();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MICROSECS);


//	//VGrid1
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0001; //(AN000)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGrid1[count] = (uint16_t)(S12AD.ADDR0);
//	S12AD.ADCSR.BIT.ADST = 0;
//	//VGrid2
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0002; //(AN001)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGrid2[count] = (uint16_t)(S12AD.ADDR1);
//	S12AD.ADCSR.BIT.ADST = 0;
//	//VGrid3
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0004; //(AN002)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGrid3[count] = (uint16_t)(S12AD.ADDR2);
//	S12AD.ADCSR.BIT.ADST = 0;
//	//VGen1
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0008; //(AN003)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGen1[count] = (uint16_t)(S12AD.ADDR4);
//	S12AD.ADCSR.BIT.ADST = 0;
//	//VGen2
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0010; //(AN004)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGen2[count] = (uint16_t)(S12AD.ADDR4);
//	S12AD.ADCSR.BIT.ADST = 0;
//	//VGen3
//	S12AD.ADCSR.BIT.ADST = 0;
//	S12AD.ADANSA0.WORD = 0x0020; //(AN005)
//	S12AD.ADCSR.BIT.ADST = 1;
//	R_BSP_SoftwareDelay(80, BSP_DELAY_MICROSECS);
//	ADC_VGen3[count] = (uint16_t)(S12AD.ADDR4);
//	S12AD.ADCSR.BIT.ADST = 0;
//
//    S12AD.ADANSA0.WORD = _0001_AD_AN000_USED | _0002_AD_AN001_USED | _0004_AD_AN002_USED | _0008_AD_AN003_USED |  _0010_AD_AN004_USED | _0020_AD_AN005_USED;
//	S12AD.ADCSR.BIT.ADST = 1;
//	S12AD.ADCSR.BIT.ADST = 0;


	adc_completed=0;
}

/* End user code. Do not edit comment generated here */
