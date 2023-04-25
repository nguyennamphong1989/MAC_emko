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
* File Name        : Config_MTU3.c
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
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_Create
* Description  : This function initializes the MTU3 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_Create(void)
{
    /* Release MTU channel 3 from stop state */
    MSTP(MTU) = 0U;

    /* Enable read/write to MTU3 registers */
    MTU.TRWER.BIT.RWE = 1U;

    /* Stop MTU channel 3 counter */
    MTU.TSTR.BIT.CST3 = 0U;

    /* Set TGIA/TGIB/TGIC/TGID interrupt priority level */
    IPR(MTU3, TGIA3) = _0F_MTU_PRIORITY_LEVEL15;

    /* MTU channel 3 is used as normal mode */
    MTU.TSYR.BIT.SYNC3 = 0U;
    MTU3.TCR.BYTE = _04_MTU_PCLK_256 | _00_MTU_CKEG_RISE | _00_MTU_CKCL_DIS;
    MTU3.TIER.BYTE = _01_MTU_TGIEA_ENABLE | _00_MTU_TGIEB_DISABLE | _04_MTU_TGIEC_ENABLE | _00_MTU_TGIED_DISABLE | 
                     _00_MTU_TCIEV_DISABLE | _00_MTU_TTGE_DISABLE;
    MTU3.TIORH.BYTE = _08_MTU_IOA_IR | _00_MTU_IOB_DISABLE;
    MTU3.TIORL.BYTE = _08_MTU_IOC_IR | _00_MTU_IOD_DISABLE;
    MTU3.TGRB = _000C_TGRB3_VALUE;
    MTU3.TGRD = _000C_TGRD3_VALUE;

    /* Disable read/write to MTU3 registers */
    MTU.TRWER.BIT.RWE = 0U;

    /* Set MTIOC3A pin */
    MPC.P17PFS.BYTE = 0x01U;
    PORT1.PMR.BYTE |= 0x80U;

    /* Set MTIOC3C pin */
    MPC.P16PFS.BYTE = 0x01U;
    PORT1.PMR.BYTE |= 0x40U;

    R_Config_MTU3_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_Start
* Description  : This function starts the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_Start(void)
{
    /* Enable TGIA3 interrupt in ICU */
    IEN(MTU3, TGIA3) = 1U;
    
    /* Enable TGIC3 interrupt in ICU */
    IEN(MTU3, TGIC3) = 1U;
    
    /* Start MTU channel 3 counter */
    MTU.TSTR.BIT.CST3 = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_Stop
* Description  : This function stops the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_Stop(void)
{
    /* Disable TGIA3 interrupt in ICU */
    IEN(MTU3, TGIA3) = 0U;
    
    /* Disable TGIC3 interrupt in ICU */
    IEN(MTU3, TGIC3) = 0U;
    
    /* Stop MTU channel 3 counter */
    MTU.TSTR.BIT.CST3 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
