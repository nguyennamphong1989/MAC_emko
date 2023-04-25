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
* Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Pin.c
* Version          : 1.0.2
* Device(s)        : R5F51305AxFM
* Description      : This file implements SMC pin code generation.
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
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Pins_Create
* Description  : This function initializes Smart Configurator pins
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Pins_Create(void)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_MPC);

    /* Set AN000 pin */
    MPC.P40PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xFEU;
    PORT4.PDR.BYTE &= 0xFEU;

    /* Set AN001 pin */
    MPC.P41PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xFDU;
    PORT4.PDR.BYTE &= 0xFDU;

    /* Set AN002 pin */
    MPC.P42PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xFBU;
    PORT4.PDR.BYTE &= 0xFBU;

    /* Set AN003 pin */
    MPC.P43PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xF7U;
    PORT4.PDR.BYTE &= 0xF7U;

    /* Set AN004 pin */
    MPC.P44PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xEFU;
    PORT4.PDR.BYTE &= 0xEFU;

    /* Set AN005 pin */
    MPC.P45PFS.BYTE = 0x80U;
    PORT4.PMR.BYTE &= 0xDFU;
    PORT4.PDR.BYTE &= 0xDFU;

    /* Set IRQ0 pin */
    MPC.PH1PFS.BYTE = 0x40U;
    PORTH.PMR.BYTE &= 0xFDU;
    PORTH.PDR.BYTE &= 0xFDU;

    /* Set IRQ1 pin */
    MPC.PH2PFS.BYTE = 0x40U;
    PORTH.PMR.BYTE &= 0xFBU;
    PORTH.PDR.BYTE &= 0xFBU;

    /* Set IRQ4 pin */
    MPC.P14PFS.BYTE = 0x40U;
    PORT1.PMR.BYTE &= 0xEFU;
    PORT1.PDR.BYTE &= 0xEFU;

    /* Set MTIOC0B pin */
    MPC.P15PFS.BYTE = 0x01U;
    PORT1.PMR.BYTE |= 0x20U;

    /* Set MTIOC3A pin */
    MPC.P17PFS.BYTE = 0x01U;
    PORT1.PMR.BYTE |= 0x80U;

    /* Set MTIOC3C pin */
    MPC.P16PFS.BYTE = 0x01U;
    PORT1.PMR.BYTE |= 0x40U;

    /* Set RXD1 pin */
    MPC.P30PFS.BYTE = 0x0AU;
    PORT3.PMR.BYTE |= 0x01U;

    /* Set RXD5 pin */
    MPC.PA3PFS.BYTE = 0x0AU;
    PORTA.PMR.BYTE |= 0x08U;

    /* Set RXD12 pin */
    MPC.PE2PFS.BYTE = 0x0CU;
    PORTE.PMR.BYTE |= 0x04U;

    /* Set TXD1 pin */
    PORT2.PODR.BYTE |= 0x40U;
    MPC.P26PFS.BYTE = 0x0AU;
    PORT2.PDR.BYTE |= 0x40U;
    // PORT2.PMR.BIT.B6 = 1U; // Please set the PMR bit after TE bit is set to 1.

    /* Set TXD5 pin */
    PORTA.PODR.BYTE |= 0x10U;
    MPC.PA4PFS.BYTE = 0x0AU;
    PORTA.PDR.BYTE |= 0x10U;
    // PORTA.PMR.BIT.B4 = 1U; // Please set the PMR bit after TE bit is set to 1.

    /* Set TXD12 pin */
    PORTE.PODR.BYTE |= 0x02U;
    MPC.PE1PFS.BYTE = 0x0CU;
    PORTE.PDR.BYTE |= 0x02U;
    // PORTE.PMR.BIT.B1 = 1U; // Please set the PMR bit after TE bit is set to 1.

    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_MPC);
}

