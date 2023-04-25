/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : r_fw_up_rx_private.h
 * Version      : 1.20
 * Device       : RX130
 * Description  : Firmware update API private header file
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 18.05.2018 1.00     First Release
 *         : 09.02.2022 1.20     Fixed coding rule violations
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/

/** Interface information for this package. */
#include "r_fw_up_rx_if.h"

/** Interface information for Flash API */
#include "r_flash_rx_if.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef R_FW_UP_RX_PRIVATE_H_
#define R_FW_UP_RX_PRIVATE_H_

/** Write buffer size */
#define FW_UP_BINARY_BUF_SIZE (256u)

/** Number of write buffers */
#define FW_UP_BINARY_BUF_NUM (2u)

/** Number of buffer for put in the firmware data */
#define FW_UP_BUF_NUM (60u)

/** Start address of the firmware area */
#define FW_UP_FIRM_ST_ADDRESS (FLASH_CF_BLOCK_127)

/** End address of the firmware area */
#define FW_UP_FIRM_EN_ADDRESS (FLASH_CF_BLOCK_37 - 1)

/** Start address of the constants data area 1 */
#define FW_UP_CONST_1_ST_ADDRESS (FLASH_CF_BLOCK_37)

/** Last write start address of the constants data area 1 */
#define FW_UP_CONST_1_FIN_ST_ADDRESS (FLASH_CF_BLOCK_34 - 256)

/** Start address of the write complete information storage area 1 */
#define FW_UP_COMPLETE_1_ST_ADDRESS (FLASH_CF_BLOCK_34 - 8)

/** End address of the constants data area 1 */
#define FW_UP_CONST_1_EN_ADDRESS (FLASH_CF_BLOCK_34 - 1)

/** Start address of the constants data area 2 */
#define FW_UP_CONST_2_ST_ADDRESS (FLASH_CF_BLOCK_34)

/** Last write start address of the constants data area 2 */
#define FW_UP_CONST_2_FIN_ST_ADDRESS (FLASH_CF_BLOCK_31 - 256)

/** Start address of the write complete information storage area 2 */
#define FW_UP_COMPLETE_2_ST_ADDRESS (FLASH_CF_BLOCK_31 - 8)

/** End address of the constants data area 2 */
#define FW_UP_CONST_2_EN_ADDRESS (FLASH_CF_BLOCK_31 - 1)

/** Start address of the start-up area */
#define FW_UP_STUP_ST_ADDRESS (FLASH_CF_BLOCK_15)

/** End address of the start-up area */
#define FW_UP_STUP_EN_ADDRESS (FLASH_CF_BLOCK_END)

/** Number of block of Firmware area (block 38 - 127) */
#define FW_UP_FIRM_BLOCK_NUM (90u)

/** Number of block of constants data area (block 32 - 34, 35 - 37) */
#define FW_UP_CONST_BLOCK_NUM (3u)

/** Number of block of the start-up area (block 0 - 15) */
#define FW_UP_STUP_BLOCK_NUM (16u)

/** Address of reset vector of Firmware */
#define FW_UP_FIRM_RESETVECT (FLASH_CF_BLOCK_15 - 4)

/** The value of blank of code flash */
#define FW_UP_BLANK_VALUE (0xFFFFFFFFu)

/** Key code for writing PRCR register */
#define PRCR_KEY  (0xA500u)

/** Set bit PRCR.PRC1 */
#define SET_PRC1  (0x0002u)

/** Write to SWRR register for software reset */
#define MCU_RESET (0xA501u)

#endif /* R_FW_UP_RX_PRIVATE_H_ */
