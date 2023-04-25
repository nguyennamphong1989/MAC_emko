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
 * File Name    : r_fw_up_rx_if.h
 * Version      : 1.20
 * Description  : Firmware update API interface file
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 01.07.2017 1.00     First Release
 *         : 09.02.2022 1.20     Fixed coding rule violations
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef R_FW_UP_RX_IF_H_
#define R_FW_UP_RX_IF_H_

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/

/** Firmware update modes */
typedef enum e_update_mode_t
{
    UPDATE_FW,      /* Update firmware program mode */
    UPDATE_FW_UP_1, /* Update firmware update program mode 1 */
    UPDATE_FW_UP_2, /* Update firmware update program mode 2 */
} update_mode_t;

/** Firmware update API error codes */
typedef enum e_fw_up_return_t
{
    FW_UP_SUCCESS,               /* Processing completed successfully */
    FW_UP_ERR_OPENED,            /* Firmware update API initialized. R_FW_UP_Open has been run. */
    FW_UP_ERR_NOT_OPEN,          /* Firmware update API uninitialized. R_FW_UP_Open has not been run. */
    FW_UP_ERR_NULL_PTR,          /* Value of the pointer is NULL or FIT_NO_PTR */
    FW_UP_ERR_INVALID_RECORD,    /* Record format is invalid */
    FW_UP_ERR_BUF_FULL,          /* Buffer is filled */
    FW_UP_ERR_BUF_EMPTY,         /* Buffer is empty */
    FW_UP_ERR_INITIALIZE,        /* Flash API open error */
    FW_UP_ERR_ERASE,             /* Erase error */
    FW_UP_ERR_WRITE,             /* Write error */
    FW_UP_ERR_VERIFY,            /* Verify error */
    FW_UP_ERR_SWITCH_AREA,       /* Switching the Start-Up area error */
    FW_UP_ERR_INVALID_ADDRESS,   /* Address of writing is invalid */
    FW_UP_ERR_INVALID_RESETVECT, /* Reset vector of the firmware is invalid */
    FW_UP_ERR_INTERNAL           /* Module internal error is detected. */
} fw_up_return_t;

/** Structure of the write data */
typedef struct st_fw_up_fl_data_t
{
    uint32_t src_addr; /* Address of write buffer containing the data to write to flash */
    uint32_t dst_addr; /* Address of ROM to write */
    uint32_t len;      /* Number of bytes contained in the write buffer specified with arc_addr */
    uint16_t count;    /* Number of bytes put in the buffer */
} fw_up_fl_data_t;

/***********************************************************************************************************************
 Exported global variables and functions (to be accessed by other files)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_open_flash
 * Description  : Initialize Flash function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS        - Processing completed successfully.
 *                FW_UP_ERR_INITIALIZE - Processing is failed.
 **********************************************************************************************************************/
fw_up_return_t fw_up_open_flash (void);

/***********************************************************************************************************************
 * Function Name: fw_up_open
 * Description  : Initialize firmware update function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS    - Processing completed successfully.
 *                FW_UP_ERR_OPENED - fw_up_open has already been run.
 **********************************************************************************************************************/
fw_up_return_t fw_up_open (void);

/***********************************************************************************************************************
 * Function Name: fw_up_close
 * Description  : Initialize firmware update function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS      - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN - fw_up_open has not been run.
 **********************************************************************************************************************/
fw_up_return_t fw_up_close (void);

/***********************************************************************************************************************
 * Function Name: analyze_and_write_data
 * Description  : Analyze mot s format and program data.
 * Arguments    : *p_recv_data - mot file data.
 *                data_size    - mot file data size.
 *                update_mode  - Update mode.
 * Return Value : FW_UP_SUCCESS            - Processing completed successfully.
 *                FW_UP_ERR_INVALID_RECORD - Motorola S record format data is invalid.
 *                FW_UP_ERR_WRITE          - Flash API(erase) error.
 **********************************************************************************************************************/
fw_up_return_t analyze_and_write_data (const uint8_t * p_recv_data, uint32_t data_size, uint8_t update_mode);

/***********************************************************************************************************************
 * Function Name: erase_firmware
 * Description  : Erase flash memory to write firmware
 * Arguments    : update_mode - Update mode.
 * Return Value : FW_UP_SUCCESS         - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN    - fw_up_open_flash has not been run.
 *                FW_UP_ERR_ERASE       - Flash API(erase) error.
 *                FW_UP_ERR_SWITCH_AREA - Flash API(switch start-up area) error.
 **********************************************************************************************************************/
fw_up_return_t erase_firmware (uint8_t update_mode);

/***********************************************************************************************************************
 * Function Name: switch_start_up_and_reset
 * Description  : Switch the start-up area and execute software reset.
 * Arguments    : none
 * Return Value : FW_UP_ERR_NOT_OPEN          - fw_up_open_flash has not been run.
 *                FW_UP_ERR_INVALID_RESETVECT - The value of reset vector of the firmware is invalid.
 *                FW_UP_ERR_SWITCH_AREA       - Flash API(switch start-up area) error.
 **********************************************************************************************************************/
fw_up_return_t switch_start_up_and_reset (void);

#endif /* R_FW_UP_RX_IF_H_ */
