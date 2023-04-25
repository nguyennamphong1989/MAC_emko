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
 * File Name    : r_fw_up_buf.h
 * Version      : 1.20
 * Description  : Firmware update buffer process header file
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 01.07.2017 1.00     First Release
 *         : 09.02.2022 1.20     Fixed coding rule violations
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include "r_fw_up_rx_if.h"
#include "r_fw_up_rx_private.h"

/***********************************************************************************************************************
 Macro definitions
 **********************************************************************************************************************/
#ifndef R_FW_UP_BUF_H_
#define R_FW_UP_BUF_H_

/** number of byte check sum field in Motorola s record */
#define MOT_S_CHECK_SUM_FIELD (0x02u)

/** Address length of S1 format */
#define ADDRESS_LENGTH_S1 (0x04u)

/** Address length of S2 format */
#define ADDRESS_LENGTH_S2 (0x06u)

/** Address length of S3 format */
#define ADDRESS_LENGTH_S3 (0x08u)

/** State of ring buffer */
#define BUF_LOCK   (1u)
#define BUF_UNLOCK (0u)

/** Motorola S-record */
#define MOT_RECORD_S0 (0u)
#define MOT_RECORD_S1 (1u)
#define MOT_RECORD_S2 (2u)
#define MOT_RECORD_S3 (3u)
#define MOT_RECORD_S7 (7u)
#define MOT_RECORD_S8 (8u)
#define MOT_RECORD_S9 (9u)

/** Masking lower 1 byte */
#define MASK_LOWER_BYTE (0x000000FFu)

/** ASCII code */
#define ASCII_CODE_0       (0x30u)
#define ASCII_CODE_9       (0x39u)
#define ASCII_CODE_UPPER_A (0x41u)
#define ASCII_CODE_UPPER_F (0x46u)
#define ASCII_CODE_LOWER_A (0x61u)
#define ASCII_CODE_LOWER_F (0x66u)

/** ASCII code */
#define CONVERT_HEX_NUM        (0x0Fu)
#define CONVERT_HEX_UPPER_CHAR (0x37u)
#define CONVERT_HEX_LOWER_CHAR (0x57u)

/***********************************************************************************************************************
 Typedef definitions
 **********************************************************************************************************************/

/** Analysis step of Motorola S record format */
typedef enum fw_up_mot_s_cnt_t
{
    STATE_MOT_S_RECORD_MARK = 0,
    STATE_MOT_S_RECORD_TYPE,
    STATE_MOT_S_LENGTH_1,
    STATE_MOT_S_LENGTH_2,
    STATE_MOT_S_ADDRESS,
    STATE_MOT_S_DATA,
    STATE_MOT_S_CHKSUM_1,
    STATE_MOT_S_CHKSUM_2
} fw_up_mot_s_cnt_t;

/** buffer structure for Motorola s record format */
typedef struct MotSBufS
{
    uint8_t           addr_length;
    uint8_t           data_length;
    uint8_t         * p_address;
    uint8_t         * p_data;
    uint8_t           type;
    uint8_t           act;
    struct MotSBufS * p_next;
} fw_up_mot_s_buf_t;

/** buffer structure for write data */
typedef struct WriteDataS
{
    uint32_t            addr;
    uint32_t            len;
    uint8_t             data[FW_UP_BINARY_BUF_SIZE];
    struct WriteDataS * p_next;
    struct WriteDataS * p_prev;
} fw_up_write_data_t;

/***********************************************************************************************************************
 Exported global variables and functions (to be accessed by other files)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_buf_init
 * Description  : Initialize buffer.
 * Arguments    : none
 * Return Value : none
 **********************************************************************************************************************/
void           fw_up_buf_init (void);

/***********************************************************************************************************************
 * Function Name: fw_up_memory_init
 * Description  : Initialize variable for store address of allocated memory.
 * Arguments    : none
 * Return Value : none
 **********************************************************************************************************************/
void           fw_up_memory_init (void);

/***********************************************************************************************************************
 * Function Name: fw_up_put_mot_s
 * Description  : Put in ring buffer and analysis Motorola s record format data.
 * Arguments    : mot_s_data - Motorola s record format data
 * Return Value : FW_UP_SUCCESS            - Processing completed successfully.
 *                FW_UP_ERR_BUF_FULL       - Ring buffer is not empty.
 *                FW_UP_ERR_INVALID_RECORD - Motorola S record format data is invalid.
 *                FW_UP_ERR_INTERNAL       - Memory allocation is failed.
 **********************************************************************************************************************/
fw_up_return_t fw_up_put_mot_s (uint8_t mot_s_data);

/***********************************************************************************************************************
 * Function Name: fw_up_get_binary
 * Description  : Put in ring buffer and analysis Motorola s record format data.
 * Arguments    : *p_fl_data          - Pointer to initial address, write address, data size
 * Return Value : FW_UP_SUCCESS       - Processing completed successfully.
 *                FW_UP_ERR_BUF_EMPTY - Ring buffer is empty.
 **********************************************************************************************************************/
fw_up_return_t fw_up_get_binary (fw_up_fl_data_t * p_fl_data);


#endif /* R_FW_UP_BUF_H_ */
