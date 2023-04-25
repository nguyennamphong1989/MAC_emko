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
 * File Name    : r_fw_up_buf.c
 * Version      : 1.20
 * Description  : Firmware update buffer process file
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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "r_fw_up_rx_if.h"
#include "r_fw_up_buf.h"
#include "r_fw_up_rx_private.h"

/***********************************************************************************************************************
 Private variables and functions
 **********************************************************************************************************************/
static fw_up_mot_s_buf_t  * sp_app_put_mot_s_buf = NULL;
static fw_up_mot_s_buf_t  * sp_app_get_mot_s_buf = NULL;
static fw_up_mot_s_buf_t    s_mot_s_buf[FW_UP_BUF_NUM];

static fw_up_write_data_t * sp_app_write_buf = NULL;
static fw_up_write_data_t   s_write_buf[FW_UP_BINARY_BUF_NUM];

static fw_up_mot_s_cnt_t    s_mot_s_data_state = STATE_MOT_S_RECORD_MARK;

static uint32_t             s_write_current_address;

static bool                 s_detect_terminal_flag = false;

static bool fw_up_ascii_to_hexbyte (uint8_t in_upper, uint8_t in_lower, uint8_t * p_hex_buf);

/***********************************************************************************************************************
 * Function Name: fw_up_buf_init
 * Description  : Initialize buffer.
 * Arguments    : none
 * Return Value : none
 **********************************************************************************************************************/
void fw_up_buf_init(void)
{
    fw_up_mot_s_buf_t  * p_mot_s_buf;
    fw_up_write_data_t * p_write_buf;
    uint32_t             i;

    /* WAIT_LOOP */
    for (i = 0; i < FW_UP_BINARY_BUF_NUM; i++ )
    {

        /* Casting is valid because it is cast to
         * a pointer of type fw_up_write_data_t */
        p_write_buf = (fw_up_write_data_t *) &s_write_buf[i];

        /** Clear write address */
        p_write_buf->addr = 0x00000000;

        /** Clear data length in buffer */
        p_write_buf->len = 0;

        /** Clear data buffer */
        (void)memset((p_write_buf->data), (0xFF), (FW_UP_BINARY_BUF_SIZE));

        /** Set next buffer address */
        /** Casting is valid because it is cast to a pointer of type fw_up_write_data_t */
        p_write_buf->p_next = (fw_up_write_data_t *) &s_write_buf[i + 1u];

        /** Set previous buffer address */
        /** Casting is valid because it is cast to a pointer of type fw_up_write_data_t */
        p_write_buf->p_prev = (fw_up_write_data_t *) &s_write_buf[i - 1u];
    }

    /** Set next buffer address of last buffer is first buffer */
    /** Casting is valid because it is cast to a pointer of type fw_up_write_data_t */
    p_write_buf->p_next = (fw_up_write_data_t *) &s_write_buf[0];

    /** Set previous buffer address of first buffer is last buffer */
    s_write_buf[0].p_prev = p_write_buf;

    /** Set currently buffer in use is first buffer */
    /** Casting is valid because it is cast to a pointer of type fw_up_write_data_t */
    sp_app_write_buf = (fw_up_write_data_t *) &s_write_buf[0];

    /** Set currently write address is initial address of option setting memory */
    s_write_current_address = 0u;

    /** Set write address of currently buffer in use is "write_current_address" */
    sp_app_write_buf->addr = s_write_current_address;

    /* WAIT_LOOP */
    for (i = 0; i < FW_UP_BUF_NUM; i++ )
    {

        /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
        p_mot_s_buf = (fw_up_mot_s_buf_t *) &s_mot_s_buf[i];

        /** Clear record type */
        p_mot_s_buf->type = 0;

        /** Clear data length */
        p_mot_s_buf->data_length = 0;

        /** Clear address length */
        p_mot_s_buf->addr_length = 0;

        /** Clear initial address of allocated memory for address */
        if (NULL != p_mot_s_buf->p_address)
        {
            free(p_mot_s_buf->p_address);
            p_mot_s_buf->p_address = NULL;
        }

        /** Clear initial address of allocated memory for data */
        if (NULL != p_mot_s_buf->p_data)
        {
            free(p_mot_s_buf->p_data);
            p_mot_s_buf->p_data = NULL;
        }

        /** Clear lock flag */
        p_mot_s_buf->act = BUF_UNLOCK;

        /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
        p_mot_s_buf->p_next = (fw_up_mot_s_buf_t *) &s_mot_s_buf[i + 1u];
    }

    /** Set next buffer address of last buffer is first buffer */
    /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
    p_mot_s_buf->p_next = (fw_up_mot_s_buf_t *) &s_mot_s_buf[0];

    /** Set currently buffer in use is first buffer */
    /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
    sp_app_put_mot_s_buf = (fw_up_mot_s_buf_t *) &s_mot_s_buf[0];
    sp_app_get_mot_s_buf = (fw_up_mot_s_buf_t *) &s_mot_s_buf[0];

    /** Clear analysis state of Motorola s record format */
    s_mot_s_data_state = STATE_MOT_S_RECORD_MARK;

    /** Clear detect terminal record flag */
    s_detect_terminal_flag = false;

}
/***********************************************************************************************************************
 * End of function fw_up_buf_init
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_memory_init
 * Description  : Initialize variable for store address of allocated memory.
 * Arguments    : none
 * Return Value : none
 **********************************************************************************************************************/
void fw_up_memory_init(void)
{
    fw_up_mot_s_buf_t * p_mot_s_buf;
    uint32_t            i;

    /* WAIT_LOOP */
    for (i = 0; i < FW_UP_BUF_NUM; i++ )
    {

        /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
        p_mot_s_buf = (fw_up_mot_s_buf_t *) &s_mot_s_buf[i];

        /** Clear initial address of allocated memory for address */
        p_mot_s_buf->p_address = NULL;

        /** Clear initial address of allocated memory for data */
        p_mot_s_buf->p_data = NULL;

        /** Casting is valid because it is cast to a pointer of type fw_up_mot_s_buf_t */
        p_mot_s_buf->p_next = (fw_up_mot_s_buf_t *) &s_mot_s_buf[i + 1u];
    }

}
/***********************************************************************************************************************
 * End of function fw_up_memory_init
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_put_mot_s
 * Description  : Put in ring buffer and analysis Motorola s record format data.
 * Arguments    : mot_s_data - Motorola s record format data
 * Return Value : FW_UP_SUCCESS            - Processing completed successfully.
 *                FW_UP_ERR_BUF_FULL       - Ring buffer is not empty.
 *                FW_UP_ERR_INVALID_RECORD - Motorola S record format data is invalid.
 *                FW_UP_ERR_INTERNAL       - Memory allocation is failed.
 **********************************************************************************************************************/
fw_up_return_t fw_up_put_mot_s(uint8_t mot_s_data)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    static uint32_t   s_cnt;
    static uint32_t   s_sum_value;
    static uint8_t    s_mot_buf;
    static uint8_t  * sp_work_addr;
    uint8_t           len_buf;
    uint8_t           chk_sum;
    bool              chk_ascii;

    if (BUF_LOCK == sp_app_put_mot_s_buf->act)
    {
        return FW_UP_ERR_BUF_FULL;
    }

    if (true == s_detect_terminal_flag)
    {
        return FW_UP_SUCCESS;
    }

    switch (s_mot_s_data_state)
    {
        case STATE_MOT_S_RECORD_MARK:

            /** Header field (upper 4bits) */
            /** Casting is valid because the width is a constant expressible with 8 bits. */
            if ((uint8_t)'S' == mot_s_data)
            {

                /** Update analysis state */
                s_mot_s_data_state = STATE_MOT_S_RECORD_TYPE;
            }

            /** Casting is valid because the width is a constant expressible with 8 bits. */
            else if (((uint8_t)'\n' == mot_s_data) || ((uint8_t)'\r' == mot_s_data))
            {

                /** Do nothing, ignore CR+LF */
                ;
            }
            else
            {
                ret = FW_UP_ERR_INVALID_RECORD;
            }
            break;

        case STATE_MOT_S_RECORD_TYPE:

            /** Header field (lower 4bits) */
            /** Convert to hex byte from ASCII data */
            /** Casting is valid because the width is a constant expressible with 8 bits. */
            chk_ascii = fw_up_ascii_to_hexbyte((uint8_t)'0', mot_s_data, &(sp_app_put_mot_s_buf->type));

            if (false == chk_ascii)
            {
                return FW_UP_ERR_INVALID_RECORD;
            }

            /** Update analysis state */
            s_mot_s_data_state = STATE_MOT_S_LENGTH_1;
            break;

        case STATE_MOT_S_LENGTH_1:

            /** Length field (upper 4bits) */
            /** Put in buffer */
            s_mot_buf = mot_s_data;

            /** Clear check sum compare value */
            s_sum_value = 0;

            /** Update analysis state */
            s_mot_s_data_state = STATE_MOT_S_LENGTH_2;
            break;

        case STATE_MOT_S_LENGTH_2:

            /** Length field (lower 4bits) */
            chk_ascii = fw_up_ascii_to_hexbyte(s_mot_buf, mot_s_data, &len_buf);

            if (false == chk_ascii)
            {
                return FW_UP_ERR_INVALID_RECORD;
            }

            /** Check address length */
            /** Casting is valid because the width is a constant expressible with 8 bits. */
            if ((((uint8_t)MOT_RECORD_S0 == sp_app_put_mot_s_buf->type)
                || ((uint8_t)MOT_RECORD_S1 == sp_app_put_mot_s_buf->type))
                || ((uint8_t)MOT_RECORD_S9 == sp_app_put_mot_s_buf->type))
            {
                sp_app_put_mot_s_buf->addr_length = (ADDRESS_LENGTH_S1 / 2);
            }

            /** Casting is valid because the width is a constant expressible with 8 bits. */
            else if (((uint8_t)MOT_RECORD_S2 == sp_app_put_mot_s_buf->type)
                     || ((uint8_t)MOT_RECORD_S8 == sp_app_put_mot_s_buf->type))
            {
                sp_app_put_mot_s_buf->addr_length = (ADDRESS_LENGTH_S2 / 2);
            }

            /** Casting is valid because the width is a constant expressible with 8 bits. */
            else if (((uint8_t)MOT_RECORD_S3 == sp_app_put_mot_s_buf->type)
                     || ((uint8_t)MOT_RECORD_S7 == sp_app_put_mot_s_buf->type))
            {
                sp_app_put_mot_s_buf->addr_length = (ADDRESS_LENGTH_S3 / 2);
            }
            else
            {
                return FW_UP_ERR_INVALID_RECORD;
            }

            /** Allocate memory of the address data */
            /** Casting is valid because conversion of void pointer to concrete type pointer. */
            sp_app_put_mot_s_buf->p_address = (uint8_t *)malloc(sp_app_put_mot_s_buf->addr_length);

            if (NULL == sp_app_put_mot_s_buf->p_address)
            {
                return FW_UP_ERR_INTERNAL;
            }

            /** Calculate data length */
            /** Casting is valid because the width is a constant expressible with 8 bits. */
            sp_app_put_mot_s_buf->data_length = (len_buf
                    - (sp_app_put_mot_s_buf->addr_length + (uint8_t)(MOT_S_CHECK_SUM_FIELD / 2)));

            /** Allocate memory of the data */
            if ((uint8_t)0 != sp_app_put_mot_s_buf->data_length)
            {

                /** Casting is valid because conversion of void pointer to concrete type pointer. */
                sp_app_put_mot_s_buf->p_data = (uint8_t *)malloc(sp_app_put_mot_s_buf->data_length);

                if (NULL == sp_app_put_mot_s_buf->p_data)
                {
                    return FW_UP_ERR_INTERNAL;
                }
            }

            /** Copy initial address of address data */
            sp_work_addr = sp_app_put_mot_s_buf->p_address;

            /** Add length value to check sum value */
            s_sum_value += len_buf;

            /** Clear 2bytes convert counter */
            s_cnt = 0;

            /** Update analysis state */
            s_mot_s_data_state = STATE_MOT_S_ADDRESS;
            break;

        case STATE_MOT_S_ADDRESS:

            /** Length field */
            if (1u == (s_cnt % 2u))
            {
                chk_ascii = fw_up_ascii_to_hexbyte(s_mot_buf, mot_s_data, sp_work_addr);

                if (false == chk_ascii)
                {
                    return FW_UP_ERR_INVALID_RECORD;
                }

                /** Add address data to check sum value */
                s_sum_value += (*sp_work_addr);

                /** Increment address */
                sp_work_addr++;
            }

            /** Increment 2bytes convert counter */
            s_cnt++;

            /** put in buffer */
            s_mot_buf = mot_s_data;

            /** Casting is valid because conversion to wide type. */
            if (((uint32_t)sp_app_put_mot_s_buf->addr_length * 2u) <= s_cnt)
            {

                /** Clear 2bytes convert counter */
                s_cnt = 0;

                /** Copy initial address of data */
                sp_work_addr = sp_app_put_mot_s_buf->p_data;

                /** Update analysis state */
                if ((uint8_t)0 != sp_app_put_mot_s_buf->data_length)
                {
                    s_mot_s_data_state = STATE_MOT_S_DATA;
                }
                else
                {
                    s_mot_s_data_state = STATE_MOT_S_CHKSUM_1;
                }
            }
            break;

        case STATE_MOT_S_DATA:

            /** Data field */
            if (1u == (s_cnt % 2u))
            {
                chk_ascii = fw_up_ascii_to_hexbyte(s_mot_buf, mot_s_data, sp_work_addr);

                if (false == chk_ascii)
                {
                    return FW_UP_ERR_INVALID_RECORD;
                }

                /** Add data to check sum value */
                s_sum_value += (*sp_work_addr);

                /** Increment address */
                sp_work_addr++;
            }

            /** Increment 2bytes convert counter */
            s_cnt++;

            /** Put in buffer */
            s_mot_buf = mot_s_data;

            /** Casting is valid because conversion to wide type. */
            if (((uint32_t)sp_app_put_mot_s_buf->data_length * 2u) <= s_cnt)
            {

                /** Clear 2bytes convert counter */
                s_cnt = 0;

                /** Update analysis state */
                s_mot_s_data_state = STATE_MOT_S_CHKSUM_1;
            }
            break;

        case STATE_MOT_S_CHKSUM_1:

            /** Check sum field (upper 4bits) */
            /** Put in buffer */
            s_mot_buf = mot_s_data;

            /** Update analysis state */
            s_mot_s_data_state = STATE_MOT_S_CHKSUM_2;
            break;

        case STATE_MOT_S_CHKSUM_2:
            chk_ascii = fw_up_ascii_to_hexbyte(s_mot_buf, mot_s_data, &chk_sum);

            if (false == chk_ascii)
            {
                return FW_UP_ERR_INVALID_RECORD;
            }

            /** Compare check sum value */
            /** Casting is valid because higher 24 bits is not used. */
            if ((uint8_t)((~s_sum_value) & MASK_LOWER_BYTE) != chk_sum)
            {
                return FW_UP_ERR_INVALID_RECORD;
            }

            /** Lock currently buffer in use */
            sp_app_put_mot_s_buf->act = BUF_LOCK;

            /** Check address length */
            /** Casting is valid because the width is a constant expressible with 8 bits. */
            if ((((uint8_t)MOT_RECORD_S7 == sp_app_put_mot_s_buf->type)
                || ((uint8_t)MOT_RECORD_S8 == sp_app_put_mot_s_buf->type))
                || ((uint8_t)MOT_RECORD_S9 == sp_app_put_mot_s_buf->type))
            {
                /** Detect terminal record */
                s_detect_terminal_flag = true;
            }

            /** Set currently buffer in use is next buffer */
            sp_app_put_mot_s_buf = sp_app_put_mot_s_buf->p_next;

            /** Clear analysis state */
            s_mot_s_data_state = STATE_MOT_S_RECORD_MARK;
            break;

        default:
            ;
            break;
    }

    return ret;

}
/***********************************************************************************************************************
 * End of function fw_up_put_mot_s
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_get_binary
 * Description  : Put in ring buffer and analysis Motorola s record format data.
 * Arguments    : *p_fl_data          - Pointer to initial address, write address, data size
 * Return Value : FW_UP_SUCCESS       - Processing completed successfully.
 *                FW_UP_ERR_BUF_EMPTY - Ring buffer is empty.
 **********************************************************************************************************************/
fw_up_return_t fw_up_get_binary(fw_up_fl_data_t *p_fl_data)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    fw_up_write_data_t * p_prev_write_buf;
    fw_up_write_data_t * p_local_write_buf;

    uint32_t   addr_buf = 0;
    uint32_t   put_number;
    uint32_t   i;
    uint8_t  * p_work_addr;

    p_work_addr = NULL;

    p_prev_write_buf = sp_app_write_buf->p_prev;

    p_prev_write_buf->len = 0;
    (void)memset((p_prev_write_buf->data), (0xFF), (FW_UP_BINARY_BUF_SIZE));

    /* WAIT_LOOP */
    while (FW_UP_BINARY_BUF_SIZE > sp_app_write_buf->len)
    {
        if (BUF_UNLOCK == sp_app_get_mot_s_buf->act)
        {
            if (true == s_detect_terminal_flag)
            {
                if (0u != sp_app_write_buf->len)
                {
                    sp_app_write_buf->len = FW_UP_BINARY_BUF_SIZE;
                    break;
                }
                else
                {
                    return FW_UP_ERR_BUF_EMPTY;
                }
            }
            else
            {
                return FW_UP_ERR_BUF_EMPTY;
            }
        }

        /** Check data record */
        /** Casting is valid because the width is a constant expressible with 8 bits. */
        if ((((uint8_t)MOT_RECORD_S1 == sp_app_get_mot_s_buf->type)
            || ((uint8_t)MOT_RECORD_S2 == sp_app_get_mot_s_buf->type))
            || ((uint8_t)MOT_RECORD_S3 == sp_app_get_mot_s_buf->type))
        {

            p_work_addr = sp_app_get_mot_s_buf->p_address;

            addr_buf = 0;

            /* WAIT_LOOP */
            for (i = 0; i < sp_app_get_mot_s_buf->addr_length; i++ )
            {
                addr_buf  =  (addr_buf << 8u);
                addr_buf |=  (*p_work_addr);

                /** Increment address */
                p_work_addr++;
            }

            if (0u == sp_app_write_buf->len)
            {

                /** Calculate currently write address */
                s_write_current_address = (addr_buf / FW_UP_BINARY_BUF_SIZE) * FW_UP_BINARY_BUF_SIZE;

                /** Set currently write address to write address in buffer */
                sp_app_write_buf->addr = s_write_current_address;

                /** Set difference of less than write size to data length */
                sp_app_write_buf->len = addr_buf % FW_UP_BINARY_BUF_SIZE;
            }
            else
            {

                /** Set difference between currently write address and write address in buffer to data length */
                sp_app_write_buf->len = addr_buf - sp_app_write_buf->addr;
            }

            if (FW_UP_BINARY_BUF_SIZE <= sp_app_write_buf->len)
            {
                sp_app_write_buf->len = FW_UP_BINARY_BUF_SIZE;
                break;
            }

            put_number = FW_UP_BINARY_BUF_SIZE - sp_app_write_buf->len;

            if (put_number >= sp_app_get_mot_s_buf->data_length)
            {
                p_work_addr = sp_app_get_mot_s_buf->p_data;

                /* WAIT_LOOP */
                for (i = 0; i < sp_app_get_mot_s_buf->data_length; i++ )
                {
                    sp_app_write_buf->data[i + sp_app_write_buf->len] = *p_work_addr;
                    p_work_addr++;
                }

                sp_app_write_buf->len += sp_app_get_mot_s_buf->data_length;
            }
            else
            {
                p_work_addr = sp_app_get_mot_s_buf->p_data;

                /* WAIT_LOOP */
                for (i = 0; i < put_number; i++ )
                {
                    sp_app_write_buf->data[i + sp_app_write_buf->len] = *p_work_addr;
                    p_work_addr++;
                }

                sp_app_write_buf->len += put_number;

                p_local_write_buf = sp_app_write_buf->p_next;

                /* WAIT_LOOP */
                for (i = 0; i < (sp_app_get_mot_s_buf->data_length - put_number); i++ )
                {
                    p_local_write_buf->data[i] = *p_work_addr;
                    p_work_addr++;
                }

                p_local_write_buf->len = i;
            }
        }

        /** Release address and data memory */
        free(sp_app_get_mot_s_buf->p_address);
        free(sp_app_get_mot_s_buf->p_data);

        /** Set NULL */
        sp_app_get_mot_s_buf->p_address = NULL;
        sp_app_get_mot_s_buf->p_data    = NULL;

        /** Release buffer */
        sp_app_get_mot_s_buf->act = BUF_UNLOCK;

        /** Set currently buffer in use is next buffer */
        sp_app_get_mot_s_buf = sp_app_get_mot_s_buf->p_next;
    }

    /** Set data address, write address, data length to argument */
    /** Casting is valid because the width of address value is 32bits. */
    p_fl_data->src_addr = (uint32_t) sp_app_write_buf->data;
    p_fl_data->dst_addr = sp_app_write_buf->addr;
    p_fl_data->len      = sp_app_write_buf->len;

    /** Set currently write buffer in use is next buffer */
    sp_app_write_buf = sp_app_write_buf->p_next;

    /** Increase currently write address */
    s_write_current_address += FW_UP_BINARY_BUF_SIZE;

    /** Set currently write address to write buffer */
    sp_app_write_buf->addr = s_write_current_address;

    return ret;

}
/***********************************************************************************************************************
 * End of function fw_up_get_binary
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_ascii_to_hexbyte
 * Description  : Convert 2 bytes of ASCII data to 1 byte of hexadecimal.
 * Arguments    : in_upper   - upper data for ASCII.
 *                in_lower   - lower data for ASCII.
 *                *p_hex_buf - Pointer to store converted 1 byte of hexadecimal
 * Return Value : true       - Processing completed successfully.
 *                false      - ASCII data is invalid.
 **********************************************************************************************************************/
static bool fw_up_ascii_to_hexbyte(uint8_t in_upper, uint8_t in_lower, uint8_t *p_hex_buf)
{
    bool ret = true;

    /** Convert upper 4 bits */
    /** Casting is valid because the width is a constant expressible with 8 bits. */
    if (((uint8_t)ASCII_CODE_0 <= in_upper) && ((uint8_t)ASCII_CODE_9 >= in_upper))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_upper = (in_upper & (uint8_t)CONVERT_HEX_NUM);
    }

    /** Casting is valid because the width is a constant expressible with 8 bits. */
    else if (((uint8_t)ASCII_CODE_UPPER_A <= in_upper) && ((uint8_t)ASCII_CODE_UPPER_F >= in_upper))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_upper = (in_upper - (uint8_t)CONVERT_HEX_UPPER_CHAR);
    }

    /** Casting is valid because the width is a constant expressible with 8 bits. */
    else if (((uint8_t)ASCII_CODE_LOWER_A <= in_upper) && ((uint8_t)ASCII_CODE_LOWER_F >= in_upper))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_upper = (in_upper - (uint8_t)CONVERT_HEX_LOWER_CHAR);
    }
    else
    {
        ret = false;
    }

    /** Convert lower 4 bits */
    /** Casting is valid because the width is a constant expressible with 8 bits. */
    if (((uint8_t)ASCII_CODE_0 <= in_lower) && ((uint8_t)ASCII_CODE_9 >= in_lower))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_lower = (in_lower & (uint8_t)CONVERT_HEX_NUM);
    }

    /** Casting is valid because the width is a constant expressible with 8 bits. */
    else if (((uint8_t)ASCII_CODE_UPPER_A <= in_lower) && ((uint8_t)ASCII_CODE_UPPER_F >= in_lower))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_lower = (in_lower - (uint8_t)CONVERT_HEX_UPPER_CHAR);
    }

    /** Casting is valid because the width is a constant expressible with 8 bits. */
    else if (((uint8_t)ASCII_CODE_LOWER_A <= in_lower) && ((uint8_t)ASCII_CODE_LOWER_F >= in_lower))
    {

        /** Casting is valid because the width is a constant expressible with 8 bits. */
        in_lower = (in_lower - (uint8_t)CONVERT_HEX_LOWER_CHAR);
    }
    else
    {
        ret = false;
    }

    if (true == ret)
    {

        /** Combine upper and lower */
        *p_hex_buf = (in_upper << 4) | in_lower;
    }

    return ret;
}
/***********************************************************************************************************************
 * End of function fw_up_ascii_to_hexbyte
 **********************************************************************************************************************/
