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
 * File Name    : r_fw_up_rx.c
 * Version      : 1.20
 * Description  : Firmware update 
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 01.07.2017 1.00     First Release
 *         : 09.02.2022 1.20     Fixed coding rule violations
 **********************************************************************************************************************/

/***********************************************************************************************************************
 Includes <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <string.h>
#include <stdint.h>

/** Access to peripherals and board defines. */
#include "platform.h"

/** Access to Flash API */
#include "r_flash_rx_if.h"

/** Defines for Firmware update support */
#include "r_fw_up_rx_if.h"
#include "r_fw_up_buf.h"
#include "r_fw_up_rx_private.h"

/***********************************************************************************************************************
 Private variables and functions
 **********************************************************************************************************************/
static bool s_is_opened  = false;

static void           copy_update_ramprog (void);
static fw_up_return_t fw_up_put_data (fw_up_fl_data_t * p_fl_data);
static fw_up_return_t fw_up_get_data (fw_up_fl_data_t * p_fl_data);
static fw_up_return_t write_firmware (fw_up_fl_data_t * p_fl_data, uint8_t update_mode);

/***********************************************************************************************************************
 * Function Name: fw_up_open_flash
 * Description  : Initialize Flash function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS        - Processing completed successfully.
 *                FW_UP_ERR_INITIALIZE - Processing is failed.
 **********************************************************************************************************************/
fw_up_return_t fw_up_open_flash(void)
{
    flash_err_t    ret_flash;
    fw_up_return_t ret;

    ret_flash = R_FLASH_Open();

    if (FLASH_SUCCESS == ret_flash)
    {
        ret = FW_UP_SUCCESS;
    }
    else
    {
        ret = FW_UP_ERR_INITIALIZE;
    }

    return ret;
}
/***********************************************************************************************************************
 * End of function fw_up_open_flash
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_open
 * Description  : Initialize firmware update function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS    - Processing completed successfully.
 *                FW_UP_ERR_OPENED - fw_up_open has already been run.
 **********************************************************************************************************************/
fw_up_return_t fw_up_open(void)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    /** Check that the fw_up_open() has been executed. */
    if (true == s_is_opened)
    {
        return FW_UP_ERR_OPENED;
    }

    /** Copy functions which operate on the RAM. */
    copy_update_ramprog();

    /** Initialize variable for store address of allocated memory */
    fw_up_memory_init();

    /** Initialize ring buffer */
    fw_up_buf_init();

    /** Set initialize complete flag */
    s_is_opened = true;

    return ret;
}
/***********************************************************************************************************************
 * End of function fw_up_open
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_close
 * Description  : Initialize firmware update function.
 * Arguments    : none
 * Return Value : FW_UP_SUCCESS      - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN - fw_up_open has not been run.
 **********************************************************************************************************************/
fw_up_return_t fw_up_close(void)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Initialize ring buffer */
    fw_up_buf_init();

    /** Clear initialize complete flag */
    s_is_opened = false;

    return ret;
}
/***********************************************************************************************************************
 * End of function fw_up_close
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: copy_update_ramprog
 * Description  : Copy firmware update function which operate on the RAM.
 * Arguments    : none
 * Return Value : none
 **********************************************************************************************************************/
static void copy_update_ramprog(void)
{

    /** Pointer to beginning of RAM section where the code will be copied to. */
    uint8_t * p_ram_section;

    /** Pointer to beginning of ROM section with the code. */
    uint8_t * p_rom_section;

    /** Number of bytes copied. */
    uint32_t  bytes_copied;

    /** Initialize RAM section pointer. */
    p_ram_section = (uint8_t *)__sectop("RPFW_UP_RAM");

    /** Initialize ROM section pointer. */
    p_rom_section = (uint8_t *)__sectop("PFW_UP_RAM");

    /** Copy over code from ROM to RAM. */
    /* WAIT_LOOP */
    for (bytes_copied = 0; bytes_copied < __secsize("PFW_UP_RAM"); bytes_copied++)
    {

        /** Copy over data 1 byte at a time. */
        p_ram_section[bytes_copied] = p_rom_section[bytes_copied];
    }
}
/***********************************************************************************************************************
 * End of function copy_update_ramprog
 **********************************************************************************************************************/

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
fw_up_return_t analyze_and_write_data(const uint8_t *p_recv_data, uint32_t data_size, uint8_t update_mode)
{
    fw_up_return_t  ret1;
    fw_up_return_t  ret2;
    fw_up_return_t  ret3;
    fw_up_fl_data_t fl_data;
    fw_up_fl_data_t bin_data;

    /** Put received data in buffer */
    /* Casting is valid because the pointer value is 32 bits
     * and the reference destination of the buffer is never changed */
    fl_data.src_addr = (uint32_t) p_recv_data;
    fl_data.len      = data_size;
    fl_data.count    = 0;

    do
    {

        /** Put received data in buffer */
        ret1 = fw_up_put_data(&fl_data);

        if (FW_UP_ERR_INVALID_RECORD == ret1)
        {
            return ret1;
        }
        ret3 = FW_UP_SUCCESS;
        do
        {

            /** Get Pointer to firmware data and data size, write address. */
            ret2 = fw_up_get_data(&bin_data);

            if (FW_UP_SUCCESS == ret2)
            {

                /** Write firmware to temporary area */
                ret3 = write_firmware(&bin_data, update_mode);

                if (FW_UP_SUCCESS != ret3)
                {
                    ret3 = FW_UP_ERR_WRITE;
                }
            }
        }
        while ((FW_UP_SUCCESS == ret3) && (FW_UP_SUCCESS == ret2)); /* WAIT_LOOP */
    }
    while (FW_UP_SUCCESS != ret1); /* WAIT_LOOP */

    return ret3;
}
/***********************************************************************************************************************
 * End of function analyze_and_write_data
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_put_data
 * Description  : Put in firmware data to ring buffer.
 * Arguments    : *p_fl_data - Pointer to initial address and data size of firmware data.
 * Return Value : FW_UP_SUCCESS            - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN       - fw_up_open has not been run.
 *                FW_UP_ERR_NULL_PTR       - The argument p_fl_data is null pointer.
 *                FW_UP_ERR_BUF_FULL       - Ring buffer is not empty.
 *                FW_UP_ERR_INVALID_RECORD - Motorola S record format data is invalid.
 *                FW_UP_ERR_INTERNAL       - Memory allocation is failed.
 **********************************************************************************************************************/
static fw_up_return_t fw_up_put_data(fw_up_fl_data_t *p_fl_data)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    uint16_t   cnt;
    uint8_t  * p_data_tmp;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Make sure that the argument is correct. */
    /* To cast in order to compare the address. There is no problem
     * because the information is not lost even if the cast. */
    if (NULL == p_fl_data)
    {
        return FW_UP_ERR_NULL_PTR;
    }

    /** Set initial address of firmware data to be put in ring buffer */
    p_data_tmp = ((uint8_t *)p_fl_data->src_addr) + p_fl_data->count;

    /* WAIT_LOOP */
    for (cnt = p_fl_data->count; cnt < p_fl_data->len; cnt++ )
    {

        /** Put firmware data in ring buffer */
        ret = fw_up_put_mot_s(*p_data_tmp);

        if (FW_UP_SUCCESS != ret)
        {

            /** Set values of data size to be put in ring buffer */
            p_fl_data->count = cnt;

            return ret;
        }

        p_data_tmp++;
    }

    return ret;

}
/***********************************************************************************************************************
 * End of function fw_up_put_data
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: fw_up_get_data
 * Description  : Stores firmware update data to ring buffer.
 * Arguments    : *p_fl_data - Pointer to binary data of initial address.
 * Return Value : FW_UP_SUCCESS       - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN  - fw_up_open has not been run.
 *                FW_UP_ERR_NULL_PTR  - The argument p_fl_data is null pointer.
 *                FW_UP_ERR_BUF_EMPTY - Ring buffer is empty.
 **********************************************************************************************************************/
static fw_up_return_t fw_up_get_data(fw_up_fl_data_t *p_fl_data)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Make sure that the argument is correct. */
    /* To cast in order to compare the address. There is no problem
     * because the information is not lost even if the cast. */
    if (NULL == p_fl_data)
    {
        return FW_UP_ERR_NULL_PTR;
    }

    /** Get initial address and write address, data size of firmware data */
    ret = fw_up_get_binary(p_fl_data);

    return ret;
}
/***********************************************************************************************************************
 * End of function fw_up_get_data
 **********************************************************************************************************************/

/***********************************************************************************************************************
    The following functions operate on the RAM.
 **********************************************************************************************************************/
#pragma section FW_UP_RAM

/***********************************************************************************************************************
 * Function Name: erase_firmware
 * Description  : Erase flash memory to write firmware
 * Arguments    : update_mode - Update mode.
 * Return Value : FW_UP_SUCCESS         - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN    - fw_up_open_flash has not been run.
 *                FW_UP_ERR_ERASE       - Flash API(erase) error.
 *                FW_UP_ERR_SWITCH_AREA - Flash API(switch start-up area) error.
 **********************************************************************************************************************/
fw_up_return_t erase_firmware(uint8_t update_mode)
{
    fw_up_return_t  ret = FW_UP_SUCCESS;

    flash_err_t ret_flash;
    uint8_t     swap_area;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Disable interrupts */
    clrpsw_i();

    /** Switch the start-up area temporarily. */
    swap_area = FLASH_SAS_SWITCH_AREA;

    /** Casting to void * type is valid */
    ret_flash = R_FLASH_Control(FLASH_CMD_SWAPSTATE_SET, (void *)&swap_area);

    if (FLASH_SUCCESS != ret_flash)
    {
        ret = FW_UP_ERR_SWITCH_AREA;
    }
    else
    {

        /** update firmware program mode */
        if (UPDATE_FW == update_mode)
        {

            /** Erase the firmware area. */
            ret_flash = R_FLASH_Erase(FW_UP_FIRM_ST_ADDRESS, FW_UP_FIRM_BLOCK_NUM);
        }

        /** update firmware update program mode 1 */
        else if (UPDATE_FW_UP_1 == update_mode)
        {

            /** Erase the firmware area and constants data area 1. */
            ret_flash = R_FLASH_Erase(FW_UP_CONST_1_ST_ADDRESS, FW_UP_CONST_BLOCK_NUM);
        }

        /** update firmware update program mode 2 */
        else
        {

            /** Erase the firmware area and constants data area 2. */
            ret_flash = R_FLASH_Erase(FW_UP_CONST_2_ST_ADDRESS, FW_UP_CONST_BLOCK_NUM);
        }

        if (FLASH_SUCCESS != ret_flash)
        {
            ret = FW_UP_ERR_ERASE;
        }

        /** Erase the start-up area. */
        ret_flash = R_FLASH_Erase(FW_UP_STUP_ST_ADDRESS, FW_UP_STUP_BLOCK_NUM);

        if (FLASH_SUCCESS != ret_flash)
        {
            ret = FW_UP_ERR_ERASE;
        }
    }

    /** Return the start-up area according to the start-up area setting of the extra area. */
    swap_area = FLASH_SAS_EXTRA;

    /** Casting to void * type is valid */
    ret_flash = R_FLASH_Control(FLASH_CMD_SWAPSTATE_SET, (void *)&swap_area);

    if (FLASH_SUCCESS != ret_flash)
    {
        ret = FW_UP_ERR_SWITCH_AREA;
    }

    /** Enable interrupts */
    setpsw_i();

    return ret;
}
/***********************************************************************************************************************
 * End of function erase_firmware
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: write_firmware
 * Description  : Write firmware data
 * Arguments    : *p_fl_data  - Pointer to binary data of initial address, write address, data size.
 *                update_mode - Update mode.
 * Return Value : FW_UP_SUCCESS             - Processing completed successfully.
 *                FW_UP_ERR_NOT_OPEN        - fw_up_open_flash has not been run.
 *                FW_UP_ERR_NULL_PTR        - The argument p_fl_data is null pointer.
 *                FW_UP_ERR_INVALID_ADDRESS - The write address is invalid.
 *                FW_UP_ERR_WRITE           - Flash API(write) error.
 *                FW_UP_ERR_VERIFY          - Verify data does not match.
 *                FW_UP_ERR_SWITCH_AREA     - Flash API(switch start-up area) error.
 **********************************************************************************************************************/
static fw_up_return_t write_firmware(fw_up_fl_data_t *p_fl_data, uint8_t update_mode)
{
    fw_up_return_t ret = FW_UP_SUCCESS;

    flash_err_t ret_flash;
    uint8_t     swap_area;

    uint32_t   wrt_addr;
    uint32_t   wrt_addr_end;
    uint8_t  * p_src_data;
    uint8_t  * p_dst_data;
    uint32_t   i;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Make sure that the argument is correct. */
    /* To cast in order to compare the address. There is no problem
     * because the information is not lost even if the cast. */
    if (NULL == p_fl_data)
    {
        return FW_UP_ERR_NULL_PTR;
    }

    /** Check programming address. */
    wrt_addr     = p_fl_data->dst_addr;
    wrt_addr_end = (p_fl_data->dst_addr + p_fl_data->len) - 1u;

    if (wrt_addr <= wrt_addr_end)
    {

        /** Update firmware program mode. */
        if (UPDATE_FW == update_mode)
        {

            /** Check the firmware area and start-up area */
            /** Casting is valid because the width is a constant expressible with 32 bits. */
            if ((((uint32_t)FW_UP_FIRM_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_FIRM_EN_ADDRESS >= wrt_addr_end))
                || (((uint32_t)FW_UP_STUP_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_STUP_EN_ADDRESS >= wrt_addr_end)))
            {

                /** Valid address */
                ;
            }
            else
            {
                return FW_UP_ERR_INVALID_ADDRESS;
            }
        }

        /** Update firmware update program mode 1 */
        else if (UPDATE_FW_UP_1 == update_mode)
        {

            /** Check the constants data area 1 and start-up area */
            /** Casting is valid because the width is a constant expressible with 32 bits. */
            if ((((uint32_t)FW_UP_CONST_1_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_CONST_1_EN_ADDRESS >= wrt_addr_end))
                || (((uint32_t)FW_UP_STUP_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_STUP_EN_ADDRESS >= wrt_addr_end)))
            {

                /** Valid address */
                ;
            }
            else
            {
                return FW_UP_ERR_INVALID_ADDRESS;
            }
        }

        /** Update firmware update program mode 2 */
        else
        {

            /** Check the constants data area 2 and start-up area */
            /** Casting is valid because the width is a constant expressible with 32 bits. */
            if ((((uint32_t)FW_UP_CONST_2_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_CONST_2_EN_ADDRESS >= wrt_addr_end))
                || (((uint32_t)FW_UP_STUP_ST_ADDRESS <= wrt_addr) && ((uint32_t)FW_UP_STUP_EN_ADDRESS >= wrt_addr_end)))
            {

                /** Valid address */
                ;
            }
            else
            {
                return FW_UP_ERR_INVALID_ADDRESS;
            }
        }
    }
    else
    {
        return FW_UP_ERR_INVALID_ADDRESS;
    }

    /** Disable interrupts */
    clrpsw_i();

    /** Switch the start-up area temporarily. */
    swap_area = FLASH_SAS_SWITCH_AREA;

    /** Casting to void * type is valid */
    ret_flash = R_FLASH_Control(FLASH_CMD_SWAPSTATE_SET, (void *)&swap_area);

    if (FLASH_SUCCESS != ret_flash)
    {
        ret = FW_UP_ERR_SWITCH_AREA;
    }
    else
    {

    	/** If last write start address of the constants data area */
    	/** Casting is valid because the width is a constant expressible with 32 bits. */
        if (((uint32_t)FW_UP_CONST_1_FIN_ST_ADDRESS == wrt_addr)
            || ((uint32_t)FW_UP_CONST_2_FIN_ST_ADDRESS == wrt_addr))
    	{

    		/** Write the firmware except for the write complete information storage area */
    		ret_flash = R_FLASH_Write(p_fl_data->src_addr, p_fl_data->dst_addr, (p_fl_data->len - 8));
    	}
    	else
    	{

    		/** Write the firmware */
    		ret_flash = R_FLASH_Write(p_fl_data->src_addr, p_fl_data->dst_addr, p_fl_data->len);
    	}

        if (FLASH_SUCCESS != ret_flash)
        {
            ret = FW_UP_ERR_WRITE;
        }
        else
        {

            /** Verify the firmware */
            p_src_data = (uint8_t *)p_fl_data->src_addr;
            p_dst_data = (uint8_t *)p_fl_data->dst_addr;

            /* WAIT_LOOP */
            for (i = 0; i < p_fl_data->len; i++ )
            {
                if ((*p_src_data) != (*p_dst_data))
                {
                    ret = FW_UP_ERR_VERIFY;
                }
                p_src_data++;
                p_dst_data++;
            }
        }
    }

    /** Return the start-up area according to the start-up area setting of the extra area. */
    swap_area = FLASH_SAS_EXTRA;

    /** Casting to void * type is valid */
    ret_flash = R_FLASH_Control(FLASH_CMD_SWAPSTATE_SET, (void *)&swap_area);

    if (FLASH_SUCCESS != ret_flash)
    {
        ret = FW_UP_ERR_SWITCH_AREA;
    }

    /** Enable interrupts */
    setpsw_i();

    return ret;
}
/***********************************************************************************************************************
 * End of function write_firmware
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function Name: switch_start_up_and_reset
 * Description  : Switch the start-up area and execute software reset.
 * Arguments    : none
 * Return Value : FW_UP_ERR_NOT_OPEN          - fw_up_open_flash has not been run.
 *                FW_UP_ERR_INVALID_RESETVECT - The value of reset vector of the firmware is invalid.
 *                FW_UP_ERR_SWITCH_AREA       - Flash API(switch start-up area) error.
 **********************************************************************************************************************/
fw_up_return_t switch_start_up_and_reset(void)
{
    flash_err_t   ret_flash;
    uint32_t    * p_reset_vector_firm;

    /** Check that the fw_up_open() has been executed. */
    if (false == s_is_opened)
    {
        return FW_UP_ERR_NOT_OPEN;
    }

    /** Check the value of reset vector of the firmware. */
    p_reset_vector_firm = (uint32_t *)FW_UP_FIRM_RESETVECT;

    /* To cast in order to compare the address. There is no problem
     * because the information is not lost even if the cast. */
    if (((uint32_t)FW_UP_FIRM_ST_ADDRESS > (*p_reset_vector_firm))
        || (((uint32_t)FW_UP_FIRM_EN_ADDRESS < (*p_reset_vector_firm))
        && ((uint32_t)FW_UP_STUP_ST_ADDRESS > (*p_reset_vector_firm))))
    {
        return FW_UP_ERR_INVALID_RESETVECT;
    }
    else if (FW_UP_BLANK_VALUE == (*p_reset_vector_firm))
    {
        return FW_UP_ERR_INVALID_RESETVECT;
    }
    else
    {

        /** Do nothing */
        ;
    }

    /** Disable interrupts */
    clrpsw_i();

    /** Switch the start-up area temporarily. */
    ret_flash = R_FLASH_Control(FLASH_CMD_SWAPFLAG_TOGGLE, FIT_NO_PTR);

    if (FLASH_SUCCESS != ret_flash)
    {

        /** Enable interrupts */
        setpsw_i();

        return FW_UP_ERR_SWITCH_AREA;
    }

    /** Disable register write protect of SWRR */
    SYSTEM.PRCR.WORD = (SYSTEM.PRCR.WORD | SET_PRC1) | PRCR_KEY;

    /** execute software reset by write SWRR */
    SYSTEM.SWRR = MCU_RESET;

    /* WAIT_LOOP */
    while (true)
    {

        /** Do nothing */
        ;
    }
}
/***********************************************************************************************************************
 * End of function switch_start_up_and_reset
 **********************************************************************************************************************/

#pragma section /* end FW_UP_RAM section */
/***********************************************************************************************************************
    End of RAM program section.
 **********************************************************************************************************************/
