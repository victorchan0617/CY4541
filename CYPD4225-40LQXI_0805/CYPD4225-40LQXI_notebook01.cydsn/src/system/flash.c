/**
 * @file flash.c
 *
 * @brief @{Flash command handler source file.@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <stdbool.h>
#include <config.h>
#include <utils.h>
#include <status.h>
#include <system.h>
#include <boot.h>
#include <ccg3_regs.h>
#include <flash_config.h>
#include <flash.h>

/* 
 * Flags to indicate Flashing mode. Flash read and write requests
 * are honoured only when any of these flags are set.
 */
static uint8_t gl_flash_mode_en = 0;

/* Lowest flash row number that can be accessed. */
static uint16_t gl_flash_access_first = 0;

/* Highest flash row number that can be accessed. */
static uint16_t gl_flash_access_last = CCG_LAST_FLASH_ROW_NUM;

/* Flash row containing metadata for the alternate firmware image. */
static uint16_t gl_flash_metadata_row = CCG_LAST_FLASH_ROW_NUM + 1;

/* Last boot loader flash row. Used for read protection. */
static uint16_t gl_flash_bl_last_row = CCG_LAST_FLASH_ROW_NUM;

#if (FLASH_ENABLE_NB_MODE == 1)

/* SPC Interrupt Number. */
#define FLASH_SPC_INTR                  (0x0C)

/* MACROS for SROM APIs. */

#define SROM_API_RETURN_VALUE                   \
    (((CY_FLASH_CPUSS_SYSARG_REG & 0xF0000000u) \
      == 0xA0000000u) ? CYRET_SUCCESS :         \
     (CY_FLASH_CPUSS_SYSARG_REG & 0x0000000Fu))

/* Keys used in SROM APIs. */
#define SROM_FLASH_API_KEY_ONE          (0xB6)

#define SROM_FLASH_API_KEY_TWO(x)       (uint32_t)(0xD3u + x)

#define SROM_FLASH_KEY_TWO_OFFSET       (0x08)

/* Offset of Argument 1 and 2 (Words) for SROM APIs in SRAM Buffer. */
#define SROM_API_ARG0_OFFSET            (0x00)
#define SROM_API_ARG1_OFFSET            (0x01)

/* SROM LOAD FLASH API. */
#define SRPM_LOAD_FLASH_API_OPCODE      (0x04)
#define SROM_LOAD_FLASH_DATA_OFFSET     (0x02)
#define SROM_LOAD_FLASH_BYTE_ADDR       (0x00)
#define SROM_LOAD_FLASH_BYTE_ADDR_OFFSET        (0x10)
#define SROM_LOAD_FLASH_MACRO_OFFSET    (0x18)

/* Clock Backup and Configuration APIs */
#define SROM_CLK_CONFIG_SIZE            (0x06)
#define SROM_CLK_BACKUP_API_OPCODE      (0x16)
#define SROM_CLK_CONFIG_API_OPCODE      (0x15)

/* Clock Restore API */
#define SROM_CLK_RESTORE_API_OPCODE     (0x17)

/* Non-BLOCKING Write Flash Row API */
#define SROM_NB_FLASH_ROW_API_OPCODE    (0x07)
#define SROM_NB_FLASH_ROW_NUM_OFFSET    (0x10)

/* Resume Non-Blocking API */
#define SROM_RESUME_NB_API_OPCODE       (0x09)
    
/* Abort Non-Blokcing Flash Row Write API Opcode. */
#define SROM_ABORT_FLASH_WRITE_OPCODE   (0x1C)

/* CPUSS SYSARG return value mask. */
#define CPUSS_SYSARG_RETURN_VALUE_MASK  (0xF0000000)

/* CPUSS SYSARG return value mask. */
#define CPUSS_SYSARG_ERROR_RETURN_VALUE (0xF0000000)
    
/* Buffer used for SROM APIs */
static uint32_t gl_srom_arg_buf[CCG_FLASH_ROW_SIZE + 2];

/* SRAM Variable to hold Clock Settings before starting Flash Write operation. */
static uint32_t gl_clockSettings[SROM_CLK_CONFIG_SIZE];

/* Callback function registered by user for Non Blokcing Flash Write Row operation. */
static flash_cbk_t flash_notify;

/* Counter to keep track of the SPC Interrupts while Non-Blocking Flash update. */
static uint8_t gl_spc_intr_counter = 0;

/* Flag to indciate Abort request for current Non Blokcing Flash write operation was received. */
static bool gl_flash_nb_write_abort = false;

CY_ISR_PROTO(flash_spc_intr_handler);

/* Refer BROS 001-88589 Section 4.6.2.5 for SROM API's description. */

/*
 * @brief Execute SROM LOAD FLASH API
 *
 * Description
 * This API loads the page latch buffer with the data to be programmed in flash,
 * This is the first API in FLASH ROW operation.
 *
 * @param data Pointer to data to be flashed
 * @param flash_macro_index Flash macro number
 *
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_load_flash_bytes_api(uint8_t *data, uint8_t flash_macro_index)
{
    uint8_t retValue;
    /* Write 128 bytes in a temp buf which is eventually passed to SROM API. */
    memcpy ((void *)&gl_srom_arg_buf[SROM_LOAD_FLASH_DATA_OFFSET], data,
            CCG_FLASH_ROW_SIZE);

    /* Fill in the arguments for API. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET] = (uint32_t)
                ((flash_macro_index << SROM_LOAD_FLASH_MACRO_OFFSET) |
                (SROM_LOAD_FLASH_BYTE_ADDR << SROM_LOAD_FLASH_BYTE_ADDR_OFFSET) |
                (SROM_FLASH_API_KEY_TWO(SRPM_LOAD_FLASH_API_OPCODE) <<
                SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* Number of bytes to flash - 1 */
    gl_srom_arg_buf[SROM_API_ARG1_OFFSET] = CCG_FLASH_ROW_SIZE-1;

    /* SYSARG */
    CPUSS->sysarg = (uint32)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SRPM_LOAD_FLASH_API_OPCODE);
    /* Read the Result. */
    retValue = SROM_API_RETURN_VALUE;
    if (retValue != 0)
    {
        return false;
    }
    return true;
}

/*
 * @brief Execute SROM Clock Backup and Config APIs
 *
 * Description
 * This API backup the current clock settings and configures
 * IMO, HF_CLK and pump clock for Flashing operation.
 *
 * @param None
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_clock_backup_config_api(void)
{
    uint8_t retValue;

    /* Clock Backup API */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET] = (uint32_t)
            ((SROM_FLASH_API_KEY_TWO(SROM_CLK_BACKUP_API_OPCODE) <<
            SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    gl_srom_arg_buf[SROM_API_ARG1_OFFSET] = (uint32_t)&gl_clockSettings[0u];

    /* SYSARG */
    CPUSS->sysarg = (uint32)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_CLK_BACKUP_API_OPCODE);
    /* Read the Result. */
    retValue = SROM_API_RETURN_VALUE;

    if (retValue == 0)
    {
        /* Clock Configuration API. */
        /* SYSARG */
        CPUSS->sysarg = (uint32) ((SROM_FLASH_API_KEY_TWO(SROM_CLK_CONFIG_API_OPCODE)
                <<  SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
        /* SYSREQ */
        CPUSS->sysreq  = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_CLK_CONFIG_API_OPCODE);
        /* Read the Result. */
        retValue = SROM_API_RETURN_VALUE;
        if (retValue == 0)
        {
           return true;
        }
    }
    return false;
}

/*
 * @brief Execute Non Blocking Write Row
 *
 * Description
 * This API performs the first part of the write row operation, which is
 * the pre-program operation.
 *
 * @param row_num Flash Row Number to be programmed
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_nb_flash_write_api(uint16_t row_num)
{
    /* Arguments */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)
            ((row_num << SROM_NB_FLASH_ROW_NUM_OFFSET) |
            (SROM_FLASH_API_KEY_TWO(SROM_NB_FLASH_ROW_API_OPCODE) <<
            SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);

    /* This command results in three SPC interrupts. Reset the counter. */
    gl_spc_intr_counter = 0;
    /* SYSARG */
    CPUSS->sysarg = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_NB_FLASH_ROW_API_OPCODE);

    /*
     * Check if the status in SYSARG is failure. If yes, then return false.
     * Otherwise, request was successful.
     */
    if ((CPUSS->sysarg & CPUSS_SYSARG_RETURN_VALUE_MASK) ==
        CPUSS_SYSARG_ERROR_RETURN_VALUE)
    {
        return false;
    }
    return true;
}

/*
 * @brief Execute SROM Clock Restore APIs
 *
 * @param None
 * @rerurn Status of API. true if success, false otherwise
 */
static bool srom_clock_restore_api(void)
{
    uint8_t retValue;

    /* Arguments. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET] = (uint32_t)
            ((SROM_FLASH_API_KEY_TWO(SROM_CLK_RESTORE_API_OPCODE) <<
            SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    gl_srom_arg_buf[SROM_API_ARG1_OFFSET] = (uint32) &gl_clockSettings[0u];

    /* SYSARG */
    CPUSS->sysarg = (uint32) &gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_CLK_RESTORE_API_OPCODE);
    retValue = SROM_API_RETURN_VALUE;
    if (retValue != 0)
    {
       return false;
    }
    return true;
}

/*
 * @brief Execute SROM Non Blocking Resume APIs
 *
 * @param None
 * @rerurn None
 */
static void srom_resume_non_blokcing(void)
{
    /* Execute Resume Non Blocking API */
    
    /* Arguments. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)((SROM_FLASH_API_KEY_TWO(SROM_RESUME_NB_API_OPCODE)
        << SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* SYSARG */
    CPUSS->sysarg = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_RESUME_NB_API_OPCODE);
}

/*
 * @brief Execute SROM Non Blocking Abort APIs
 *
 * @param None
 * @rerurn None
 */
static void srom_abort_flash_write(void)
{
    /* Execute Abort Non Blaokcing Flash Row Write operation. */
    
    /* Arguments. */
    gl_srom_arg_buf[SROM_API_ARG0_OFFSET]  = (uint32_t)((SROM_FLASH_API_KEY_TWO(SROM_ABORT_FLASH_WRITE_OPCODE)
        << SROM_FLASH_KEY_TWO_OFFSET) | SROM_FLASH_API_KEY_ONE);
    /* SYSARG */
    CPUSS->sysarg = (uint32_t)&gl_srom_arg_buf[SROM_API_ARG0_OFFSET];
    /* SYSREQ */
    CPUSS->sysreq = (CPUSS_SYSREQ_SYSCALL_REQ | SROM_ABORT_FLASH_WRITE_OPCODE);
}

/* SPC Interrupt for Resume Non-Blocking SROM API */
CY_ISR(flash_spc_intr_handler)
{
    flash_write_status_t status = FLASH_WRITE_COMPLETE;

    /* Check if Abort request is pending. */
    if (gl_flash_nb_write_abort)
    {
        /*
         * See if we are already in last phase of Flash update i.e the
         * third interrupt. If yes, there is no point in aborting the flash as there
         * is nothing left to abort.
         */
        if (gl_spc_intr_counter < 2)
        {
           srom_abort_flash_write ();
           status = FLASH_WRITE_ABORTED;
        }
        else
        {
            srom_resume_non_blokcing ();
            status = FLASH_WRITE_COMPLETE_AND_ABORTED;
        }            
        gl_flash_nb_write_abort = false;        
    }
    else
    {
        /*
         * Once Non-Blocking Flash row update process starts, this interrupt
         * will fire three times. FW is expected to call Resume Non-Blocking
         * SROM API from here.
         */
        gl_spc_intr_counter++;

        /* Resume Non-Blokcing Operation. */
        srom_resume_non_blokcing ();
    }
    
    /*
     * See if this is the third interrupt of flash write sequence or if there
     * was an abort request. In both cases, reset the counters/clocks and invoke
     * the registered callback.
     */
    if ((gl_spc_intr_counter == 3) || (status != FLASH_WRITE_COMPLETE))
    {
        /* Restore Clock at the end of one flash row write or abort sequence. */
        srom_clock_restore_api ();
        /* Reset counter. */
        gl_spc_intr_counter = 0;
        /* Invoke Callback. */
        if (flash_notify != NULL)
        {
            /* Callback should notify the status of flash write as well. */
            flash_notify (status);
        }
    }
}
#endif /* FLASH_ENABLE_NB_MODE */

void flash_enter_mode(bool is_enable, flash_interface_t mode)
{
    /* Enter or Exit the Flashing mode. Only one mode will be active at a time. */
    if (is_enable)
    {
        gl_flash_mode_en = (1 << mode);
    }
    else
    {
        gl_flash_mode_en = 0;
    }
}

bool flash_access_get_status (uint8_t modes)
{
    return ((bool)((gl_flash_mode_en & modes) != 0));
}

void flash_set_access_limits (uint16_t start_row, uint16_t last_row, uint16_t md_row,
        uint16_t bl_last_row)
{
    /* Store the access limits if they are valid. */
    if ((start_row <= CCG_LAST_FLASH_ROW_NUM) && (last_row <= CCG_LAST_FLASH_ROW_NUM))
    {
        gl_flash_access_first = start_row;
        gl_flash_access_last  = last_row;
        gl_flash_metadata_row = md_row;
    }

    /* Store the BL last row information. */
    gl_flash_bl_last_row = bl_last_row;
}

static ccg_status_t flash_blocking_row_write(uint16_t row_num, uint8_t *data)
{
    /* Invoke Flash Write API. */
    if (CYRET_SUCCESS != CySysFlashWriteRow (row_num, data))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }

    return CCG_STAT_SUCCESS;
}

/*
 * @brief Handle Clear Flash Row operation.
 *
 * Description
 * This function clears spcified flash row
 *
 * @param row_num Flash Row Number
 * @return ccg_status_t Status Code
 */
ccg_status_t flash_row_clear(uint16_t row_num)
{
    uint8_t buffer[CCG_FLASH_ROW_SIZE] = {0};
    return flash_blocking_row_write (row_num, buffer);
}

#if (FLASH_ENABLE_NB_MODE == 1)

void flash_non_blocking_write_abort(void)
{
    /* Set a flag which which will be sampled in next SPCIF interrupt. */
    gl_flash_nb_write_abort = true;
}

static ccg_status_t flash_non_blocking_row_write(uint16_t row_num, uint8_t *data,
        flash_cbk_t cbk)
{
    uint8_t flash_macro_index;

    /* Determine the Flash Macro from Row number. */
    flash_macro_index = 0;
    if (row_num > 0x1FF)
        flash_macro_index++;

    /* Load Flash Bytes SROM API. */
    if (!srom_load_flash_bytes_api (data, flash_macro_index))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }

    /* Clock Backup and Configuration APIs. */
    if (!(srom_clock_backup_config_api ()))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }

    /* Set SPC Interrupt Vector and enable Interrupt. */
    CyIntDisable (FLASH_SPC_INTR);
    CyIntSetVector (FLASH_SPC_INTR, &flash_spc_intr_handler);
    CyIntEnable (FLASH_SPC_INTR);

    /* Flash Callback. */
    flash_notify = cbk;

    /* Non Blocking Write Row API. */
    if (!(srom_nb_flash_write_api (row_num)))
    {
        return CCG_STAT_FLASH_UPDATE_FAILED;
    }
    /*
     * Non-Blocking Flash Write has started. Response will
     * go back only after write completes.
     */
    return CCG_STAT_NO_RESPONSE;
}
#endif /* FLASH_ENABLE_NB_MODE */

ccg_status_t flash_row_write(uint16_t row_num, uint8_t *data, flash_cbk_t cbk)
{
    /* Initialize Return Status Value. */
    ccg_status_t status = CCG_STAT_NO_RESPONSE;

#if ((defined(CCG_BOOT)) || (defined(CCG_PSEUDO_METADATA_DISABLE)))
    uint32_t seq_num;
    uint16_t offset;
#else /* !CCG_PSEUDO_METADATA_DISABLE */
    sys_fw_metadata_t *fw_metadata;
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    /* Can't handle flash update request if Flashing mode is not active. */
    if (gl_flash_mode_en == 0)
    {
        return CCG_STAT_NOT_READY;
    }

    if ((data == NULL) || (row_num < gl_flash_access_first) ||
            ((row_num > gl_flash_access_last) && (row_num != gl_flash_metadata_row)))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }
    
#if CCG_BOOT
    /*
     * Ensure boot loader is not allowed to write to reserved rows (if any) in FW2 Image area.
     * Certain applications use FW Image 2 area to store APP priority, Customer info etc.
     * This rows are sandwiched between last row of image 2 and image 2's metadata table row.
     */
    if ((row_num > CCG_IMG2_LAST_FLASH_ROW_NUM) && (row_num < CCG_IMG2_METADATA_ROW_NUM))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }
#endif /* CCG_BOOT */

#if ((defined(CCG_BOOT)) || (defined(CCG_PSEUDO_METADATA_DISABLE)))
    /* Byte offset to the sequence number field in metadata. */
    offset  = (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE + CCG_FW_METADATA_BOOTSEQ_OFFSET);
    if (row_num == CCG_IMG1_METADATA_ROW_NUM)
    {
        /* Set sequence number to 1 + that of FW2. */
        seq_num = boot_get_boot_seq (SYS_FW_MODE_FWIMAGE_2) + 1;
        ((uint32_t *)data)[offset / 4] = seq_num;
    }
    if (row_num == CCG_IMG2_METADATA_ROW_NUM)
    {
        /* Set sequence number to 1 + that of FW1. */
        seq_num = boot_get_boot_seq (SYS_FW_MODE_FWIMAGE_1) + 1;
        ((uint32_t *)data)[offset / 4] = seq_num;
    }
#else /* CCG_PSEUDO_METADATA_DISABLE */
    /*
     * Refer JITM#2, In FW mode, For metadata rows write,
     * FW image updates the coresponding pseudo metadata
     * row instead of actual metadata row.
     */
    if (row_num == CCG_IMG1_METADATA_ROW_NUM)
    {
        row_num = gl_img2_fw_metadata->boot_last_row;
        /*
         * Mark the METADATA_VALID as "CP" which indicates that FW flashing is
         * now complete. After RESET, this will signal the current FW to
         * validate the other image and then jump to it.
         */
        fw_metadata= (sys_fw_metadata_t *)(data + (CCG_FLASH_ROW_SIZE
                - CCG_METADATA_TABLE_SIZE));
        fw_metadata->metadata_valid = SYS_PSEUDO_METADATA_VALID_SIG;
    }
    else if (row_num == CCG_IMG2_METADATA_ROW_NUM)
    {
        row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        /* 
         * Mark the METADATA_VALID as "CP" which indicates that FW flashing is
         * now complete. After RESET, this will signal the current FW to validate
         * the other image and then jump to it.
         */
        fw_metadata= (sys_fw_metadata_t *)(data + (CCG_FLASH_ROW_SIZE
                - CCG_METADATA_TABLE_SIZE));
        fw_metadata->metadata_valid = SYS_PSEUDO_METADATA_VALID_SIG;
    }
#endif /* CCG_PSEUDO_METADATA_DISABLE */

#if (FLASH_ENABLE_NB_MODE == 1)
   /*
    * Determine mode of flashing: Blocking or Non-Blocking based
    * on the callback function pointer.
    */
    if (cbk == NULL)
    {
        status = flash_blocking_row_write (row_num, data);
    }
    else
    {
        status = flash_non_blocking_row_write (row_num, data, cbk);
    }
#else
    /* Blocking Flash Row Write in Bootloader mode. */
    /* Handle only if Flashing mode is active. */
    status = flash_blocking_row_write (row_num, data);
#endif /* FLASH_ENABLE_NB_MODE */
    return status;
}

ccg_status_t flash_row_read(uint16_t row_num, uint8_t* data)
{
    /* Can't handle flash update request if Flashing mode is not active. */
    if (gl_flash_mode_en == 0)
    {
        return CCG_STAT_NOT_READY;
    }

    /* We allow any row outside of the boot-loader to be read. */
    if ((data == NULL) || (row_num <= gl_flash_bl_last_row) ||
            (row_num > CCG_LAST_FLASH_ROW_NUM))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    memcpy (data, (void *)(row_num << CCG_FLASH_ROW_SHIFT_NUM), CCG_FLASH_ROW_SIZE);
    return CCG_STAT_SUCCESS;
}

ccg_status_t flash_set_app_priority(flash_app_priority_t app_priority)
{
    uint8_t temp_buf[CCG_FLASH_ROW_SIZE] = {0};
    /* Ensure APP Priority value is valid. */
    if (app_priority > FLASH_APP_PRIORITY_IMAGE_2)
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }
    else
    {
        /* Set APP Priority Field. */
        temp_buf[0] = app_priority;
        if (CYRET_SUCCESS == CySysFlashWriteRow (CCG_APP_PRIORITY_ROW_NUM, temp_buf))
        {
            return CCG_STAT_SUCCESS;
        }
        else
        {
            return CCG_STAT_FAILURE;
        }
    }
}
/* [] END OF FILE */
