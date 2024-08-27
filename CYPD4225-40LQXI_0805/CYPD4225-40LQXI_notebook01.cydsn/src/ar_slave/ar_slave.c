/**
 * @file ar_slave.c
 *
 * @brief @{Alpine Ridge I2C slave interface source file.@}
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

#include "i2c.h"
#include "ar_slave.h"
#include "utils.h"
#include "pd.h"
#include "dpm.h"
#include "psource.h"

#if AR_SLAVE_IF_ENABLE

/* SCB used for the Alpine Ridge Slave interface. */
static uint8_t            ar_slave_scb_idx = AR_SLAVE_SCB_INDEX;

/* Command register for each PD port. */
static volatile uint32_t  ar_slave_cmd_reg[NO_OF_TYPEC_PORTS];

/* Status register for each PD port. */
static volatile uint32_t  ar_slave_stat_reg[NO_OF_TYPEC_PORTS];

/* Scratch buffer used to hold incoming data from Alpine Ridge. */
static uint8_t            ar_slave_write_buf[AR_SLAVE_MAX_WRITE_SIZE];

/* Scratch buffer used to provide read data back to Alpine Ridge. */
static uint8_t            ar_slave_read_buf[AR_SLAVE_MAX_READ_SIZE];

/* Current read pointer. */
static uint8_t           *ar_slave_read_ptr = 0;

/* Limit upto which I2C data read is allowed. */
static uint8_t           *ar_slave_read_limit = 0;

/* Function called by Alt. Mode layer to update the status register. */
void ar_slave_status_update(uint8_t  port, uint32_t status)
{
    if (port == 0)
    {
        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ar_slave_stat_reg[0] != status)
        {
            ar_slave_stat_reg[0]  = status;
            ar_slave_cmd_reg[0]  &= ~AR_CMD_INT_CLEAR;
            AR_INT_P1_Write (0);
        }
    }

#if CCG_PD_DUALPORT_ENABLE
    if (port == 1)
    {
        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ar_slave_stat_reg[1] != status)
        {
            ar_slave_stat_reg[1]  = status;
            ar_slave_cmd_reg[1]  &= ~AR_CMD_INT_CLEAR;
            AR_INT_P2_Write (0);
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

/*
 * @brief I2C command callback function that implements the actual Alpine Ridge
 * interface logic. This is called from SCB interrupt handler. Since the work
 * to be done is limited, it is completely handled from the callback.
 *
 * @param cmd I2C operation that caused this callback.
 * @param i2c_state Current state of the I2C slave state machine.
 * @param count Size of data written in the case of a write command.
 *
 * @return true if the command is valid, false if it is invalid.
 */
static bool ar_slave_i2c_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count)
{
    bool retval = false;
    uint16_t size;
    uint8_t temp, port = 0;
    uint8_t regaddrloc = 1;

    (void)i2c_state;

    if (cmd == I2C_CB_CMD_WRITE)
    {
#if CCG_PD_DUALPORT_ENABLE
        /* Select the port being read/written. */
        switch (ar_slave_write_buf[0] >> 1)
        {
            case AR_SLAVE_ADDR_P0:
                port = 0;
                break;
            case AR_SLAVE_ADDR_P1:
                port = 1;
                break;
            default:
                return false;
        }
#else
        port = 0;
        regaddrloc = 0;
        count++;
#endif

        /* If we get a single byte only, allow the existing data in the read buf to be read. */
        switch (count)
        {
            case 0:
            case 1:
                /* This will never happen. */
            case 3:
                /* Nothing to do as there is no real data provided. */
                retval = true;
                break;

            case 2:
                /* Only address has been set. Set the read pointer and return. */
                {
                    /* Start with an invalid address. */
                    ar_slave_read_ptr   = 0;
                    ar_slave_read_limit = 0;

                    /* Clear the read data to start with. */
                    memset ((void *)ar_slave_read_buf, 0, sizeof (ar_slave_read_buf));
                    switch (ar_slave_write_buf[regaddrloc])
                    {
                        case AR_REG_CCG_COMMAND:
                            /* Copy in the relevant command register value. */
                            ar_slave_read_buf[0] = 0x04;
                            ar_slave_read_buf[1] = DWORD_GET_BYTE0 (ar_slave_cmd_reg[port]);
                            ar_slave_read_buf[2] = DWORD_GET_BYTE1 (ar_slave_cmd_reg[port]);
                            ar_slave_read_buf[3] = DWORD_GET_BYTE2 (ar_slave_cmd_reg[port]);
                            ar_slave_read_buf[4] = DWORD_GET_BYTE3 (ar_slave_cmd_reg[port]);
                            break;

                        case AR_REG_CCG_STATUS:
                            /* Copy in the relevant command register value. */
                            ar_slave_read_buf[0] = 0x04;
                            ar_slave_read_buf[1] = DWORD_GET_BYTE0 (ar_slave_stat_reg[port]);
                            ar_slave_read_buf[2] = DWORD_GET_BYTE1 (ar_slave_stat_reg[port]);
                            ar_slave_read_buf[3] = DWORD_GET_BYTE2 (ar_slave_stat_reg[port]);
                            ar_slave_read_buf[4] = DWORD_GET_BYTE3 (ar_slave_stat_reg[port]);
                            break;

                        default:
                            return false;
                    }

                    /* Set the valid address range for the read operation. */
                    ar_slave_read_ptr   = (uint8_t *)ar_slave_read_buf;
                    ar_slave_read_limit = (uint8_t *)(ar_slave_read_buf + AR_SLAVE_MAX_READ_SIZE);
                }

                retval = true;
                break;

            default:
                /* Valid write request. The only write-able register is the command register. */
                if (ar_slave_write_buf[regaddrloc] == AR_REG_CCG_COMMAND)
                {
                    /* Handle soft reset request. */
                    if ((ar_slave_write_buf[regaddrloc + 2] & AR_CMD_CCG_RESET) != 0)
                    {
                        AR_INT_P1_Write (1);
#if CCG_PD_DUALPORT_ENABLE
                        AR_INT_P2_Write (1);
#endif /* CCG_PD_DUALPORT_ENABLE */
                        CySoftwareReset ();
                    }

                    /* Handle interrupt clear command. */
                    if ((ar_slave_write_buf[regaddrloc + 2] & AR_CMD_INT_CLEAR) != 0)
                    {
                        if (port == 0)
                        {
                            ar_slave_cmd_reg[port] |= AR_CMD_INT_CLEAR;
                            AR_INT_P1_Write (1);
                        }
#if CCG_PD_DUALPORT_ENABLE
                        if (port != 0)
                        {
                            ar_slave_cmd_reg[port] |= AR_CMD_INT_CLEAR;
                            AR_INT_P2_Write (1);
                        }
#endif /* CCG_PD_DUALPORT_ENABLE */
                    }

                    retval = true;
                }
                break;
        }
    }

    if (cmd == I2C_CB_CMD_READ)
    {
        /* Read handler: return data while it is available. */
        if ((ar_slave_read_ptr != 0) && (ar_slave_read_ptr < ar_slave_read_limit))
        {
            size = GET_MIN (I2C_SCB_TX_FIFO_SIZE, (unsigned int)(ar_slave_read_limit - ar_slave_read_ptr));
            if (size != 0)
            {
                i2c_scb_write (ar_slave_scb_idx, ar_slave_read_ptr, (uint8_t)size, &temp);
                ar_slave_read_ptr += temp;
                retval = true;
            }
        }
    }

    if (cmd == I2C_CB_CMD_XFER_END)
    {
        /* Just roll the read pointer back to the base of the read buffer. */
        ar_slave_read_ptr   = (uint8_t *)ar_slave_read_buf;
        ar_slave_read_limit = (uint8_t *)(ar_slave_read_buf + AR_SLAVE_MAX_READ_SIZE);
        retval = true;
    }

    return retval;
}

/* Alpine Ridge interface initialization. */
void ar_slave_init(uint8_t ar_scbnum, uint8_t portsel)
{
    /* Clear all structures and status. */
    memset ((void *)ar_slave_write_buf, 0, AR_SLAVE_MAX_WRITE_SIZE);
    memset ((void *)ar_slave_stat_reg, 0, sizeof (ar_slave_stat_reg));

    ar_slave_scb_idx    = ar_scbnum;
    ar_slave_read_ptr   = 0;
    ar_slave_read_limit = 0;

#if CCG_PD_DUALPORT_ENABLE
    i2c_scb_init (ar_slave_scb_idx, I2C_SCB_MODE_ALP_RIDGE, AR_SLAVE_SCB_CLOCK_FREQ,
            AR_SLAVE_ADDR_P0, AR_SLAVE_ADDR_MASK,
            ar_slave_i2c_cmd_callback, ar_slave_write_buf, AR_SLAVE_MAX_WRITE_SIZE);
#else
    i2c_scb_init (ar_slave_scb_idx, I2C_SCB_MODE_ALP_RIDGE, AR_SLAVE_SCB_CLOCK_FREQ,
            ((portsel != 0) ? AR_SLAVE_ADDR_P1 : AR_SLAVE_ADDR_P0), I2C_SLAVE_ADDR_MASK_DEFAULT,
            ar_slave_i2c_cmd_callback, ar_slave_write_buf, AR_SLAVE_MAX_WRITE_SIZE);
#endif

    /* De-assert the interrupt at start-up. */
    ar_slave_cmd_reg[0] = AR_CMD_INT_CLEAR;
    AR_INT_P1_Write (1);

#if CCG_PD_DUALPORT_ENABLE
    ar_slave_cmd_reg[1] = AR_CMD_INT_CLEAR;
    AR_INT_P2_Write (1);
#endif
}

/* Check if the Alpine Ridge interface is idle and prepare for deep sleep. */
bool ar_slave_sleep(void)
{
    /* Assume that we can sleep by default. */
    bool stat = true;

    /* Sleep is not if the I2C interface is busy. */
    if (!i2c_scb_is_idle (ar_slave_scb_idx))
    {
        stat = false;
    }
    else
    {
        /* Enable wakeup due to HPI activity. */
        i2c_scb_enable_wakeup (ar_slave_scb_idx);
    }

    return stat;
}

#endif /* AR_SLAVE_IF_ENABLE */

/* [] END OF FILE */

