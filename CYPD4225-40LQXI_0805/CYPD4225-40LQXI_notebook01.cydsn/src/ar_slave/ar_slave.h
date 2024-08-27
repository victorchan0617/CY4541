/**
 * @file ar_slave.h
 *
 * @brief @{Alpine Ridge I2C slave interface header file.@}
 */

/*
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

#ifndef _AR_SLAVE_H_
#define _AR_SLAVE_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "status.h"
#include "i2c.h"

/**
 * @brief Default SCB index used for Alpine Ridge Slave interface.
 */
#define AR_SLAVE_SCB_INDEX                      (0x1)

/**
 * @brief AR Slave interface clock frequency can be upto 1 MHz.
 */
#define AR_SLAVE_SCB_CLOCK_FREQ                 (I2C_SCB_CLOCK_FREQ_1_MHZ)

/**
 * @brief Minimum AR slave write size: Slave address + Register Address.
 */
#define AR_SLAVE_MIN_WRITE_SIZE                 (2)

/**
 * @brief Maximum AR slave write size: We should never get writes longer than 32 bytes.
 */
#define AR_SLAVE_MAX_WRITE_SIZE                 (32)

/**
 * @brief Maximum AR slave read size.
 */
#define AR_SLAVE_MAX_READ_SIZE                  (32)

/**
 * @brief Slave address associated with USB-PD port number 0. This is defined by Intel.
 */
#define AR_SLAVE_ADDR_P0                        (0x38)

/**
 * @brief Slave address associated with USB-PD port number 1. This is defined by Intel.
 */
#define AR_SLAVE_ADDR_P1                        (0x3F)

/**
 * @brief I2C slave address mask to be applied on the incoming slave address.
 *
 * Since a single I2C block is used to handle transaction on two different slave addresses,
 * a mask needs to be applied while checking the incoming preamble for an address match.
 * This mask allows comparison of only the address bits that do not change across the two
 * addresses.
 */
#define AR_SLAVE_ADDR_MASK                      (0xF0)

/**
 * @brief Alpine Ridge command value that requests a CCG device reset.
 */
#define AR_CMD_CCG_RESET                        (0x02)

/**
 * @brief Alpine Ridge command value to clear the interrupt from CCG.
 */
#define AR_CMD_INT_CLEAR                        (0x04)

/**
 * @brief List of Alpine Ridge slave interface registers.
 *
 * The Thunderbolt Alternate Mode specification defines the following set of registers
 * that should be implemented by a USB-PD port controller in Thunderbolt enabled systems.
 */
typedef enum
{
    AR_REG_CCG_COMMAND = 0x50,                  /**< CCG command register. */
    AR_REG_CCG_STATUS  = 0x5F                   /**< CCG status register. */
} ar_slave_reg_addr_t;

/**
 * @brief Initialize the Alpine Ridge slave interface module.
 *
 * This function initializes the Alpine Ridge slave interface module and configures it
 * to use the specified SCB block. The SCB will be configured as an I2C slave block,
 * and the interrupt output will also be initialized to a de-asserted state.
 *
 * Since only two registers are to be implemented, and the commands to be implemented are
 * simple, the complete module is implemented using the I2C command callbacks.
 *
 * @param ar_scbnum SCB index to be used for the Alpine Ridge slave interface.
 * @param portsel Alpine Ridge slave port selection. Only applicable for CCG3 designs.
 *
 * @return None
 */
void ar_slave_init(uint8_t ar_scbnum, uint8_t portsel);

/**
 * @brief Update the AR status register and send an event to the Alpine Ridge.
 *
 * This function is used by the application layer to update the content of the Alpine
 * Ridge status register. If the content of the register is changing, CCG asserts
 * the corresponding interrupt to notify Alpine Ridge about the status change.
 *
 * @param port USB-PD port index corresponding to the status update.
 * @param status Value to be written into the status register.
 *
 * @return None
 */
void ar_slave_status_update(uint8_t port, uint32_t status);

/**
 * @brief Check whether the AR slave interface is idle so that device can be placed into sleep.
 *
 * This function should be called prior to placing the CCG device in deep sleep. Deep sleep
 * entry is only allowed if this function returns true.
 *
 * @return true if the interface is idle, false otherwise.
 */
bool ar_slave_sleep(void);

#endif /* _AR_SLAVE_H_ */

/* [] END OF FILE */
