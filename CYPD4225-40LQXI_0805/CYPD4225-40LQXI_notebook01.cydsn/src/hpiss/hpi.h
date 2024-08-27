/**
 * @file hpi.h
 *
 * @brief @{Host Processor Interface (HPI) header file.@}
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

#ifndef _HPI_H_
#define _HPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <config.h>
#include "status.h"
#include "i2c.h"

#if CCG_HPI_PD_ENABLE
#include "pd.h"
#include "dpm.h"
#endif /* CCG_HPI_PD_ENABLE */

/**************************************************************************************************
 ******************************************** MACROS **********************************************
 *************************************************************************************************/

/**
 * @brief Timer ID used for command retry checks.
 */
#define HPI_PD_CMD_TIMER                (0x80)

/**************************************************************************************************
 ****************************************** DATA TYPES ********************************************
 *************************************************************************************************/

/**
 * @typedef hpi_reg_section_t
 * @brief HPI register section definitions.
 *
 * HPI registers are grouped into sections corresponding to the functions that are supported.
 */
typedef enum
{
    HPI_REG_SECTION_DEV = 0,            /**< Device information registers. */
    HPI_REG_SECTION_PORT_0,             /**< USB-PD Port 0 related registers. */
    HPI_REG_SECTION_PORT_1,             /**< USB-PD Port 1 related registers. */
    HPI_REG_SECTION_ALL                 /**< Special definition to select all register spaces. */
} hpi_reg_section_t;

/**
 * @typedef hpi_reg_part_t
 * @brief Types of HPI register/memory regions.
 */
typedef enum
{
    HPI_REG_PART_REG = 0,               /**< Register region. */
    HPI_REG_PART_DATA = 1,              /**< Data memory for device section. */
    HPI_REG_PART_FLASH = 2,             /**< Flash memory. */
    HPI_REG_PART_PDDATA_READ = 4,       /**< Read Data memory for port section. */
    HPI_REG_PART_PDDATA_READ_H = 5,     /**< Upper fraction of read data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE = 8,      /**< Write Data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE_H = 9     /**< Upper fraction of write data memory for port section. */
} hpi_reg_part_t;

/**
 * @typedef hpi_write_cb_t
 * @brief Handler for HPI register writes.
 * @return Type of response to be sent to the EC. Only a single byte response
 * can be sent from here. Use hpi_reg_enqueue_event to send longer responses.
 */
typedef uint8_t (*hpi_write_cb_t)(
        uint16_t  reg_addr,             /**< Address of register that got written. */
        uint8_t   wr_size,              /**< Size of write operation. */
        uint8_t  *wr_data               /**< Buffer containing data written. */
        );

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/

/**
 * @brief Initialize the HPI interface.
 *
 * This function initializes the I2C interface and EC_INT GPIO used for HPI hardware interface, and
 * initializes all HPI registers to their default values.
 *
 * @param scb_idx Index of SCB block to be used for HPI. Please note that this parameter is
 * not validated, and it is the caller's responsibility to pass the correct value.
 * @return None
 */
void hpi_init(uint8_t scb_idx);

/**
 * @brief HPI task handler.
 *
 * This function handles the commands from the EC through the HPI registers. HPI writes from the EC
 * are handled in interrupt context, and any associated work is queued to be handled by this function.
 * The hpi_task is expected to be called periodically from the main task loop of the firmware
 * application.
 *
 * @return None
 */
void hpi_task(void);

/**
 * @brief Enqueue an event to the EC through the HPI interface.
 *
 * This function is used by the PD stack and application layers to send event
 * notifications to the EC through the HPI registers.
 *
 * @param section Register section through which event is to be reported.
 * @param status The event code to be stored into the response register.
 * @param length Length of the data associated with the event.
 * @param data Pointer to buffer containing data associated with the event.
 *
 * @return true if the event queue has space for the event, false if there
 * is an overflow.
 */
bool hpi_reg_enqueue_event(hpi_reg_section_t section, uint8_t status, uint16_t length,
    uint8_t *data);

#if CCG_HPI_PD_ENABLE

/**
 * @brief Handler for PD events reported from the stack.
 *
 * Internal function used to receive PD events from the stack and to update the HPI registers.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that needs to be de-referenced
 * based on event type.
 *
 * @return None
 */
void hpi_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

#endif /* CCG_HPI_PD_ENABLE */

/**
 * @brief Check whether EC init complete event has been received.
 *
 * This function is used by the application to check whether the EC has sent
 * the EC initialization complete event notification.
 *
 * @return true if EC init has been received, false otherwise.
 */
bool hpi_is_ec_ready (void);

/**
 * @brief Update firmware version information in HPI registers.
 *
 * This is an internal function used to update the firmware version information
 * in the HPI registers.
 *
 * @param bl_version Buffer containing Bootloader version information.
 * @param fw1_version Buffer containing firmware-1 version information.
 * @param fw2_version Buffer containing firmware-2 version information.
 *
 * @return void
 */
void hpi_update_versions (uint8_t *bl_version, uint8_t *fw1_version,
        uint8_t *fw2_version);

/**
 * @brief Set device mode and reason register values.
 *
 * This is an internal function used to update the device mode and boot mode
 * reason HPI registers.
 *
 * @param dev_mode Value to be set into the device mode register.
 * @param mode_reason Value to be set into the boot mode reason register.
 *
 * @return void
 */
void hpi_set_mode_regs (uint8_t dev_mode, uint8_t mode_reason);

/**
 * @brief Update the firmware location HPI registers.
 *
 * This is an internal function used to update the firmware binary location
 * HPI registers.
 *
 * @param fw1_location Flash row where FW1 is located.
 * @param fw2_location Flash row where FW2 is located.
 *
 * @return void
 */
void hpi_update_fw_locations (uint16_t fw1_location, uint16_t fw2_location);

/**
 * @brief Debug function to set reserved register values.
 *
 * This is an internal function that is used to write debug data to one of the
 * HPI reserved registers. This can be used as a logging mechanism for firmware
 * debugging.
 *
 * @param offset Offset of the reserved register to be set.
 * @param value Value to be set into the reserved register.
 *
 * @return None
 */
void hpi_set_reserved_reg (uint8_t offset, uint8_t value);

/**
 * @brief Check whether EC control of VDMs is enabled.
 *
 * @param port USB-PD port to check the configuration for.
 *
 * @return true if EC control is enabled, false otherwise.
 */
bool hpi_is_vdm_ec_ctrl_enabled (uint8_t port);

/**
 * @brief Check whether handling of extended messages by EC is enabled. If not enabled,
 * CCG firmware will automatically respond with NOT_SUPPORTED messages.
 *
 * @param port USB-PD port to check the configuration for.
 *
 * @return true if EC handling of extended messages is enabled, false otherwise.
 */
bool hpi_is_extd_msg_ec_ctrl_enabled (uint8_t port);

/**
 * @brief Get the active EC alternate modes value.
 *
 * @param port USB-PD to check the configuration for.
 *
 * @return The Active EC modes setting programmed by EC.
 */
uint8_t hpid_get_ec_active_modes (uint8_t port);

/**
 * @brief Check if the CCG device can be put into deep-sleep.
 *
 * @return true if deep sleep is possible, false otherwise.
 */
bool hpi_sleep_allowed (void);

/**
 * @brief Prepare the HPI interface for device deep sleep.
 *
 * This function checks whether the I2C interface is idle so that the CCG device can enter
 * deep sleep mode. It also enables an I2C address match as a wake-up trigger from deep sleep.
 * hpi_sleep_allowed should have been called prior to calling this function.
 *
 * @return true if the HPI interface is ready for sleep, false otherwise.
 */
bool hpi_sleep (void);

/**
 * @brief Get the Port Enable register value.
 *
 * @return The Port Enable HPI register value.
 */
uint8_t hpi_get_port_enable (void);

/**
 * @brief Configure HPI to operate in No-boot support mode.
 * @param enable Whether to enable no-boot mode.
 * @return None
 */
void hpi_set_no_boot_mode (bool enable);

/**
 * @brief Set the I2C slave address to be used for the HPI interface.
 * @param slave_addr Slave address to be used.
 * @return None
 */
void hpi_set_fixed_slave_address (uint8_t slave_addr);

/**
 * @brief Send a FW ready notification through HPI to the EC. This event is sent
 * to the EC to indicate that the device is out of reset and has loaded firmware.
 *
 * @return None
 */
void hpi_send_fw_ready_event (void);

/**
 * @brief Set the CCG device flash parameters. These values are used for the
 * device status reporting and firmware update implementation.
 *
 * @param flash_size Total device flash size in bytes.
 * @param row_size Size of each flash row in bytes.
 * @param row_cnt Number of flash rows on the device.
 * @param bl_last_row Last flash row assigned to boot-loader.
 * @return None
 */
void hpi_set_flash_params (uint32_t flash_size, uint16_t row_size, uint16_t row_cnt, uint16_t bl_last_row);

#if (CCG_BOOT == 0)

/**
 * @brief This function initializes the user-defined HPI registers.
 *
 * This function is used to initialize the contents of the user-defined
 * registers that are part of the HPI register space.
 *
 * @param reg_addr The base address of registers to be updated. Should be in
 * the user defined address region.
 * @param size Number of registers to be updated. The upper limit should not
 * exceed the user defined address region.
 * @param data Buffer containing data to be copied into HPI registers.
 *
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t hpi_init_userdef_regs (uint16_t reg_addr, uint8_t size,
        uint8_t *data);

/**
 * @brief Enable handling of user-defined register writes in the Application.
 *
 * This function is used to enable handling of EC writes to the user-defined HPI
 * register region in the application code.
 *
 * @param wr_handler Pointer to function that handles the received HPI writes.
 */
void hpi_set_userdef_write_handler (hpi_write_cb_t wr_handler);

#endif

#if (CCG_HPI_BB_ENABLE != 0)

/**
 * @brief Set BB related register data.
 *
 * @param port_idx Port index.
 * @param bb_reg_addr BB related register address.
 * @param data Pointer to data which writes to BB related register.
 * @return None.
 */
void hpi_bb_reg_update (uint8_t port_idx, uint8_t bb_reg_addr, void *data);

/**
 * @brief Get BB related register data.
 *
 * @param port_idx Port index.
 * @param bb_reg_addr BB related register address.
 * 
 * @return Pointer to data of selected BB related register.
 */
void * hpi_bb_get_reg (uint8_t port_idx, uint8_t bb_reg_addr);

#endif /* (CCG_HPI_BB_ENABLE != 0) */

#endif /* _HPI_H_ */

/* [] END OF FILE */
