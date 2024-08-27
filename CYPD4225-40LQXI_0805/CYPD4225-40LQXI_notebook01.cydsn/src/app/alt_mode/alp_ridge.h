/**
 * @file alp_ridge.h
 *
 * @brief @{Alpine Ridge control interface header file.@}
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

#ifndef _ALP_RIDGE_H_
#define _ALP_RIDGE_H_

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include <alt_mode_hw.h>

/*****************************************************************************
 * Data Struct Definition
 *****************************************************************************/

/**
  @union ar_reg_t
  @brief Union to hold AR Registers
 */
typedef union
{
    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing USB-PD controller status to be reported to Alpine Ridge. */
    struct USBPD_STATUS_REG
    {
        uint32_t data_conn_pres  : 1;   /**< B0: Whether data connection is present. */
        uint32_t conn_orien      : 1;   /**< B1: CC polarity. */
        uint32_t active_cbl      : 1;   /**< B2: Active cable. From B22 of Cable MODE Response. */
        uint32_t ovc_indn        : 1;   /**< B3: Over-Current indication. */
        uint32_t usb2_conn       : 1;   /**< B4: USB 2.0 connection. Set when no ALT MODES are active. */
        uint32_t usb3_conn       : 1;   /**< B5: USB 3.1 connection. Set when no ALT MODES are active. */
        uint32_t usb3_speed      : 1;   /**< B6: USB Gen2/Gen1 speed. From B18-B16 of Cable MODE Response. */
        uint32_t usb_dr          : 1;   /**< B7: Data role. DFP=0, UFP=1. */
        uint32_t dp_conn         : 1;   /**< B8: DP connection status. */
        uint32_t dp_role         : 1;   /**< B9: DP direction. Source=0, Sink=1. */
        uint32_t dp_pin_assign   : 2;   /**< B[11-10]: DP pin assignment. 4-lane='b00 2-lane='b01 */
        uint32_t rsvd1           : 4;   /**< B[15-12]: Reserved. */
        uint32_t tbt_conn        : 1;   /**< B16: TBT connection status. */
        uint32_t tbt_type        : 1;   /**< B17: TBT type. From B16 of UFP MODE Response. */
        uint32_t cbl_type        : 1;   /**< B18: TBT cable type. From B21 of Cable MODE Response. */
        uint32_t rsvd2           : 1;   /**< B19: Reserved. */
        uint32_t act_link_train  : 1;   /**< B20: Active TBT link training. From B23 of Cable MODE Response. */
        uint32_t rsvd3           : 2;   /**< B[22-21]: Reserved. */
        uint32_t force_lsx       : 1;   /**< B23: Set to zero. */
        uint32_t rsvd4           : 1;   /**< B24: Reserved. */
        uint32_t tbt_cbl_spd     : 3;   /**< B[27-25]: Cable speed. From B18-B16 of Cable MODE Response. */
        uint32_t tbt_cbl_gen     : 2;   /**< B[29-28]: Cable generation. From B20-19 of Cable MODE Response. */
        uint32_t rsvd5           : 2;   /**< B[31-30]: Reserved. */

    }ar_status;                         /**< PD-controller status. */

    /** @brief Struct containing USB-PD controller command register fields for Alpine Ridge. */
    struct USBPD_CMD_REG
    {
        uint32_t host_conn       : 1;   /**< B0: No host connected. */
        uint32_t soft_rst        : 1;   /**< B1: Issue USB-PD Controller soft reset. */
        uint32_t intrpt_ack      : 1;   /**< B2: Alpine Ridge acknowledge for the interrupt. */
        uint32_t rsvd            : 29;  /**< B[31-3]: No host connected. */

    }usbpd_cmd_reg;                     /**< PD-controller command register. */

}ar_reg_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function set AR registers in accordance to input parameters
 *
 * @param port Port index the AR settings are performed for.
 * @param cfg MUX configuration.
 * @param polarity Attached target Type-C Polarity.
 * @param ar_cfg Contains AR register settings in case of TBT alt mode is active.
 *
 * @return true if AR was set successful, in the other case - false
 */
bool ar_set_mux(uint8_t port, mux_select_t cfg, uint8_t polarity, uint32_t ar_cfg);

#endif /*_ALP_RIDGE_H_ */

/* [] END OF FILE */
