/**
 * @file alp_ridge.c
 *
 * @brief @{Alpine Ridge control interface source file.@}
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

#include <config.h>
#include <pd.h>
#include <alp_ridge.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <dpm.h>
#include <ar_slave.h>

typedef struct
{
    ar_reg_t ar_stat;
    ar_reg_t cmd_reg;
    uint8_t polarity;

}ar_t;

ar_t ar[NO_OF_TYPEC_PORTS];

/* Sets AR lines as isolate */
static bool ar_set_isolate(uint8_t port);
/* Sets AR lines as USB 2.0 */
static bool ar_set_2_0(uint8_t port);
/* Sets AR lines as USB SS */
static bool ar_set_usb_ss(uint8_t port);
/* Sets AR lines as DP 2 lanes */
static bool ar_set_dp_2_lane(uint8_t port);
/* Sets AR lines as DP 4 lanes */
static bool ar_set_dp_4_lane(uint8_t port);
/* Sets AR as custom TBT configuration */
static bool ar_set_custom(uint8_t port,uint32_t ar_cfg);

/************************** Function definitions *****************************/

static bool ar_set_isolate(uint8_t port)
{
    ar_reg_t*     reg = &ar[port].ar_stat;

    /* No connection active. */
    reg->val = NO_DATA;

    return ar_set_custom(port, reg->val);
}

static bool ar_set_2_0(uint8_t port)
{
    ar_reg_t*     reg = &ar[port].ar_stat;

    /* Set Data Connection bit and USB 2.0 bit. */
    reg->val = NO_DATA;
    reg->ar_status.data_conn_pres = true;
    reg->ar_status.usb2_conn = true;

    return ar_set_custom(port, reg->val);
}

static bool ar_set_usb_ss(uint8_t port)
{
    ar_reg_t *reg = &ar[port].ar_stat;
    pd_do_t   cbl_vdo;

    /* We don't support USB 3.0 connection as UFP. */
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        return false;
    }

    reg->val = NO_DATA;

    /* Check if no USB 2.0 cable */
    cbl_vdo.val = dpm_get_info(port)->cbl_vdo.val;
    if ((cbl_vdo.val != NO_DATA) && (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_2_0_SUPP))
    {
        return ar_set_2_0(port);
    }

    /* Data connection present */
    reg->ar_status.data_conn_pres = true;

    /* Enable USB 2.0 and 3.1 connections as well. */
    reg->ar_status.usb2_conn = true;
    reg->ar_status.usb3_conn = true;

    /* Set GEN2 speed only if the cable supports it. */
    if ((cbl_vdo.val == NO_DATA) || (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_GEN_2_SUPP))
    {
        reg->ar_status.usb3_speed = true;
    }

    /* Set AR */
    return ar_set_custom(port, reg->val);
}

static bool ar_set_dp_2_lane(uint8_t port)
{
    ar_reg_t *reg = &ar[port].ar_stat;
    pd_do_t   cbl_vdo;

    /* We don't support DisplayPort as UFP. */
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        return false;
    }

    reg->val = NO_DATA;
    reg->ar_status.data_conn_pres = true;

    /* Set DP parameters. */
    reg->ar_status.dp_conn = true;
    reg->ar_status.dp_role = false;         /* Only DP source supported for now. */
    reg->ar_status.dp_pin_assign = 0x01;    /* 2-lane connection. */

    /* Enable USB 2.0 and 3.1 connections as well. */
    reg->ar_status.usb2_conn = true;
    reg->ar_status.usb3_conn = true;

    /* Set GEN2 speed only if the cable supports it. */
    cbl_vdo.val = dpm_get_info(port)->cbl_vdo.val;
    if ((cbl_vdo.val == NO_DATA) || (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_GEN_2_SUPP))
    {
        reg->ar_status.usb3_speed = true;
    }

    /* Set AR */
    return ar_set_custom(port, reg->val);
}

static bool ar_set_dp_4_lane(uint8_t port)
{
    ar_reg_t*     reg = &ar[port].ar_stat;

    /* We don't support DisplayPort as UFP. */
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        return false;
    }

    reg->val = NO_DATA;
    reg->ar_status.data_conn_pres = true;

    /* Set DP parameters. */
    reg->ar_status.dp_conn = true;
    reg->ar_status.dp_role = false;         /* Only DP source supported for now. */
    reg->ar_status.dp_pin_assign = 0x00;    /* 4-lane connection. */

    /* Enable USB 2.0 connection as well. */
    reg->ar_status.usb2_conn = true;

    /* Set AR */
    return ar_set_custom(port, reg->val);
}

static bool ar_set_custom(uint8_t port, uint32_t ar_cfg)
{
    ar_reg_t*     reg = &ar[port].ar_stat;

    /* Copy ar config to AR struct */
    reg->val = ar_cfg;
    /* Update polarity field. */
    reg->ar_status.conn_orien = ar[port].polarity;

    /* Update data role field. */
    reg->ar_status.usb_dr = false;
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        reg->ar_status.usb_dr = true;
    }

#if AR_SLAVE_IF_ENABLE

    /* Update the status register content and raise an interrupt to the Alpine Ridge. */
    ar_slave_status_update (port, reg->val);

#endif /* AR_SLAVE_IF_ENABLE */

    return true;
}

bool ar_set_mux(uint8_t      port, mux_select_t cfg, uint8_t polarity, uint32_t ar_cfg)
{
    ar[port].polarity = polarity;
    switch (cfg)
    {
        case MUX_CONFIG_ISOLATE:
            return ar_set_isolate(port);
        case MUX_CONFIG_2_0:
            return ar_set_2_0(port);
        case MUX_CONFIG_SS_ONLY:
            return ar_set_usb_ss(port);
        case MUX_CONFIG_DP_2_LANE:
            return ar_set_dp_2_lane(port);
        case MUX_CONFIG_DP_4_LANE:
            return ar_set_dp_4_lane(port);
        case MUX_CONFIG_AR_CUSTOM:
            return ar_set_custom(port, ar_cfg);
        default:
            break;
    }

    return false;
}

/* [] END OF FILE */
