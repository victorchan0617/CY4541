/**
 * @file alt_mode_hw.c
 *
 * @brief @{Hardware control for Alternate mode implementation.@}
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
#include <hpi.h>
#include <alp_ridge.h>
#include <dpm.h>
#include <alt_mode_hw.h>
#include <app.h>
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
#include <gpio.h>
#include <hpd.h>
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

/* Holds hw solution event/command data */
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static uint32_t hw_sln_data[NO_OF_TYPEC_PORTS];
/* Holds command callback information. */
static alt_mode_hw_cmd_cbk_t gl_hw_cmd_cbk[NO_OF_TYPEC_PORTS];
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

/* Holds current HPD command status. */
static volatile bool gl_alt_mode_cmd_pending[NO_OF_TYPEC_PORTS];
/* Holds current HPD pin status */
#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static volatile hpd_event_type_t gl_alt_mode_hpd_state[NO_OF_TYPEC_PORTS];
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

#if DP_DFP_SUPP
/* DFP HPD callback */
static void
dp_src_hpd_cbk(uint8_t port, hpd_event_type_t event);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* UFP HPD callback */
static void dp_snk_hpd_cbk(uint8_t port, hpd_event_type_t event);
#endif /* DP_UFP_SUPP */

/************************** Function definitions *****************************/

bool eval_app_alt_hw_cmd(uint8_t port, uint8_t *cmd_param)
{
    uint8_t             hw_type, data_role;
    alt_mode_hw_evt_t     cmd_info;

    /* Convert received cmd bytes as info and data */
    cmd_info  = (alt_mode_hw_evt_t)MAKE_DWORD(cmd_param[3], cmd_param[2], cmd_param[1], cmd_param[0]);
    hw_type   = cmd_info.hw_evt.hw_type;
    data_role = cmd_info.hw_evt.data_role;

    if (data_role == gl_dpm_port_type[port])
    {
        switch (hw_type)
        {
            case ALT_MODE_MUX:
                return eval_mux_cmd(port, cmd_info.hw_evt.evt_data);

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
            case ALT_MODE_HPD:
                return eval_hpd_cmd(port, cmd_info.hw_evt.evt_data);
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */

            default:
                break;
        }
    }

    return false;
}

bool eval_mux_cmd(uint8_t port, uint32_t cmd)
{
    if ((cmd >= MUX_CONFIG_ISOLATE)&&(cmd <= MUX_CONFIG_DP_4_LANE))
    {
        return set_mux(port, (mux_select_t)cmd, NO_DATA);
    }

    return false;
}

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
void alt_mode_hw_set_cbk(uint8_t port, alt_mode_hw_cmd_cbk_t cbk)
{
    /* Register the callback if the port is valid. */
    if (port < NO_OF_TYPEC_PORTS)
    {
        gl_hw_cmd_cbk[port] = cbk;
    }
}

bool eval_hpd_cmd(uint8_t  port, uint32_t cmd)
{
    if (cmd == HPD_DISABLE_CMD)
    {
        gl_alt_mode_cmd_pending[port] = false;
        hpd_deinit(port);
        return true;
    }

#if DP_DFP_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        if (cmd == HPD_ENABLE_CMD)
        {
            gl_alt_mode_cmd_pending[port] = false;
            hpd_transmit_init(port, dp_src_hpd_cbk);
            return true;
        }
        else
        {
            if (cmd < HPD_DISABLE_CMD)
            {
                gl_alt_mode_cmd_pending[port] = true;
                gl_alt_mode_hpd_state[port]   = (hpd_event_type_t)cmd;
                hpd_transmit_sendevt(port, (hpd_event_type_t)cmd , false);
                return true;
            }
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        if (cmd == HPD_ENABLE_CMD)
        {
            gl_alt_mode_hpd_state[port] = HPD_EVENT_UNPLUG;
            hpd_receive_init(port, dp_snk_hpd_cbk);
            return true;
        }
    }
#endif /* DP_UFP_SUPP */

    return false;
}

#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */


#if DP_UFP_SUPP
static void dp_snk_hpd_cbk(uint8_t port, hpd_event_type_t event)
{
    alt_mode_hw_evt_t alt_mode_hw_data;

    if ((event > HPD_ENABLE_CMD) && (event < HPD_DISABLE_CMD))
    {
        alt_mode_hw_data.hw_evt.data_role = PRT_TYPE_UFP;
        alt_mode_hw_data.hw_evt.hw_type   = ALT_MODE_HPD;

        /* Save first 4 bytes of event */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        hw_sln_data[port] = (uint32_t)alt_mode_hw_data.val;
        
        /* Store current HPD event. */
        gl_alt_mode_hpd_state[port] = event;

        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (port, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        sln_pd_event_handler (port, APP_EVT_APP_HW, &(hw_sln_data[port]));
    }
}

bool dp_snk_get_hpd_state(uint8_t port)
{
    /*
     * Return HPD state based on last HPD event from HAL.
     * If last event was UNPLUG, HPD is not connected. If it was
     * PLUG or IRQ, HPD is connected.
     */
    if (gl_alt_mode_hpd_state[port] == HPD_EVENT_UNPLUG)
    {
        return false;
    }
    else
    {
        return true;
    }
}
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
static void dp_src_hpd_cbk(uint8_t port, hpd_event_type_t event)
{
    if (event == HPD_COMMAND_DONE)
    {
        alt_mode_hw_evt_t alt_mode_hw_data;

        /* ALT. MODE command completed. */
        gl_alt_mode_cmd_pending[port] = false;

        /* Set data role and HW type */
        alt_mode_hw_data.hw_evt.data_role = PRT_TYPE_DFP;
        alt_mode_hw_data.hw_evt.hw_type   = ALT_MODE_HPD;

        /* If HPD command done */
        alt_mode_hw_data.hw_evt.evt_data = (uint32_t)event;
        hw_sln_data[port] = (uint32_t)alt_mode_hw_data.val;

        /* Call the event callback, if it exists. */
        if (gl_hw_cmd_cbk[port] != NULL)
        {
            gl_hw_cmd_cbk[port] (port, alt_mode_hw_data.val);
        }

        /* Send notification to the solution. */
        sln_pd_event_handler(port, APP_EVT_APP_HW, &(hw_sln_data[port]));
    }
}
#endif /* DP_DFP_SUPP */

void alt_mode_hw_deinit(uint8_t port)
{
    /* If we still have a device connected, switch MUX to USB mode. */
    if (dpm_get_info(port)->attach)
    {
        set_mux(port, MUX_CONFIG_SS_ONLY, NO_DATA);
    }

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
    /* De-init the HPD block. */
    hpd_deinit (port);

    /* Clear state variables. */
    gl_alt_mode_cmd_pending[port] = false;
    gl_alt_mode_hpd_state[port]   = HPD_EVENT_NONE;
    gl_hw_cmd_cbk[port]           = NULL;
#endif /* (DP_DFP_SUPP) || (DP_UFP_SUPP) */
}

bool set_mux(uint8_t port, mux_select_t cfg, uint32_t custom_data)
{
    bool retval = true;

    (void)custom_data;
#if (MUX_TYPE == DP_MUX)
    if (cfg <= MUX_CONFIG_DP_4_LANE)
    {
        retval = mux_ctrl_set_cfg (port, cfg,  dpm_get_polarity (port));
    }
    else
    {
        retval = false;
    }
#elif (MUX_TYPE == AR_MUX)
    /* In TBT use cases, this call is used to configure the SBU Mux.
     * This has to be configured before notifying Alpine Ridge. */
    if (cfg <= MUX_CONFIG_AR_CUSTOM)
    {
        retval = mux_ctrl_set_cfg (port, cfg,  dpm_get_polarity (port));
    }
    /* Update the Alpine-Ridge status register. */
    if (retval)
    {
        retval = ar_set_mux (port, cfg, dpm_get_polarity(port), custom_data);
    }
#endif

    return retval;
}

bool alt_mode_hw_is_idle(uint8_t port)
{
    return (!gl_alt_mode_cmd_pending[port]);
}

void alt_mode_hw_sleep(uint8_t port)
{
#if DP_DFP_SUPP
    /* We can use the presence of a callback as indication that the HPD block is active. */
    if (gl_hw_cmd_cbk[port] != NULL)
    {
        /* Set the value of the HPD GPIO based on the last event. */
        if (port == 0)
        {
            gpio_set_value (HPD_P0_PORT_PIN, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
        }
        else
        {
            gpio_set_value (HPD_P1_PORT_PIN, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
        }

        /* Move the HPD pin from HPD IO mode to GPIO mode. */
        hpd_sleep_entry (port);
    }
#endif

#if DP_UFP_SUPP
    /* CDT 245126 workaround: Prepare to enter deep sleep. */
    hpd_rx_sleep_entry (port, dp_snk_get_hpd_state(port));
#endif /* DP_UFP_SUPP */
}

void alt_mode_hw_wakeup(uint8_t port)
{
#if DP_DFP_SUPP
    /* We can use the presence of a callback as indication that the HPD block is active. */
    if (gl_hw_cmd_cbk[port] != NULL)
    {
        /* Move the HPD pin back to HPD IO mode. */
        hpd_wakeup (port, (gl_alt_mode_hpd_state[port] > HPD_EVENT_UNPLUG));
    }
#endif

#if DP_UFP_SUPP
    /* CDT 245126 workaround: Wakeup and revert HPD RX configurations. */
    hpd_rx_wakeup (port);
#endif /* DP_UFP_SUPP */
}

/* [] END OF FILE */
