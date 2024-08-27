/**
 * @file app.c
 *
 * @brief @{PD application handler source file.@}
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
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <vdm_task_mngr.h>
#include <timer.h>
#include <hpi.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <hal_ccgx.h>
#include <gpio.h>
#if AR_SLAVE_IF_ENABLE
#include <ar_slave.h>
#endif /* AR_SLAVE_IF_ENABLE */

#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if DP_UFP_SUPP
#include <hpd.h>
#endif /* DP_UFP_SUPP */

#if ((CCG_HPI_ENABLE == 0) && (VBUS_OCP_RETRY_ENABLE == 1))
uint8_t gl_vbus_ocp_retry_count[NO_OF_TYPEC_PORTS] = {0};

static void ocp_off_tmr_cbk(uint8_t port, timer_id_t id)
{
    (void) id;
    dpm_start(port);
}
#endif /* (CCG_HPI_ENABLE == 0) && (VBUS_OCP_RETRY_ENABLE == 1) */

app_status_t app_status[NO_OF_TYPEC_PORTS];

bool app_is_cbl_disc_done(uint8_t port)
{
    /* Assume cable discovery finished when device is UFP. */
    bool retval = true;

#if DFP_ALT_MODE_SUPP
	
    const dpm_status_t *dpm_stat = dpm_get_info (port);

    /* This check only makes sense for DFP. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        /*
         * Set the cable discovered flag if:
         * 1. Cable discovery is disabled.
         * 2. EMCA present flag in DPM is set.
         */
        if ((dpm_stat->cbl_dsc == false) || (dpm_stat->emca_present != false))
        {
            app_get_status(port)->cbl_disc_id_finished = true;
        }

        /* Return the status of Cable discovered flag. */
        retval = app_get_status(port)->cbl_disc_id_finished;
    }

#endif /* DFP_ALT_MODE_SUPP */

    return retval;
}

uint8_t app_task(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* If vdm processing allowed */
    if (app_get_status(port)->vdm_task_en != false)
    {
        /* Wait for cable discovery completion before going on Alt. Modes. */
        if (app_is_cbl_disc_done (port))
        {
            vdm_task_mngr (port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */	

#if (CCG_BB_ENABLE != 0)
    if (bb_is_present(port) != false)
    {
        bb_task(port);
    }
#endif /* (CCG_BB_ENABLE != 0) */

    return true;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_BB_ENABLE
        if (!bb_enter_deep_sleep(port))
        {
            stat = false;
            break;
        }
#endif /* CCG_BB_ENABLE */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        if (!is_vdm_task_idle(port))
        {
            stat = false;
            break;
        }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if DP_UFP_SUPP
        /* CDT 245126 workaround: Check if HPD RX Activity timer is running.
         * If yes, don't enter deep sleep. */
        if (!is_hpd_rx_state_idle (port))
        {
            stat = false;
            break;
        }
#endif /* DP_UFP_SUPP */
    }

#if AR_SLAVE_IF_ENABLE
    /* Make sure the AR slave interface is not busy. */
    if (stat)
    {
        stat = ar_slave_sleep();
    }
#endif /* AR_SLAVE_IF_ENABLE */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    if (stat)
    {
        for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
            /* Prepare for deep-sleep entry. */
            alt_mode_mngr_sleep(port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return stat;
}

void app_wakeup(void)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        alt_mode_mngr_wakeup (port);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

#if (CCG_BB_ENABLE != 0)
/* Alternate mode entry timeout callback function. */
static void ame_tmr_cbk(uint8_t port, timer_id_t id)
{
    (void)id;

    /* TODO: Should the alternate mode status be reset. */
    bb_enable(port, BB_CAUSE_AME_TIMEOUT);
}
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_PD_REV3_ENABLE
static void app_extd_msg_handler(uint8_t port, pd_packet_extd_t *pd_pkt_p)
{
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.data_size >
               ((pd_pkt_p->hdr.hdr.chunk_no + 1) * MAX_EXTD_MSG_LEGACY_LEN)))
    {
        dpm_pd_cmd_buf_t extd_dpm_buf;
        uint32_t extd_dummy_data;

        extd_dpm_buf.cmd_sop = pd_pkt_p->sop;
        extd_dpm_buf.extd_type = pd_pkt_p->msg;
        extd_dpm_buf.extd_hdr.val = 0;
        extd_dpm_buf.extd_hdr.extd.chunked = true;
        extd_dpm_buf.extd_hdr.extd.request = true;
        extd_dpm_buf.extd_hdr.extd.chunk_no = pd_pkt_p->hdr.hdr.chunk_no + 1;
        extd_dpm_buf.dat_ptr = (uint8_t*)&extd_dummy_data;
        extd_dpm_buf.timeout = 0;

        /* Send next chunk request */
        dpm_pd_command_ec(port, DPM_CMD_SEND_EXTENDED,
                &extd_dpm_buf, NULL);
    }
    else
    {
        /* Send Not supported message */
        dpm_pd_command_ec(port, DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
    }
}
#endif /* CCG_PD_REV3_ENABLE */

void app_event_handler(uint8_t port, app_evt_t evt, const void* dat)
{
    const app_req_status_t* result;
    const pd_contract_info_t* contract_status;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
#ifdef CCG3
            /* Sink FET need to be enabled in dead battery to power CCG3 board. */
            if(dpm_stat->dead_bat == true)
            {
                psnk_enable(port);
            }
#endif /* CCG3 */
            /* Initialize the MUX to its default settings (isolate). */
            mux_ctrl_init (port);
            app_get_status(port)->vdm_prcs_failed = false;
            break;

        case APP_EVT_CONNECT:
            set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            app_get_status(port)->vdm_prcs_failed = false;

#if (CCG_BB_ENABLE != 0)
            /* Enable the AME timer on attach if in sink mode. */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                timer_start(port, APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
            }
#endif /* (CCG_BB_ENABLE != 0) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((dpm_stat->pd_connected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            vdm_task_mngr_deinit (port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            
            /*
             * Re-enable MUX in USB mode if hard reset has been completed.
             */
            if (hardreset_cplt)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    set_mux (port, MUX_CONFIG_ISOLATE, 0);
                }
            }

#if ((CCG_HPI_ENABLE == 0) && (VBUS_OCP_RETRY_ENABLE == 1))
            gl_vbus_ocp_retry_count[port] = 0;
#endif
            break;

        case APP_EVT_EMCA_DETECTED:
        case APP_EVT_EMCA_NOT_DETECTED:
            app_get_status(port)->cbl_disc_id_finished = true;
            app_get_status(port)->vdm_prcs_failed = false;
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const app_req_status_t*)dat ;
            if(*result == REQ_ACCEPT)
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                vdm_task_mngr_deinit (port);

                /* Device data role changed. Enable the VDM task manager for alt. mode support. */
                enable_vdm_task_mngr(port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */    				

#if (CCG_BB_ENABLE != 0)
                /* Start tAME Timer to enable BB functionality */
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    timer_start(port, APP_AME_TIMEOUT_TIMER , APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
                }
#endif /* (CCG_BB_ENABLE != 0) */
            }
            break;

        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_status[port].vdm_retry_pending)
                skip_soln_cb = true;
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:

            /* Set VDM version based on active PD revision. */
#if CCG_PD_REV3_ENABLE
            if (dpm_stat->spec_rev_sop_live >= PD_REV3)
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV3;
            }
            else
#endif /* CCG_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV2;
            }

            contract_status = (pd_contract_info_t*)dat;
            if((contract_status->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == PD_CONTRACT_CAP_MISMATCH_DETECTED))
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /*
                 * Contract established.  Enable VDM task manager for Alt. Mode support.
                 * This function will have no effect if the Alt. Modes are already running.
                 */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_UFP) ||
                        (app_get_status(port)->vdm_prcs_failed == false)
                   )
                {
                    enable_vdm_task_mngr(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            }

            break;

        case APP_EVT_VBUS_OCP_FAULT:
#if ((CCG_HPI_ENABLE == 0) && (VBUS_OCP_RETRY_ENABLE == 1))
            if (gl_vbus_ocp_retry_count[port] < (get_pd_port_config(port)->ocp_retry_cnt))
            {
                /* Start the VBUS OCP Off timer. VBUS OCP off time is in 10ms units. */
                timer_start(port, APP_VBUS_OCP_OFF_TIMER, 10 * (get_pd_port_config(port)->ocp_offtime),
                        ocp_off_tmr_cbk);
                gl_vbus_ocp_retry_count[port]++;
            }
#endif /* (CCG_HPI_ENABLE == 0) && (VBUS_OCP_RETRY_ENABLE == 1) */
            break;

#if CCG_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
#if (CCG_HPI_PD_ENABLE)
            /* Handle the extended message locally if forwarding to EC is not enabled. */
            if (hpi_is_extd_msg_ec_ctrl_enabled (port) == false)
#endif
            {
                app_extd_msg_handler(port, (pd_packet_extd_t *)dat);
            }
            skip_soln_cb  = true;
            break;
#endif /* CCG_PD_REV3_ENABLE */

#if 0
        /* Default handlers are sufficient. */
        case APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS:
        case APP_EVT_RP_CHANGE:
        case APP_EVT_PKT_RCVD:
        case APP_EVT_PR_SWAP_COMPLETE:
        case APP_EVT_VCONN_SWAP_COMPLETE:
        case APP_EVT_SENDER_RESPONSE_TIMEOUT:
        case APP_EVT_SOFT_RESET_SENT:
        case APP_EVT_CBL_RESET_SENT:
#endif
        default:
            /* Nothing to do. */
            break;
    }

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
        sln_pd_event_handler(port, evt, dat);
    }
}

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].app_resp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

void app_init(void)
{
    uint8_t port;

    /* For now, only the VDM handlers require an init call. */
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        vdm_data_init(port);

#if (CCG_BB_ENABLE != 0)
        /*
         * Initialize the billboard interface. The billboard
         * interface shall not get initialized if it is not
         * enabled in configuration table.
         */
        bb_init(port);
#endif /* (CCG_BB_ENABLE != 0) */
    }
}

/* Implements CCG deep sleep functionality for power saving. */
bool system_sleep(void)
{
    uint8_t intr_state;
    bool dpm_slept = false;
    bool app_slept = false;
    bool retval = false;

    intr_state = CyEnterCriticalSection();

    /*
     * We have to check the application layer, HPI and the Device Policy
     * Manager (DPM) to see if all of these modules are ready for sleep.
     * CCG can only enter deep sleep if all of these blocks are in an idle
     * state.
     *
     * Note: The respective sleep functions might be performing some
     * state updates as part of the idle check function; and therefore
     * the corresponding wakeup function needs to be called if they have
     * returned true to indicate that sleep is allowed.
     */
    if (app_sleep())
    {
        app_slept = true;

        if (
#if CCG_HPI_ENABLE
                (hpi_sleep_allowed()) &&
#endif /* CCG_HPI_ENABLE */
                (dpm_deepsleep())
           )
        {
            dpm_slept = true;
            timer_enter_sleep();

#if CCG_HPI_ENABLE
            /*
             * CDT 224642: The I2C IDLE check needs to be done as the last step
             * before device enters into sleep. Otherwise, the device may fail
             * to wake up when there is an address match on the I2C interface.
             *
             * Note: The hpi_sleep_allowed() function must have been called
             * and have returned true prior to this call.
             */
            if (hpi_sleep())
#endif /* CCG_HPI_ENABLE */
            {
                /* Device sleep entry. */
                retval = true;
                CySysPmDeepSleep();
            }
        }
    }

    CyExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if(dpm_slept)
    {
        dpm_wakeup();
    }

    /* Call app_wakeup() if app_sleep() had returned true. */
    if(app_slept)
    {
        app_wakeup();
    }

    return retval;
}

void vconn_enable(uint8_t port, uint8_t channel)
{
#if VCONN_OCP_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P1,APP_VCONN_MON_AMUX_INPUT_P1);
#if CCG_PD_DUALPORT_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P2,APP_VCONN_MON_AMUX_INPUT_P2);
#endif /* CCG_PD_DUALPORT_ENABLE */
    /*
     * 120us delay required as a settiling time after HSIOM config to get a stable
     * ADC reading.
     */
    CyDelayUs(120);
    system_vconn_ocp_en(port, app_psrc_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

    pd_vconn_enable(port, channel);
    /* Reset RX Protocol for cable */
    dpm_prot_reset_rx(port, SOP_PRIME);
    dpm_prot_reset_rx(port, SOP_DPRIME);
}

void vconn_disable(uint8_t port, uint8_t channel)
{
    pd_vconn_disable(port, channel);
}

bool vconn_is_present(uint8_t port)
{
    return pd_is_vconn_present( port, dpm_get_info(port)->rev_pol);
}

bool vbus_is_present(uint8_t port, uint16_t volt, int8 per)
{
    uint8_t level;
    uint8_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling.
     * To avoid false output on OVP Trip pin when VBUS is polled
     * OVP trip pin is disconnected from OVP comp output and last
     * value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4*/

    /*
     * Re-run caliberation every time to ensure that VDDD or the measurement
     * does not break.
     */
    pd_adc_calibrate (port, APP_VBUS_POLL_ADC_ID);
    level = pd_get_vbus_adc_level(port, APP_VBUS_POLL_ADC_ID, volt, per);
    retVal = pd_adc_comparator_sample (port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT,
            level);
#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/
    return retVal;
}

#if VCONN_OCP_ENABLE
uint8_t system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk)
{
    if(cbk == NULL)
    {
        return false;
    }
    /* Enable VBUS OCP protection */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, APP_VCONN_OCP_ADC_INPUT,
            APP_VCONN_TRIGGER_LEVEL, PD_ADC_INT_RISING, cbk);
    return true;
}

uint8_t system_vconn_ocp_dis(uint8_t port)
{
    /* Enable VBUS OCP protection */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, 0, 0, 0, NULL);
    return true;
}

#endif /* VCONN_OCP_ENABLE */

#if VBUS_OVP_ENABLE
/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(uint8_t port, uint16_t volt_50mV, bool pfet, PD_ADC_CB_T ovp_cb)
{
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
    uint8_t level;
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */
    uint8_t intr_state;

    if (get_pd_port_config(port)->protect_en & CFG_TABLE_OVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection();

#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        /* Set OVP threshold. */
        level = pd_get_vbus_adc_level(port, APP_OVP_ADC_ID, volt_50mV, get_pd_port_config(port)->ovp_threshold);
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, APP_OVP_ADC_INPUT, level, PD_ADC_INT_FALLING, ovp_cb);
#else
#ifdef CCG3
        pd_internal_vbus_ovp_en(volt_50mV, get_pd_port_config(port)->ovp_threshold, ovp_cb,
                pfet, VBUS_OVP_MODE, VBUS_OVP_FILT_SEL);
#endif /* CCG3 */
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

        CyExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(uint8_t port, bool pfet)
{
    if (get_pd_port_config(port)->protect_en & CFG_TABLE_OVP_EN_MASK)
    {
        /* Disable OVP. */
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, 0, 0, 0, NULL);
#else
#ifdef CCG3
        pd_internal_vbus_ovp_dis(pfet);
#endif /* CCG3 */
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */
    }
}

#endif /* VBUS_OVP_ENABLE */

/* End of File */

