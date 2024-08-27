/**
 * @file psink.c
 *
 * @brief @{Power Sink (Consumer) manager source file.@}
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <psink.h>
#include <app.h>
#include <timer.h>

void app_psnk_vbus_ovp_cbk(uint8_t port, bool comp_out);

void sink_fet_off(uint8_t port)
{
    
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SNK_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}


void sink_fet_on(uint8_t port)
{
    
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SNK_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

#if VBUS_OVP_ENABLE

void snk_vbus_ovp(uint8_t port)
{
    /*OVP fault*/
    /*Quickly turning off fets here, although dpm_stop will also do that*/
    sink_fet_off(port);
    /*Enqueue HPI OVP fault event*/
    app_event_handler(port, APP_EVT_VBUS_OVP_FAULT, NULL);
    dpm_stop(port);
}

void app_snk_db_fet_off_delay_expire(uint8_t port, timer_id_t id)
{
    snk_vbus_ovp(port);
}

void app_psnk_vbus_ovp_cbk(uint8_t port, bool comp_out)
{
    if(dpm_get_info(port)->dead_bat == false)
    {
        snk_vbus_ovp(port);
    }
    else
    {
        /* If dead battery case then delaying cfet off so that terminations
         * removed can be seen by peer otherwise if fet is off immdiately then
         * chip loose power and in dead battery state Rd is still seen by peer
         * so peer will never see Rd removed */
        /* Remove rd terminations */
        dpm_typec_deassert_rp_rd(port, CC_CHANNEL_1);
        dpm_typec_deassert_rp_rd(port, CC_CHANNEL_2);
        /* Start DB sink fet disable timer*/
        timer_start(port, APP_DB_SNK_FET_DIS_DELAY_TIMER, APP_DB_SNK_FET_DIS_DELAY_TIMER_PERIOD,
                app_snk_db_fet_off_delay_expire);
    }
}

#endif /* VBUS_OVP_ENABLE */

void psnk_set_voltage (uint8_t port, uint16_t volt_50mV)
{
#if VBUS_OVP_ENABLE
#ifdef CCG3
    app_ovp_enable(port, volt_50mV, CCG3_SNK_FET, app_psnk_vbus_ovp_cbk);
#else
    app_ovp_enable(port, volt_50mV, false, app_psnk_vbus_ovp_cbk);
#endif
#endif /* VBUS_OVP_ENABLE */
}

void psnk_set_current (uint8_t port, uint16_t cur_10mA)
{
    /*
     * There are no implementation to update the current settings as of today.
     * This implementation needs to be updated when the CCGx solution has
     * capability to control the sink current capability.
     */
}

void psnk_enable (uint8_t port)
{
    uint8_t intr_state;
    intr_state = CyEnterCriticalSection();

    if (dpm_get_info(port)->dpm_enabled)
    {
        sink_fet_on(port);
    }
    CyExitCriticalSection(intr_state);
}

/*Timer Callback*/
static void app_psnk_tmr_cbk(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSINK_DIS_TIMER:
            timer_stop(port, APP_PSINK_DIS_MONITOR_TIMER);
            vbus_discharge_off(port);
            break;

        case APP_PSINK_DIS_MONITOR_TIMER:
            if(vbus_is_present(port, VSAFE_5V, 0) == false)
            {
                timer_stop(port, APP_PSINK_DIS_TIMER);
                vbus_discharge_off(port);
                app_stat->snk_dis_cbk(port);
            }
            else
            {
                /*Start Monitor Timer again*/
                timer_start(port, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
            }
            break;
        default:
            break;
    }
}


void psnk_disable (uint8_t port, sink_discharge_off_cbk_t snk_discharge_off_handler)
{
    uint8_t intr_state;
    app_status_t* app_stat = app_get_status(port);

    intr_state = CyEnterCriticalSection();

#if VBUS_OVP_ENABLE
#ifdef CCG3
    app_ovp_disable (port, CCG3_SNK_FET);
#else
    app_ovp_disable (port, false);
#endif /* CCG3 */
#endif /* VBUS_OVP_ENABLE */

    sink_fet_off(port);
    vbus_discharge_off(port);
    timer_stop_range(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_MONITOR_TIMER);

    if ((snk_discharge_off_handler != NULL) && (dpm_get_info(port)->dpm_enabled))
    {
        /*Turn On discharge*/
        vbus_discharge_on(port);
        app_stat->snk_dis_cbk = snk_discharge_off_handler;
        /*Start Power source enable and monitor timer*/
        timer_start(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_TIMER_PERIOD, app_psnk_tmr_cbk );
        timer_start(port, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
    }
    CyExitCriticalSection(intr_state);
}

/* End of File */

