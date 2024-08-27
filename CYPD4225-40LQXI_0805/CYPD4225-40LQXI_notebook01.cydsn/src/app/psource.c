/**
 * @file psource.c
 *
 * @brief @{Power source (Provider) manager source file.@}
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
#include <psource.h>
#include <app.h>
#include <timer.h>
#include <psink.h>
#include <hal_ccgx.h>
#include <pdss_hal.h>
#include <gpio.h>

/* Type-C current levels in 10mA units. */
#define CUR_LEVEL_3A    300
#define CUR_LEVEL_1_5A  150
#define CUR_LEVEL_DEF   90

static void psrc_dis_ovp(uint8_t port);
void psrc_en_ovp(uint8_t port);
static void psrc_dis_ocp(uint8_t port);
static void psrc_shutdown(uint8_t port, bool discharge_dis);

static void app_psrc_tmr_cbk(uint8_t port, timer_id_t id);
void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out);
void app_psrc_vbus_ocp_cbk(uint8_t port);

static void psrc_select_voltage(uint8_t port);

static void vbus_fet_on(uint8_t port)
{
    app_get_status(port)->is_vbus_on = true;
   
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SNK_FET_OFF_P2();
        CyDelayUs(10);
        APP_VBUS_SRC_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

static void vbus_fet_off(uint8_t port)
{
    app_get_status(port)->is_vbus_on = false;
    
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

void vbus_discharge_on(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_ON_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

void vbus_discharge_off(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

/*Timer Callback*/
static void app_psrc_tmr_cbk(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSOURCE_EN_TIMER:
            timer_stop_range(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_HYS_TIMER);
            app_stat->psrc_volt_old = VSAFE_0V;
            psrc_shutdown(port, true);
            break;

        case APP_PSOURCE_EN_MONITOR_TIMER:
            if(
                    ((app_stat->psrc_rising == true) &&
                     (vbus_is_present(port, app_stat->psrc_volt, VBUS_TURN_ON_MARGIN) == true)) ||
                    ((app_stat->psrc_rising == false) &&
                     (vbus_is_present(port, app_stat->psrc_volt, VBUS_DISCHARGE_MARGIN) == false))
              )
            {
                timer_stop(port, APP_PSOURCE_EN_TIMER);
                /* Start Source enable hysteresis Timer */
                timer_start(port, APP_PSOURCE_EN_HYS_TIMER, APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk );
                break;
            }

            /*Start Monitor Timer again*/
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk );
            break;

        case APP_PSOURCE_EN_HYS_TIMER:

            vbus_discharge_off(port);
            if(app_stat->psrc_rising == false)
            {
                psrc_en_ovp(port);
            }
            app_stat->psrc_volt_old = app_stat->psrc_volt;
            app_stat->pwr_ready_cbk(port);
            break;

        case APP_PSOURCE_DIS_TIMER:
            timer_stop(port, APP_PSOURCE_DIS_MONITOR_TIMER);
            psrc_shutdown(port, true);
            break;

        case APP_PSOURCE_DIS_MONITOR_TIMER:
            if(vbus_is_present(port, VSAFE_5V, VBUS_DISCHARGE_TO_5V_MARGIN) == false)
            {
                psrc_shutdown(port, false);
            }
            if(vbus_is_present(port, VSAFE_0V, VBUS_TURN_ON_MARGIN) == false)
            {
                timer_stop(port, APP_PSOURCE_DIS_TIMER);
                /* Start Extra discharge to allow proper discharge below Vsafe0V */
                timer_start(port, APP_PSOURCE_DIS_EXT_DIS_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD,
                        app_psrc_tmr_cbk );                
            }
            else
            {
                /*Start Monitor Timer again*/
                timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER, APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD,
                        app_psrc_tmr_cbk );
            }
            break;
        case APP_PSOURCE_DIS_EXT_DIS_TIMER:
            vbus_discharge_off(port);
            app_stat->pwr_ready_cbk(port);            
            break;
        default:
            break;
    }
}

#if VBUS_OCP_ENABLE
void app_psrc_vbus_ocp_cbk(uint8_t port)
{
    /* OCP fault. */
    /* Quickly turning off fets here, although dpm_stop will also do that. */
    psrc_shutdown(port, true);
    /* Stopping dpm before sending app event. If OCP retry logic is enabled, app will
     * restart the dpm. */
    dpm_stop(port);
    /* Enqueue HPI OVP fault event. */
    app_event_handler(port, APP_EVT_VBUS_OCP_FAULT, NULL);

}
#endif /* VBUS_OCP_ENABLE */

#if VCONN_OCP_ENABLE
void app_psrc_vconn_ocp_cbk(uint8_t port, bool comp_out)
{
    /*OCP fault*/
    /*Quickly turning off fets here*/
    vconn_disable(port, dpm_get_info(port)->rev_pol);

    /*Enqueue HPI OVP fault event*/
    app_event_handler(port, APP_EVT_VCONN_OCP_FAULT, NULL);
    system_vconn_ocp_dis(port);
}

#endif /* VCONN_OCP_ENABLE */

#if VBUS_OVP_ENABLE
void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out)
{
    /*OVP fault*/
    /*Quickly turning off fets here, although dpm_stop will also do that*/
    psrc_shutdown(port, true);
    /*Enqueue HPI OVP fault event*/
    app_event_handler(port, APP_EVT_VBUS_OVP_FAULT, NULL);
    dpm_stop(port);
}
#endif /* VBUS_OVP_ENABLE */

static void psrc_select_voltage(uint8_t port)
{
    uint8_t intr_state;
    app_status_t* app_stat = app_get_status(port);

    intr_state = CyEnterCriticalSection();
    if(port == TYPEC_PORT_0_IDX)
    {
        ;
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case VSAFE_9V:
                APP_VBUS_SET_9V_P2();
                break;
            case VSAFE_12V:
                APP_VBUS_SET_12V_P2();
                break;
            case VSAFE_13V:
                APP_VBUS_SET_13V_P2();
                break;
            case VSAFE_15V:
                APP_VBUS_SET_15V_P2();
                break;
            case VSAFE_19V:
                APP_VBUS_SET_19V_P2();
                break;
            case VSAFE_20V:
                APP_VBUS_SET_20V_P2();
                break;
            default:
                app_stat->psrc_volt = VSAFE_5V;
                APP_VBUS_SET_5V_P2();
                break;
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    CyExitCriticalSection(intr_state);
}

void psrc_set_voltage (uint8_t port, uint16_t volt_50mV)
{
    app_status_t* app_stat = app_get_status(port);
    app_stat->psrc_volt = volt_50mV;

    if ((app_stat->psrc_volt >= app_stat->psrc_volt_old) && (volt_50mV != VSAFE_0V))
    {
        psrc_en_ovp(port);
    }

    psrc_select_voltage(port);
}

void psrc_set_current (uint8_t port, uint16_t cur_10mA)
{
}

#if VBUS_OCP_ENABLE
static const uint32_t cc_rp_to_cur_map[] = {
    CUR_LEVEL_DEF,
    CUR_LEVEL_1_5A,
    CUR_LEVEL_3A
};
#endif

void psrc_enable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    app_status_t* app_stat = app_get_status(port);
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint8_t       intr_state;

#if VBUS_OCP_ENABLE
    uint32_t ocp_cur;
#endif

    intr_state = CyEnterCriticalSection();

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

    if (dpm_stat->dpm_enabled)
    {
#if VBUS_OCP_ENABLE
        if (get_pd_port_config(port)->protect_en & CFG_TABLE_OCP_EN_MASK)
        {
            if (dpm_stat->pd_connected)
            {
                ocp_cur = dpm_stat->src_sel_pdo.src_gen.max_cur_power;
            }
            else
            {
                ocp_cur = cc_rp_to_cur_map[dpm_stat->src_cur_level];
            }

            system_vbus_ocp_en(port, ocp_cur, app_psrc_vbus_ocp_cbk);
        }
#endif /* VBUS_OCP_ENABLE */

        /*Turn on PSource FET*/
        vbus_fet_on(port);

        /* Turn off VBus Discharge by default. */
        vbus_discharge_off(port);

        if(pwr_ready_handler != NULL)
        {
            app_stat->psrc_rising = true;

            /* If the VBus voltage is dropping, turn the discharge path on. */
            if(app_stat->psrc_volt_old > app_stat->psrc_volt)
            {
                app_stat->psrc_rising = false;
                vbus_discharge_on(port);
            }

            app_stat->pwr_ready_cbk = pwr_ready_handler;

            /* Start Power source enable and monitor timer */
            timer_start(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_EN_TIMER_PERIOD, app_psrc_tmr_cbk );
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk );
        }
    }

    CyExitCriticalSection(intr_state);
}

void psrc_disable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    app_status_t* app_stat = app_get_status(port);
    uint8_t intr_state;

    intr_state = CyEnterCriticalSection();

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

    psrc_set_voltage(port, VSAFE_0V);
    app_stat->psrc_volt_old = VSAFE_0V;

    if ((pwr_ready_handler != NULL) && (dpm_get_info(port)->dpm_enabled))
    {
        /*Turn On discharge*/
        vbus_discharge_on(port);
        app_stat->pwr_ready_cbk = pwr_ready_handler;

        /*Start Power source enable and monitor timer*/
        timer_start(port, APP_PSOURCE_DIS_TIMER, APP_PSOURCE_DIS_TIMER_PERIOD, app_psrc_tmr_cbk );
        timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER, APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk );
    }
    else
    {
        psrc_shutdown(port, true);
    }

    CyExitCriticalSection(intr_state);
}

static void psrc_dis_ovp(uint8_t port)
{
#if VBUS_OVP_ENABLE
#ifdef CCG3
    app_ovp_disable (port, CCG3_SRC_FET);
#else
    app_ovp_disable (port, false);
#endif
#endif /* VBUS_OVP_ENABLE */
}

static void psrc_dis_ocp(uint8_t port)
{
#if VBUS_OCP_ENABLE
    if (get_pd_port_config(port)->protect_en & CFG_TABLE_OCP_EN_MASK)
    {
        system_vbus_ocp_dis(port);
    }
#endif /* VBUS_OCP_ENABLE */
}

static void psrc_shutdown(uint8_t port, bool discharge_dis)
{
    /*Turn Off Source FET*/
    vbus_fet_off(port);

    if(discharge_dis == true)
    {
        vbus_discharge_off(port);
    }

    /* Disable OVP/OCP */
    psrc_dis_ovp(port);
    psrc_dis_ocp(port);
}

void psrc_en_ovp(uint8_t port)
{
#if VBUS_OVP_ENABLE
#ifdef CCG3
    app_ovp_enable (port, app_get_status(port)->psrc_volt, CCG3_SRC_FET, app_psrc_vbus_ovp_cbk);
#else
    app_ovp_enable (port, app_get_status(port)->psrc_volt, false, app_psrc_vbus_ovp_cbk);
#endif
#endif /* VBUS_OVP_ENABLE */
}

/* End of File */
