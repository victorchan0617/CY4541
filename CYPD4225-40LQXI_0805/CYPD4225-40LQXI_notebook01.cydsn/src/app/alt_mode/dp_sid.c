/**
 * @file dp_sid.c
 *
 * @brief @{DisplayPort alternate mode handler source file.@}
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
#include <app.h>
#include <dpm.h>
#include <dp_sid.h>
#include <hpd.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>

typedef struct
{
    alt_mode_info_t info;
    pd_do_t vdo[MAX_DP_VDO_NUMB];
    dp_state_t state;
    mux_select_t dp_mux_cfg;
    pd_do_t config_vdo;
    pd_do_t sink_mode_vdo;
    uint8_t dp_active_flag;
    /*CCG's DP pin supported mask*/
    uint8_t ccg_dp_pins_supp;
    /*Port partner's DP pin supported mask*/
    uint8_t partner_dp_pins_supp;
    uint8_t dp_2_lane_active;
    uint16_t hpd_state;
    uint8_t queue_read_index;
    pd_do_t status_vdo;

#if DP_DFP_SUPP
    bool dp_exit;
    uint8_t dp_4_lane;
    uint8_t dp_2_lane;
    uint8_t dp_cfg_ec_ctrl;
    uint8_t tgt_obj_pos;
#endif /* DP_DFP_SUPP */

}dp_status;

dp_status dp[NO_OF_TYPEC_PORTS];

#if DP_DFP_SUPP
/* Analyse receved DP sink DISC MODE VDO */
static dp_state_t analyse_dp_sink_vdo(uint8_t port, pd_do_t svid_vdo);
/* Evaluates DP sink Status Update/Attention VDM */
static dp_state_t eval_ufp_status(uint8_t port);
/* Inits DP DFP alt mode */
static void init_dp(uint8_t port);
/* Checks if cable supports DP handling */
static bool is_cable_dp_capable(uint8_t port, const atch_tgt_info_t *atch_tgt_info);
/* HPD callback function */
static void dp_src_hpd_resp_cbk(uint8_t port, uint32_t event);
/* Add reseived DP source HPD status to queue */
static void dp_dfp_enqueue_hpd(uint8_t port, uint32_t status);
/* Dequues next HPD status */
static void dp_dfp_dequeue_hpd(uint8_t port);
/* Main DP Source alt mode function */
static void dp_dfp_run(uint8_t port);
/* Analyses received status update VDO */
static void analyse_status_update_vdo(uint8_t port);
/* Analyse if DFP and UFP DP modes consistent */
static bool is_prtnr_ccg_consistent(uint8_t port,uint8_t config);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* HPD UFP callback function */
static void dp_snk_hpd_resp_cbk(uint8_t port, uint32_t cmd);
/* Main DP Sink alt mode function */
static void dp_ufp_run(uint8_t port);
/* Updates DP Sink Status */
static bool dp_ufp_update_status_field (uint8_t port, dp_stat_bm_t bit_pos, bool status);
/* Add reseived DP sink HPD status to queue */
static void dp_ufp_enqueue_hpd(uint8_t port, hpd_event_type_t status);
/* Dequues DP Sink HPD status */
static void dp_ufp_dequeue_hpd(uint8_t port);
/* Verifies is input configuration valid */
static bool is_config_correct(uint8_t port);
#endif /* DP_UFP_SUPP */

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
/* Composes VDM for sending by alt mode manager */
static void send_cmd(uint8_t port);
/* Returns pointer to DP alt mode cmd info structure */
static alt_mode_info_t* dp_info(uint8_t port);
/* Evaluates DP sink Gonfiguration command response */
static mux_select_t dp_eval_config(uint8_t port);
/* Exits DP alt mode */
static void dp_exit(uint8_t port);
/* Inits HPD functionality */
static void init_hpd(uint8_t port, uint8_t port_role);
/* Returns pointer to DP VDO buffer */
static pd_do_t* dp_get_vdo(uint8_t     port);
/* Resets DP variables when exits DP alt mode */
static void reset_dp_var(uint8_t port);
/* Stores given VDO in DP VDO buffer for sending */
static void dp_assign_vdo(uint8_t port, uint32_t vdo);
/* Evaluates received command from application */
static bool dp_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data);
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

#if DP_DFP_SUPP
static void dp_set_app_evt(uint8_t port, uint32_t evtype, uint32_t evdata)
{
    dp[port].info.app_evt_needed = true;
    dp_info(port)->app_evt_data.alt_mode_event_data.evt_type = evtype;
    dp_info(port)->app_evt_data.alt_mode_event_data.evt_data = evdata;
}
#endif

/********************* Alt modes manager register function ********************/

alt_mode_info_t* reg_dp_modes(uint8_t port, alt_mode_reg_info_t* reg_info)
{
    /* Check if DP SVID discover mode received  VDO relates to DP alt mode */
    if (reg_info->svid_vdo.std_dp_vdo.rsvd == DP_ALT_MODE_ID)
    {
#if DP_UFP_SUPP
        /* If DP sink */
        if (reg_info->data_role == PRT_TYPE_UFP)
        {
            /* Reset DP info struct */
            reset_alt_mode_info(&(dp[port].info));
            reset_dp_var(port);
            reg_info->alt_mode_id = DP_ALT_MODE_ID;
            dp_info(port)->cbk = dp_ufp_run;
            dp_info(port)->vdm_header.std_vdm_hdr.svid = DP_SVID;
            dp_info(port)->vdo[SOP] = dp[port].vdo;
            dp_info(port)->vdo_max_numb = MAX_DP_VDO_NUMB;
            dp_info(port)->eval_app_cmd = dp_eval_app_cmd;
            /* Set application event */
            reg_info->app_evt = AM_EVT_ALT_MODE_SUPP;
            /* Get supported DP sink pin assigment. */
            if (reg_info->svid_vdo.std_dp_vdo.recep)
            {
                dp[port].ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.ufp_d_pin;
            }
            else
            {
                dp[port].ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.dfp_d_pin;
            }

            return &(dp[port].info);
        }
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
        if (reg_info->atch_type == ATCH_TGT)
        {
            /* Reset DP info struct */
            reset_alt_mode_info(&(dp[port].info));
            reg_info->alt_mode_id = DP_ALT_MODE_ID;

            /* Check if cable consistent with DP alt mode if cable is detected */
            if (is_cable_dp_capable(port, reg_info->atch_tgt_info) == false)
            {
                /* Set application event */
                reg_info->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
                return NULL;
            }

            reset_dp_var(port);
            /* Set EC ctrl as false at the start */
            dp[port].dp_cfg_ec_ctrl = false;
            /* analyse sink VDO */
            dp[port].state = analyse_dp_sink_vdo(port, reg_info->svid_vdo);

            if (dp[port].state == DP_STATE_EXIT)
            {
                /* Set application event */
                reg_info->app_evt = AM_EVT_NOT_SUPP_PARTNER_CAP;
                return NULL;
            }

            /* Notify application which DP MUX config is supported by both DFP and UFP */
            dp_set_app_evt(port, DP_ALLOWED_MUX_CONFIG_EVT,
                    (dp[port].partner_dp_pins_supp & dp[port].ccg_dp_pins_supp));

            /* Save sink VDO */
            dp[port].sink_mode_vdo = reg_info->svid_vdo;
            /* Set application event */
            reg_info->app_evt = AM_EVT_ALT_MODE_SUPP;

            /* If SOP prime not used return SOP_INVALID */
            /* SOP' disc svid not needed */
            reg_info->cbl_sop_flag = SOP_INVALID;
            /* Prepare DP Enter command */
            dp_info(port)->mode_state = ALT_MODE_STATE_INIT;
            dp_dfp_run(port);

            return &dp[port].info;
        }
#endif /* DP_DFP_SUPP */
    }

    /* If received VDO not DP alt mode VDO */
    reg_info->alt_mode_id = MODE_NOT_SUPPORTED;
    return NULL;
}

/************************* DP Source Functions definitions ********************/

#if DP_DFP_SUPP
static void dp_dfp_run(uint8_t port)
{
    alt_mode_state_t dp_mode_state = dp_info(port)->mode_state;

    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_INIT:
            /* Enable DP functionality */
            init_dp(port);
            /* Analyse DP sink VDO */
            dp[port].state = analyse_dp_sink_vdo(port, dp[port].sink_mode_vdo);
            send_cmd(port);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            dp[port].state = (dp_state_t)dp_info(port)->vdm_header.std_vdm_hdr.cmd;
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            switch (dp[port].state)
            {
                case DP_STATE_ENTER:
                    /* Init HPD */
                    init_hpd(port, PRT_TYPE_DFP);
                    /* Go to Status update */
                    dp[port].state = DP_STATE_STATUS_UPDATE;
                    dp_assign_vdo(port, STATUS_UPDATE_VDO);
                    break;
                case DP_STATE_STATUS_UPDATE:
                    /* Analyse received VDO */
                    analyse_status_update_vdo(port);
                    if (dp[port].dp_cfg_ec_ctrl == EC_DP_MUX_CTRL_CMD)
                    {
                        return;
                    }
                    break;
                case DP_STATE_CONFIG:
                    /* If EC controls DP MUX Config - send ACK to EC */
                    if (dp[port].dp_cfg_ec_ctrl == EC_DP_MUX_CTRL_CMD)
                    {
                        dp_set_app_evt(port, DP_STATUS_UPDATE_EVT, DP_EC_CFG_CMD_ACK_MASK);
                    }

                    dp[port].dp_mux_cfg = dp_eval_config(port);
                    /* CDT 251518 fix - set HPD to LOW when USB config is set */
                    if (dp[port].config_vdo.val == EMPTY_VDO)
                    {
                        dp_dfp_enqueue_hpd(port, EMPTY_VDO);
                    }
                    else
                    {
                        /* Add HPD to queue*/
                        dp_dfp_enqueue_hpd(port, dp[port].status_vdo.val);
                    }

                    set_mux(port, dp[port].dp_mux_cfg, NO_DATA);
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    /* Exit DP required */
                    if (dp[port].dp_exit == true)
                    {
                        dp[port].state = DP_STATE_EXIT;
                        dp[port].dp_exit = false;
                        break;
                    }
                    return;
                case DP_STATE_EXIT:
                    dp_exit(port);
                    return;
                default:
                    return;
            }
            send_cmd(port);
            break;

        case ALT_MODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((dp_state_t)dp_info(port)->vdm_header.std_vdm_hdr.cmd == DP_STATE_ATT)
            {
                /* Handle HPD status */
                if (dp[port].dp_active_flag != false)
                {
                    /* Add HPD to queue*/
                    dp_dfp_enqueue_hpd(port, (*dp_get_vdo(port)).val);
                }
                /* Analyse received VDO */
                analyse_status_update_vdo(port);
                if (dp[port].dp_cfg_ec_ctrl == EC_DP_MUX_CTRL_CMD)
                {
                    return;
                }
                send_cmd(port);
            }
            break;

        case ALT_MODE_STATE_FAIL:
            if ((dp[port].state == DP_STATE_ENTER) || (dp[port].state == DP_STATE_EXIT))
            {
                dp_exit(port);
            }
            else if ((dp[port].state == DP_STATE_CONFIG) || (dp[port].state == DP_STATE_STATUS_UPDATE))
            {
                dp[port].state = DP_STATE_EXIT;
                send_cmd(port);
            }
            else
            {
                dp[port].state = DP_STATE_IDLE;
                dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            }
            break;

        case ALT_MODE_STATE_EXIT:
            dp[port].state = DP_STATE_CONFIG;
            dp[port].dp_exit = true;

            dp_assign_vdo(port, DP_USB_SS_CONFIG);
            send_cmd(port);
            break;

        default:
            break;
    }
}

static void analyse_status_update_vdo(uint8_t port)
{

    /* If EC controls DP MUX Config */
    if (dp[port].dp_cfg_ec_ctrl == EC_DP_MUX_CTRL_CMD)
    {
        dp_set_app_evt(port, DP_STATUS_UPDATE_EVT, (*dp_get_vdo(port)).val);

        /* Set DP to idle and wait for config cmd from EC */
        dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
        return;
    }

    /* CCG handles Status Update in auto mode */
    dp[port].state = eval_ufp_status(port);
    if (dp[port].state == DP_STATE_CONFIG)
    {
        set_mux(port, MUX_CONFIG_ISOLATE, NO_DATA);
        if (dp[port].config_vdo.val != DP_USB_SS_CONFIG)
        {
            dp[port].config_vdo.dp_cfg_vdo.sign = DP_1_3_SIGNALING;
            dp[port].config_vdo.dp_cfg_vdo.sel_conf = DP_CONFIG_SELECT;
        }

        dp_assign_vdo(port,dp[port].config_vdo.val);
    }
}

static dp_state_t analyse_dp_sink_vdo(uint8_t port, pd_do_t svid_vdo)
{
    if ((svid_vdo.std_dp_vdo.port_cap == DP_PORT_CAP_UFP_D) ||
            (svid_vdo.std_dp_vdo.port_cap == DP_PORT_CAP_BOTH))
    {
        /* Read DP_MODES_SUPPORTED in DP structure. */
        dp[port].ccg_dp_pins_supp = get_pd_port_config(port)->dp_config_supported;

        /* If DP Interface is presented on a USB TYPE C receptacle,
         * compare with UFP_D Pin Assignments advertised by UFP. */
        if (svid_vdo.std_dp_vdo.recep)
        {
            dp[port].partner_dp_pins_supp = svid_vdo.std_dp_vdo.ufp_d_pin;
        }
        else
        {
            dp[port].partner_dp_pins_supp = svid_vdo.std_dp_vdo.dfp_d_pin;
        }

        /* Check if either C/E is supported by both UFP and CCG. If yes.
         * we are good to go. */
        if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_C))
        {
            dp[port].dp_4_lane = DP_DFP_D_CONFIG_C;
        }
        else if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_E))
        {
            dp[port].dp_4_lane = DP_DFP_D_CONFIG_E;
        }
        else
        {
            return DP_STATE_EXIT;
        }
    }
    else
    {
        /* UFP does not support UFP_D mode. DP not possible. */
        return DP_STATE_EXIT;
    }

    /* See if 2 lane Pin Configuration is supported by both devices.
     * If yes, store the preference. */
    if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_D))
    {
        dp[port].dp_2_lane = DP_DFP_D_CONFIG_D;
    }
    else if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_F))
    {
        dp[port].dp_2_lane = DP_DFP_D_CONFIG_F;
    }
    else
    {
        /* Multi Function not possible. */
        dp[port].dp_2_lane = DP_INVALID_CFG;
    }

    return DP_STATE_ENTER;
}

static dp_state_t eval_ufp_status(uint8_t port)
{
    dp_state_t ret = DP_STATE_CONFIG;
    bool       dp_mode = false;
    uint32_t   dp_asgn = DP_INVALID_CFG;

    pd_do_t status = *dp_get_vdo(port);
    dp[port].status_vdo = status;

    dp[port].config_vdo.val = 0;

    /* Handle the STATUS UPDATE sent by UFP. */
    if ((status.dp_stat_vdo.usb_cfg != 0) || (status.dp_stat_vdo.exit != 0))
    {
        dp[port].config_vdo.val = DP_USB_SS_CONFIG;
        /* If Exit request bit is set then send Exit cmd */
        if (status.dp_stat_vdo.exit != 0)
        {
            dp[port].dp_exit = true;
        }
    }
    else if (dp[port].dp_active_flag == false)
    {
        /* USB Configuration mode. */
        /* Check if UFP_D is connected. */
        if (status.dp_stat_vdo.dfp_ufp_conn > DP_CONN_DFP_D)
        {
            /* Check if Multi Function requested and CCG can support 2 lane. */
            if ((status.dp_stat_vdo.mult_fun == true) &&
                    (dp[port].dp_2_lane != DP_INVALID_CFG))
            {
                dp_asgn = dp[port].dp_2_lane;
            }
            else
            {
                dp_asgn = dp[port].dp_4_lane;
            }

            dp_mode = true;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }
    /* DP Config is active. */
    else
    {
        /* DP Config mode. */
        /* Check if UFP_D no longer connected. */
        if (status.dp_stat_vdo.dfp_ufp_conn < DP_CONN_UFP_D)
        {
            /* Move to USB Config. */
            dp[port].config_vdo.val = DP_USB_SS_CONFIG;
        }
        /* Check if DP sink is requesting for 2 lane and current
         * config is not 2 DP lane. */
        else if ((status.dp_stat_vdo.mult_fun == true) && (dp[port].dp_2_lane_active == false)
                && (dp[port].dp_2_lane != DP_INVALID_CFG))
        {
            /* Move to DP 2 lane. */
            dp_mode = true;
            dp_asgn = dp[port].dp_2_lane;
        }
        /* Check if DP Sink is requesting for NON-MULTI Funtion Mode
         * and current mode is MULTI FUNCTION. In this case switch to
         * 4 lane DP. */
        else if ((dp[port].dp_2_lane_active == true) && (status.dp_stat_vdo.mult_fun == false))
        {
            /* Move to DP 4 lane. */
            dp_mode = true;
            dp_asgn = dp[port].dp_4_lane;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }

    if (dp_mode)
    {
        dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt = dp_asgn;
    }

    return ret;
}

static bool is_cable_dp_capable(uint8_t port, const atch_tgt_info_t* atch_tgt_info)
{
    if ((atch_tgt_info->cbl_vdo != NULL) && (atch_tgt_info->cbl_vdo->val != NO_DATA))
    {
        /* DP cannot be supported if the USB mode supported is not 3.1 Gen1/Gen2; or
           if this is an ACTIVE CABLE and the directionality bits are not supported. */
        if (
                (atch_tgt_info->cbl_vdo->std_cbl_vdo.usb_ss_sup == USB_2_0_SUPP) ||
                ((dpm_get_info(port)->cbl_type == PROD_TYPE_ACT_CBL) &&
                 ((atch_tgt_info->cbl_vdo->val & CBL_DIR_SUPP_MASK) != CBL_DIR_SUPP_MASK))
           )
        {
            return false;
        }
    }

    /* Allow DP mode to go through if no cable marker is found. */
    return true;
}

static void init_dp(uint8_t port)
{
    dp_info(port)->eval_app_cmd = dp_eval_app_cmd;
    dp_info(port)->vdo_max_numb = MAX_DP_VDO_NUMB;
    dp_info(port)->vdo[SOP] = dp[port].vdo;
    /* Goto enter state */
    dp[port].state = DP_STATE_ENTER;
    dp_info(port)->cbk = dp_dfp_run;

}

void dp_dfp_enqueue_hpd(uint8_t port, uint32_t status)
{
    switch(GET_HPD_IRQ_STAT(status))
    {
        case HPD_LOW_IRQ_LOW:
            /* Empty queue */
            dp[port].queue_read_index = DP_QUEUE_STATE_SIZE;
            dp[port].hpd_state = (uint32_t) HPD_EVENT_UNPLUG;
            break;
        case HPD_LOW_IRQ_HIGH:
            dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
            dp[port].hpd_state |= (uint32_t) HPD_EVENT_IRQ;
            dp[port].queue_read_index += DP_QUEUE_STATE_SIZE;
            break;
        case HPD_HIGH_IRQ_LOW:
            /* Add to queue High HPD state */
            dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
            dp[port].hpd_state |= (uint32_t) HPD_EVENT_PLUG;
            dp[port].queue_read_index += DP_QUEUE_STATE_SIZE;
            break;
        case HPD_HIGH_IRQ_HIGH:
            /*  Add to queue High HPD state and IRQ HPD*/
            dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
            dp[port].hpd_state |= (uint32_t) HPD_EVENT_PLUG;
            dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
            dp[port].hpd_state |= (uint32_t) HPD_EVENT_IRQ;
            dp[port].queue_read_index += (DP_QUEUE_STATE_SIZE << 1);
            break;
        default:
            break;
    }
    dp_dfp_dequeue_hpd(port);
}

void dp_dfp_dequeue_hpd(uint8_t port)
{
    hpd_event_type_t hpd_evt = HPD_EVENT_NONE;

    if (dp[port].queue_read_index != DP_QUEUE_EMPTY_INDEX)
    {
        hpd_evt = (hpd_event_type_t)(DP_HPD_STATE_MASK &
                (dp[port].hpd_state >> (dp[port].queue_read_index - DP_QUEUE_STATE_SIZE))
                );
        eval_hpd_cmd (port, (uint32_t)hpd_evt);
    }
}

static void dp_src_hpd_resp_cbk(uint8_t port, uint32_t event)
{
    if (((event & 0xFFFF) == HPD_COMMAND_DONE) && (dp_info(port)->is_active == true))
    {
        if (dp[port].queue_read_index != DP_QUEUE_EMPTY_INDEX)
        {
            dp[port].queue_read_index -= DP_QUEUE_STATE_SIZE;
            dp_dfp_dequeue_hpd(port);
        }
    }
}
#endif /* DP_DFP_SUPP */

/************************* DP Sink Functions definitions **********************/

#if DP_UFP_SUPP

#if DP_GPIO_CONFIG_SELECT

/* Global to hold DP Pin configuration when GPIO based selection is enabled. */
uint8_t gl_dp_sink_config = 0;

void dp_sink_set_pin_config(uint8_t dp_config)
{
    /* Store DP Configuration supported as DP Sink. */
    gl_dp_sink_config = dp_config;
}

uint8_t dp_sink_get_pin_config(void)
{
    return gl_dp_sink_config;
}
#endif /* DP_GPIO_CONFIG_SELECT */

static void dp_ufp_run(uint8_t port)
{
    /* Get alt modes state and VDM command */
    alt_mode_state_t dp_mode_state = dp_info(port)->mode_state;
    dp[port].state = (dp_state_t)dp_info(port)->vdm_header.std_vdm_hdr.cmd;
    /* If exit all modes cmd received */
    if (dp_info(port)->vdm_header.std_vdm_hdr.cmd == EXIT_ALL_MODES)
    {
        dp[port].state = DP_STATE_EXIT;
    }
    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_IDLE:

            switch(dp[port].state)
            {
                case DP_STATE_ENTER:
                    /* Enable HPD receiver */
                    init_hpd (port, PRT_TYPE_UFP);
                    /* Fill Status Update with appropiate bits */
                    dp[port].status_vdo.val = EMPTY_VDO;
                    /* Update UFP Enabled status field */
                    dp_ufp_update_status_field(port, DP_STAT_EN, true);
#if DP_UFP_MONITOR
                    /* Update UFP connection status */
                    dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
#endif /* DP_UFP_MONITOR */

                    /* Check if Multi-function allowed */
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_F) != NO_DATA))
                    {
                        dp_ufp_update_status_field(port, DP_STAT_MF, true);
                    }

                    /* No VDO is needed in response */
                    dp_assign_vdo(port, NONE_VDO);
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    return;

                case DP_STATE_STATUS_UPDATE:
                    /* Update current HPD status */
                    dp_ufp_update_status_field(port, DP_STAT_HPD, dp_snk_get_hpd_state(port));
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_C)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, dp_snk_get_hpd_state(port));
                    }
#else
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_C) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA))
                    {
                        /* Always report UFP connected. */
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN,
                                dp_snk_get_hpd_state(port));
                    }
#endif /* DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Respond with current DP sink status */
                    dp_assign_vdo(port, dp[port].status_vdo.val);
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    return;

                case DP_STATE_CONFIG:
                    /* Check if Config VDO is correct */
                    if (
                            /* IF DP configuration */
                            ((dp_get_vdo(port)->dp_cfg_vdo.sel_conf == DP_CONFIG_SELECT) &&
                             (dp_get_vdo(port)->dp_cfg_vdo.sign == DP_1_3_SIGNALING)) ||
                            /* If USB configuration requested */
                            (dp_get_vdo(port)->val == EMPTY_VDO)
                       )
                    {
                        /* Save port partner pin config */
                        dp[port].partner_dp_pins_supp = dp_get_vdo(port)->dp_cfg_vdo.dfp_asgmnt;
                        /* Check if both UFP and DFP support selected pin configuration */
#if DP_GPIO_CONFIG_SELECT
                        /* Pin configuration supported is based on GPIO status. */
                        uint8_t dp_config = dp_sink_get_pin_config ();
#else
                        /* Get DP Pin configuration supported from config table. */
                        uint8_t dp_config = get_pd_port_config(port)->dp_config_supported;
#endif /* DP_GPIO_CONFIG_SELECT */
                        if (
                                ((is_config_correct(port)) &&
                                 (dp_config & dp[port].partner_dp_pins_supp)) ||
                                (dp_get_vdo(port)->dp_cfg_vdo.sel_conf == USB_CONFIG_SELECT)
                           )
                        {
                            /* Save config VDO */
                            dp[port].config_vdo = *(dp_get_vdo(port));
                            /* Get DP MUX configuration */
                            dp[port].dp_mux_cfg = dp_eval_config(port);
                            /* Set DP MUX */
                            set_mux(port, dp[port].dp_mux_cfg, NO_DATA);
                            dp_assign_vdo(port, NONE_VDO);
                            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                            return;
                        }
                    }
                    break;

                case DP_STATE_EXIT:
                    /* Set MUX to SS */
                    set_mux(port, MUX_CONFIG_SS_ONLY, NO_DATA);
                    /* Deinit alt mode and reset dp variables */
                    dp_exit(port);
                    dp_assign_vdo(port, NONE_VDO);
                    return;

                default:
                    break;
            }
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            /* Analyse Attention callback */
            if (dp[port].state == DP_STATE_ATT)
            {
                /* Clear Request and IRQ status bits */
                dp_ufp_update_status_field(port, DP_STAT_USB_CNFG, false);
                dp_ufp_update_status_field(port, DP_STAT_EXIT, false);
                dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                /* Check if any HPD event in queue */
                dp_ufp_dequeue_hpd(port);
                return;
            }
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            return;

        default:
            break;
    }

    /* Send NACK */
    dp_info(port)->mode_state = ALT_MODE_STATE_FAIL;
}

static bool is_config_correct(uint8_t port)
{
    bool retval = false;

    /* If input configuration is valid then return true */
    switch (dp[port].partner_dp_pins_supp)
    {
        case DP_DFP_D_CONFIG_C:
        case DP_DFP_D_CONFIG_D:
        case DP_DFP_D_CONFIG_E:
        case DP_DFP_D_CONFIG_F:
            retval = true;
            break;
        default:
            retval = false;
            break;
    }

    return retval;
}

static bool dp_ufp_update_status_field(uint8_t port, dp_stat_bm_t bit_pos, bool status)
{
    uint32_t    prev_status;

    /* Save current DP sink status */
    prev_status = dp[port].status_vdo.val;

    /* Update status field*/
    if (status == true)
    {
        SET_FLAG(dp[port].status_vdo.val, (uint8_t)bit_pos);
    }
    else
    {
        REMOVE_FLAG(dp[port].status_vdo.val, (uint8_t)bit_pos);
    }

    /* Check if status changed */
    if (prev_status == dp[port].status_vdo.val)
    {
        return false;
    }

    return true;
}

static void dp_snk_hpd_resp_cbk(uint8_t port, uint32_t  cmd)
{
    hpd_event_type_t event = (hpd_event_type_t)cmd;

    /* Handle hpd only when DP sink was entered */
    if (
            (event > HPD_EVENT_NONE)   &&
            (event < HPD_COMMAND_DONE) &&
            (dp_info(port)->is_active == true)
       )
    {
        /* If HPD received after Status update command was sent then add to HPD queue */
        dp_ufp_enqueue_hpd(port, event);
    }
}

static void dp_ufp_enqueue_hpd(uint8_t port, hpd_event_type_t status)
{
    /* Check if queue is not full */
    if (dp[port].queue_read_index <= (DP_UFP_MAX_QUEUE_SIZE * DP_QUEUE_STATE_SIZE))
    {
        switch (status)
        {
            case HPD_EVENT_UNPLUG:
                /* Empty queue */
                dp[port].queue_read_index = DP_QUEUE_STATE_SIZE;
                dp[port].hpd_state = (uint32_t) status;
                break;
            case HPD_EVENT_PLUG:
            case HPD_EVENT_IRQ:
                /* Add to queue High HPD state or IRQ */
                dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
                dp[port].hpd_state |= (uint32_t) status;
                dp[port].queue_read_index += DP_QUEUE_STATE_SIZE;
                break;

            default:
                return;
        }
        /* Dequeue HPD events */
        dp_ufp_dequeue_hpd(port);
    }
}

static void dp_ufp_dequeue_hpd(uint8_t port)
{
    hpd_event_type_t hpd_evt = HPD_EVENT_NONE;
    bool    is_att_needed = false;

    if (dp[port].queue_read_index != (uint8_t)DP_QUEUE_EMPTY_INDEX)
    {
        /* Get queued HPD event */
        hpd_evt = (hpd_event_type_t)(DP_HPD_STATE_MASK & (dp[port].hpd_state >>
                    (dp[port].queue_read_index - DP_QUEUE_STATE_SIZE)));
        /* Decrease queue size */
        dp[port].queue_read_index -= DP_QUEUE_STATE_SIZE;
        /* Get queued HPD event */
        switch(hpd_evt)
        {
            case HPD_EVENT_UNPLUG:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_HPD, false))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_E)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, false);
                    }
#else
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_C) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA))
                    {
                        /* Always report UFP connected. */
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        /* Update UFP connection status */
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, false);
                    }
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* Zero IRQ status if Unattached */
                dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                break;
            case HPD_EVENT_PLUG:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_HPD, true))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_E)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
                    }
#else
                    /* Update UFP connection status */
                    dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* If next queued event is IRQ we can combine in one Attention */
                if ((((hpd_event_type_t)(DP_HPD_STATE_MASK & (dp[port].hpd_state >>
                                        (dp[port].queue_read_index - DP_QUEUE_STATE_SIZE)))) == HPD_EVENT_IRQ) &&
                        (dp[port].queue_read_index != (uint8_t)DP_QUEUE_EMPTY_INDEX))
                {
                    /* Decrease queue size */
                    dp[port].queue_read_index -= DP_QUEUE_STATE_SIZE;
                    /* Update IRQ field in status */
                    dp_ufp_update_status_field(port, DP_STAT_IRQ, true);
                    is_att_needed = true;
                }
                else
                {
                    /* Zero IRQ status */
                    dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                }
                break;
            case HPD_EVENT_IRQ:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_IRQ, true))
                {
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                break;
            default:
                break;
        }

        /* If status changed then send attention */
        if (is_att_needed == true)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            dp[port].state = DP_STATE_ATT;
            dp_assign_vdo(port, dp[port].status_vdo.val);
            send_cmd(port);
            return;
        }
    }
    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
}
#endif /* DP_UFP_SUPP */

/************************* Common DP Functions definitions ********************/

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static void send_cmd(uint8_t port)
{
    if (dp[port].state != DP_STATE_IDLE)
    {
        dp_info(port)->vdm_header.val = NO_DATA;
        dp_info(port)->vdm_header.std_vdm_hdr.cmd = (uint32_t)dp[port].state;
        dp_info(port)->vdm_header.std_vdm_hdr.svid = DP_SVID;

        dp_info(port)->vdo_numb[SOP] = MAX_DP_VDO_NUMB;
        if ((dp[port].state == DP_STATE_EXIT) || (dp[port].state == DP_STATE_ENTER))
        {
            dp_info(port)->vdo_numb[SOP] = NO_DATA;
        }

        dp_info(port)->sop_state[SOP] = ALT_MODE_STATE_SEND;
        dp_info(port)->mode_state = ALT_MODE_STATE_SEND;
    }
}

static alt_mode_info_t* dp_info(uint8_t port)
{
    return &(dp[port].info);
}

#if DP_DFP_SUPP
static bool is_prtnr_ccg_consistent(uint8_t port, uint8_t config)
{
    bool ret = false;

    if (
            /* DP MUX pin */
            ((dp[port].ccg_dp_pins_supp & config) &&
             (dp[port].partner_dp_pins_supp & config)) ||
            /* USB config */
            (config == NO_DATA)
       )
    {
        ret = true;
    }

    return ret;
}
#endif /* DP_DFP_SUPP */

static mux_select_t dp_eval_config(uint8_t port)
{

    switch (dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt)
    {
        case DP_DFP_D_CONFIG_C:
        case DP_DFP_D_CONFIG_E:
            dp[port].dp_active_flag = true;
            dp[port].dp_2_lane_active = false;
            return MUX_CONFIG_DP_4_LANE;
        case DP_DFP_D_CONFIG_D:
        case DP_DFP_D_CONFIG_F:
            dp[port].dp_active_flag = true;
            dp[port].dp_2_lane_active = true;
            return MUX_CONFIG_DP_2_LANE;
        case DP_USB_SS_CONFIG:
            dp[port].dp_active_flag = false;
            dp[port].dp_2_lane_active = false;
        default:
            break;
    }

    return MUX_CONFIG_SS_ONLY;
}

static void reset_dp_var(uint8_t port)
{
    /* Zeros DP flags */
    dp[port].dp_active_flag = false;
    dp[port].dp_2_lane_active = false;
    dp[port].config_vdo.val = EMPTY_VDO;
    dp[port].status_vdo.val = EMPTY_VDO;
    dp[port].hpd_state = false;
    dp[port].queue_read_index = false;
    dp[port].state = DP_STATE_IDLE;
    dp[port].partner_dp_pins_supp = false;
    dp_assign_vdo(port, EMPTY_VDO);

#if DP_DFP_SUPP
    dp[port].dp_2_lane = false;
    dp[port].dp_4_lane = false;
    dp[port].dp_exit = false;
#endif /* DP_DFP_SUPP */
}

static void dp_assign_vdo(uint8_t port, uint32_t vdo)
{
    /* No DP VDOs needed while send VDM */
    if (vdo == NONE_VDO)
    {
        dp_info(port)->vdo_numb[SOP] = NO_DATA;
    }
    else
    {
        /* Include given VDO as part of VDM */
        dp[port].vdo[DP_VDO_IDX].val = vdo;
        dp_info(port)->vdo_numb[SOP] = MAX_DP_VDO_NUMB;
    }
}

static pd_do_t* dp_get_vdo(uint8_t port)
{
    return &(dp[port].vdo[DP_VDO_IDX]);
}

static void dp_exit(uint8_t port)
{
    reset_dp_var(port);
    eval_hpd_cmd(port, HPD_DISABLE_CMD);
    alt_mode_hw_set_cbk(port, NULL);
    dp_info(port)->mode_state = ALT_MODE_STATE_EXIT;
}

static void init_hpd(uint8_t port, uint8_t port_role)
{
#if DP_DFP_SUPP
    if (port_role == PRT_TYPE_DFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        alt_mode_hw_set_cbk (port, dp_src_hpd_resp_cbk);
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if (port_role == PRT_TYPE_UFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        alt_mode_hw_set_cbk (port, dp_snk_hpd_resp_cbk);
    }
#endif /* DP_UFP_SUPP */

    eval_hpd_cmd (port, HPD_ENABLE_CMD);
    dp[port].hpd_state = 0;
    dp[port].queue_read_index = 0;
}

/****************************HPI command evaluation***************************/

static bool dp_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data)
{
#if ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0))
    uint32_t data = cmd_data.alt_mode_event_data.evt_data;
#endif /* ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0)) */

#if DP_DFP_SUPP
    uint32_t config = NO_DATA;
#endif /* DP_DFP_SUPP */
#if DP_UFP_SUPP
    bool is_att_needed = false;
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
    /* EC mux configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == EC_DP_MUX_CTRL_CMD)
    {
        if (data == EC_DP_MUX_CTRL_CMD)
        {
            dp[port].dp_cfg_ec_ctrl = true;
        }
        else
        {
            dp[port].dp_cfg_ec_ctrl = false;
        }
        return true;
    }
    /* DP config command received */
    else if ((cmd_data.alt_mode_event_data.evt_type == DP_EC_CFG_CMD) &&
            (dp[port].dp_cfg_ec_ctrl == true))
    {
        if (data == DP_EC_CFG_USB_IDX)
        {
            dp[port].state = DP_STATE_CONFIG;

            dp[port].config_vdo.val = DP_USB_SS_CONFIG;
            dp_assign_vdo(port, dp[port].config_vdo.val);
            send_cmd(port);
        }
        else if(data <= DP_EC_CFG_CMD_MAX_NUMB)
        {
            /* Check if received DP config is possible */
            SET_FLAG(config, data);
            /* Check pin assigment compatibility */
            if ((is_prtnr_ccg_consistent(port, config)) &&
                    (dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt != config))
            {
                dp[port].state = DP_STATE_CONFIG;

                /* Prepare Config cmd and send VDM */
                dp[port].config_vdo.val = 0;
                dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt = config;
                dp[port].config_vdo.dp_cfg_vdo.sign = DP_1_3_SIGNALING;
                dp[port].config_vdo.dp_cfg_vdo.sel_conf = DP_CONFIG_SELECT;
                dp_assign_vdo(port, dp[port].config_vdo.val);
                send_cmd(port);
            }
        }
        else
        {
            return false;
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    /* EC mux configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == EC_DP_SINK_CTRL_CMD)
    {
        /* Check if received command is Exit/USB Request */
        if ((data < DP_STAT_HPD) && (data > DP_STAT_MF))
        {
            /* Update DP sink status */
            if (dp_ufp_update_status_field(port, (dp_stat_bm_t)data, true))
            {
                /* Set send Attention flag */
                is_att_needed = true;
            }
        }
        /* Check if received command is enabling of Multi-function */
        if (data == DP_STAT_MF)
        {
            /* Check if MF is supported by CCG */
            if (
                    ((dp[port].ccg_dp_pins_supp == DP_DFP_D_CONFIG_D) ||
                     (dp[port].ccg_dp_pins_supp == DP_DFP_D_CONFIG_F)) &&
                    /* If status changed from previous */
                    (dp_ufp_update_status_field(port, DP_STAT_MF, true) != false)
               )
            {
                is_att_needed = true;
            }
        }
        /* Check if received command is disabling of Multi-function */
        if (
                (data == (DP_STAT_MF - 1)) &&
                (dp_ufp_update_status_field(port, DP_STAT_MF, false) != false)
           )
        {
            is_att_needed = true;
        }
        /* If status changed then send attention */
        if (is_att_needed == true)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            dp[port].state = DP_STATE_ATT;
            dp_assign_vdo(port, dp[port].status_vdo.val);
            send_cmd(port);
            return true;
        }
        else
        {
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
        }
    }
#endif /* DP_UFP_SUPP */

    return false;
}
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

/* [] END OF FILE */
