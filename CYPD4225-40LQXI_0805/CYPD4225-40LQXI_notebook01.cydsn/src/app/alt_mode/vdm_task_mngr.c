/**
 * @file vdm_task_mngr.c
 *
 * @brief @{VDM task manager source file.@}
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

#include <vdm.h>
#include <pd.h>
#include <dpm.h>
#include <timer.h>
#include <app.h>
#include <alt_mode_hw.h>
#include <hpi.h>
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#include <vdm_task_mngr.h>
#include <alt_modes_mngr.h>

/**
 * @typedef vdm_task_status_t
 * @brief struct to hold vdm manager status
 */
typedef struct
{
    /* Current VDM manager state */
    vdm_task_t      vdm_task;
    /* Current VDM manager event */
    vdm_evt_t       vdm_evt;
    /* Info about cable, device/AMA */
    atch_tgt_info_t atch_tgt;
    /* Struct with received/sent VDM info */
    vdm_msg_info_t  vdm_msg;
    /* Sent/received VDM retry counters */
    uint8_t         rec_retry_cnt;
    /* Holds svid index if we have more than one Disc SVID response */
    uint8_t         svid_idx;
    /* Holds count of D_SVID requests sent. */
    uint8_t         dsvid_cnt;
}vdm_task_status_t;

/* Main structure to hold variables for VDM task manager */
vdm_task_status_t vdm_stat[NO_OF_TYPEC_PORTS];

/* Init VDM task mngr */
static vdm_task_t vdm_task_mngr_init(uint8_t port);

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
/* Discover ID cmd handler */
static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt);
/* Discovery SVID cmd handler */
static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

/* Sets recevied VDM response command as failed */
static void set_vdm_failed(uint8_t port);
/* Check VDM retries */
static bool vdm_retry_check(uint8_t port);
/* Checks if received response is expected */
static bool is_rec_vdm_expected (uint8_t port, const pd_packet_t *rec_vdm);
/* Parses received VDM resp to vdm_msg_info_t struct */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm);
/* Callback function for sent VDM */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t *rec_vdm);
/* Function to send VDM */
static vdm_task_t send_vdm(uint8_t port);
/* Checks if VDM mngr is enabled */
static bool is_vdm_mngr_enabled(uint8_t     port);
/* Returns pointer to vdm_msg_info_t struct */
static vdm_msg_info_t* get_msg(uint8_t port);
/* Composes vdm_msg_info_t data to dpm_pd_cmd_buf_t struct */
static uint8_t compose_vdm(uint8_t port);
/* Reset vdm mngr info */
static void reset_vdm_mngr(uint8_t port);

/* VDM field set/get functions. */
vdm_evt_t get_vdm_evt(uint8_t port);
vdm_task_t get_vdm_mngr_task(uint8_t port);
void set_mngr_task(uint8_t port, vdm_task_t task);
void set_mngr_event(uint8_t port, vdm_evt_t evt);

void enable_vdm_task_mngr(uint8_t port)
{
    /* If the VDM Task is already running, do nothing. */
    if (app_get_status(port)->vdm_task_en == false)
    {
        vdm_stat[port].vdm_task = VDM_TASK_INIT;
        app_get_status(port)->vdm_task_en = true;
        app_get_status(port)->vdm_prcs_failed = false;
    }
}

static vdm_task_t vdm_task_mngr_init(uint8_t port)
{
    /* Reset vdm mngr info */
    reset_vdm_mngr(port);
    set_mngr_event(port, VDM_EVT_RUN);

    /* Check if current data role is DFP */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
#if DFP_ALT_MODE_SUPP
        return VDM_TASK_DISC_ID;
#else
        return VDM_TASK_EXIT;
#endif /* DFP_ALT_MODE_SUPP */
    }

#if UFP_ALT_MODE_SUPP
    /* If data role is UFP */
    return VDM_TASK_REG_ATCH_TGT_INFO;
#else
    return VDM_TASK_EXIT;
#endif /* DFP_ALT_MODE_SUPP */

}

bool is_vdm_task_idle(uint8_t port)
{
    bool stat = true;

    /*
       If VDM manager is enabled, check
       1. Whether BUSY timer is running.
       2. Whether the ALT mode tasks are idle.
     */
    if (is_vdm_mngr_enabled (port))
    {
        if (
                (timer_is_running (port, APP_VDM_BUSY_TIMER)) ||
                (vdm_stat[port].vdm_task != VDM_TASK_ALT_MODE) ||
                (is_alt_mode_mngr_idle(port) == false)
           )
        {
            stat = false;
        }
    }

    return stat;
}

void vdm_task_mngr(uint8_t port)
{
    if (
            (is_vdm_mngr_enabled(port) == false) ||
            (timer_is_running(port, APP_VDM_BUSY_TIMER) == true)
       )
    {
        return;
    }

    /* Get current vdm task */
    switch (get_vdm_mngr_task(port))
    {
        case VDM_TASK_WAIT:
            break;
        case VDM_TASK_INIT:
            set_mngr_task(port, vdm_task_mngr_init(port));
            break;

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        case VDM_TASK_DISC_ID:
            set_mngr_task(port, vdm_task_mng_disc_id(port, get_vdm_evt(port)));
            break;
        case VDM_TASK_DISC_SVID:
            set_mngr_task(port, vdm_task_mng_disc_svid(port, get_vdm_evt(port)));
            break;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

        case VDM_TASK_SEND_MSG:
            compose_vdm(port);
            set_mngr_task(port, send_vdm(port));
            break;
        case VDM_TASK_REG_ATCH_TGT_INFO:
            set_mngr_task(port, reg_alt_mode_mngr(port, &vdm_stat[port].atch_tgt, &vdm_stat[port].vdm_msg));
            set_mngr_event(port, VDM_EVT_RUN);
            break;
        case VDM_TASK_ALT_MODE:
            set_mngr_task(port, vdm_task_mngr_alt_mode_process(port, get_vdm_evt(port)));
            if (vdm_stat[port].vdm_task == VDM_TASK_SEND_MSG)
            {
                set_mngr_event(port, VDM_EVT_RUN);
            }
            if (vdm_stat[port].vdm_task == VDM_TASK_ALT_MODE)
            {
                set_mngr_event(port, VDM_EVT_RUN);
            }
            break;
        case VDM_TASK_EXIT:
            vdm_task_mngr_deinit(port);
            app_get_status(port)->vdm_prcs_failed = true;
            break;
        default:
            break;
    }
}

static bool vdm_retry_check(uint8_t port)
{
    bool ret = false;

    (vdm_stat[port].rec_retry_cnt)++;

    /* If we don't receive response more than rec_retry_cnt notify that fail */
    if (vdm_stat[port].rec_retry_cnt > MAX_RETRY_CNT)
    {
        set_vdm_failed(port);
    }
    else
    {
        /* Try to send msg again */
        set_mngr_task(port, VDM_TASK_SEND_MSG);

        /* Start a timer to delay the retry attempt. */
        timer_start(port, APP_VDM_BUSY_TIMER, APP_VDM_FAIL_RETRY_PERIOD, NULL);
        ret = true;
    }

    return ret;
}

static void set_vdm_failed(uint8_t port)
{
    /* Reset retry counters */
    vdm_stat[port].rec_retry_cnt = 0;
    set_mngr_event(port, VDM_EVT_FAIL);
    /* Set failed and save failed command */
    get_msg(port)->cmd = get_vdm_buff(port)->cmd_do->std_vdm_hdr.cmd;
}

/* Received VDM callback */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t* rec_vdm)
{
    uint32_t response;
    bool     run_task_flag = false;

    /* If response ACK */
    if (status == RES_RCVD)
    {
        response = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd_type;

#if CCG_PD_REV3_ENABLE
        if (rec_vdm->sop == SOP)
        {
            app_get_status(port)->vdm_version =
                GET_MIN (app_get_status(port)->vdm_version, rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.st_ver);
        }
#endif /* CCG_PD_REV3_ENABLE */

        /* Actual response received. */
        switch (response)
        {
            case CMD_TYPE_RESP_ACK:
                {
                    if (is_rec_vdm_expected(port, rec_vdm))
                    {
                        parse_vdm(port, rec_vdm);
                        set_mngr_event(port, VDM_EVT_EVAL);

                        /* Reset timer counter when ACK */
                        vdm_stat[port].rec_retry_cnt = 0;
                    }
                    else
                    {
                        /* Set failed and save failed command */
                        set_vdm_failed(port);
                    }

                    /* Continue the state machine operation. */
                    run_task_flag = true;
                }
                break;

            case CMD_TYPE_RESP_BUSY:
                /* Target is BUSY. */
                {
                    /* Check if command should be retried. We always have to continue task execution. */
                    vdm_stat[port].rec_retry_cnt++;
                    if (vdm_stat[port].rec_retry_cnt > MAX_RETRY_CNT)
                    {
                        set_vdm_failed(port);
                        run_task_flag = true;
                    }
                    else
                    {
                        /* Try to send msg again */
                        set_mngr_task(port, VDM_TASK_SEND_MSG);
                        /* Start a timer. Command will be retried when timer expires. */
                        timer_start(port, APP_VDM_BUSY_TIMER, APP_VDM_BUSY_TIMER_PERIOD, NULL);
                    }
                }
                break;
            default:
                /* Target NAK-ed the message. */
                {
                    /* Notify manager with failed event */
                    set_vdm_failed(port);
                    run_task_flag = true;
                }
                break;
        }
    }
    /* Attention related handler */
    else if (get_msg(port)->cmd == VDM_CMD_ATTENTION)
    {
        /* This statement need to notify alt mode that Attention VDM was successfuly sent */
        if ((status == CMD_SENT) && (get_vdm_mngr_task(port) == VDM_TASK_WAIT))
        {
            /* Start a timer. Command will be retried when timer expires. */
            timer_stop(port, APP_VDM_BUSY_TIMER);
            set_mngr_event(port, VDM_EVT_EVAL);
            /* Reset retry counter */
            vdm_stat[port].rec_retry_cnt = 0;
            /* Continue the state machine operation. */
            run_task_flag = true;
        }
        else if (status == SEQ_ABORTED)
        {
            /* Try to send msg again */
            run_task_flag = true;
        }
    }
    else
    {
        /* Good CRC not received or no response (maybe corrupted packet). */
        if ((status == CMD_FAILED) || (status == RES_TIMEOUT))
        {
            /* Check for retries. If failure persists after all retries, go to exit. */
            if (vdm_retry_check (port) == false)
            {
                run_task_flag = true;
            }
        }
        else
        {
            if (status == SEQ_ABORTED)
            {
                /* Try to send msg again */
                set_mngr_task(port, VDM_TASK_SEND_MSG);
            }
        }
    }

    /* Check if we need run any task */
    if (run_task_flag == true)
    {
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        switch (vdm_stat[port].vdm_msg.cmd)
        {
            case VDM_CMD_DSC_IDENTITY:
                set_mngr_task(port, VDM_TASK_DISC_ID);
                break;
            case VDM_CMD_DSC_SVIDS:
                set_mngr_task(port, VDM_TASK_DISC_SVID);
                break;
            default:
                set_mngr_task(port, VDM_TASK_ALT_MODE);
                break;
        }
#else /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
        set_mngr_task(port, VDM_TASK_ALT_MODE);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    }
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_task_t ret = VDM_TASK_EXIT;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover ID VDM packet. */
            get_msg(port)->svid = STD_SVID;
            get_msg(port)->sop_type = SOP;
            get_msg(port)->cmd = VDM_CMD_DSC_IDENTITY;
            get_msg(port)->obj_pos = NO_DATA;
            get_msg(port)->vdo_numb = NO_DATA;
            ret = VDM_TASK_SEND_MSG;
            break;

            /* Evaluate received response */
        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
                /* Check if SOP */
                if (get_msg(port)->sop_type == (uint8_t)SOP)
                {
                    /* If alt modes not supported. */
                    if (get_msg(port)->vdo[VDO_START_IDX - 1].std_id_hdr.mod_support == false)
                    {
                        break;
                    }

                    /* Set svid idx to zero before start DISC SVID */
                    vdm_stat[port].svid_idx  = NO_DATA;
                    vdm_stat[port].dsvid_cnt = 0;
                }

                /* Copy ID header to info struct */
                vdm_stat[port].atch_tgt.ama_vdo.val = get_msg(port)->vdo[PD_SVID_ID_HDR_VDO_START_IDX - 2].val;
                /* Copy AMA VDO to info struct */
                vdm_stat[port].atch_tgt.tgt_id_header.val = get_msg(port)->vdo[VDO_START_IDX - 1].val;
                /* Send Disc SVID cmd */
                set_mngr_event(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_mngr_event(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        case VDM_EVT_FAIL:
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_mngr_event(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        default:
            break;
    }

    return ret;
}

/* Checks if input SVID already saved. */
static bool is_svid_stored(uint16_t *svid_arr, uint16_t svid)
{
    uint8_t  idx;
    bool     stat = false;

    /* Go through all SVIDs and compare with input SVID */
    for (idx = 0; idx < MAX_SVID_VDO_SUPP; idx++)
    {
        /* If input SVID already saved */
        if (svid_arr[idx] == svid)
        {
            stat = true;
            break;
        }
    }

    return stat;
}

/*
   This function saves received Discover SVID resp,
   returns true if a NULL SVID was received.
 */
static bool save_svids(uint8_t port, uint16_t *svid_arr, uint8_t max_svid)
{
    uint8_t     idx, vdo_count;
    uint8_t     svid_idx = vdm_stat[port].svid_idx;
    uint16_t    svid;
    bool        retval = false;

    /* Compare received SVIDs with supported SVIDs */
    vdo_count = get_msg(port)->vdo_numb;

    /* Stop further discovery if this response does not have the maximum no. of DOs. */
    if (vdo_count < (MAX_NO_OF_DO - 1))
    {
        retval = true;
    }

    for (idx = 0; idx < (vdo_count * 2); idx++)
    {
        if ((idx & 1) == 0)
        {
            /* Upper half of the DO. */
            svid = get_msg(port)->vdo[idx >> 1].val >> 16u;
        }
        else
        {
            /* Lower half of the DO. */
            svid = get_msg(port)->vdo[idx >> 1].val & 0xFFFFu;
        }

        if (svid_idx < (max_svid - 1))
        {
            /* Stop on NULL SVID. */
            if (svid == NO_DATA)
            {
                retval = true;
            }
            else
            {
                /* If SVID not saved previously then save */
                if (is_svid_stored(svid_arr, svid) == false)
                {
#if SAVE_SUPP_SVID_ONLY
                    if (is_svid_supported(svid) != false)
#endif /* SAVE_SUPP_SVID_ONLY */
                    {
                        svid_arr[svid_idx] = svid;
                        svid_idx++;
                    }
                }
            }
        }
        else
        {
            /* Cannot continue as we have no more space. */
            retval = true;
            break;
        }
    }

    /* Set zero after last SVID in info */
    svid_arr[svid_idx] = NO_DATA;
    vdm_stat[port].svid_idx = svid_idx;

    /* Terminate discovery after MAX_DISC_SVID_COUNT attempts. */
    vdm_stat[port].dsvid_cnt++;
    if (vdm_stat[port].dsvid_cnt >= MAX_DISC_SVID_COUNT)
        retval = true;

    return retval;
}

static void set_disc_svid_param (uint8_t port, uint8_t sop)
{
    vdm_msg_info_t *msg_p = get_msg (port);

    msg_p->sop_type = sop;
    msg_p->svid     = STD_SVID;
    msg_p->cmd      = VDM_CMD_DSC_SVIDS;
    msg_p->obj_pos  = NO_DATA;
    msg_p->vdo_numb = NO_DATA;
}

static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_task_t ret = VDM_TASK_EXIT;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover SVID VDM packet */
            set_disc_svid_param (port, SOP);
            ret = VDM_TASK_SEND_MSG;
            break;

        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
                /* For attached target response */
                if (get_msg(port)->sop_type == (uint8_t)SOP)
                {
                    if (get_msg(port)->vdo[VDO_START_IDX - 1].val == NONE_VDO)
                    {
                        /* No SVID supported */
                        break;
                    }

                    /* Save received SVIDs and check if a NULL SVID was received. */
                    if (save_svids (port, vdm_stat[port].atch_tgt.tgt_svid, MAX_SVID_VDO_SUPP) != false)
                    {
                        /* If cable was detected send SOP' disc svid */
                        if ((dpm_get_info(port)->emca_present != false) &&
                                (dpm_get_info(port)->cbl_mode_en != false))
                        {
                            vdm_stat[port].svid_idx  = 0;
                            vdm_stat[port].dsvid_cnt = 0;
                            set_disc_svid_param (port, SOP_PRIME);
                            ret = VDM_TASK_SEND_MSG;
                        }
                        else
                        {
                            /* Goto register info in alt modes mngr */
                            set_mngr_event(port, VDM_EVT_RUN);
                            ret = VDM_TASK_REG_ATCH_TGT_INFO;
                        }
                    }
                    else
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        set_disc_svid_param (port, SOP);
                        ret = VDM_TASK_SEND_MSG;
                    }
                }
                /* For cable response */
                else
                {
                    if (get_msg(port)->vdo[VDO_START_IDX - 1].val != NONE_VDO)
                    {
                        /* Save received SVIDs and stop if a NULL SVID is received. */
                        if (save_svids(port, vdm_stat[port].atch_tgt.cbl_svid, MAX_CABLE_SVID_SUPP) != false)
                        {
                            set_mngr_event(port, VDM_EVT_RUN);
                            ret = VDM_TASK_REG_ATCH_TGT_INFO;
                        }
                        else
                        {
                            /* If not all SVID received, ask for the next set of SVIDs. */
                            set_disc_svid_param (port, SOP_PRIME);
                            ret = VDM_TASK_SEND_MSG;
                        }
                    }
                    else
                    {
                        /* Move to the next step. */
                        set_mngr_event(port, VDM_EVT_RUN);
                        ret = VDM_TASK_REG_ATCH_TGT_INFO;
                    }
                }
            }

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_mngr_event(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
            /* If cable SVID fails */
            if (get_msg(port)->sop_type == (uint8_t)SOP_PRIME)
            {
                set_mngr_event(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
            break;

        default:
            break;
    }

    return ret;
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static void reset_vdm_mngr(uint8_t port)
{
    vdm_stat[port].rec_retry_cnt = NO_DATA;
    vdm_stat[port].svid_idx = 0;
    vdm_stat[port].dsvid_cnt = 0;

    /* Clear arrays which hold SVIDs */
    memset(&vdm_stat[port].atch_tgt, 0, sizeof(atch_tgt_info_t));

    /* Store the pointer to the cable VDO discovered by PD stack. */
    vdm_stat[port].atch_tgt.cbl_vdo = &(dpm_get_info(port)->cbl_vdo);
}

void vdm_task_mngr_deinit(uint8_t port)
{
    /* CDT 247011 re-fix */
    app_get_status(port)->vdm_prcs_failed = false;
    /* If VDM task is not active, no need to go through the rest of the steps. */
    if (app_get_status(port)->vdm_task_en != false)
    {
        reset_vdm_mngr(port);
        app_get_status(port)->vdm_task_en = false;
        app_get_status(port)->cbl_disc_id_finished = false;
        app_get_status(port)->alt_mode_entered = false;
        /* Exit from alt mode manager */
        vdm_task_mngr_alt_mode_process(port, VDM_EVT_EXIT);

        /* Deinit App HW */
        alt_mode_hw_deinit(port);
    }
}


/* Fill DPM cmd buffer with properly VDM info */
static uint8_t compose_vdm(uint8_t port)
{
    dpm_pd_cmd_buf_t *vdm_buf;
    uint8_t idx;
    pd_do_t cmd_do;

    vdm_buf = get_vdm_buff(port);

    vdm_buf->cmd_sop = (get_msg(port)->sop_type);
    vdm_buf->no_of_cmd_do = (get_msg(port)->vdo_numb + VDO_START_IDX);

    cmd_do.val                 = 0;
    cmd_do.std_vdm_hdr.svid    = get_msg(port)->svid;
    if (vdm_buf->cmd_sop == SOP)
    {
        cmd_do.std_vdm_hdr.st_ver = app_get_status(port)->vdm_version;
    }
    else
    {
        cmd_do.std_vdm_hdr.st_ver = dpm_get_info(port)->cbl_vdm_version;
    }

    cmd_do.std_vdm_hdr.cmd     = get_msg(port)->cmd;
    cmd_do.std_vdm_hdr.obj_pos = get_msg(port)->obj_pos;
    cmd_do.std_vdm_hdr.vdm_type = VDM_TYPE_STRUCTURED;
    vdm_buf->cmd_do[VDM_HEADER_IDX].val = cmd_do.val;

    /* Set exceptions for Enter/Exit mode cmd period */
    switch (get_msg(port)->cmd)
    {
        case VDM_CMD_ENTER_MODE:
            vdm_buf->timeout = PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD;
            break;
        case VDM_CMD_EXIT_MODE:
            vdm_buf->timeout = PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD;
            break;
        case VDM_CMD_ATTENTION:
            /* No timeout required for attention messages. */
            vdm_buf->timeout = NO_DATA;
            break;
        default:
            vdm_buf->timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
            break;
    }

    if (get_msg(port)->vdo_numb > NO_DATA)
    {
        for (idx = 0; idx < get_msg(port)->vdo_numb; idx++)
        {
            vdm_buf->cmd_do[idx + VDO_START_IDX].val = get_msg(port)->vdo[idx].val;
        }
    }

    return true;
}

static bool is_rec_vdm_expected(uint8_t port, const pd_packet_t *rec_vdm)
{
    /* check command id compatibility */
    if ((rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd)
            == (get_vdm_buff(port)->cmd_do[VDM_HEADER_IDX].std_vdm_hdr.cmd))
    {
        return true;
    }

    return false;
}

/* Parse received VDM and save info in  */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm)
{
    uint8_t vdo_idx;

    vdm_stat[port].vdm_msg.svid = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid;
    vdm_stat[port].vdm_msg.sop_type = rec_vdm->sop;
    vdm_stat[port].vdm_msg.obj_pos = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos;
    vdm_stat[port].vdm_msg.cmd = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd;
    vdm_stat[port].vdm_msg.vdo_numb = (rec_vdm->len - VDO_START_IDX);

    /* If VDO is present in received VDM */
    if (rec_vdm->len > VDO_START_IDX)
    {
        for (vdo_idx = 0; vdo_idx < rec_vdm->len; vdo_idx++)
        {
            vdm_stat[port].vdm_msg.vdo[vdo_idx].val = rec_vdm->dat[vdo_idx + VDO_START_IDX].val;
        }
    }

    return true;
}

static vdm_task_t send_vdm(uint8_t port)
{
    ccg_status_t stat;

    if (vdm_stat[port].rec_retry_cnt == MAX_RETRY_CNT)
        app_get_status(port)->vdm_retry_pending = false;
    else
        app_get_status(port)->vdm_retry_pending = true;

    stat = dpm_pd_command (port, DPM_CMD_SEND_VDM, get_vdm_buff(port), vdm_rec_cbk);
    if (stat == CCG_STAT_SUCCESS)
    {
        return VDM_TASK_WAIT;
    }
    /* If fails - try to send VDM again */
    return VDM_TASK_SEND_MSG;
}

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
bool is_ufp_disc_started(uint8_t port)
{
    if (is_vdm_task_idle(port) == true)
    {
        /* Set VDM Task to send DISC ID VDM */
        set_mngr_task(port, VDM_TASK_DISC_ID);
        set_mngr_event(port, VDM_EVT_RUN);
        return true;
    }

    return false;
}
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

bool is_vdm_mngr_enabled(uint8_t port)
{
    return (bool)app_get_status(port)->vdm_task_en;
}

vdm_evt_t get_vdm_evt(uint8_t port)
{
    return vdm_stat[port].vdm_evt;
}

vdm_task_t get_vdm_mngr_task(uint8_t port)
{
    return vdm_stat[port].vdm_task;
}

void set_mngr_task(uint8_t port, vdm_task_t task)
{
    vdm_stat[port].vdm_task = task;
}

void set_mngr_event(uint8_t port, vdm_evt_t evt)
{
    vdm_stat[port].vdm_evt = evt;
}

static vdm_msg_info_t* get_msg(uint8_t port)
{
    return &vdm_stat[port].vdm_msg;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

/* [] END OF FILE */
