/**
 * @file alt_modes_mngr.c
 *
 * @brief @{Alternate Mode Manager source file.@}
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

#include "config.h"
#include <alt_modes_mngr.h>
#include <alt_modes_config.h>
#include <pd.h>
#include <dpm.h>
#include <app.h>
#include <vdm.h>
#include <timer.h>
#include <vdm_task_mngr.h>
#include <vdm.h>

#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

/**
 * @typedef alt_mode_status_t
 * @brief struct to hold alt modes manager status
 */
typedef struct
{
    /* Holds info when register alt mode */
    alt_mode_reg_info_t   reg_info;
    /* Supported alternate modes. */
    uint32_t              am_supported_modes;
    /* Exited alternate modes. */
    uint32_t              am_exited_modes;
    /* Active alternate modes. */
    uint32_t              am_active_modes;
    /* Pointers to each alt mode info structure */
    alt_mode_info_t*      alt_mode_info[MAX_SUPP_ALT_MODES];
    /* Number of existed alt modes */
    uint8_t               alt_modes_numb;
    /* Buffer to hold VDM */
    dpm_pd_cmd_buf_t      vdm_buf;
    /* Holds application event data */
    uint32_t              app_evt_data[ALT_MODE_EVT_SIZE];
    /* Current alt modes mngr status */
    alt_mode_mngr_state_t state;
    /* Pointer to vdm_msg_info_t struct in vdm task mngr */
    vdm_msg_info_t       *vdm_info;
    /* Hold current SVID index for discovery mode command */
    uint8_t               svid_idx;
    /* Check whether the device is a PD 3.0 supporting UFP. */
    uint8_t               pd3_ufp;
}alt_mode_mngr_status_t;

/*Main structure to hold alt modes manager status */
alt_mode_mngr_status_t alt_mode[NO_OF_TYPEC_PORTS];

/* Composes alt mode info to vdm_msg_info_t struct before sending */
static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);
/* Parces received VDM info and moves it to alt mode info struct */
static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);

#if DFP_ALT_MODE_SUPP
/* Find next avaliable alt mode for processing if previous exited */
static uint8_t get_next_alt_mode(uint8_t port);
#endif /* DFP_ALT_MODE_SUPP   */

/* Run alt mode callback function */
static vdm_task_t analyse_alt_mode(uint8_t port, uint8_t alt_mode_idx);

#if UFP_ALT_MODE_SUPP
/* This function verifies possibility of entering of corresponding UFP alt mode */
static bool is_mode_activated(uint8_t port, const pd_packet_t *vdm);
/* UFP function for alt modes processing */
static bool ufp_reg_alt_mode(uint8_t port, uint32_t svid);
#endif /* UFP_ALT_MODE_SUPP */

/* Returns number of supported alt modes */
static uint8_t get_alt_mode_numb(uint8_t port);
/* Returns alt mode index for given svid */
static uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid);
/* Resets alt mode mngr info */
static void reset_mngr_info(uint8_t port);
/* Check for presence of any alt modes in alt modes compatibility table*/
static uint8_t is_alt_modes_available(uint8_t port);
/* Returns alt mode index in compatibility table */
static uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id);

/* Fast access functions */
alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx);
alt_mode_state_t get_alt_mode_state(alt_mode_info_t* alt_mode_info_ptr);
vdm_msg_info_t* get_vdm_info(uint8_t port);
const atch_tgt_info_t* get_atch(uint8_t port);

/* Alt modes mngr AMS Prototypes */
static vdm_task_t run_disc_mode(uint8_t port);
static vdm_task_t eval_disc_mode(uint8_t port);
static vdm_task_t disc_mode_fail(uint8_t port);
static vdm_task_t monitor_alt_modes(uint8_t port);
static vdm_task_t eval_alt_modes(uint8_t port);
static vdm_task_t fail_alt_modes(uint8_t port);
static vdm_task_t alt_mode_mngr_deinit(uint8_t port);
static vdm_task_t wait_ec_trigger(uint8_t port);

/*State Table*/
static vdm_task_t (*const alt_mode_ams_table [ALT_MODE_MNGR_STATE_EXIT + 1]
        [VDM_EVT_EXIT + 1]) (uint8_t port) = {
    {
        /* Send next discovery svid */
        run_disc_mode,
        /* Evaluate disc svid response */
        eval_disc_mode,
        /* Process failed disc svid response */
        disc_mode_fail,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    },
    {
        /* Wait for EC triggering */
        wait_ec_trigger,
        wait_ec_trigger,
        wait_ec_trigger,
        alt_mode_mngr_deinit,
    },
    {
        /* Monitor if any changes appears in modes  */
        monitor_alt_modes,
        /* Evaluate alt mode response */
        eval_alt_modes,
        /* Process failed alt modes response */
        fail_alt_modes,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    },
    {
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit,
        alt_mode_mngr_deinit,
        alt_mode_mngr_deinit,
        alt_mode_mngr_deinit
    }
};

/************************* DFP Related Function definitions *******************/

vdm_task_t reg_alt_mode_mngr(uint8_t port, atch_tgt_info_t* atch_tgt_info, vdm_msg_info_t* vdm_msg_info)
{
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_live = (bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3);
#endif

    alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);
    alt_mode[port].vdm_info       = vdm_msg_info;

    /* Check device role to start with. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
            /* UFP does not support any of the SVIDs of interest. Exit VDM manager. */
            return VDM_TASK_EXIT;
        }
        else
        {
            if (is_alt_modes_available(port) != false)
            {
                /* Register pointers to VDM mngr info */
                alt_mode[port].reg_info.atch_tgt_info = atch_tgt_info;

                /* Set alt modes mngr state to Discovery Mode process */
                alt_mode[port].reg_info.data_role = PRT_TYPE_DFP;
                alt_mode[port].state              = ALT_MODE_MNGR_STATE_DISC_MODE;

                /* Set alt mode trigger based on config */
                app_get_status(port)->alt_mode_trigger = get_pd_port_config(port)->dp_mode_trigger;
                return VDM_TASK_ALT_MODE;
            }
        }
    }
    else
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        alt_mode[port].pd3_ufp = pd3_live;
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
            /* If we have no SVIDs to evaluate by UFP then go to regular monitoring */
            alt_mode[port].vdm_info = vdm_msg_info;
            alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
            return VDM_TASK_ALT_MODE;
        }
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        else
        {
            if (pd3_live)
            {
                /* If PD spec revision is 3.0, we can start with discover mode process. */
                return VDM_TASK_ALT_MODE;
            }
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
    }

    return VDM_TASK_EXIT;
}

bool is_alt_mode_mngr_idle(uint8_t port)
{
    bool    is_idle = true;
    uint8_t alt_mode_idx;

    /* If alt mode mngr waits for EC trigger to enter alt mode - idle */
    if (alt_mode[port].state == ALT_MODE_MNGR_WAIT_EC_TRIGGER)
        return is_idle;

    if (alt_mode[port].state == ALT_MODE_MNGR_STATE_DISC_MODE)
        return false;
    if (!alt_mode_hw_is_idle(port))
        return false;

    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            /* If alt mode not Idle then return current alt mode state */
            if ((get_mode_info(port, alt_mode_idx)->mode_state != ALT_MODE_STATE_IDLE) ||
                    (get_mode_info(port, alt_mode_idx)->app_evt_needed != false))
            {
                is_idle = false;
            }
        }
    }

    return (is_idle);
}

void alt_mode_mngr_sleep(uint8_t port)
{
    alt_mode_hw_sleep (port);
}

void alt_mode_mngr_wakeup(uint8_t port)
{
    alt_mode_hw_wakeup (port);
}

vdm_task_t vdm_task_mngr_alt_mode_process(uint8_t port, vdm_evt_t vdm_evt)
{
    /* Run alt modes manager ams table */
    return alt_mode_ams_table[alt_mode[port].state][vdm_evt](port);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static void set_disc_mode_params(uint8_t port, sop_t sop)
{
    vdm_msg_info_t *vdm_p = get_vdm_info(port);

    vdm_p->svid     = get_atch(port)->tgt_svid[alt_mode[port].svid_idx];
    vdm_p->sop_type = sop;
    vdm_p->cmd      = VDM_CMD_DSC_MODES;
    vdm_p->obj_pos  = 0;
    vdm_p->vdo_numb = 0;
}

static void send_sln_event_nodata(uint8_t port, uint16_t svid, uint8_t am_id, alt_mode_app_evt_t evtype)
{
    sln_pd_event_handler (port, APP_EVT_ALT_MODE,
            form_alt_mode_event (port, svid, am_id, evtype, NO_DATA)
            );
}

static void send_sln_app_evt(uint8_t port, uint32_t data)
{
    if (data == NO_DATA)
    {
        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                form_alt_mode_event (port, get_vdm_info(port)->svid,
                    alt_mode[port].reg_info.alt_mode_id,
                    alt_mode[port].reg_info.app_evt, NO_DATA)
                );
    }
    else
    {
        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                form_alt_mode_event (port, get_vdm_info(port)->svid,
                    alt_mode[port].reg_info.alt_mode_id,
                    AM_EVT_DATA_EVT, data)
                );
    }
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t run_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    uint16_t cur_svid;

    /* Set cable sop flag not needed at start*/
    alt_mode[port].reg_info.cbl_sop_flag = SOP_INVALID;

    /* Search for next SVID until svid array is not empty */
    while ((cur_svid = get_atch(port)->tgt_svid[alt_mode[port].svid_idx]) != 0)
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        /* Check is current port date role UFP */
        if (alt_mode[port].pd3_ufp)
        {
            /* Send Disc Mode cmd */
            set_disc_mode_params (port, SOP);
            return VDM_TASK_SEND_MSG;
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

#if DFP_ALT_MODE_SUPP
        if (is_svid_supported(cur_svid) != false)
        {
            /* Send notifications to the solution. */
            send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_SUPP);

            /* If SVID is supported send Disc Mode cmd */
            set_disc_mode_params (port, SOP);
            return VDM_TASK_SEND_MSG;
        }

#if (SAVE_SUPP_SVID_ONLY == 0)
        /* Send notifications to the solution. */
        send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_NOT_SUPP);
#endif /* SAVE_SUPP_SVID_ONLY */

        /* If svid not supported */
        alt_mode[port].svid_idx++;
#endif /* DFP_ALT_MODE_SUPP */
    }

    if (alt_mode[port].am_supported_modes == NONE_MODE_MASK)
    {
        /* No supp modes */
        return VDM_TASK_EXIT;
    }

    /* Send SVID discovery finished notification to the solution. */
    send_sln_event_nodata (port, 0, 0, AM_EVT_DISC_FINISHED);

    if (app_get_status(port)->alt_mode_trigger)
    {
        alt_mode[port].state = ALT_MODE_MNGR_WAIT_EC_TRIGGER;
    }
    else
    {
#if DFP_ALT_MODE_SUPP
        /* Goto alt mode process */
        get_next_alt_mode(port);
#endif /* DFP_ALT_MODE_SUPP */
        alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
    }

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
static void handle_cbl_disc_mode(uint8_t port, bool failed)
{
    alt_mode_info_t *alt_mode_ptr;
    uint8_t tbl_svid_idx, tbl_alt_mode_idx;
    uint8_t vdo_idx;

    /*
       Get index of function in alt_mode_config related to receive SVID.
       This is expected to be valid as we would already have gone through
       Discover MODE for the UFP.
     */
    tbl_svid_idx = get_alt_mode_svid_idx(port, get_vdm_info(port)->svid);
    alt_mode[port].reg_info.atch_type = CABLE;

    /* Analyse all received VDOs. */
    for (vdo_idx = 0; ((failed) || (vdo_idx < get_vdm_info(port)->vdo_numb)); vdo_idx++)
    {
        if (failed)
        {
            alt_mode[port].reg_info.cbl_sop_flag = SOP_INVALID;
        }
        else
        {
            /* Save current VDO and its position in svid structure */
            alt_mode[port].reg_info.svid_emca_vdo = get_vdm_info(port)->vdo[vdo_idx];
        }

        /* Check if DFP support attached target alt mode */
        alt_mode_ptr = is_alt_mode_allowed[tbl_svid_idx] (port, &alt_mode[port].reg_info);
        if (alt_mode_ptr == NULL)
        {
            /* Get index of alt mode in the compatibility table */
            tbl_alt_mode_idx = get_alt_mode_tbl_idx(port, get_vdm_info(port)->svid,
                    alt_mode[port].reg_info.alt_mode_id);
            if (tbl_alt_mode_idx != MODE_NOT_SUPPORTED)
            {
                /* Remove pointer to alt mode info struct */
                alt_mode[port].alt_mode_info[tbl_alt_mode_idx] = NULL;
                /* Remove flag that alt mode could be runned */
                REMOVE_FLAG(alt_mode[port].am_supported_modes, tbl_alt_mode_idx);
            }
        }
        else
        {
            if (!failed)
                alt_mode_ptr->cbl_obj_pos = (vdo_idx + VDO_START_IDX);
        }

        if (alt_mode[port].reg_info.app_evt != AM_NO_EVT)
        {
            /* Send notifications to the solution. */
            send_sln_app_evt (port, NO_DATA);
            alt_mode[port].reg_info.app_evt = AM_NO_EVT;
        }

        if (failed)
            break;
    }
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t eval_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    uint8_t             vdo_idx, tbl_svid_idx, tbl_alt_mode_idx;
    alt_mode_info_t*    alt_mode_ptr;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    /* Evaluate SOP response */
    if (get_vdm_info(port)->sop_type == (uint8_t)SOP)
    {
        /* Get index of function in alt_mode_config related to receive SVID */
        tbl_svid_idx = get_alt_mode_svid_idx(port, get_vdm_info(port)->svid);
        /* Additional check if we support current SVID operations */
        if (tbl_svid_idx != MODE_NOT_SUPPORTED)
        {
            alt_mode[port].reg_info.atch_type = ATCH_TGT;
            /* Analyse all rec VDOs */
            for (vdo_idx = 0; vdo_idx < get_vdm_info(port)->vdo_numb; vdo_idx++)
            {
                /* Save current VDO and its position in svid structure */
                alt_mode[port].reg_info.svid_vdo = get_vdm_info(port)->vdo[vdo_idx];
                /* Check if DFP support attached target alt mode */
                alt_mode_ptr = is_alt_mode_allowed[tbl_svid_idx]
                    (port, &alt_mode[port].reg_info);
                /* If VDO relates to any of supported alt modes */
                if (alt_mode[port].reg_info.alt_mode_id != MODE_NOT_SUPPORTED)
                {
                    if (alt_mode[port].reg_info.app_evt != AM_NO_EVT)
                    {
                        /* Send notifications to the solution. */
                        send_sln_app_evt(port, NO_DATA);
                        alt_mode[port].reg_info.app_evt = AM_NO_EVT;
                    }

                    /* If alternate modes discovered and could be runned */
                    if (alt_mode_ptr != NULL)
                    {
                        /* Save alt mode ID and object position */
                        alt_mode_ptr->alt_mode_id = alt_mode[port].reg_info.alt_mode_id;
                        alt_mode_ptr->obj_pos = (vdo_idx + VDO_START_IDX);

                        if (alt_mode_ptr[port].app_evt_needed != false)
                        {
                            /* Send notifications to the solution. */
                            send_sln_app_evt (port, alt_mode_ptr[port].app_evt_data.val);
                            alt_mode_ptr[port].app_evt_needed = false;
                        }

                        /* Get index of alt mode in the compatibility table */
                        tbl_alt_mode_idx = get_alt_mode_tbl_idx(port, get_vdm_info(port)->svid,
                                alt_mode[port].reg_info.alt_mode_id);
                        if (tbl_alt_mode_idx != MODE_NOT_SUPPORTED)
                        {
                            /* Save pointer to alt mode info struct */
                            alt_mode[port].alt_mode_info[tbl_alt_mode_idx] = alt_mode_ptr;
                            /* Set flag that alt mode could be runned */
                            SET_FLAG(alt_mode[port].am_supported_modes, tbl_alt_mode_idx);
                        }
                    }
                }
            }

            /* If cable DISC Mode is needed - send VDM */
            if (alt_mode[port].reg_info.cbl_sop_flag != SOP_INVALID)
            {
                set_disc_mode_params (port, SOP_PRIME);
                return VDM_TASK_SEND_MSG;
            }
        }
    }
    /* Evaluate cable response: Packet type will be SOP' or SOP'' here. */
    else
    {
        handle_cbl_disc_mode(port, false);

        /* If cable DISC Mode is needed - send VDM */
        if (alt_mode[port].reg_info.cbl_sop_flag != SOP_INVALID)
        {
            set_disc_mode_params (port, SOP_DPRIME);
            return VDM_TASK_SEND_MSG;
        }
    }

    /* If no any result goto next SVID */
    alt_mode[port].svid_idx++;

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    return VDM_TASK_ALT_MODE;
}

static vdm_task_t disc_mode_fail(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    if (get_vdm_info(port)->sop_type == (uint8_t)SOP_PRIME)
    {
        handle_cbl_disc_mode(port, true);
    }

    /* If Disc Mode cmd fails goto next SVID */
    alt_mode[port].svid_idx++;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

static vdm_task_t monitor_alt_modes(uint8_t port)
{
    uint8_t          alt_mode_idx;
    alt_mode_state_t alt_mode_state;
    vdm_task_t       stat = VDM_TASK_ALT_MODE;
    alt_mode_info_t  *am_info_p = NULL;

    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);

            /* Get alt mode state */
            alt_mode_state = get_alt_mode_state(am_info_p);
            switch (alt_mode_state)
            {
                case ALT_MODE_STATE_SEND:
                    /* This case activates when VDM sequence for given alt mode */
                    /* was interrupted by alt mode with higher priority */
                case ALT_MODE_STATE_WAIT_FOR_RESP:

                    /* Check if SOP' is needed */
                    if (am_info_p->sop_state[SOP_PRIME] == ALT_MODE_STATE_SEND)
                    {
                        /* Copy to vdm info and send vdm */
                        move_to_vdm_info(port, am_info_p, SOP_PRIME);
                    }
                    else if (am_info_p->sop_state[SOP_DPRIME] == ALT_MODE_STATE_SEND)
                    {
                        /* Copy to vdm info and send vdm */
                        move_to_vdm_info(port, am_info_p, SOP_DPRIME);
                    }
                    else
                    {
                        /* Copy to vdm info and send vdm */
                        move_to_vdm_info(port, am_info_p, SOP);
                    }
                    am_info_p->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
                    stat = VDM_TASK_SEND_MSG;
                    break;

                case ALT_MODE_STATE_EXIT:
#if DFP_ALT_MODE_SUPP
                    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
                    {
                        /* Remove flag from active mode */
                        REMOVE_FLAG(alt_mode[port].am_active_modes, alt_mode_idx);
                        /* Set alt mode as disabled */
                        am_info_p->mode_state = ALT_MODE_STATE_DISABLE;
                        /* Set flag as exited mode */
                        SET_FLAG(alt_mode[port].am_exited_modes, alt_mode_idx);

                        /* If any active modes are present */
                        if (alt_mode[port].am_active_modes == NONE_MODE_MASK)
                        {
                            /* Notify APP layer that ALT mode has been exited. */
                            app_get_status(port)->alt_mode_entered = false;
                            /* Set MUX to SS config */
                            set_mux(port, MUX_CONFIG_SS_ONLY, NO_DATA);

                            /* If EC choose next alt mode or if no other alt modes are supported, wait for EC trigger. */
                            if ((app_get_status(port)->alt_mode_trigger) || (get_next_alt_mode(port) == false))
                            {
                                alt_mode[port].state = ALT_MODE_MNGR_WAIT_EC_TRIGGER;
                            }
                        }
                    }
#endif /* DFP_ALT_MODE_SUPP   */
#if UFP_ALT_MODE_SUPP
                    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                    {
                        if (alt_mode[port].am_active_modes != NONE_MODE_MASK)
                        {
                            /* Notify APP layer that ALT mode has been exited. */
                            app_get_status(port)->alt_mode_entered = false;

                            /*
                             * TODO: Need to identify if billboard needs to be shown
                             * on alternate mode exit. By spec billboard is required
                             * only during the connection.
                             */
                        }

                        /* Send notifications to the solution if alt mode was entered. */
                        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                form_alt_mode_event (port,
                                    am_info_p->vdm_header.std_vdm_hdr.svid,
                                    am_info_p->alt_mode_id,
                                    AM_EVT_ALT_MODE_EXITED, NO_DATA));

                        /* Set alt mode not active */
                        am_info_p->is_active = false;
                        /* Remove flag that alt mode could be processed */
                        REMOVE_FLAG(alt_mode[port].am_active_modes, alt_mode_idx);
                    }
#endif /* UFP_ALT_MODE_SUPP   */
                    stat = VDM_TASK_ALT_MODE;
                    break;

                case ALT_MODE_STATE_IDLE:
                    /* If alt modes need to send app event data */
                    if (am_info_p->app_evt_needed != false)
                    {
                        /* Send notifications to the solution. */
                        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                form_alt_mode_event (port,
                                    am_info_p->vdm_header.std_vdm_hdr.svid,
                                    am_info_p->alt_mode_id,
                                    AM_EVT_DATA_EVT, am_info_p->app_evt_data.val));
                        am_info_p->app_evt_needed = false;
                    }
                    break;

                case ALT_MODE_STATE_RUN:
                    /* Run ufp evaluation function */
                    am_info_p->cbk(port);
                    stat = VDM_TASK_ALT_MODE;
                    break;

                default:
                    break;
            }
        }
    }

    return stat;
}

static vdm_task_t eval_alt_modes(uint8_t port)
{
    uint8_t            alt_mode_idx;
    alt_mode_state_t   alt_mode_state;
    vdm_task_t         stat = VDM_TASK_ALT_MODE;
    alt_mode_app_evt_t appevt_type = AM_NO_EVT;
    alt_mode_info_t    *am_info_p  = NULL;
    uint32_t           appevt_data = NO_DATA;

    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);

            /* Get alt mode state */
            alt_mode_state = get_alt_mode_state(am_info_p);
            switch (alt_mode_state)
            {
                case ALT_MODE_STATE_WAIT_FOR_RESP:
                    /* Set flag that send transaction passed successful */
                    am_info_p->sop_state[get_vdm_info(port)->sop_type] = ALT_MODE_STATE_IDLE;
                    /* Copy received resp to alt mode info struct */
                    get_vdm_info_vdo(port, am_info_p, get_vdm_info(port)->sop_type);

                    /* If received VDM is SOP */
                    if (get_vdm_info(port)->sop_type == SOP)
                    {
                        /* Run alt mode analisys function */
                        stat = analyse_alt_mode(port, alt_mode_idx);

                        /* If alt mode entered */
                        if (get_vdm_info(port)->cmd == VDM_CMD_ENTER_MODE)
                        {
                            /* Notify APP layer that ALT mode has been entered. */
                            app_get_status(port)->alt_mode_entered = true;
                            /* Set mode as active */
                            am_info_p->is_active = true;

                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_ALT_MODE_ENTERED;
                        }

                        if (get_vdm_info(port)->cmd == VDM_CMD_EXIT_MODE)
                        {
                            /* Set mode as not active */
                            am_info_p->is_active = false;

                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_ALT_MODE_EXITED;
                        }

                        /* If alt modes need to send app event data */
                        if (am_info_p->app_evt_needed != false)
                        {
                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_DATA_EVT;
                            appevt_data = am_info_p->app_evt_data.val;

                            am_info_p->app_evt_needed = false;
                        }

                        /* Send event to solution space, if required. */
                        if (appevt_type != AM_NO_EVT)
                        {
                            sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                    form_alt_mode_event (port,
                                        am_info_p->vdm_header.std_vdm_hdr.svid,
                                        am_info_p->alt_mode_id,
                                        appevt_type, appevt_data));
                        }
                    }
                    else
                    {
                        /* If received VDM is SOP' type, check if SOP'' is needed.
                           If received VDM is SOP'', the check on SOP_DPRIME will fail trivially. */
                        if (am_info_p->sop_state[SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed send SOP'' VDM */
                            move_to_vdm_info(port,am_info_p, SOP_DPRIME);
                            stat = VDM_TASK_SEND_MSG;
                        }
                        else if (am_info_p->sop_state[SOP] == ALT_MODE_STATE_SEND)
                        {
                            /* If SOP'' not needed - send SOP VDM */
                            move_to_vdm_info(port,am_info_p, SOP);
                            stat = VDM_TASK_SEND_MSG;
                        }
                        else
                        {
                            stat = analyse_alt_mode(port, alt_mode_idx);
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }

    return stat;
}

static vdm_task_t fail_alt_modes(uint8_t port)
{
    uint8_t             alt_mode_idx;
    alt_mode_state_t    alt_mode_state;
    alt_mode_app_evt_t  appevt_type = AM_EVT_CBL_RESP_FAILED;
    alt_mode_info_t     *am_info_p = NULL;

    /* Look through all alt modes */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);

            /* Get alt mode state */
            alt_mode_state = get_alt_mode_state(am_info_p);
            /* If mode waits for response */
            if (alt_mode_state == ALT_MODE_STATE_WAIT_FOR_RESP)
            {
                /* Change status to fail */
                am_info_p->sop_state[get_vdm_info(port)->sop_type] = ALT_MODE_STATE_FAIL;

                if (get_vdm_info(port)->sop_type == SOP)
                {
                    appevt_type = AM_EVT_SOP_RESP_FAILED;
                }

                /* Send notifications to the solution. */
                sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                        form_alt_mode_event (port,
                            am_info_p->vdm_header.std_vdm_hdr.svid,
                            am_info_p->alt_mode_id, appevt_type, NO_DATA));

                /* Run alt mode analisys function */
                am_info_p->mode_state = ALT_MODE_STATE_FAIL;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                }
            }
        }
    }

    return VDM_TASK_ALT_MODE;
}

static vdm_task_t wait_ec_trigger(uint8_t port)
{
    return VDM_TASK_ALT_MODE;
}

#if DFP_ALT_MODE_SUPP
uint8_t is_svid_supported(uint16_t svid)
{
    uint8_t     idx, am_numb;

    am_numb = sizeof(dfp_compatibility_mode_table) / sizeof(dfp_compatibility_mode_table[0]);

    /* Look through all alt modes */
    for (idx = 0; idx < am_numb; idx++)
    {
        /* if alt mode with given index is supported by CCG */
        if (dfp_compatibility_mode_table[idx][idx].svid == svid)
        {
            return true;
        }
    }

    return false;
}

static uint8_t get_next_alt_mode(uint8_t port)
{
    uint32_t idx;

    /* Set active modes bit map to zero */
    alt_mode[port].am_active_modes = NONE_MODE_MASK;

    /* Start looking for next supported alt modes */
    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
        /* If mode supported by CCG and not processed yet */
        if (IS_FLAG_CHECKED(((alt_mode[port].am_supported_modes) & (~alt_mode[port].am_exited_modes)), idx))
        {
            /* Check if this entry has a valid SVID. */
            if (dfp_compatibility_mode_table[idx][idx].svid != NO_DATA)
            {
                SET_FLAG(alt_mode[port].am_active_modes, idx);
            }

            return true;
        }
    }

    return false;
}

#endif /* DFP_ALT_MODE_SUPP   */

static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t *alt_mode_vdm, sop_t sop_type)
{
    /* Copy info from alt mode to vdm manager */
    get_vdm_info(port)->cmd = alt_mode_vdm->vdm_header.std_vdm_hdr.cmd;
    get_vdm_info(port)->sop_type = sop_type;
    get_vdm_info(port)->svid = alt_mode_vdm->vdm_header.std_vdm_hdr.svid;
    get_vdm_info(port)->vdo_numb = NO_DATA;
    get_vdm_info(port)->obj_pos = alt_mode_vdm->obj_pos;
    if (sop_type != SOP)
    {
        get_vdm_info(port)->obj_pos = alt_mode_vdm->cbl_obj_pos;
    }
    if ((alt_mode_vdm->vdo[sop_type] != NULL) && (alt_mode_vdm->vdo_numb[sop_type] != NO_DATA))
    {

        get_vdm_info(port)->vdo_numb = alt_mode_vdm->vdo_numb[sop_type];
        /* Save received VDO */
        memcpy(get_vdm_info(port)->vdo, alt_mode_vdm->vdo[sop_type],
                (alt_mode_vdm->vdo_numb[sop_type]) << 2);
    }

    return true;
}

static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type)
{
    uint8_t vdo_numb;

    /* Copy object position to the VDM Header */
    alt_mode_vdm->vdm_header.std_vdm_hdr.obj_pos = get_vdm_info(port)->obj_pos;

    vdo_numb = get_vdm_info(port)->vdo_numb;
    /* Copy received VDO to alt mode info */
    if ((vdo_numb != 0) && (alt_mode_vdm->vdo[sop_type] != NULL))
    {
        alt_mode_vdm->vdo_numb[sop_type] = get_vdm_info(port)->vdo_numb;
        /* Save Rec VDO */
        if (get_vdm_info(port)->vdo_numb <= alt_mode_vdm->vdo_max_numb)
        {
            /* Save received VDO */
            memcpy(alt_mode_vdm->vdo[sop_type], get_vdm_info(port)->vdo,
                    (get_vdm_info(port)->vdo_numb) << 2);
        }
    }
}

static vdm_task_t analyse_alt_mode(uint8_t port, uint8_t alt_mode_idx)
{
    /* If pointer to cbk is not NULL thrn run */
    if (get_mode_info(port, alt_mode_idx)->cbk != NULL)
    {
        get_mode_info(port, alt_mode_idx)->cbk(port);
    }
    return VDM_TASK_ALT_MODE;
}

static vdm_task_t alt_mode_mngr_deinit(uint8_t port)
{
    uint8_t             idx;

    /* Find and reset alt modes info structures */
    for (idx = 0; idx <= alt_mode[port].alt_modes_numb; idx++)
    {
        if (alt_mode[port].alt_mode_info[idx] != NULL)
        {
            /* If current data role is UFP - set mode to idle */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                /* CDT 232310 fix */
               get_mode_info(port, idx)->mode_state = ALT_MODE_STATE_IDLE;
            }
            else
            {
                /* If DFP - wait for response */
                get_mode_info(port, idx)->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
            }

            /* Exit from alt mode */
            alt_mode[port].alt_mode_info[idx]->vdm_header.std_vdm_hdr.cmd = VDM_CMD_EXIT_MODE;
            alt_mode[port].alt_mode_info[idx]->cbk(port);
            /* Reset alt mode info */
            reset_alt_mode_info(get_mode_info(port, idx));
        }
    }

    /* Reset alt mode mngr info */
    reset_mngr_info(port);

#if (CCG_BB_ENABLE != 0)
    bb_disable(port, true);
    bb_update_all_status(port, BB_ALT_MODE_STATUS_INIT_VAL);
#endif /* (CCG_BB_ENABLE != 0) */

    return VDM_TASK_EXIT;
}


/******************** Common ALT Mode DFP and UFP functions *******************/

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))

static uint8_t get_alt_mode_numb(uint8_t port)
{
    uint8_t am_numb = NO_DATA;

#if DFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        am_numb = sizeof(dfp_compatibility_mode_table) / sizeof(dfp_compatibility_mode_table[0]);
    }
#endif /* DFP_ALT_MODE_SUPP */

#if UFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        am_numb = sizeof(ufp_compatibility_mode_table) / sizeof(ufp_compatibility_mode_table[0]);
    }
#endif /* UFP_ALT_MODE_SUPP */

    return am_numb;
}

static uint8_t is_alt_modes_available(uint8_t port)
{
    uint8_t idx;

#if DFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* if any alt mode is avaliable */
            if (dfp_compatibility_mode_table[idx][idx].svid != NO_DATA)
            {
                /* Set svid idx to zero to start Disc Mode process */
                alt_mode[port].svid_idx = 0;
                return true;
            }
        }
    }
#endif /* DFP_ALT_MODE_SUPP */
#if UFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* if any alt mode is avaliable */
            if (ufp_compatibility_mode_table[idx][idx].svid != NO_DATA)
            {
                return true;
            }
        }
    }
#endif /* UFP_ALT_MODE_SUPP */

    return false;
}

static uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id)
{
    uint8_t idx;

#if DFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* if any alt mode is avaliable */
            if ((dfp_compatibility_mode_table[idx][idx].svid == svid) &&
                    (dfp_compatibility_mode_table[idx][idx].alt_mode_id == id))
            {
                return idx;
            }
        }
    }
#endif
#if UFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* if any alt mode is avaliable */
            if ((ufp_compatibility_mode_table[idx][idx].svid == svid) &&
                    (ufp_compatibility_mode_table[idx][idx].alt_mode_id == id))
            {
                return idx;
            }
        }
    }
#endif

    return MODE_NOT_SUPPORTED;
}

static uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid)
{
    uint8_t idx = MODE_NOT_SUPPORTED;

    /* Look through all alt modes */
    for (idx = 0; idx <= alt_mode[port].alt_modes_numb; idx++)
    {
        /* if alt mode with given index is supported by CCG */
        if (supp_svid_tbl[idx] == svid)
        {
            return idx;
        }
    }
    return idx;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

void reset_alt_mode_info(alt_mode_info_t *info)
{
    memset(info, 0, sizeof(*info));
}

void reset_mngr_info(uint8_t port)
{
    memset(&alt_mode[port], 0, sizeof(alt_mode[port]));
}

dpm_pd_cmd_buf_t* get_vdm_buff(uint8_t port)
{
    return &(alt_mode[port].vdm_buf);
}

/******************* ALT MODE Solution Related Functions ****************************/

#if ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE))

bool eval_app_alt_mode_cmd(uint8_t port_idx, uint8_t *cmd, uint8_t *data)
{
    alt_mode_evt_t  cmd_info;
    alt_mode_info_t *am_info_p = NULL;
    uint32_t        cmd_data;
    uint8_t         alt_mode_idx;
    bool            found = false;

    /* Convert received cmd bytes as info and data */
    cmd_info = (alt_mode_evt_t)MAKE_DWORD(cmd[3], cmd[2], cmd[1], cmd[0]);
    cmd_data = MAKE_DWORD(data[3], data[2], data[1], data[0]);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check if received app command is start discover process for UFP when PD 3.0 supported */
    if (
            (alt_mode[port_idx].pd3_ufp) &&
            (cmd_info.alt_mode_event.data_role == PRT_TYPE_UFP) &&
            (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_RUN_UFP_DISC)
       )
    {
        /* Try to start Discovery process if VDM manager is not busy  */
        return is_ufp_disc_started(port_idx);
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    /* Look for the alternate mode entry which matches the SVID and alt mode id. */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port_idx].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is supported. */
        if (IS_FLAG_CHECKED(alt_mode[port_idx].am_supported_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info (port_idx, alt_mode_idx);
            if ((am_info_p->vdm_header.std_vdm_hdr.svid == cmd_info.alt_mode_event.svid) &&
                    (am_info_p->alt_mode_id == cmd_info.alt_mode_event.alt_mode))
            {
                found = true;
                break;
            }
        }
    }

#if DFP_ALT_MODE_SUPP
    if ((gl_dpm_port_type[port_idx] != PRT_TYPE_UFP) && (cmd_info.alt_mode_event.data_role != PRT_TYPE_UFP))
    {
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_EN_TRIG)
        {
            app_get_status(port_idx)->alt_mode_trigger = true;
            return true;
        }

        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_DIS_TRIG)
        {
            app_get_status(port_idx)->alt_mode_trigger = false;
            return true;
        }

        /* Check if Enter command and trigger is set */
        if ((cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_ENTER) &&
                (alt_mode[port_idx].state == ALT_MODE_MNGR_WAIT_EC_TRIGGER))
        {
            /* If we found an alternate mode entry for this {SVID, ID} pair. */
            if (found)
            {
                if (am_info_p->cbk != NULL)
                {
                    /* Set flag as active mode */
                    SET_FLAG(alt_mode[port_idx].am_active_modes, alt_mode_idx);
                    REMOVE_FLAG(alt_mode[port_idx].am_exited_modes, alt_mode_idx);

                    /* Inits alt mode */
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(port_idx);

                    /* Goto alt mode processing */
                    alt_mode[port_idx].state = ALT_MODE_MNGR_STATE_PROCESS;
                    return true;
                }
            }
            return false;
        }
    }
#endif /* DFP_ALT_MODE_SUPP */

    if ((found) && (am_info_p->is_active == true) && (am_info_p->mode_state == ALT_MODE_STATE_IDLE))
    {
        /* If received cmd is specific alt mode command */
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_SPEC)
        {
            return (am_info_p->eval_app_cmd(port_idx, (alt_mode_evt_t)cmd_data));
        }
#if DFP_ALT_MODE_SUPP
        else if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_EXIT)
        {
            if (gl_dpm_port_type[port_idx] != PRT_TYPE_UFP)
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port_idx);
                    return true;
                }
            }
        }
#endif /* DFP_ALT_MODE_SUPP */
    }

    return false;
}

#endif /* ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE)) */

const uint32_t* form_alt_mode_event(uint8_t port, uint16_t svid, uint8_t am_idx, alt_mode_app_evt_t evt, uint32_t data)
{
    alt_mode_evt_t temp;

    temp.alt_mode_event.svid = (uint32_t)svid;
    temp.alt_mode_event.alt_mode = (uint32_t)am_idx;
    temp.alt_mode_event.alt_mode_evt = (uint32_t)evt;
    temp.alt_mode_event.data_role = (uint32_t)gl_dpm_port_type[port];
    alt_mode[port].app_evt_data[ALT_MODE_EVT_IDX] = temp.val;
    alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = 0;

    if (data != 0)
    {
        alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = data;
    }

    return alt_mode[port].app_evt_data;
}


const atch_tgt_info_t* get_atch(uint8_t port)
{
    return alt_mode[port].reg_info.atch_tgt_info;
}

vdm_msg_info_t* get_vdm_info(uint8_t port)
{
    return alt_mode[port].vdm_info;
}

alt_mode_state_t get_alt_mode_state(alt_mode_info_t* alt_mode_info_ptr)
{
    return (alt_mode_state_t)alt_mode_info_ptr->mode_state;
}

alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx)
{
    return alt_mode[port].alt_mode_info[alt_mode_idx];
}

#if UFP_ALT_MODE_SUPP

static bool ufp_reg_alt_mode(uint8_t port, uint32_t svid)
{
    uint8_t idx, vdo_idx, vdo_numb, tbl_alt_mode_idx = 0;
    alt_mode_info_t* alt_mode_ptr;
    pd_do_t* dobj;

    /* Check if any alt mode is supported by DFP */
    if (is_alt_modes_available(port) == true)
    {
        /* Get index of related svid register function */
        idx = get_alt_mode_svid_idx(port, svid);
        /* If SVID supported by CCG */
        if (idx != MODE_NOT_SUPPORTED)
        {
            alt_mode[port].reg_info.data_role = PRT_TYPE_UFP;
            /* Get SVID related VDOs and number of VDOs */
            if (get_modes_vdo_info (port, svid, &dobj, &vdo_numb) == false)
            {
                return false;
            }
            for (vdo_idx = VDO_START_IDX; vdo_idx < vdo_numb; vdo_idx++)
            {
                /* Save Disc mode VDO and object position */
                alt_mode[port].reg_info.svid_vdo = dobj[vdo_idx];
                /* Check if UFP support attached target alt mode */
                alt_mode_ptr = is_alt_mode_allowed[idx](port, &alt_mode[port].reg_info);
                /* If VDO relates to any of supported alt modes */
                if (alt_mode[port].reg_info.alt_mode_id != MODE_NOT_SUPPORTED)
                {
                    /* If alternate modes discovered and could be runned */
                    if (alt_mode_ptr != NULL)
                    {
                        /* Get index of alt mode in the compatibility table */
                        tbl_alt_mode_idx = get_alt_mode_tbl_idx(port, svid,
                                alt_mode[port].reg_info.alt_mode_id);
                        if (tbl_alt_mode_idx != MODE_NOT_SUPPORTED)
                        {
                            /* Save alt mode ID and obj position */
                            alt_mode_ptr->alt_mode_id = alt_mode[port].reg_info.alt_mode_id;
                            alt_mode_ptr->obj_pos = vdo_idx;
                            /* Save pointer to alt mode info struct */
                            alt_mode[port].alt_mode_info[tbl_alt_mode_idx] = alt_mode_ptr;
                            /* Set flag that alt mode could be runned */
                            SET_FLAG(alt_mode[port].am_supported_modes, tbl_alt_mode_idx);
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

static bool is_mode_activated(uint8_t port, const pd_packet_t *vdm)
{
    uint8_t      idx;

    /* If any alt mode already registered */
    if (alt_mode[port].am_supported_modes != NONE_MODE_MASK)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* Try to find alt mode among supported alt modes */
            if (
                    (alt_mode[port].alt_mode_info[idx] != NULL) &&
                    (get_mode_info(port, idx)->vdm_header.std_vdm_hdr.svid ==
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                    (get_mode_info(port, idx)->obj_pos ==
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos)
               )
            {
                /* return true if ufp alt mode already registered in alt mode mngr */
                return true;
            }
        }
    }

    /* Register alt mode and check possibility of alt mode entering
     * CDT 237168 */
    if (ufp_reg_alt_mode(port, vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) == true)
    {
        return true;
    }

    return false;
}

#endif /* UFP_ALT_MODE_SUPP */

bool eval_rec_vdm(uint8_t port, const pd_packet_t *vdm)
{
    uint8_t    idx;

#if UFP_ALT_MODE_SUPP
    uint8_t    idx2;
    bool       enter_flag = false;
#endif /* UFP_ALT_MODE_SUPP */

    alt_mode_info_t *am_info_p = NULL;

    /* Discovery commands not processed by alt modes manager */
    if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd < VDM_CMD_ENTER_MODE)
    {
        return false;
    }

    /* Save number of available alt modes */
    alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);

#if UFP_ALT_MODE_SUPP
    /* If Enter mode cmd */
    if (
            (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_ENTER_MODE) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP)
       )
    {
        if (is_mode_activated(port, vdm) == true)
        {
            enter_flag = true;
        }
    }
#endif /* UFP_ALT_MODE_SUPP */

    /* Go throught all alt modes */
    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
        am_info_p = get_mode_info(port, idx);

        /* Check if received command processing allowed */
        if (
                (am_info_p != NULL) &&
                (am_info_p->vdm_header.std_vdm_hdr.svid == vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                (am_info_p->obj_pos == vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos)
           )
        {
#if UFP_ALT_MODE_SUPP
            /* If Enter mode cmd */
            if (enter_flag == true)
            {
                /* If alt mode already active then ACK */
                if (am_info_p->is_active == true)
                {
                    return true;
                }

                /* If all alt modes not active and just entered  */
                if (alt_mode[port].am_active_modes == NONE_MODE_MASK)
                {
                    /* Save cmd */
                    am_info_p->vdm_header.std_vdm_hdr.cmd = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd;
                    am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;

                    if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                    {
                        /* Save received VDO */
                        memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                                (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);
                    }

                    /* Run ufp alt mode cbk */
                    am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                    am_info_p->cbk(port);

                    if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                    {
                        /* Set mode as active */
                        am_info_p->is_active = true;

                        /* Notify APP layer that ALT mode has been entered. */
                        app_get_status(port)->alt_mode_entered = true;

                        /* Send notifications to the solution if alt mode entered. */
                        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                form_alt_mode_event (port,
                                    am_info_p->vdm_header.std_vdm_hdr.svid,
                                    am_info_p->alt_mode_id,
                                    AM_EVT_ALT_MODE_ENTERED, NO_DATA));

#if (CCG_BB_ENABLE != 0)
                        if (bb_is_present(port) != false)
                        {
                            /* Disable AME timeout timer. */
                            timer_stop(port, APP_AME_TIMEOUT_TIMER);
                            /* Update BB alt mode status register as successful config */
                            bb_update_alt_status(port, idx, BB_ALT_MODE_STAT_SUCCESSFUL);
                            /* Enable BB controller */
                            bb_enable(port, BB_CAUSE_AME_SUCCESS);
                        }
#endif /* (CCG_BB_ENABLE != 0) */

                        /* Set flag that alt mode could be processed */
                        SET_FLAG(alt_mode[port].am_active_modes, idx);
                        return true;
                    }

#if (CCG_BB_ENABLE != 0)
                    if (bb_is_present(port) != false)
                    {
                        /* Disable AME timeout timer. */
                        timer_stop(port, APP_AME_TIMEOUT_TIMER);
                        /* Update BB alt mode status register as Error */
                        bb_update_alt_status(port, idx, BB_ALT_MODE_STAT_UNSUCCESSFUL);
                        /* Enable BB controller */
                        bb_enable(port, BB_CAUSE_AME_FAILURE);
                    }
#endif /* (CCG_BB_ENABLE != 0) */
                    return false;
                }
                else
                {
                    /* Check alt mode in ufp consistent table  */
                    for (idx2 = 0; idx2 < alt_mode[port].alt_modes_numb; idx2++)
                    {
                        /* find table index of any active alt mode */
                        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, idx2))
                        {
                            /* If alt mode consistent with active alt mode */
                            if (ufp_compatibility_mode_table[idx2][idx].svid != NO_DATA)
                            {
                                /* Save cmd */
                                am_info_p->vdm_header.std_vdm_hdr.cmd = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd;
                                am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;
                                /* Save Rec VDO */
                                if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                                {
                                    /* Copy received VDO */
                                    memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                                            (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);
                                }
                                /* Run ufp alt mode cbk */
                                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                                am_info_p->cbk(port);
                                if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                                {
                                    /* Set mode as active */
                                    am_info_p->is_active = true;
                                    /* Set flag that alt mode could be processed */
                                    SET_FLAG(alt_mode[port].am_active_modes, idx);
                                    return true;
                                }
                                return false;
                            }
                        }
                    }
                    return false;
                }
            }
            /* Any other received command */
            else
#endif /* UFP_ALT_MODE_SUPP */
            {
                /* Check if alt mode is active */
                if (am_info_p->is_active == false)
                {
                    return false;
                }

                /* Save cmd */
                am_info_p->vdm_header.std_vdm_hdr.cmd = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd;
                am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;
                /* Save Rec VDO */
                if ((vdm->len > VDO_START_IDX) &&
                        (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                {
                    /* Save received VDO */
                    memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                            ((vdm->len - VDO_START_IDX) * PD_WORD_SIZE));
                }

                /* Set alt mode state as idle */
                alt_mode[port].alt_mode_info[idx]->mode_state = ALT_MODE_STATE_IDLE;
                /* Run alt mode cbk */
                am_info_p->cbk(port);

                /* If command processed successful */
                if (alt_mode[port].alt_mode_info[idx]->mode_state != ALT_MODE_STATE_FAIL)
                {
                    /* Set number of data objects */
                    app_get_status(port)->vdm_resp.do_count = am_info_p->vdo_numb[SOP] + VDO_START_IDX;
                    if (am_info_p->vdo_numb[SOP] != NO_DATA)
                    {
                        /* If VDO resp is needed */
                        memcpy( &(app_get_status(port)->vdm_resp.resp_buf[VDO_START_IDX]),
                                am_info_p->vdo[SOP],
                                (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);

                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        /* If Exit all modes */
        if (
                (am_info_p->vdm_header.std_vdm_hdr.svid == vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                (am_info_p->is_active) &&
                (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos == EXIT_ALL_MODES) &&
                (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_EXIT_MODE)
            )
        {
            /* Save cmd */
            am_info_p->vdm_header.std_vdm_hdr.cmd = EXIT_ALL_MODES;
            /* Run ufp alt mode cbk */
            am_info_p->cbk(port);
            return true;
        }
    }

    return false;
}

/* [] END OF FILE */

