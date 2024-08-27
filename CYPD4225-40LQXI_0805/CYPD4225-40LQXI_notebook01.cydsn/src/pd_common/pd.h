/**
 * @file pd_common\pd.h
 *
 * @brief @{Common definitions and structures used in the CCG USB-PD stack.@}
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

#ifndef _PD_H_
#define _PD_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <utils.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/********************************* PD macros **********************************/

/* Externally powered bit position in Source PDO mask. */
#define PD_EXTERNALLY_POWERED_BIT_POS       (7u)

/*
 * Config Table OCP/OVP macros.
 * Byte 136: Protection schemes (OVP/OCP) enable.
 * Bit 0 => OCP method: 1=External, 0=UVP
 * Bit 1 => OCP enable
 * Bit 2 => OVP enable
 */
#define CFG_TABLE_OCP_METHOD_MASK           (0x1u)
#define CFG_TABLE_OCP_EN_MASK               (0x2u)
#define CFG_TABLE_OVP_EN_MASK               (0x4u)

#define GIVE_BACK_MASK                      (0x8000u)
#define SNK_MIN_MAX_MASK                    (0x3FFu)

#define GET_DR_SWAP_RESP(resp)              ((resp) & 0x3u)
#define GET_PR_SWAP_RESP(resp)              (((resp) & 0xCu) >> 2u)
#define GET_VCONN_SWAP_RESP(resp)           (((resp) & 0x30u) >> 4u)

#define APP_RESP_ACCEPT                     (0u)
#define APP_RESP_REJECT                     (1u)
#define APP_RESP_WAIT                       (2u)
#define APP_RESP_NOT_SUPPORTED              (3u)

/* Macro to skip length check in pd_is_msg API. */
#define LENGTH_CHECK_SKIP                   (0xFFFF)

/* Maximum retries of Source Capability. */
#define MAX_SRC_CAP_COUNT                   (50u)

/* Maximum hard reset retry count. */
#define MAX_HARD_RESET_COUNT                (3u)

/* nDiscoverId to cable. */
#define MAX_CBL_DSC_ID_COUNT                (20u)

/* Maximum number of DOs in a packet. */
#define MAX_NO_OF_DO                        (7u)
/* Maximum number of PDOs in a packet. */
#define MAX_NO_OF_PDO                       (MAX_NO_OF_DO)
/* Maximum number of VDOs in a packet. */
#define MAX_NO_OF_VDO                       (MAX_NO_OF_DO)

/* Rx buffer indices. */
#define VDM_HEADER_IDX                      (0u)
#define BDO_HDR_IDX                         (0u)
#define ID_HEADER_IDX                       (1u)
#define RDO_IDX                             (0u)

/* Index of cable info in Discover Identity response message. */
#define CBL_VDO_INDEX                       (4u)
#define CBL_VDO_COUNT                       (5u)

/* Protocol revision. */
#define PD_REV                              (1u) /* Version 2.0. */
#define PD_REV_POS                          (6u)
#define TX_SOP_GD_CRC_HDR_DFLT              ((PD_REV << PD_REV_POS) | 0x0001u)
#define TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT ((TX_SOP_GD_CRC_HDR_DFLT <<16u) |TX_SOP_GD_CRC_HDR_DFLT)

/* Maximum extended message size in bytes. */
#define MAX_EXTD_PKT_SIZE                   (260u)

/* Maximum extended message 32-bit words. Each word is 32 bit. */
#define MAX_EXTD_PKT_WORDS                  (65u)

/* Maximum legacy Extended message size in bytes. */
#define MAX_EXTD_MSG_LEGACY_LEN             (26u)

/* Maximum PD words in one packet. */
#define MAX_PD_PKT_WORDS                    (8u)
/* Size of each PD word in bytes. */
#define PD_WORD_SIZE                        (4u)
/* Maximum bytes in a PD packet. */
#define MAX_PD_PKT_BYTES                    (MAX_PD_PKT_WORDS * PD_WORD_SIZE)

/* PD Header Message id is 3 bits. */
#define MAX_MESSAGE_ID                      (7u)

/*
 * PD message header macros.
 */

/* Macro to form a PD header. */
#define PD_HEADER(type, id, cnt)            ((type) | (PD_REV << 6) | \
                                            ((id) << 9) | ((cnt) << 12))

#define PD_HEADER_REV3(type, id, cnt, ext)  ((type) | ((id) << 9) | \
                                            ((cnt) << 12) | ((ext) << 15))

/* Macro to update DR and PR role in PD header. */
#define PD_DR_PR_ROLE(data_role,pwr_role)   (((pwr_role) << 8) | ((data_role) << 5))

/* Return message count from header. */
#define GET_PD_HDR_CNT(header)              (((header) >> 12) & 0x7)

/* Return PR role bit from header. */
#define GET_PD_HDR_PR_ROLE(header)          (((header) >> 8) & 0x1)

/* Return Cable Plug bit from header. */
#define GET_PD_HDR_CBL_PLUG(header)         (((header) >> 8) & 0x1)

/* Return spec revision from header. */
#define GET_PD_HDR_SPEC_REV(header)         (((header) >> 6) & 0x3)

/* Return DR role bit from header. */
#define GET_PD_HDR_DR_ROLE(header)          (((header) >> 5) & 0x1)

/* Return message ID from header. */
#define GET_PD_HDR_ID(header)               (((header) >> 9) & 0x7)

/* Return Message Type from header. */
#define GET_PD_HDR_TYPE(header)             ((header) & 0xF)

/* PD header mask. */
#define PD_HDR_MASK                         (0x0000FFFFu)

/* Message ID mask. */
#define MSG_ID_MASK                         ((0x7u) << 9u)

/* Return complete PD header Rx buffer[0]. */
#define GET_PD_HDR(buf0)                    ((buf0) & PD_HDR_MASK)

/*
 * Max SOP types excluding hard reset, cable reset, SOP_PDEBUG and SOP_DPDEBUG.
 */
#define MAX_SOP_TYPES                       (3)

/* RDO macros. */
#define GET_RDO_OBJ_POS(rdo)                (((rdo) >> 28) & 0x7)
#define GET_RDO_GV_BACK(rdo)                (((rdo) >> 27) & 0x1)
#define GET_RDO_CAP_MIS(rdo)                (((rdo) >> 26) & 0x1)
#define GET_RDO_USB_COM(rdo)                (((rdo) >> 25) & 0x1)
#define GET_RDO_NO_SSPND(rdo)               (((rdo) >> 24) & 0x1)
#define GET_RDO_OP_CUR(rdo)                 (((rdo) >> 10) & 0x3FF)
#define GET_RDO_OP_PWR(rdo)                 (((rdo) >> 10) & 0x3FF)
#define GET_RDO_MAX_OP_CUR(rdo)             (((rdo)) & 0x3FF)
#define GET_RDO_MIN_OP_CUR(rdo)             (((rdo)) & 0x3FF)
#define GET_RDO_MAX_OP_PWR(rdo)             (((rdo)) & 0x3FF)
#define GET_RDO_MIN_OP_PWR(rdo)             (((rdo)) & 0x3FF)

/* VDM macros. */
#define STD_SVID                            (0xFF00u)
#define DP_SVID                             (0xFF01u)
#define CY_VID                              (0x04B4u)
#define TBT_SVID                            (0x8087u)

#define GET_VID(vdm_hdr)                    ((vdm_hdr) >> 16)
#define GET_VDM_TYPE(vdm_hdr)               (((vdm_hdr) >> 15) & 0x1)
#define GET_SVDM_VDM_VER(vdm_hdr)           (((vdm_hdr) >> 13) & 0x3)
#define GET_SVDM_OBJ_POS(vdm_hdr)           (((vdm_hdr) >> 8) & 0x7)
#define GET_SVDM_CMD_TYPE(vdm_hdr)          (((vdm_hdr) >> 6) & 0x3)
#define GET_SVDM_CMD(vdm_hdr)               (((vdm_hdr)) & 0x1F)

#define STD_VDM_VERSION_IDX                 (13u)
   
#define STD_VDM_VERSION_REV3                     (1u)
#define STD_VDM_VERSION_REV2                     (0u)
#define STD_VDM_VERSION                          (0u)

#define STD_VDM_HEADER_IDENTITY_REQ                                             \
                                            (                                   \
                                                (STD_SVID << 16) |              \
                                                (VDM_TYPE_STRUCTURED << 15) |   \
                                                (VDM_CMD_DSC_IDENTITY)          \
                                            )

#define STD_VDM_HEADER_SVID_REQ                                                 \
                                            (                                   \
                                                (STD_SVID << 16) |              \
                                                (VDM_TYPE_STRUCTURED << 15) |   \
                                                (VDM_CMD_DSC_SVIDS)             \
                                            )

/* BIST macros. */
#define GET_BIST_MODE(bist_hdr)             ((bist_hdr) >> 28)

/* Message masks. */
#define CTRL_MSG_RSRVD_MASK                 (0x1 << 0)
#define CTRL_MSG_GOOD_CRC_MASK              (0x1 << 1)
#define CTRL_MSG_GO_TO_MIN_MASK             (0x1 << 2)
#define CTRL_MSG_ACCEPT_MASK                (0x1 << 3)
#define CTRL_MSG_REJECT_MASK                (0x1 << 4)
#define CTRL_MSG_PING_MASK                  (0x1 << 5)
#define CTRL_MSG_PS_RDY_MASK                (0x1 << 6)
#define CTRL_MSG_GET_SOURCE_CAP_MASK        (0x1 << 7)
#define CTRL_MSG_GET_SINK_CAP_MASK          (0x1 << 8)
#define CTRL_MSG_DR_SWAP_MASK               (0x1 << 9)
#define CTRL_MSG_PR_SWAP_MASK               (0x1 << 10)
#define CTRL_MSG_VCONN_SWAP_MASK            (0x1 << 11)
#define CTRL_MSG_WAIT_MASK                  (0x1 << 12)
#define CTRL_MSG_SOFT_RESET_MASK            (0x1 << 13)
#define CTRL_MSG_NOT_SUPPORTED_MASK         (0x1 << 16)
#define CTRL_MSG_GET_SRC_CAP_EXTD_MASK      (0x1 << 17)
#define CTRL_MSG_GET_STATUS_MASK            (0x1 << 18)
#define CTRL_MSG_FR_SWAP_MASK               (0x1 << 19)
#define DATA_MSG_SRC_CAP_MASK               (0x1 << 1)
#define DATA_MSG_REQUEST_MASK               (0x1 << 2)
#define DATA_MSG_BIST_MASK                  (0x1 << 3)
#define DATA_MSG_SNK_CAP_MASK               (0x1 << 4)
#define DATA_MSG_BAT_STATUS_MASK            (0x1 << 5)
#define DATA_MSG_SRC_ALERT_MASK             (0x1 << 6)
#define DATA_MSG_VDM_MASK                   (0x1 << 15)
#define EXTD_MSG_SRC_CAP_EXTD_MASK          (0x1 << 1)
#define EXTD_MSG_STATUS_MASK                (0x1 << 2)
#define EXTD_MSG_GET_BAT_CAP_MASK           (0x1 << 3)
#define EXTD_MSG_GET_BAT_STATUS_MASK        (0x1 << 4)
#define EXTD_MSG_BAT_CAP_MASK               (0x1 << 5)
#define EXTD_MSG_GET_MANU_INFO_MASK         (0x1 << 6)
#define EXTD_MSG_MANU_INFO_MASK             (0x1 << 7)
#define EXTD_MSG_SECURITY_REQ_MASK          (0x1 << 8)
#define EXTD_MSG_SECURITY_RESP_MASK         (0x1 << 9)
#define EXTD_MSG_FW_UPDATE_REQ_MASK         (0x1 << 10)
#define EXTD_MSG_FW_UPDATE_RESP_MASK        (0x1 << 11)    

/*
 * EMCA macros.
 */

/* Cable capabilities (in 10mA units). */
#define CBL_CAP_0A                          (0u)        /* 0A   */
#define CBL_CAP_3A                          (300u)      /* 3A   */
#define CBL_CAP_5A                          (500u)      /* 5A   */
/*
 * The default cable current capability. This value should not be
 * modified. If the default setting needs to be modified, use the
 * dpm_update_def_cable_cap() function.
 */
#define DEFAULT_CBL_CAP                     (CBL_CAP_3A)

/* VBUS macros in 50mV units. */
#define VSAFE_0V                            (16u)       /* 0.8V */
#define VSAFE_5V                            (100u)      /* 5V   */
#define VSAFE_9V                            (180u)      /* 9V   */
#define VSAFE_12V                           (240u)      /* 12V  */
#define VSAFE_13V                           (260u)      /* 13V  */
#define VSAFE_15V                           (300u)      /* 15V  */
#define VSAFE_19V                           (380u)      /* 19V  */
#define VSAFE_20V                           (400u)      /* 20V  */
#define VSAFE_0V_PR_SWAP_SNK_SRC            (60u)       /* 3V   */
#define VSAFE_0V_HARD_RESET                 (60u)       /* 3V   */

/* Voltage unit in PDOs (50mV). */
#define PD_VOLT_PER_UNIT                    (50u)

/* Current macros in 10mA units. */
#define I_SAFE_0mA                          (0u)
#define ISAFE_DEF                           (50u)

/* VBUS margins. */
#define VSAFE_0V_SNK_MARGIN                 (0)
#define VSAFE_0V_PR_SWAP_SNK_SRC_MARGIN     (0)
#define VSAFE_0V_HARD_RESET_MARGIN          (0)

#define VSAFE_0V_SRC_MARGIN             (-50)
#define VSAFE_0V_SRC_TURN_ON_MARGIN     (0)
#define VSAFE_5V_SNK_TURN_ON_MARGIN     (-20)
#define VSAFE_5V_SNK_TURN_OFF_MARGIN    (-27)
#define VSAFE_SNK_TURN_OFF_MARGIN       (-20)
#define VSAFE_5V_SRC_TURN_ON_MARGIN     (-20)

#define VSAFE_5V_FRS_SWAP_RX_MARGIN     (10)
#define VSAFE_5V_FRS_SWAP_TX_MARGIN     (10)    

/* PE events macros. */
#define PE_EVT_NONE                         (0x00000000u)
#define PE_EVT_HARD_RESET_RCVD              (0x00000001u)
#define PE_EVT_SOFT_RESET_RCVD              (0x00000002u)
#define PE_EVT_ENTRY                        (0x00000004u)
#define PE_EVT_TX_SUCCESS                   (0x00000008u)
#define PE_EVT_TX_DISCARDED                 (0x00000010u)
#define PE_EVT_TX_FAIL                      (0x00000020u)
#define PE_EVT_PKT_RCVD                     (0x00000040u)
#define PE_EVT_PWR_RDY                      (0x00000080u)
#define PE_EVT_TIMEOUT                      (0x00000100u)
#define PE_EVT_DPM_CMD_RCVD                 (0x00000200u)
#define PE_EVT_APP_RESP_RCVD                (0x00000400u)
#define PE_EVT_VDM_RESP_RCVD                (0x00000800u)
#define PE_EVT_CABLE_TIMEOUT                (0x00001000u)
#define PE_EVT_NO_RESPONSE_TIMEOUT          (0x00002000u)
#define PE_EVT_FR_SIGNAL_RCVD               (0x00004000u)
#define PE_EVT_FR_SIGNAL_SENT               (0x00008000u)
#define PE_EVT_ALL_MASK                     (0xFFFFFFFFu)

/******************************* Type-C macros ********************************/

/**
 * Minimum DRP toggling period, in ms. See Table 4-16 of the Type-C spec Rev1.
 */
#define DRP_TOGGLE_PERIOD                   (75u)

/**
 * Minimum percentage of DRP period for a source. See Table 4-16 of the Type-C
 * spec Rev1.
 */
#define SRC_DRP_MIN_DC                      (30)

#define CC_CHANNEL_1                        (0u)
#define CC_CHANNEL_2                        (1u)

/* Rp combinations macros. */
#define RP_CC1_OPEN_CC2_OPEN                (((RP_OPEN) << 8)|RP_OPEN)
#define RP_CC1_OPEN_CC2_RD                  (((RP_RD) << 8)|RP_OPEN)
#define RP_CC1_OPEN_CC2_RA                  (((RP_RA) << 8)|RP_OPEN)
#define RP_CC1_RD_CC2_OPEN                  (((RP_OPEN) << 8)|RP_RD)
#define RP_CC1_RA_CC2_OPEN                  (((RP_OPEN) << 8)|RP_RA)
#define RP_CC1_RA_CC2_RD                    (((RP_RD) << 8)|RP_RA)
#define RP_CC1_RA_CC2_RA                    (((RP_RA) << 8)|RP_RA)
#define RP_CC1_RD_CC2_RA                    (((RP_RA) << 8)|RP_RD)
#define RP_CC1_RD_CC2_RD                    (((RP_RD) << 8)|RP_RD)

/* Rd combinations macros. */
#define RD_CC1_RA_CC2_USB                   (((RD_USB) << 8)|RD_RA)
#define RD_CC1_RA_CC2_1_5A                  (((RD_1_5A) << 8)|RD_RA)
#define RD_CC1_RA_CC2_3A                    (((RD_3A) << 8)|RD_RA)
#define RD_CC1_USB_CC2_RA                   (((RD_RA) << 8)|RD_USB)
#define RD_CC1_1_5A_CC2_RA                  (((RD_RA) << 8)|RD_1_5A)
#define RD_CC1_3A_CC2_RA                    (((RD_RA) << 8)|RD_3A)
#define RD_CC1_RA_CC2_RA                    (((RD_RA) << 8)|RD_RA)
#define RD_CC1_ERR_CC2_ERR                  (((RD_ERR) << 8)|RD_ERR)

/* Type-C events macros. */
#define TYPEC_EVT_NONE                      (0x00000000u)
#define TYPEC_EVT_ERR_RECOVERY              (0x00000001u)
#define TYPEC_EVT_ENTRY                     (0x00000002u)
#define TYPEC_EVT_TIMEOUT                   (0x00000004u)
#define TYPEC_EVT_DETACH                    (0x00000008u)
#define TYPEC_EVT_ATTACH                    (0x00000010u)
#define TYPEC_EVT_PWR_RDY                   (0x00000020u)
#define TYPEC_EVT_DPM_CMD_RCVD              (0x00000040u)
#define TYPEC_EVT_ALL_MASK                  (0xFFFFFFFFu)

/* TypeC_FSM enable macros. */
#define TYPEC_FSM_NONE                      (0x00000000u)
#define TYPEC_FSM_GENERIC                   (0x00000001u)

#define TYPEC_FSM_ALL_MASKS                 (0xFFFFFFFFu)

/************************* Stack timers and timeouts **************************/

/*
 * PD Timers.
 *
 * Timer ID and period settings, in milliseconds, of various timers. Period
 * should not be zero.
 * These timer IDs should not be changed.
 * Timer values are as per Section 6.5 of USB PD spec Rev2.0 v1.2
 */
#define PD_TIMERS_START_ID                  (0u)
#define PD_TIMERS_END_ID                    (14u)

#define TYPEC_TIMERS_START_ID               (15u)
#define TYPEC_TIMERS_END_ID                 (24u)

#define APP_TIMERS_START_ID                 (30u)

#define PD_CABLE_TIMER                      (1u)
#define PD_NO_RESPONSE_TIMER                (2u)
#define PD_CBL_DSC_ID_TIMER                 (3u)
#define PD_CBL_DELAY_TIMER                  (4u)
#define PD_PHY_BUSY_TIMER                   (5u)
#define PD_GOOD_CRC_TX_TIMER                (6u)
#define PD_HARD_RESET_TX_TIMER              (7u)
#define PD_VCONN_SWAP_INITIATOR_TIMER       (8u)
#define PD_GENERIC_TIMER                    (9u)
    
#define PD_SINK_TX_TIMER                    (11u)
    
#define PD_OCP_DEBOUNCE_TIMER               (25u)

#define HPD_RX_ACTIVITY_TIMER_ID            (26u)
#define HPD_RX_ACTIVITY_TIMER_PERIOD_MIN    (5u)
#define HPD_RX_ACTIVITY_TIMER_PERIOD_MAX    (105u)

/* See Section 6.5.7 of USBPD Spec Rev2 v1.2 */    
#define PD_NO_RESPONSE_TIMER_PERIOD         (5000u)
    
/* See Section 6.5.15 of USBPD Spec Rev2 v1.2 */     
#define PD_CBL_DSC_ID_TIMER_PERIOD          (49u)
#define PD_CBL_DSC_ID_START_TIMER_PERIOD    (43u)
    
/* See Section 6.5.14 of USBPD Spec Rev2 v1.2 */      
#define PD_CBL_DELAY_TIMER_PERIOD           (2u)

/* This timer used internally by stack to prevent tx lockup */    
#define PD_PHY_BUSY_TIMER_PERIOD            (15u)
    
/* See Section 6.3.13 of USBPD Spec Rev2 v1.2 */      
#define PD_HARD_RESET_TX_TIMER_PERIOD       (20u)
    
/* This timer used by stack to do auto retry VCONN swap before PR swap (if DUT is sink)
 * Minimum gap between VCONN swap request shall be a minimum 100ms, to be safe 110ms
 * is used
 */    
#define PD_VCONN_SWAP_INITIATOR_TIMER_PERIOD (110u)
    
/* See Table 7-22 of USBPD Spec Rev2 v1.2 */    
#define PD_VBUS_TURN_ON_TIMER_PERIOD        (275u)
#define PD_VBUS_TURN_OFF_TIMER_PERIOD       (625u)
    
/* See Section 6.5.6.1 of USBPD Spec Rev2 v1.2 */      
#define PD_PS_SRC_TRANS_TIMER_PERIOD        (400u)

/* See Section 6.5.6.2 of USBPD Spec Rev2 v1.2 */     
#define PD_PS_SRC_OFF_TIMER_PERIOD          (900u)
    
/* See Section 6.5.6.3 of USBPD Spec Rev2 v1.2 */      
#define PD_PS_SRC_ON_TIMER_PERIOD           (450u)
    
/* See Section 6.5.6.1 of USBPD Spec Rev2 v1.2 */     
#define PD_PS_SNK_TRANSITION_TIMER_PERIOD   (500u)
    
/* See Section 7.6.1 of USBPD Spec Rev2 v1.2 */     
#define PD_SRC_RECOVER_TIMER_PERIOD         (750u)
    
/* See Section 6.5.2 of USBPD Spec Rev2 v1.2 */    
#define PD_SENDER_RESPONSE_TIMER_PERIOD     (27u)
    
/* See Section 6.5.2 of USBPD Spec Rev2 v1.2 */     
#define PD_RECEIVER_RESPONSE_TIMER_PERIOD   (15u)
    
/* See Section 6.5.4.2 of USBPD Spec Rev2 v1.2 */     
#define PD_SINK_WAIT_CAP_TIMER_PERIOD       (400u)
    
/* See Section 6.5.4.1 of USBPD Spec Rev2 v1.2 */     
#define PD_SRC_CAP_TIMER_PERIOD             (110u)

/* See Section 6.5.9.2 of USBPD Spec Rev2 v1.2 */     
#define PD_SWAP_SRC_START_TIMER_PERIOD      (55u)
    
 /* See Table 7-22 of USBPD Spec Rev2 v1.2 */    
#define PD_SOURCE_TRANSITION_TIMER_PERIOD   (28u)
 
/* See Section 6.5.13 of USBPD Spec Rev2 v1.2 */     
#define PD_VCONN_OFF_TIMER_PERIOD           (25u)
#define PD_VCONN_ON_TIMER_PERIOD            (100u)
    
/* This is internal timer used to monitor VCONN if it is turned on*/    
#define PD_VCONN_TURN_ON_TIMER_PERIOD       (10u)

/* This timer is the delay between PD startup and sending cable Disocver ID request
 * to ensure cable is ready to respond */    
#define PD_CBL_READY_TIMER_PERIOD           (50u)

/* See Section 6.5.12.1 of USBPD Spec Rev2 v1.2 */ 
#define PD_VDM_RESPONSE_TIMER_PERIOD        (25u)

/* See Section 6.5.12.2 of USBPD Spec Rev2 v1.2 */     
#define PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD (45u)
    
/* See Section 6.5.12.3 of USBPD Spec Rev2 v1.2 */     
#define PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD  (45u)
    
/* See Section 6.5.12.1 of USBPD Spec Rev2 v1.2 
 * This timer is slightly relaxed
 */ 
#define PD_DPM_RESP_REC_RESP_PERIOD         (20u)
    
/* See Section 6.5.8.4 of USBPD Spec Rev2 v1.2 */     
#define PD_BIST_CONT_MODE_TIMER_PERIOD      (55u)

/* These timers are used by Sink to monitor VBUS during Hard reset
 * These values are derived by understanding  Table 7-22
 */     
#define PD_SINK_VBUS_TURN_OFF_TIMER_PERIOD  (750u)
#define PD_SINK_VBUS_TURN_ON_TIMER_PERIOD   (1300u)
 
/* See Section 6.5.11.2 of USBPD Spec Rev2 v1.2 */     
#define PD_PS_HARD_RESET_TIMER_PERIOD       (27u)

/* This is an internal timer used to prevent RX lock up */
#define PD_GOOD_CRC_TX_TIMER_PERIOD         (3u)
    
/* These timers are meant for PD 3.0 collision support */     
#define PD_COLLISION_SRC_COOL_OFF_TIMER_PERIOD (5u)
#define PD_SINK_TX_TIMER_PERIOD                (18u)
#define PD_FRS_SRC_SNK_MAX_WAIT_FOR_FR_SWAP     (20u)    
#define PD_FRS_SNK_SRC_MAX_WAIT_FOR_RP          (14u)
/*
 * Type-C Timers.
 *
 * Timer ID and period settings, in milliseconds, of various timers. Period
 * should not be zero.
 *
 * These timer IDs should not be changed.
 */

/* CC debounce timers must have consecutive id numbers */
#define TYPEC_CC1_DEBOUNCE_TIMER                    (15u)
#define TYPEC_CC2_DEBOUNCE_TIMER                    (16u)
#define TYPEC_RD_DEBOUNCE_TIMER                     (17u)
#define TYPEC_VBUS_DISCHARGE_TIMER                  (18u)
#define TYPEC_ACTIVITY_TIMER                        (19u)
#define TYPEC_GENERIC_TIMER                         (20u)

#define TYPEC_CC_DEBOUNCE_TIMER_PERIOD              (140u)
#define TYPEC_PD_DEBOUNCE_TIMER_PERIOD              (11u)   
#define TYPEC_PD_DEBOUNCE_TIMER_REV3_SINK_PERIOD    (2u)
#define TYPEC_RD_DEBOUNCE_TIMER_PERIOD              (12u)
#define TYPEC_ERROR_RECOVERY_TIMER_PERIOD           (50u)
#define TYPEC_DRP_TRY_TIMER_PERIOD                  (130u)
#define TYPEC_SNK_TRY_TIMER_PERIOD                  (30u)
#define TYPEC_DRP_TIMER_PERIOD                      (37u)
#define TYPEC_VBUS_DISCHARGE_TIMER_PERIOD           (10u)
#define VBUS_TURN_ON_TIMER_PERIOD                   (250u)
#define VBUS_TURN_OFF_TIMER_PERIOD                  (625u)
#define VCONN_TURN_ON_TIMER_PERIOD                  (10u)
#define TYPEC_SINK_VBUS_DISCHARGE_PERIOD            (275u)
#define TYPEC_ACTIVITY_TIMER_PERIOD                 (20u)
#define TYPEC_SYNC_TOGGLE_PERIOD                    (25u)    

/**< FRS transmit enable flag in config table setting. */
#define CCG_FRS_TX_ENABLE_MASK                      (0x02u)

/**< FRS receive enable flag in config table setting. */
#define CCG_FRS_RX_ENABLE_MASK                      (0x01u)

/**< Size of extended source capabilities message in bytes. */
#define CCG_PD_EXT_SRCCAP_SIZE                      (23u)
#define CCG_PD_EXT_STATUS_SIZE                      (3u)
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/*
 * The following enumerated constant values are set based on the USB Type-C
 * Cable and Connector Specification, Revision 1.1, and the USB Power Delivery
 * Specification, Revision 2.0.
 */

/****************************** PD enumerations *******************************/

/**
 * @typedef pd_rev_t
 * @brief Enum of the PD spec revisions.
 */
typedef enum
{
    PD_REV1 = 0,
    PD_REV2,
    PD_REV3,
    PD_REV_RSVD
} pd_rev_t;

/**
 * @typedef msg_class_t
 * @brief Enum of the PD message types.
 */
typedef enum
{
    PD_CTRL_MSG = 0,
    PD_DATA_MSG,
    PD_EXTD_MSG
} pd_msg_class_t;

/**
 * @typedef rdo_type_t
 * @brief Enum of the RDO types.
 */
typedef enum
{
    FIXED_VAR_RDO = 0,
    BAT_RDO
} rdo_type_t;

/**
 * @typedef ctrl_msg_t
 * @brief Enum of the control message types.
 */
typedef enum
{
    CTRL_MSG_RSRVD = 0,
    CTRL_MSG_GOOD_CRC = 1,
    CTRL_MSG_GO_TO_MIN,
    CTRL_MSG_ACCEPT,
    CTRL_MSG_REJECT,
    CTRL_MSG_PING,
    CTRL_MSG_PS_RDY,
    CTRL_MSG_GET_SOURCE_CAP,
    CTRL_MSG_GET_SINK_CAP,
    CTRL_MSG_DR_SWAP,
    CTRL_MSG_PR_SWAP,
    CTRL_MSG_VCONN_SWAP,
    CTRL_MSG_WAIT,
    CTRL_MSG_SOFT_RESET,
    CTRL_MSG_NOT_SUPPORTED = 16,
    CTRL_MSG_GET_SRC_CAP_EXTD,
    CTRL_MSG_GET_STATUS,
    CTRL_MSG_FR_SWAP
} ctrl_msg_t;

/**
 * @typedef data_msg_t
 * @brief Enum of the data message types.
 */
typedef enum
{
    DATA_MSG_SRC_CAP = 1,
    DATA_MSG_REQUEST,
    DATA_MSG_BIST,
    DATA_MSG_SNK_CAP,
    DATA_MSG_BAT_STATUS,
    DATA_MSG_ALERT,
    DATA_MSG_VDM = 15
} data_msg_t;

/**
 * @typedef extd_msg_t
 * @brief Enum of the extended data message types.
 */
typedef enum
{
    EXTD_MSG_SRC_CAP_EXTD = 1,
    EXTD_MSG_STATUS,
    EXTD_MSG_GET_BAT_CAP,
    EXTD_MSG_GET_BAT_STATUS,
    EXTD_MSG_BAT_CAP,
    EXTD_MSG_GET_MANU_INFO,
    EXTD_MSG_MANU_INFO,
    EXTD_MSG_SECURITY_REQ,
    EXTD_MSG_SECURITY_RESP,
    EXTD_MSG_FW_UPDATE_REQ,
    EXTD_MSG_FW_UPDATE_RESP,    
} extd_msg_t;

/**
 * @typedef pdo_t
 * @brief Enum of the PDO types.
 */
typedef enum
{
    PDO_FIXED_SUPPLY = 0,
    PDO_BATTERY,
    PDO_VARIABLE_SUPPLY,
    PDO_RESERVED
} pdo_t;

/**
 * @typedef peak_cur_cap_t
 * @brief Enum of Peak Current Capability levels.
 */
typedef enum
{
    IMAX_EQ_IOC = 0,
    IMAX_EQ_130_IOC,
    IMAX_EQ_150_IOC,
    IMAX_EQ_200_IOC
} peak_cur_cap_t;

/**
 * @typedef bist_mode_t
 * @brief Enum of the BIST modes.
 */
typedef enum
{
    BIST_RX_MODE = 0,
    BIST_TX_MODE,
    BIST_RETURN_COUNTERS_MODE,
    BIST_CARRIER_MODE_0,
    BIST_CARRIER_MODE_1,
    BIST_CARRIER_MODE_2,
    BIST_CARRIER_MODE_3,
    BIST_EYE_PATTERN_MODE,
    BIST_TEST_DATA_MODE
} bist_mode_t;

/**
 * @typedef sop_t
 * @brief Enum of the SOP (Start Of Frame) types.
 */
typedef enum
{
    SOP = 0,
    SOP_PRIME,
    SOP_DPRIME,
    SOP_P_DEBUG,
    SOP_DP_DEBUG,
    HARD_RESET,
    CABLE_RESET,
    SOP_INVALID
} sop_t;

/**
 * @typedef port_role_t
 * @brief Enum of the PD port roles.
 */
typedef enum
{
    PRT_ROLE_SINK = 0,
    PRT_ROLE_SOURCE,
    PRT_DUAL

} port_role_t;

/**
 * @typedef port_type_t
 * @brief Enum of the PD port types.
 */
typedef enum
{
    PRT_TYPE_UFP = 0,
    PRT_TYPE_DFP,
    PRT_TYPE_DRP
} port_type_t;

/**
 * @typedef pd_fr_swap_supp_t
 * @brief Enum to hold FR swap options in sink capabilities
 */
typedef enum
{
    FR_SWAP_NOT_SUPPORTED = 0,
    FR_SWAP_DEF_USB,
    FR_SWAP_1_5A,
    FR_SWAP_3A
    
}fr_swap_supp_t;

/**
 * @typedef app_req_status_t
 * @brief Enum of the PD Request results. Enum fields map to the control
 * message field in the PD spec.
 */
typedef enum
{
    /*Send Hard Reset is only valid for response to DR swap*/
    REQ_SEND_HARD_RESET = 1,
    REQ_ACCEPT = 3,
    REQ_REJECT = 4,
    REQ_WAIT = 12,
    REQ_NOT_SUPPORTED = 16
} app_req_status_t;

/**
 * @typedef resp_status_t
 * @brief Enum of the response status to DPM commands.
 */
typedef enum
{
    SEQ_ABORTED = 0,
    CMD_FAILED,
    RES_TIMEOUT,
    CMD_SENT,
    RES_RCVD
} resp_status_t;

/**
 * @typedef dpm_pd_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types.
 */
typedef enum
{
    DPM_CMD_SRC_CAP_CHNG = 0,
    DPM_CMD_SNK_CAP_CHNG,
    DPM_CMD_SEND_GO_TO_MIN,
    DPM_CMD_GET_SNK_CAP,
    DPM_CMD_GET_SRC_CAP,
    DPM_CMD_SEND_HARD_RESET,
    DPM_CMD_SEND_SOFT_RESET,
    DPM_CMD_SEND_CABLE_RESET,
    DPM_CMD_SEND_SOFT_RESET_EMCA,
    DPM_CMD_SEND_DR_SWAP,
    DPM_CMD_SEND_PR_SWAP,
    DPM_CMD_SEND_VCONN_SWAP,
    DPM_CMD_SEND_VDM,
    DPM_CMD_SEND_EXTENDED,
    DPM_CMD_GET_SRC_CAP_EXTENDED,
    DPM_CMD_GET_STATUS,
    DPM_CMD_SEND_BATT_STATUS,
    DPM_CMD_SEND_ALERT,
    DPM_CMD_SEND_NOT_SUPPORTED,
    DPM_CMD_SEND_INVALID = 0xFFu
} dpm_pd_cmd_t;

/**
 * @typedef pd_devtype_t
 * @brief Enum of the attached device type.
 */
typedef enum
{
    DEV_SNK = 1,
    DEV_SRC,
    DEV_DBG_ACC,
    DEV_AUD_ACC,
    DEV_PWRD_ACC,
    DEV_UNSUPORTED_ACC
} pd_devtype_t;

/**
 * @typedef vdm_type_t
 * @brief Enum of the VDM types.
 */
typedef enum
{
    VDM_TYPE_UNSTRUCTURED = 0,
    VDM_TYPE_STRUCTURED
} vdm_type_t;

/**
 * @typedef std_vdm_cmd_t
 * @brief Enum of the standard VDM commands.
 */
typedef enum
{
    VDM_CMD_DSC_IDENTITY = 1,
    VDM_CMD_DSC_SVIDS,
    VDM_CMD_DSC_MODES,
    VDM_CMD_ENTER_MODE,
    VDM_CMD_EXIT_MODE,
    VDM_CMD_ATTENTION,
    VDM_CMD_DP_STATUS_UPDT = 16,
    VDM_CMD_DP_CONFIGURE = 17
} std_vdm_cmd_t;

/**
 * @typedef std_vdm_cmd_type_t
 * @brief Enum of the standard VDM command types.
 */
typedef enum
{
    CMD_TYPE_INITIATOR = 0,
    CMD_TYPE_RESP_ACK,
    CMD_TYPE_RESP_NAK,
    CMD_TYPE_RESP_BUSY
} std_vdm_cmd_type_t;

/**
 * @typedef std_vdm_prod_t
 * @brief Enum of the standard VDM product types.
 */
typedef enum
{
    PROD_TYPE_UNDEF = 0,
    PROD_TYPE_HUB,
    PROD_TYPE_PERI,
    PROD_TYPE_PAS_CBL,
    PROD_TYPE_ACT_CBL,
    PROD_TYPE_AMA
} std_vdm_prod_t;

/**
 * @typedef std_vdm_ver_t
 * @brief Enum for the standard VDM version.
 */
typedef enum
{
    STD_VDM_VER1 = 0,
    STD_VDM_VER2,
    STD_VDM_VER3,
    STD_VDM_VER4,
} std_vdm_ver_t;

/**
 * @typedef cbl_vbus_cur_t
 * @brief Enum of the cable current levels.
 */
typedef enum
{
    CBL_VBUS_CUR_0A = 0,
    CBL_VBUS_CUR_3A,
    CBL_VBUS_CUR_5A
} cbl_vbus_cur_t;

/**
 * @typedef cbl_term_t
 * @brief Enum of the cable termination types.
 */
typedef enum
{
    CBL_TERM_BOTH_PAS_VCONN_NOT_REQ = 0,
    CBL_TERM_BOTH_PAS_VCONN_REQ,
    CBL_TERM_ONE_ACT_ONE_PAS_VCONN_REQ,
    CBL_TERM_BOTH_ACT_VCONN_REQ
} cbl_term_t;

/*
 * PE FSMs enumerations.
 */

/**
 * @typedef pe_fsm_ams_t
 * @brief Enum of all PE AMSs.
 */
typedef enum
{
    PE_FSM_NONE = 0,
    PE_FSM_HARD_RESET_AMS,
    PE_FSM_BTD_AMS,
    PE_FSM_BCM2_AMS,
    PE_FSM_SOFT_RESET_AMS,
    PE_FSM_SRC_CONTRACT_AMS,
    PE_FSM_SNK_CONTRACT_AMS,
    PE_FSM_PR_SWAP_AMS,
    PE_FSM_DR_SWAP_AMS,
    PE_FSM_VCONN_SWAP_AMS,
    PE_FSM_GENERIC_AMS,
    PE_FSM_FR_SWAP_AMS,
    PE_FSM_FRS_SRC_SNK_AMS,
} pe_fsm_ams_t;

/**
 * @typedef pe_hard_reset_ams_state_t
 * @brief Enum of the Hard Reset AMS states.
 */
typedef enum
{
    PE_HR_AMS_SEND_HARD_RESET = 0,
    PE_HR_AMS_SRC_TRANS_TO_DFLT,
    PE_HR_AMS_SRC_RECOVER,
    PE_HR_AMS_SRC_APPLY_VBUS,
    PE_HR_AMS_SNK_TRANS_TO_DFLT,
    PE_HR_AMS_SNK_WAIT_VBUS_OFF,
    PE_HR_AMS_SNK_WAIT_VBUS_ON,
    PE_HR_AMS_MAX_STATES
} pe_hard_reset_ams_state_t;

/**
 * @typedef pe_bist_test_data_ams_state_t
 * @brief Enum of the BIST test data states.
 */
typedef enum
{
    PE_BTD_AMS_ENABLED = 0,
    PE_BTD_AMS_MAX_STATES
} pe_bist_test_data_ams_state_t;

/**
 * @typedef pe_bist_cm2_ams_state_t
 * @brief Enum of the BIST Carrier mode 2 states.
 */
typedef enum
{
    PE_BCM2_ENABLED = 0,
    PE_BCM2_AMS_MAX_STATES
} pe_bist_cm2_ams_state_t;

/**
 * @typedef tPE_PE_SOFT_RESET_AMS_STATE
 * @brief Enum of the Soft Reset AMS states.
 */
typedef enum
{
    PE_SR_AMS_SEND_SOFT_RESET = 0,
    PE_SR_AMS_RCVD_SOFT_RESET,
    PE_SR_AMS_MAX_STATES
} pe_soft_reset_ams_state_t;

/**
 * @typedef pe_src_contract_ams_state_t
 * @brief Enum of the source PD contract AMS states.
 */
typedef enum
{
    PE_SRC_AMS_SRC_STARTUP = 0,
    PE_SRC_AMS_SRC_SEND_CBL_SOFT_RESET,
    PE_SRC_AMS_SRC_SEND_CBL_DSC_ID,
    PE_SRC_AMS_SRC_SEND_CAP,
    PE_SRC_AMS_SRC_DISCOVERY,
    PE_SRC_AMS_SRC_NEG_CAP,
    PE_SRC_AMS_SRC_TRANS_SUPPLY,
    PE_SRC_AMS_SRC_SEND_PS_RDY,
    PE_SRC_AMS_SRC_WAIT_NEW_CAP,
    PE_SRC_AMS_MAX_STATES
} pe_src_contract_ams_state_t;

/**
 * @typedef pe_snk_contract_ams_state_t
 * @brief Enum of the sink PD contract states.
 */
typedef enum
{
    PE_SNK_AMS_SNK_STARTUP = 0,
    PE_SNK_AMS_WAIT_FOR_CAP,
    PE_SNK_AMS_SNK_EVAL_CAP,
    PE_SNK_AMS_SNK_SEL_CAP,
    PE_SNK_AMS_SNK_TRANS_SINK,
    PE_SNK_AMS_MAX_STATES
} pe_snk_contract_ams_state_t;

/**
 * @typedef pe_pr_swap_ams_state_t
 * @brief Enum of the power role swap states.
 */
typedef enum
{
    PE_PR_AMS_EVAL_PR_SWAP = 0,
    PE_PR_AMS_SEND_PR_SWAP,
    PE_PR_AMS_SRC_SNK_TRANSITION,
    PE_PR_AMS_SRC_SNK_REMOVE_VBUS,
    PE_PR_AMS_SRC_SNK_WAIT_PS_RDY,
    PE_PR_AMS_SNK_SRC_WAIT_PS_RDY,
    PE_PR_AMS_SNK_SRC_APPLY_VBUS,
    PE_PR_AMS_MAX_STATES
} pe_pr_swap_ams_state_t;

/**
 * @typedef pe_dr_swap_ams_state_t
 * @brief Enum of the data role swap states.
 */
typedef enum
{
    PE_DR_AMS_EVAL_DR_SWAP = 0,
    PE_DR_AMS_SEND_DR_SWAP,
    PE_DR_AMS_DFP_CHNG_DATA_ROLE,
    PE_DR_AMS_MAX_STATES
} pe_dr_swap_ams_state_t;

/**
 * @typedef pe_vconn_swap_ams_state_t
 * @brief Enum of the VCONN swap states.
 */
typedef enum
{
    PE_VS_AMS_EVAL_VCONN_SWAP = 0,
    PE_VS_AMS_SEND_VCONN_SWAP,
    PE_VS_AMS_TURN_ON_VCONN,
    PE_VS_AMS_TURN_OFF_VCONN,
    PE_VS_AMS_MAX_STATES
} pe_vconn_swap_ams_state_t;

/**
 * @typedef pe_generic_ams_state_t
 * @brief Enum of the generic PD state machine states.
 */
typedef enum
{
    PE_GN_AMS_READY = 0,
    PE_GN_AMS_REJECT,
    PE_GN_AMS_GIVE_SRC_CAP,
    PE_GN_AMS_GIVE_SNK_CAP,
    PE_GN_AMS_GET_RESP,
    PE_GN_AMS_SEND_SR_SP_DP,
    PE_GN_AMS_SEND_CABLE_RESET,
    PE_GN_AMS_EVAL_VDM,
    PE_GN_AMS_SEND_VDM,
#if CCG_PD_REV3_ENABLE    
    PE_GN_AMS_SEND_EXTD,
    PE_GN_AMS_GIVE_EXTD,
    PE_GN_AMS_SEND_MSG,
#endif /* CCG_PD_REV3_ENABLE */    
    PE_GN_AMS_MAX_STATES
} pe_generic_ams_state_t;

/**
 * @typedef pe_fr_swap_ams_state_t
 * @brief Enum of the fast role swap states.
 */
typedef enum
{
    PE_FR_AMS_EVAL_FR_SWAP = 0,
    PE_FR_AMS_CHECK_RP,    
    PE_FR_AMS_SEND_FR_SWAP,
    PE_FR_AMS_SRC_SNK_TRANSITION_TO_OFF,
    PE_FR_AMS_SRC_SNK_WAIT_PS_RDY,
    PE_FR_AMS_SNK_SRC_WAIT_PS_RDY,
    PE_FR_AMS_SNK_SRC_APPLY_VBUS,
    PE_FR_AMS_MAX_STATES
} pe_fr_swap_ams_state_t;

/**
 * @typedef pe_frs_src_snk_ams_state_t
 * @brief Enum to hold Fast role swap Sent Wait state
 */
typedef enum
{
    PE_FRS_SRC_SNK_CC_SIGNAL = 0,
    PE_FRS_SRC_SNK_MAX_STATES,
}pe_frs_src_snk_ams_state_t;


/**
 * @typedef pe_cbl_state_t
 * @brief Enum of the Policy Engine cable discovery states.
 */
typedef enum
{
    CBL_FSM_DISABLED = 0,
    CBL_FSM_ENTRY,
    CBL_FSM_SEND_SOFT_RESET,
    CBL_FSM_SEND_DSC_ID
} pe_cbl_state_t;

/**************************** Type-C enumerations *****************************/

/**
 * @typedef rp_cc_status_t
 * @brief Enum of the Rp status when Rp is asserted.
 */
typedef enum
{
    RP_RA  = 0, /* Ra present.  */
    RP_RD,      /* Rd present.  */
    RP_OPEN,    /* No pulldown. */
} rp_cc_status_t;

/**
 * @typedef rd_cc_status_t
 * @brief Enum of the Rd status when Rd is asserted.
 */
typedef enum
{
    RD_RA = 0,  /* Ra           */
    RD_USB,     /* Default USB  */
    RD_1_5A,    /* 1.5A         */
    RD_3A,      /* 3A           */
    RD_ERR      /* Error        */
} rd_cc_status_t;

/**
 * @typedef cc_state_t
 * @brief Union to hold CC status.
 */
typedef union cc_state
{
    uint16_t state; /* Combined status of CC1 and CC2. */
    uint8_t  cc[2]; /* Individual status of CC1(cc[0]) and CC2(cc[1]). */
} cc_state_t;

/**
 * @typedef rp_term_t
 * @brief Enum of the CC termination current levels.
 */
typedef enum
{
    RP_TERM_RP_CUR_DEF = 0,
    RP_TERM_RP_CUR_1_5A,
    RP_TERM_RP_CUR_3A,
} rp_term_t;

/**
 * @typedef try_src_snk_t
 * @brief Enum of the Try Source/ Try Sink options.
 */
typedef enum
{
    TRY_SRC_TRY_SNK_DISABLED = 0,
    TRY_SRC_ENABLED,
    TRY_SNK_ENABLED,
} try_src_snk_t;

/**
 * @typedef dpm_typec_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types.
 */
typedef enum
{
    DPM_CMD_SET_RP_DFLT = 0,
    DPM_CMD_SET_RP_1_5A,
    DPM_CMD_SET_RP_3A,
    DPM_CMD_PORT_DISABLE,
    DPM_CMD_TYPEC_INVALID
} dpm_typec_cmd_t;

/**
 * @typedef dpm_typec_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) response types.
 */
typedef enum
{
    DPM_RESP_FAIL = 0,
    DPM_RESP_SUCCESS,
} dpm_typec_cmd_resp_t;

/**
 * @typedef typec_generic_fsm_state_t
 * @brief Enum of the Type-C generic FSM states.
 * @warning The ordering of elements must not be altered unless the state table
 * is also updated to match.
 */
typedef enum
{
    TYPEC_GN_FSM_ERR_RECOV = 0,
    TYPEC_GN_FSM_ERR_RECOV_CMPLT,
    TYPEC_GN_FSM_ATTACH_WAIT,
    TYPEC_GN_FSM_TRY_SRC,
    TYPEC_GN_FSM_TRY_WAIT_SNK,
    TYPEC_GN_FSM_TRY_SNK,
    TYPEC_GN_FSM_TRY_WAIT_SRC,
    TYPEC_GN_FSM_UNATTACHED_SRC,
    TYPEC_GN_FSM_UNATTACHED_SNK,
    TYPEC_GN_FSM_AUD_ACC,
    TYPEC_GN_FSM_DBG_ACC,
    TYPEC_GN_FSM_ATTACHED_SRC,
    TYPEC_GN_FSM_ATTACHED_SNK,
    TYPEC_GN_FSM_MAX_STATES
} typec_generic_fsm_state_t;

/******************************* Other modules ********************************/

/**
 * @typedef pd_contract_status_t
 * @brief Enum of possible PD contract negotiation scenarios that are used to
 * signal the application event handler.
 *
 * NOTE: An APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE event signal contains a
 * status bitmap, which is defined as:
 *  Byte 0: PD Contract Status
 *      Bit 0: Set if contract negotiation was successful.
 *      Bit 1: Set if the port partner indicates a Cap Mismatch. Note that in
 *             this case Bit 0 will also be set and this bit is an extra
 *             indication.
 *      Bits 2:4: Contract failure reason
 *          0x00: CCG sends REJECT and previous contract is valid. The received
 *                RDO is appended.
 *          0x01: CCG sends REJECT and previous contract is not valid. The
 *                received RDO is appended.
 *          0x02: CCG sends REJECT and there was no previous explicit contract.
 *                The received RDO is appended.
 *          0x03: CCG receives REJECT in response to RDO and there was a
 *                previous explicit contract.
 *          0x04: CCG receives REJECT in response to RDO and there was no
 *                previous explicit contract.
 *          0x05: CCG receives ACCEPT in response to RDO but the source did not
 *                send PS_RDY within tPSTransition.
 *          0x06: Reserved.
 *          0x07: Reserved.
 *      Bits 5:7: Reserved.
 *  Bytes (1:3): Reserved.
 *  Bytes (4:7): RDO, if applicable.
 *
 *  The values listed below are corresponding HEX values of the potential
 *  scenarios listed above and helps in saving code space by handling opcodes
 *  instead of bits.
 */
typedef enum
{
    PD_CONTRACT_NEGOTIATION_SUCCESSFUL      = 0x01,
    PD_CONTRACT_CAP_MISMATCH_DETECTED       = 0x03,
    PD_CONTRACT_REJECT_CONTRACT_VALID       = 0x00,
    PD_CONTRACT_REJECT_CONTRACT_NOT_VALID   = 0x04,
    PD_CONTRACT_REJECT_NO_CONTRACT          = 0x08,
    PD_CONTRACT_REJECT_EXPLICIT_CONTRACT    = 0x0C,
    PD_CONTRACT_REJECT_NO_EXPLICIT_CONTRACT = 0x10,
    PD_CONTRACT_PS_READY_NOT_RECEIVED       = 0x14,
    PD_CONTRACT_PS_READY_NOT_SENT           = 0x18,
} pd_contract_status_t;

/**
 * @typedef app_evt_t
 * @brief Enum of events that are signalled to the application.
 */
typedef enum
{
    APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS,
    APP_EVT_TYPE_C_ERROR_RECOVERY,
    APP_EVT_CONNECT,
    APP_EVT_DISCONNECT,
    APP_EVT_EMCA_DETECTED,
    APP_EVT_EMCA_NOT_DETECTED,
    APP_EVT_ALT_MODE,
    APP_EVT_APP_HW,
    APP_EVT_RP_CHANGE,
    APP_EVT_HARD_RESET_RCVD,
    APP_EVT_HARD_RESET_COMPLETE,
    APP_EVT_PKT_RCVD,
    APP_EVT_PR_SWAP_COMPLETE,
    APP_EVT_DR_SWAP_COMPLETE,
    APP_EVT_VCONN_SWAP_COMPLETE,
    APP_EVT_SENDER_RESPONSE_TIMEOUT,
    APP_EVT_VENDOR_RESPONSE_TIMEOUT,
    APP_EVT_HARD_RESET_SENT,
    APP_EVT_SOFT_RESET_SENT,
    APP_EVT_CBL_RESET_SENT,
    APP_EVT_PE_DISABLED,
    APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE,
    APP_EVT_VBUS_OVP_FAULT,
    APP_EVT_VBUS_OCP_FAULT,
    APP_EVT_VCONN_OCP_FAULT,
    APP_EVT_VBUS_PORT_DISABLE,
    APP_EVT_TYPEC_STARTED,
    APP_EVT_FR_SWAP_COMPLETE,
    APP_EVT_TEMPERATURE_FAULT,
    APP_EVT_HANDLE_EXTENDED_MSG,
} app_evt_t;

typedef enum
{
    PD_AMS_NONE = 0,
    PD_AMS_NON_INTR,
    PD_AMS_INTR
}pd_ams_type;

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/
/**
 * @typedef pd_hdr_t
 * @brief Union to hold the PD header. Upper 16 bits hold the extended header.
 */
typedef union
{
    uint32_t val;
    struct PD_HDR
    {
        uint32_t msg_type                   : 5;
        uint32_t data_role                  : 1;
        uint32_t spec_rev                   : 2;
        uint32_t pwr_role                   : 1;
        uint32_t msg_id                     : 3;
        uint32_t len                        : 3;
        uint32_t extd                       : 1;
        uint32_t data_size                  : 9;
        uint32_t rsvd1                      : 1;
        uint32_t request                    : 1;
        uint32_t chunk_no                   : 4;
        uint32_t chunked                    : 1;
    } hdr;
} pd_hdr_t;

/**
 * @typedef pd_extd_hdr_t
 * @brief Union to hold the PD extended header.
 */
typedef union
{
    uint16_t val;
    struct
    {
        uint16_t data_size                  : 9;
        uint16_t rsvd1                      : 1;
        uint16_t request                    : 1;
        uint16_t chunk_no                   : 4;
        uint16_t chunked                    : 1;
    } extd;
} pd_extd_hdr_t;

/**
 * @union pd_do_t
 * @brief Union to hold a PD data object.
 */
typedef union
{

    uint32_t val;                                   /**< Data object interpreted as an unsigned integer value. */

    struct BIST_DO
    {
        uint32_t rsvd1                      : 16;   /**< Reserved field. */
        uint32_t rsvd2                      : 12;   /**< Reserved field. */
        uint32_t mode                       : 4;    /**< BIST mode. */
    } bist_do;                                      /**< DO interpreted as a BIST data object. */

    struct FIXED_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 100mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t pk_current                 : 2;    /**< Peak current. */
        uint32_t reserved                   : 2;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t usb_suspend_sup            : 1;    /**< USB suspend supported. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_src;                                    /**< DO interpreted as a Fixed Supply PDO - Source. */

    struct VAR_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 100mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_src;                                      /**< DO interpreted as a Variable Supply PDO - Source. */

    struct BAT_SRC
    {
        uint32_t max_power                  : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_src;                                      /**< DO interpreted as a Battery Supply PDO - Source. */

    struct SRC_GEN
    {
        uint32_t max_cur_power              : 10;
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type. */
    } src_gen;                                      /**< Generic source DO. */

    struct FIXED_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t rsrvd                      : 3;    /**< Reserved field. */
		uint32_t fr_swap                    : 2;	/**< FR swap support. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t high_cap                   : 1;    /**< Higher capability possible. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_snk;                                    /**< DO interpreted as a Fixed Supply PDO - Sink. */

    struct VAR_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_snk;                                      /**< DO interpreted as a Variable Supply PDO - Sink. */

    struct BAT_SNK
    {
        uint32_t op_power                   : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_snk;                                      /**< DO interpreted as a Battery Supply PDO - Sink. */

    struct RDO_FIXED_VAR
    {
        uint32_t max_op_current             : 10;   /**< Maximum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_fix_var;                                  /**< Fixed and Variable Request Data Object. */

    struct RDO_FIXED_VAR_GIVEBACK
    {
        uint32_t min_op_current             : 10;   /**< Minimum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_fix_var_gvb;                              /**< Fixed and Variable Request Data Object with GiveBack. */

    struct RDO_BAT
    {
        uint32_t max_op_power               : 10;   /**< Maximum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_bat;                                      /**< Battery Request Data Object. */

    struct RDO_BAT_GIVEBACK
    {
        uint32_t min_op_power               : 10;   /**< Minimum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_bat_gvb;                                  /**< Battery Request Data Object with GiveBack. */

    struct RDO_GEN
    {
        uint32_t min_max_power_cur          : 10;
        uint32_t op_power_cur               : 10;
        uint32_t rsrvd1                     : 3;
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;
        uint32_t usb_comm_cap               : 1;
        uint32_t cap_mismatch               : 1;
        uint32_t give_back_flag             : 1;
        uint32_t obj_pos                    : 3;
        uint32_t rsrvd2                     : 1;
    } rdo_gen;

    struct RDO_GEN_GVB
    {
        uint32_t max_power_cur              : 10;
        uint32_t op_power_cur               : 10;
        uint32_t rsrvd1                     : 3;
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;
        uint32_t usb_comm_cap               : 1;
        uint32_t cap_mismatch               : 1;
        uint32_t give_back_flag             : 1;
        uint32_t obj_pos                    : 3;
        uint32_t rsrvd2                     : 1;
    } rdo_gen_gvb;

    struct STD_VDM_HDR
    {
        uint32_t cmd                        : 5;
        uint32_t rsvd1                      : 1;
        uint32_t cmd_type                   : 2;
        uint32_t obj_pos                    : 3;
        uint32_t rsvd2                      : 2;
        uint32_t st_ver                     : 2;
        uint32_t vdm_type                   : 1;
        uint32_t svid                       : 16;
    } std_vdm_hdr;

    struct USTD_VDM_HDR
    {
        uint32_t cmd                        : 5;
        uint32_t seq_num                    : 3;
        uint32_t rsvd1                      : 3;
        uint32_t cmd_type                   : 2;
        uint32_t vdm_ver                    : 2;
        uint32_t vdm_type                   : 1;
        uint32_t svid                       : 16;
    } ustd_vdm_hdr;

    struct STD_VDM_ID_HDR
    {
        uint32_t usb_vid                    : 16;
#if CCG_PD_REV3_ENABLE
        uint32_t rsvd1                      : 7;
        uint32_t prod_type_dfp              : 3;
#else
        uint32_t rsvd1                      : 10;
#endif /* CCG_PD_REV3_ENABLE */	
        uint32_t mod_support                : 1;
        uint32_t prod_type                  : 3;
        uint32_t usb_dev                    : 1;
        uint32_t usb_host                   : 1;
    } std_id_hdr;

    struct STD_CERT_VDO
    {
        uint32_t usb_cmpl                   : 20;
        uint32_t rsvd1                      : 12;
    } std_cert_vdo;

    struct STD_PROD_VDO
    {
        uint32_t bcd_dev                    : 16;
        uint32_t usb_pid                    : 16;
    } std_prod_vdo;

    struct STD_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;
        uint32_t sop_dp                     : 1;
        uint32_t vbus_thru_cbl              : 1;
        uint32_t vbus_cur                   : 2;
        uint32_t ssrx2                      : 1;
        uint32_t ssrx1                      : 1;
        uint32_t sstx2                      : 1;
        uint32_t sstx1                      : 1;
        uint32_t cbl_term                   : 2;
        uint32_t cbl_latency                : 4;
        uint32_t typec_plug                 : 1;
        uint32_t typec_abc                  : 2;
        uint32_t rsvd1                      : 4;
        uint32_t cbl_fw_ver                 : 4;
        uint32_t cbl_hw_ver                 : 4;
    } std_cbl_vdo;

    struct STD_AMA_VDO
    {
        uint32_t usb_ss_sup                 : 3;
        uint32_t vbus_req                   : 1;
        uint32_t vcon_req                   : 1;
        uint32_t vcon_pwr                   : 3;
        uint32_t ssrx2                      : 1;
        uint32_t ssrx1                      : 1;
        uint32_t sstx2                      : 1;
        uint32_t sstx1                      : 1;
        uint32_t rsvd1                      : 12;
        uint32_t cbl_fw_ver                 : 4;
        uint32_t cbl_hw_ver                 : 4;
    } std_ama_vdo;

    struct STD_SVID_RESP_VDO
    {
        uint32_t svid_n1                    : 16;
        uint32_t svid_n                     : 16;
    } std_svid_res;

    struct STD_DP_VDO
    {
        uint32_t port_cap                   : 2;
        uint32_t signal                     : 4;
        uint32_t recep                      : 1;
        uint32_t usb2_0                     : 1;
        uint32_t dfp_d_pin                  : 8;
        uint32_t ufp_d_pin                  : 8;
        uint32_t rsvd                       : 8;
    } std_dp_vdo;

    struct DP_STATUS_VDO
    {
        uint32_t dfp_ufp_conn               : 2;
        uint32_t pwr_low                    : 1;
        uint32_t en                         : 1;
        uint32_t mult_fun                   : 1;
        uint32_t usb_cfg                    : 1;
        uint32_t exit                       : 1;
        uint32_t hpd_state                  : 1;
        uint32_t hpd_irq                    : 1;
        uint32_t rsvd                       : 23;
    } dp_stat_vdo;

    struct DP_CONFIG_VDO
    {
        uint32_t sel_conf                   : 2;
        uint32_t sign                       : 4;
        uint32_t rsvd1                      : 2;
        uint32_t dfp_asgmnt                 : 8;
        uint32_t ufp_asgmnt                 : 8;
        uint32_t rsvd2                      : 8;
    } dp_cfg_vdo;

    struct TBT_VDO
    {
        uint32_t intel_mode                 : 16;
        uint32_t cbl_speed                  : 3;
        uint32_t cbl_gen                    : 2;
        uint32_t cbl_type                   : 1;
        uint32_t cbl                        : 1;
        uint32_t link_training              : 1;
        uint32_t leg_adpt                   : 1;
        uint32_t rsvd                       : 7;
    } tbt_vdo;

    struct SLICE_VDO
    {
        uint32_t slice_mode                 : 16;
        uint32_t module_type                : 2;
        uint32_t rsvd                       : 14;
    } slice_vdo;

    struct SLICE_SUBHDR
    {
        uint32_t am_addr                    : 20;
        uint32_t vdo_cnt                    : 3;
        uint32_t multi_part                 : 1;
        uint32_t data_cnt                   : 8;
    } slice_subhdr;
} pd_do_t;

/**
 * @typedef pd_power_status_t
 * @brief PD port status corresponding to the Status Data Block (SSDB)
 * See Table 6-39 of USB-PD R3 specification.
 */
typedef struct
{
    uint8_t                             intl_temperature;       /**< Port's internal temperature. 0 if not supported. */
    uint8_t                             present_input;          /**< Reports current input power status. */
    uint8_t                             battery_input;          /**< Reports the current battery status. */
    uint8_t                             reserved;               /**< Reserved value. */
} pd_power_status_t;

/**
 * @typedef pd_port_status_t
 * @brief PD port status as reported to Embedded Controller.
 */
typedef union
{
    uint32_t    val;                                            /**< PD-Status register value. */

    struct PD_PORT_STAT
    {
        uint32_t dflt_data_role         : 2;
        uint32_t dflt_data_pref         : 1;
        uint32_t dflt_power_role        : 2;
        uint32_t dflt_power_pref        : 1;
        uint32_t cur_data_role          : 1;
        uint32_t reserved0              : 1;
        uint32_t cur_power_role         : 1;
        uint32_t min_state              : 1;
        uint32_t contract_exist         : 1;
        uint32_t emca_present           : 1;
        uint32_t vconn_src              : 1;
        uint32_t vconn_on               : 1;
        uint32_t rp_status              : 1;
        uint32_t pe_rdy                 : 1;
        uint32_t ccg_spec_rev           : 2;
        uint32_t peer_pd3_supp          : 1;
        uint32_t peer_unchunk_supp      : 1;
        uint32_t emca_spec_rev          : 2;
        uint32_t reserved2              : 10;
    } status;                                                   /**< Structure containing status bits. */
} pd_port_status_t;

/**
 *  @typedef pd_port_config_t
 *  @brief PD port-specific configuration data from the configuration table.
 */
typedef struct
{
    /* Byte 0: Two byte offset to the Discover ID Response VDM. */
    uint16_t    id_vdm_offset;
    /* Byte 2: Two byte length of the Discover ID Response VDM. */
    uint16_t    id_vdm_length;
    /* Byte 4: Two byte offset to the Discover SVID Response VDM. */
    uint16_t    svid_vdm_offset;
    /* Byte 6: Two byte length of the Discover SVID Response VDM. */
    uint16_t    svid_vdm_length;
    /* Byte 8: Two byte offset to the Discover Mode Response VDM. */
    uint16_t    mode_vdm_offset;
    /* Byte 10: Two byte length of the Discover Mode Response VDM. */
    uint16_t    mode_vdm_length;
    /* Byte 12: Two byte offset to the Src. Cap Extended response. */
    uint16_t    ext_src_cap_offset;
    /* Byte 14: Two byte length of the Src. Cap Extended response. */
    uint16_t    ext_src_cap_length;
    /* Byte 16: Reserved area for additional VDMs. */
    uint16_t    reserved_0[4];

    /* Byte 24: Ra removal delay for EMCA applications, measured in ms. */
    uint16_t    ra_timeout;
    /* Byte 26: Reserved area for EMCA configuration. */
    uint16_t    reserved_1[3];

    /* Byte 32: PD port role: 0=Sink, 1=Source, 2=Dual Role. */
    uint8_t     port_role;
    /* Byte 33: Default port role in case of a dual role port: 0=Sink, 1=Source. */
    uint8_t     default_role;
    /* Byte 34: Type-C current level: 0=Default, 1=1.5A, 2=3A. */
    uint8_t     current_level;
    /* Byte 35: Whether the power source is connected to a battery. */
    uint8_t     is_src_bat;
    /* Byte 36: Whether the power sink is connected to a battery. */
    uint8_t     is_snk_bat;
    /* Byte 37: Whether USB suspend is supported. */
    uint8_t     snk_usb_susp_en;
    /* Byte 38: Whether USB communication is supported. */
    uint8_t     snk_usb_comm_en;
    /*
     * Byte 39: Response to be sent for each USB-PD SWAP command:
     *  Bits 1:0 => DR_SWAP response
     *  Bits 3:2 => PR_SWAP response
     *  Bits 5:4 => VCONN_SWAP response
     * Allowed values are: 0=ACCEPT, 1=REJECT, 2=WAIT.
     */
    uint8_t     swap_response;
    /* Byte 40: Whether DRP toggle is enabled. */
    uint8_t     drp_toggle_en;
    /* Byte 41: Number of valid source PDOs in the table: Maximum supported value is 7. */
    uint8_t     src_pdo_cnt;
    /* Byte 42: Default Source PDO enable mask. */
    uint8_t     default_src_pdo_mask;
    /* Byte 43: Number of valid sink PDOs in the table: Maximum supported value is 7. */
    uint8_t     snk_pdo_cnt;
    /* Byte 44: Default Sink PDO enable mask. */
    uint8_t     default_sink_pdo_mask;
    /*
     * Byte 45: Supported Rp values. Multiple bits can be set.
     *  Bit 0 => Default current support.
     *  Bit 1 => 1.5A support.
     *  Bit 2 => 3A support.
     */
    uint8_t     rp_supported;
    /* Byte 46: Whether PD operation is supported on the port. */
    uint8_t     pd_operation_en;
    /* Byte 47: Whether Try.SRC state is supported on the port. */
    uint8_t     try_src_en;
    /* Byte 48: Whether cable discovery is supported on the port. */
    uint8_t     cable_disc_en;
    /* Byte 49: Whether firmware should force Sink operation in Dead Battery condition. */
    uint8_t     dead_bat_support;
    /* Byte 50: Whether PD error recovery is enabled. */
    uint8_t     err_recovery_en;
    /* Byte 51: Port disable flag. */
    uint8_t     port_disable;
    /* Byte 52: Fast Role Swap enable flags. */
    uint8_t     frs_enable;
    /* Byte 53: Reserved bytes for padding to 2-byte aligned address. */
    uint8_t     reserved_2;
    /* Byte 54: Reserved words for padding to 4-byte aligned address. */
    uint16_t    reserved_3[5];

    /* Byte 64: Source PDO data array. */
    uint32_t    src_pdo_list[MAX_NO_OF_PDO];
    /* Byte 92: Sink PDO data array. */
    uint32_t    snk_pdo_list[MAX_NO_OF_PDO];
    /*
     * Byte 120: Array of sink PDO parameters. For each element, the format is as below:
     *  Bit 15      => Give Back support flag
     *  Bits 14:0   => Minimum sink operating current in 10 mA units
     */
    uint16_t    snk_pdo_max_min_current_pwr[MAX_NO_OF_PDO];

    /* Byte 134: Reserved space for additional port parameters. */
    uint16_t    reserved_4;

    /*
     * Byte 136: Protection schemes (OVP/OCP) enable.
     *  Bit 0 => OCP method: 1=External, 0=UVP.
     *  Bit 1 => OCP enable.
     *  Bit 2 => OVP enable.
     */
    uint8_t     protect_en;
    /* Byte 137: OVP threshold as percentage of rated voltage. */
    uint8_t     ovp_threshold;
    /* Byte 138: OVP debounce period in us. */
    uint8_t     ovp_debounce;
    /* Byte 139: OCP threshold as percentage of nominal operating current. */
    uint8_t     ocp_threshold;
    /* Byte 140: OCP debounce period in ms. */
    uint8_t     ocp_debounce;
    /* Byte 141: OCP power-off time in 10ms units. */
    uint8_t     ocp_offtime;
    /* Byte 142: OCP retry count. */
    uint8_t     ocp_retry_cnt;
    /* Byte 143: OCP sampling period in ms. Only applies to UVP method. */
    uint8_t     ocp_sample_period;

    /*
     * Byte 144: This field is a map for MUXSEL GPIOs for the port. One byte is
     * reserved for each MUXSEL GPIO. Each byte represents the port and pin
     * info for the GPIO. Upper nibble contains the port number and lower
     * nibble contains the pin number. Value of 0xFF indicates that no GPIO is
     * assigned to the corresponding MUXSEL.
     */
    uint8_t     muxsel_gpio_map[8];
    /* Byte 152: Reserved field. */
    uint8_t     reserved_5[8];

    /*
     * Byte 160: Supported Pin configurations for DP modes
     *  0b00000000: USB SS only.
     *  0b00000001: Reserved for future use (A).
     *  0b00000010: Reserved for future use (B).
     *  0b00000100: Pin Config C.
     *  0b00001000: Pin Config D.
     *  0b00010000: Pin Config E.
     *  0b00100000: Pin Config F.
     */
    uint8_t     dp_config_supported;
    /* Byte 161: DP_MUX_CONTROL method:
     *  0 => DP MUX Controlled by CCG.
     *  1 => DP MUX Controlled by EC.
     */
    uint8_t     dp_mux_control;
    /*
     * Byte 162: DP_MODE_TRIGGER: Trigger for initializing DP modes.
     *  0 => CCG initiates DP after contract.
     *  1 => CCG waits for a trigger from EC.
     */
    uint8_t     dp_mode_trigger;
    /*
     * Byte 163: Type of DP operation supported.
     *  Bit 0: DP Sink supported
     *  Bit 1: DP Source supported.
     */
    uint8_t     dp_oper;
    /*
     * Byte 164: DP preferred mode.
     *  Bit 0:
     *      0: CCG as DP Sink prefers 4 lane DP mode only.
     *      1: CCG as DP Sink prefers 2 lane DP + USB SS Mode.
     *  All other bits are reserved. */
    uint8_t     dp_pref_mode;

    /* Byte 165: Reserved area for future expansion. */
    uint8_t     reserved_6[7];

    /*
     * Byte 172: This field indicates whether a billboard device is enabled.
     * The usage and requirement for the billboard device is application
     * specific.
     *  0 => No billboard device.
     *  1 => External billboard device.
     *  2 => Internal billboard.
     */
    uint8_t     bb_enable;
    /*
     * Byte 173: This field is valid only if the bb_enable is non-zero. The field
     * determines when to enable the billboard interface.
     *  0 => Enable the billboard device only on error.
     *  1 => Always enable billboard on connection.
     */
    uint8_t     bb_always_on;
    /*
     * Byte 174: This field provides the various functionalities supported by
     * the device.
     *  Bit 0    => 0 = No HID interface, 1 = Enable HID interface.
     *  Bits 1:7 => Reserved.
     */
    uint8_t     bb_option;
    /* Byte 175: Reserved area for future expansion. */
    uint8_t     reserved_7;
    /*
     * Byte 176: This field is valid only if bb_enable is non-zero. The field
     * determines how long the billboard interface stays on, in seconds.
     * FFFFh    => Stays on until disconnect.
     * 000Ah to FFFEh => Timeout in seconds.
     */
    uint16_t    bb_timeout;
    /* Byte 178: EC present notification to external billboard controller. */
    uint8_t     bb_ec_present;
    /* Byte 179: Reserved area for future expansion. */
    uint8_t     reserved_8[5];

    /*
     * Byte 184: This field is valid only for devices that have internal USB
     * support. The field indicates whether the device is bus powered or not.
     */
    uint8_t     bb_bus_power;
    /*
     * Byte 185: This field is valid only for devices that have  internal USB
     * support. The field indicates whether the device creates the container ID
     * descriptor or uses what is provided in the BOS descriptor.
     */
    uint8_t     bb_unique_container_id;
    /* Byte 186: This field is valid only for devices that have internal USB
     * support. The field provides the offset inside the table where the BOS
     * descriptor for the device is located. The BOS descriptor is mandatory
     * for billboard support.
     */
    uint16_t    bb_bos_dscr_offset;
    /*
     * Byte 188: This field is valid only for devices that have internal USB
     * support. The field provides the offset inside the table where the
     * various string descriptors for the device are located. The descriptors
     * are expected to be in a fixed order and is mandatory. The following are
     * the usage models for the various string indices:
     *  0    => Manufactuer string (mandatory).
     *  1    => Product string (mandatory).
     *  2    => Serial string (optional).
     *          0000h = No serial string descriptor,
     *          FFFFh = Unique serial string generated by device,
     *          Any other offset is treated as a valid serial string descriptor.
     *  3    => Configuration string (mandatory).
     *  4    => Billboard interface string (mandatory).
     *  5    => HID interface string (mandatory).
     *  6    => Additional info URL string (mandatory).
     *  7-8  => Alternate mode strings (mandatory for all valid modes).
     */
    uint16_t    bb_string_dscr_offset[15];

    /* Byte 218: Reserved area for future expansion. */
    uint8_t     reserved_10[20];

} pd_port_config_t;

/**
 * @typedef pd_config_t
 * @brief Struct to hold the PD device configuration.
 */
typedef struct
{
    /* Byte 0: Two byte signature to indicate validity of the configuration. */
    uint16_t    table_sign;
    /*
     * Byte 2: The table type indicates the type of solution.
     *  1 => EMCA
     *  2 => DFP
     *  3 => UFP
     *  4 => DRP
     */
    uint8_t     table_type;
    /*
     * Byte 3: This field specifies the type of PD application supported:
     *  0 => Notebook
     *  1 => Tablet
     *  2 => Passive Cable
     *  3 => Active Cable
     *  4 => Monitor
     *  5 => Power Adapter
     *  6 => Cable Adapter (Dongle)
     */
    uint8_t     application;
    /*
     * Byte 4: Table version: This contains 4 bit major version, 4 bit minor
     * version and 8 bit patch number.
     */
    uint16_t    table_version;
    /*
     * Byte 6: Size of the configuration table. Checksum is calculated over
     * bytes 10 to size - 1.
     */
    uint16_t    table_size;
    /*
     * Byte 8: One byte configuration checksum. Calculated over bytes 10 to
     * byte (size - 1).
     */
    uint8_t     table_checksum;
    /*
     * Byte 9: One byte flash checksum. Used to ensure that binary sum of
     * config table is 0.
     */
    uint8_t     flash_checksum;

    /*
     * Byte 10: Special VID used for flashing mode in case of CC firmware
     * updates.
     */
    uint16_t    flashing_vid;

    /* Byte 12: Special Mode used for flashing in case of CC firmware updates. */
    uint16_t    flashing_mode;

    /* Byte 14: Number of PD ports enabled in the configuration. */
    uint8_t     pd_port_cnt;

    /* Byte 15: Reserved byte for 2-byte alignment. */
    uint8_t     reserved_0;

    /* Byte 16: Offset to manufacturer information in config table. */
    uint16_t    mfg_info_offset;

    /* Byte 18: Length of manufacturer information in config table. */
    uint8_t     mfg_info_length;

    /* Byte 19: Reserved field for 2-byte alignment. */
    uint8_t     reserved_1;

    /* Byte 20: Reserved fields for future expansion. */
    uint16_t    reserved_2[6];

    /* Byte 32(272): Configuration data for each USB-PD port. */
    pd_port_config_t    port_conf[NO_OF_TYPEC_PORTS];
} pd_config_t;

/**
 * @typedef app_resp_t
 * @brief Struct to hold response to policy manager.
 */
typedef struct
{
    pd_do_t             resp_buf[MAX_NO_OF_DO]; /* Data objects buffer. */
    app_req_status_t    req_status;             /* Request status. */
    uint8_t             do_count;               /* Data objects count. */
} app_resp_t;

/**
 * @typedef vdm_resp_t
 * @brief Struct to hold response to policy manager.
 */
typedef enum
{
    VDM_AMS_RESP_READY = 0,      /* Response is ready */
    VDM_AMS_RESP_NOT_REQ,        /* No response required */
    VDM_AMS_RESP_FROM_EC         /* Response will come from EC */
} vdm_ams_t;

/**
 * @typedef vdm_resp_t
 * @brief Struct to hold response to policy manager.
 */
typedef struct
{
    pd_do_t             resp_buf[MAX_NO_OF_DO]; /* Data objects buffer */
    uint8_t             do_count;               /*Data objects count */
    vdm_ams_t           no_resp;                /* No response needed. */
} vdm_resp_t;

/**
 * @typedef dpm_pd_cmd_buf_t
 * @brief Struct to hold PD command buffer.
 * @warning When providing pointer to the extended data make sure original buffer
 * is always 4byte aligned. i.e even if 1 byte data 4 byte should be used to store that
 * data.
 */
typedef struct cmd_buf
{
    sop_t               cmd_sop;                /**< SOP type */
    extd_msg_t          extd_type;              /**< Extended Message Type */
    pd_extd_hdr_t       extd_hdr;               /**< Extended Header */
    uint8_t             no_of_cmd_do;           /**< No of data objects including VDM header */
    uint8_t*            dat_ptr;                 /**< Data Pointer in case of extended message only*/
    /*
     * Timeout value, in ms, for a response.
     * If set to zero, the PD stack will not wait for a VDM response and jump
     * to ready state after this buffer has been sent.
     *
     * For e.g. for Attention or VDM tunnneling, timeout should be 0,
     * whereas for Discover SVID, timeout should be set to the VDM response
     * timer.
     */
    uint8_t             timeout;    
    pd_do_t             cmd_do[MAX_NO_OF_DO];   /**< Command data objects. */

} dpm_pd_cmd_buf_t;

/**
 * @typedef contract_t
 * @brief Union to hold PD Contract
 */
typedef union
{
    uint32_t val;                       /**< absolute value */

    struct  FIELD
    {
        uint32_t cur_pwr     : 10;     /**< PD contract current/power. */
        uint32_t max_volt    : 10;     /**< PD contract max voltage. */
        uint32_t min_volt    : 10;     /**< PD contract min voltage. */
    } field;
} contract_t;

/**
 * @typedef pd_contract_info_t
 * @brief Stucture to hold PD Contract information passed in
 * APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE event to the application.
 */
typedef struct
{
    pd_do_t rdo;
    pd_contract_status_t status;
} pd_contract_info_t;

/**
 * @typedef pd_packet_t
 * @brief Struct to hold PD packet.
 */
typedef struct
{
    uint8_t     sop;
    uint8_t     len;
    uint8_t     msg;
    uint8_t     data_role;
    pd_hdr_t    hdr;
    pd_do_t     dat[MAX_NO_OF_DO];
} pd_packet_t;

/**
 * @typedef pd_packet_t
 * @brief Struct to hold PD packet.
 */
typedef struct
{
    uint8_t     sop;
    uint8_t     len;
    uint8_t     msg;
    uint8_t     data_role;
    pd_hdr_t    hdr;
    pd_do_t     dat[MAX_EXTD_PKT_WORDS];
} pd_packet_extd_t;

/********************** Callbacks **********************/

/**
 * @typedef: pd_cbk_t
 * @brief: PD callback prototype.
 *
 * This is a stack internal callback function used by the USB-PD Protocol
 * layer to send events to the Policy Engine. The events notified correspond
 * to Policy Engine events such as HARD RESET or SOFT RESET received.
 */
typedef void (*pd_cbk_t)(uint8_t port, uint32_t event);

/**
 * @enum: dpm_pd_cmd_cbk_t
 * @brief: DPM PD command callback
 */
typedef void (*dpm_pd_cmd_cbk_t)(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr);

/**
 * @enum: app_resp_cbk_t
 * @brief: Application response callback.
 */
typedef void (*app_resp_cbk_t)(uint8_t port, app_resp_t* resp);

/**
 * @enum: vdm_resp_cbk_t
 * @brief: VDM response callback.
 */
typedef void (*vdm_resp_cbk_t)(uint8_t port, vdm_resp_t* resp);

/**
 * @enum: pwr_ready_cbk_t
 * @brief: Power ready callback.
 */
typedef void (*pwr_ready_cbk_t)(uint8_t port);

/**
 * @enum: sink_discharge_off_cbk_t
 * @brief: Sink discharge off callback.
 */
typedef void (*sink_discharge_off_cbk_t)(uint8_t port);

/**
 * @enum: dpm_typec_cmd_cbk_t
 * @brief: Type C command response callback.
 */
typedef void (*dpm_typec_cmd_cbk_t)(uint8_t port, dpm_typec_cmd_resp_t resp);

/**
 * @typedef app_cbk_t
 * @brief Struct to hold the application interface. All the functions in this
 * structure must be non blocking and take minimum execution time.
 * @warning: The application must check the callback pointer passed by the
 * stack is not NULL.
 */
typedef struct
{
    /* App event handler callback. */
    void (*app_event_handler) (uint8_t port, app_evt_t evt ,const void* dat);

    /* Power source APIs. */
    /* Set voltage in 50mV units. */
    void (*psrc_set_voltage) (uint8_t port, uint16_t volt_50mV);
    /* Set current in 10mA units. */
    void (*psrc_set_current) (uint8_t port, uint16_t cur_10mA);
    /*
     * Enable the power supply. The pwr_ready_handler, if not NULL, will be
     * called when the power supply is ready.
     */
    void (*psrc_enable) (uint8_t port, pwr_ready_cbk_t pwr_ready_handler);
    /*
     * Disable the power supply. The pwr_ready_handler, if not NULL, will be
     * called when the power supply has been discharged to Vsafe0V.
     */
    void (*psrc_disable) (uint8_t port, pwr_ready_cbk_t pwr_ready_handler);
    /* Turn on VCONN. */
    void (*vconn_enable) (uint8_t port, uint8_t channel);
    /* Turn off VCONN. */
    void (*vconn_disable) (uint8_t port, uint8_t channel);
    /* Is supplying VCONN. */
    bool (*vconn_is_present) (uint8_t port);
    /* Is supplying Vbus. */
    bool (*vbus_is_present) (uint8_t port, uint16_t volt, int8_t per);

    /* Turn on VBUS discharge. */
    void (*vbus_discharge_on) (uint8_t port);

    /* Turn off VBUS discharge. */
    void (*vbus_discharge_off) (uint8_t port);

    /* Power sink APIs. */
    /* Set voltage, in 50mV units. */
    void (*psnk_set_voltage) (uint8_t port, uint16_t volt_50mV);
    /* Set current, in 10mA units. */
    void (*psnk_set_current) (uint8_t port, uint16_t cur_10mA);
    /* Power supply enable. */
    void (*psnk_enable) (uint8_t port);
    /* Power supply disable. */
    void (*psnk_disable) (uint8_t port, sink_discharge_off_cbk_t snk_discharge_off_handler);

    /* When response is ready, the application must call app_resp_handler. */
    /* Evaluate source cap. */
    void (*eval_src_cap) (uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler);
    /* Evaluate sink request message. */
    void (*eval_rdo)(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler);
    /* Handles DR swap request received by port. */
    void (*eval_dr_swap) (uint8_t port, app_resp_cbk_t app_resp_handler);
    /* Handles pr swap request received by port. */
    void (*eval_pr_swap) (uint8_t port, app_resp_cbk_t app_resp_handler);
    /* Handles VCONN swap request received by port. */
    void (*eval_vconn_swap) (uint8_t port, app_resp_cbk_t app_resp_handler);

    /*
     * Handles all VDMS (structured/unstructured). It is the responsibility of
     * APP to process or ignore a message. If a message must be ignored then
     * the proper response must be provided in vdm_resp_handler callback.
     *
     * If a previous response is pending from the application when this API is
     * called by the stack, the application should discard the previous
     * response and process the new VDM.
     */
    void (*eval_vdm) (uint8_t port, const pd_packet_t *vdm, vdm_resp_cbk_t vdm_resp_handler);

    /* Handle FR swap request received by the specified port. */
    void (*eval_fr_swap) (uint8_t port, app_resp_cbk_t app_resp_handler);

} app_cbk_t;

/**
 * @struct dpm_status_t
 * @brief PD Device Policy Status structure.
 * @warning Initial elements of this structure maps directly to config table
 * fields and hence must not be moved around or changed.
 */
typedef struct
{
    /***** Config table parameters start. *****/

    /** Port role: Sink, Source or Dual. */
    port_role_t port_role;

    /** Default port role: Sink or Source. */
    port_role_t dflt_port_role;

    /** Type C current level in the source role. */
    uint8_t src_cur_level;

    /** Power source is connected to a battery. */
    uint8_t is_src_bat;

    /** Power sink is connected to a battery. */
    uint8_t is_snk_bat;

    /** USB suspend supported indication. */
    uint8_t snk_usb_susp_en;

    /** USB communication supported indication. */
    uint8_t snk_usb_comm_en;

    /**
     * Response to be sent for each USB-PD SWAP command:
     * - Bits 1:0 => DR_SWAP response
     * - Bits 3:2 => PR_SWAP response
     * - Bits 5:4 => VCONN_SWAP response
     *
     * Allowed values are: 0 = ACCEPT, 1 = REJECT, 2 = WAIT
     */
    uint8_t swap_response;

    /** DRP toggle is enabled. */
    uint8_t toggle;

    /** Source PDO count from the config table or updated at runtime by the EC. */
    uint8_t src_pdo_count;

    /** Source PDO mask from the config table or updated at runtime by the EC. */
    uint8_t src_pdo_mask;

    /** Sink PDO count from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_count;

    /** Sink PDO mask from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_mask;

    /**
     * Supported Rp values.
     * This is a bit mask where each bit is interpreted as below. Multiple bits
     * may be set.
     * - Bit 0 => Default current
     * - Bit 1 => 1.5A
     * - Bit 2 => 3A
     */
    uint8_t rp_supported;

    /** USB-PD supported. */
    uint8_t pd_support;

    /** Try Source/ Try Sink control knob. */
    uint8_t try_src_snk;

    /** Cable discovery control knob. */
    uint8_t cbl_dsc;

    /** Dead battery support control knob. */
    uint8_t db_support;

    /** Error recovery control knob.*/
    uint8_t err_recov;

    /** PD port disable flag. */
    uint8_t port_disable;

    /** FRS enable flags. */
    uint8_t frs_enable;

    /** Reserved bytes for padding to 2-byte aligned address. */
    uint8_t reserved_2;

    /** Reserved words for padding to 4-byte aligned address. */
    uint16_t reserved_3[5];

    /** Source PDO loaded from the config table or updated at runtime by the EC. */
    pd_do_t src_pdo[MAX_NO_OF_PDO];

    /** Sink PDO loaded from the config table or updated at runtime by the EC. */
    pd_do_t snk_pdo[MAX_NO_OF_PDO];

    /**
     * Max min current from the config table or updated at runtime by the EC.
     *
     * Array of sink PDO parameters. For each element, the format is as below:
     * - Bit 15    => Give Back support flag.
     * - Bits 14:0 => Minimum sink operating current in 10 mA units.
     */
    uint16_t snk_max_min[MAX_NO_OF_PDO];

    /***** Config table parameters end. *****/

    /** Current port type: UFP or DFP. */
    port_type_t cur_port_type;

    /** Current Port role: Sink or Source. */
    port_role_t cur_port_role;

    /** Type C current level in sink role. */
    uint8_t volatile snk_cur_level;

    /** Flag to indicate chip bootup, used to check dead battery. */
    uint8_t volatile bootup;

    /** Flag to indicate dead battery operation. */
    uint8_t volatile dead_bat;

    /** Time period for DRP toggling. */
    uint8_t drp_period;

    /** Time period for which to stay as a SRC for a DRP device. */
    uint8_t src_period;

    /** Time period for which to stay as a SNK for a DRP device. */
    uint8_t snk_period;

    /** Skip CC scanning control knob. */
    uint8_t volatile skip_scan;

    /**
     * CC channel polarity.
     * - CC1 = 0
     * - CC2 = 1
     */
    uint8_t polarity;

    /** CC channel reverse polarity. */
    uint8_t rev_pol;

    /** Port connected but not debounced yet. */
    uint8_t volatile connect;

    /** Port attached indication. */
    uint8_t volatile attach;

    /** Port role when the port moved to the attached state. */
    port_role_t role_at_connect;

    /** Type of device attached. */
    pd_devtype_t attached_dev;

    /** PD contract exists indication. */
    uint8_t volatile contract_exist;

    /** Ports are PD connected indication. */
    uint8_t volatile pd_connected;

    /** PD disabled indication. */
    uint8_t volatile pd_disabled;

    /** Ra present indication. */
    uint8_t volatile ra_present;

    /** EMCA cable present indication, set when the cable responded to a discover ID. */
    uint8_t volatile emca_present;
    
    /** Stores the cable type. */
    std_vdm_prod_t cbl_type;
    
    /** Stores the cable VDM version. */
    std_vdm_ver_t cbl_vdm_version;    
    
    /** VCONN logical status. */
    uint8_t volatile vconn_logical;

    /** Source PDO count in the last sent source cap. */
    uint8_t cur_src_pdo_count;

    /** Sink PDO count in the last sent sink cap. */
    uint8_t cur_snk_pdo_count;

    /** PE Hard Reset AMS current state. */
    pe_hard_reset_ams_state_t pe_hr_ams_state;

    /** PE Bist Test data AMS current state. */
    pe_bist_test_data_ams_state_t pe_btd_ams_state;

    /** PE Bist Carrier mode 2 AMS current state. */
    pe_bist_cm2_ams_state_t pe_bcm2_ams_state;

    /** PE Soft Reset AMS current state. */
    pe_soft_reset_ams_state_t pe_sr_ams_state;

    /** PE Source contract AMS current state. */
    pe_src_contract_ams_state_t pe_src_ams_state;

    /** PE Sink Contract AMS current state. */
    pe_snk_contract_ams_state_t pe_snk_ams_state;

    /** PE Power role swap AMS current state. */
    pe_pr_swap_ams_state_t pe_pr_ams_state;

    /** PE Data role swap AMS current state. */
    pe_dr_swap_ams_state_t pe_dr_ams_state;

    /** PE VCONN swap AMS current state. */
    pe_vconn_swap_ams_state_t pe_vs_ams_state;

    /** PE Generic AMS state. */
    pe_generic_ams_state_t pe_gn_ams_state;

    /** Flag to indicate cable discovery is waiting for some event. */
    uint8_t cbl_wait;

    /**
     * Store cable discovery state and check if cable discovery is ongoing.
     * Cable discovery is not ongoing when cbl_state is set to
     * CBL_FSM_DISABLED.
     */
    pe_cbl_state_t cbl_state;

    /** Stores number of cable soft reset tries. */
    uint8_t cbl_soft_reset_tried;

    /** Type C generic FSM state. */
    typec_generic_fsm_state_t typec_gn_fsm_state;
    
    /** Current DPM PD command. */
    dpm_pd_cmd_t dpm_pd_cmd;

    /** Indicate DPM PD command was registered. */
    uint8_t dpm_pd_cmd_active;

    /** Indicate DPM Type C command was registered. */
    uint8_t dpm_typec_cmd_active;

    /** DPM enabled flag. */
    bool dpm_enabled;

    /** DPM Initialized flag. */
    bool dpm_init;

    /**
     * DPM safe disable flag, used to make sure the port is disabled once
     * again. Since OCP/OVP are handled in interrupt context and the state
     * machine is in the main loop it may happen that some unnecessary steps
     * are carried out by the state machine after an OCP/OVP event.
     */
    uint8_t dpm_safe_disable;

    /** Current DPM Type C command. */
    dpm_typec_cmd_t dpm_typec_cmd;

    /** Current Type C current level in the source role. */
    uint8_t src_cur_level_live;

    /** Live CC status. */
    cc_state_t volatile cc_live;

    /** Current debounced CC status. */
    cc_state_t volatile cc_status;

    /** Old CC status. */
    cc_state_t volatile cc_old_status;

    /** Rd status. */
    cc_state_t volatile cc_rd_status; 
    
    /** Current spec rev for SOP messages. */
    pd_rev_t spec_rev_sop_live;

    /** Current spec revision for SOP Prime/DPrime messages. */
    pd_rev_t spec_rev_sop_prime_live;

    /** Spec revision of the currently connected cable. */
    pd_rev_t spec_rev_cbl;

    /** Spec revision of the currently connected peer. */
    pd_rev_t spec_rev_peer;

    /** Mutual unchunk support with the currently connected peer. */
    bool unchunk_sup_live;

    /** Unchunk support of the currently connected peer. */
    bool unchunk_sup_peer;
    
    /* Flag to indicate stack is waiting for App response to a non interruptible AMS */
    pd_ams_type non_intr_response;    
    
    /** PE Fast role swap AMS current state. */
    pe_fr_swap_ams_state_t pe_fr_ams_state;

    /** FRS receive disabled by EC. */
    bool fr_rx_disabled;

    /** Fast role signal detection enabled */
    bool volatile fr_rx_en_live;    

    /** FRS transmit disabled by EC. */
    bool fr_tx_disabled;

    /** PE Fast role Sent Wait AMS current state. */
    pe_frs_src_snk_ams_state_t  pe_frs_src_snk_ams_state;
    
    /** Fast role signal auto send enabled */
    bool volatile fr_tx_en_live;       
    
    /**
     * Holds the current running AMS of all Policy Engine (PE) Atomic Message
     * Sequences (AMSs).
     */
    pe_fsm_ams_t pe_fsm;

    /** Holds the enable/disable state of all Type C sequences. */
    uint32_t typec_fsm;  
    
    /** Stores policy engine events. */
    uint32_t volatile pe_evt; 
    
    /** Stores Type C events. */
    uint32_t volatile typec_evt;
    
    /** Current pd contract. */
    contract_t contract;

    /** Stores the last received cable VDO. */
    pd_do_t cbl_vdo;

    /** Whether cable supports alternate modes. */
    bool cbl_mode_en;

    /** Application callback pointer. */
    app_cbk_t* app_cbk;

    /** Pointer to DPM PD callback function. */
    dpm_pd_cmd_cbk_t dpm_pd_cbk;

    /** Pointer to DPM Type C callback function. */
    dpm_typec_cmd_cbk_t dpm_typec_cbk;

    /** Pointer to DPM command buffer. */
    dpm_pd_cmd_buf_t* cmd_p;

    /** Local DPM command buffer. */
    dpm_pd_cmd_buf_t dpm_cmd_buf;

    /** Max min current/power of current sink capabilities. */
    uint16_t cur_snk_max_min[MAX_NO_OF_PDO];
    
    /**
     * Current source PDOs sent in source cap messages, these may be updated
     * due to cable caps or PDO mask.
     */
    pd_do_t cur_src_pdo[MAX_NO_OF_PDO];

    /**
     * Current sink PDOs sent in sink cap messages, these may be updated as per
     * sink PDO mask.
     */
    pd_do_t cur_snk_pdo[MAX_NO_OF_PDO];

    /**
     * Last RDO received, when operating as a source, that resulted in a
     * contract.
     */
    pd_do_t src_rdo;

    /**
     * last RDO sent, when operating as a sink, that resulted in a contract.
     */
    pd_do_t snk_rdo;

    /**
     * Selected PDO (from the source cap), when operating as a sink, that
     * resulted in a contract.
     */
    pd_do_t snk_sel_pdo;

    /**
     * Selected PDO (from the source cap), when operating as a source, that
     * resulted in a contract.
     */
    pd_do_t src_sel_pdo;
    
    /* Fields below need to be properly aligned to 4 byte boundary */
    uint32_t padval;

    pd_power_status_t port_status;

    /** Buffer to hold extended source caps data. Used only when PD 3.0 is enabled.
      * Four byte aligned */
    uint8_t ext_src_cap[CCG_PD_EXT_SRCCAP_SIZE + 1];

} dpm_status_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function gets the config table data.
 * @param port Port index.
 * @return Returns a pointer to the config table info structure.
 * @warning The information provided by this API must not be altered by the
 * application.
 */
const pd_config_t* get_pd_config(void);

/**
 * @brief This function gets the configuration information for the specified
 * port.
 * @param port Port index.
 * @return Returns a pointer to the port specific config table info structure.
 * @warning The information provided by this API must not be altered by the
 * application.
 */
const pd_port_config_t* get_pd_port_config(uint8_t port);

/**
 * @brief This is a utility function to check if the packet is an expected
 * message of a certain class.
 * @param pkt Packet pointer.
 * @param sop_type Sop type to match. Passing SOP_INVALID will skip this check.
 * @param msg_class Message class: Control, Data or Extended.
 * @param msg_mask Message mask is a 32 bit mask. Each bit corresponds to a
 * message type corresponding to the message class. If mask is 0 then this
 * check is skipped.
 * @param length Length corresponds to the 'Number of Data Objects' field in
 * the PD header. If length is 0xFFFF this check is skipped.
 * @return Returns true if packet matches all the conditions, else false.
 */
bool pd_is_msg(const pd_packet_t* pkt, sop_t sop_type, pd_msg_class_t msg_class, uint32_t msg_mask, uint16_t length);

#endif /* _PD_H_ */

/* End of file */

