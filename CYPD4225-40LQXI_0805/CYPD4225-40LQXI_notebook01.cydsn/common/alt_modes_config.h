/**
 * @file alt_modes_config.h
 *
 * @brief @{Header file that selects the Alternate Modes supported by CCG
 * firmware.@}
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

#ifndef _ALT_MODES_CONFIG_H_
#define _ALT_MODES_CONFIG_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <project.h>
#include <alt_modes_mngr.h>
#include <dp_sid.h>
#include <config.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* Only a single SVID (DisplayPort) is supported when CCG is DFP. */
#define DFP_MAX_SVID_SUPP                    (1u)

/* No SVIDs supported when CCG is UFP. */
#define UFP_MAX_SVID_SUPP                    (0u)

/* Maximum number of SVIDs supported across operating modes. */
#define MAX_SVID_SUPP                                                                   \
    (DFP_MAX_SVID_SUPP > UFP_MAX_SVID_SUPP ? DFP_MAX_SVID_SUPP : UFP_MAX_SVID_SUPP)

/*****************************************************************************
 * Global Variable Declaration
 *****************************************************************************/

#ifdef DFP_ALT_MODE_SUPP

/*
   This table maps the SVID for the alternate mode to an alternate mode index to
   be used in the Alt. mode manager.
 */
const comp_tbl_t
dfp_compatibility_mode_table[(DFP_MAX_SVID_SUPP)][(DFP_MAX_SVID_SUPP)] =
{
    {{DP_SVID, DP_ALT_MODE_ID}},        /* Use DisplayPort SVID and Alt. Mode index 0. */
};

#endif /* DFP_ALT_MODE_SUPP */

#if (MAX_SVID_SUPP != 0)

/* This table holds the SVIDs for the various alt. modes supported by the solution.
   Note: The order of SVIDs in this array need to match the corresponding handler
         functions in the is_alt_mode_allowed array defined below.
 */
const uint16_t
supp_svid_tbl[MAX_SVID_SUPP] =
{
    DP_SVID                             /* DP_ALT_MODE */
};


/* This table holds functions to register SVID functionality during Registering Alt Mode   */
alt_mode_info_t* (*const is_alt_mode_allowed [(uint8_t)(MAX_SVID_SUPP)]) (uint8_t, alt_mode_reg_info_t*) =
{
    reg_dp_modes                        /* DisplayPort mode handler. */
};

#endif /* MAX_SVID_SUPP */

#endif /* _ALT_MODES_CONFIG_H_ */

/* [] END OF FILE */

