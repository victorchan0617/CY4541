/**
 * @file vdm.h
 *
 * @brief @{Vendor Defined Message (VDM) handler header file.@}
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

#ifndef _VDM_H_
#define _VDM_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Store the VDM data from the configuration table.
 *
 * This function retrieves the VDM data (for CCG as UFP) that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be stored.
 *
 * @return None.
 */
void vdm_data_init (uint8_t port);

/**
 * @brief This function allows the VDM data for CCG to be changed.
 *
 * This function allows the user to change the VDM responses that CCG sends
 * for D_ID, D_SVID and D_MODE requests. The default responses are taken
 * from the configuration table. This function allows the user to change
 * the response data. The caller is responsible to ensure that the responses
 * are not changed while CCG is already in contract as a UFP.
 *
 * @param port PD port for which responses are to be changed.
 * @param id_vdo_cnt Number of VDOs in the D_ID response.
 * @param id_vdo_p Pointer to the actual D_ID response in memory.
 * @param svid_vdo_cnt Number of VDOs in the D_SVID response.
 *        Should be less than 8.
 * @param svid_vdo_p Pointer to the actual D_SVID response in memory.
 * @param mode_resp_len Total length of mode response. This includes
 *        the D_MODE responses for each supported SVID, along with the
 *        corresponding header fields.
 * @param mode_resp_p Pointer to all of the mode responses in memory.
 *
 * @return None
 */
void vdm_update_data(uint8_t port, uint8_t id_vdo_cnt, uint8_t *id_vdo_p,
        uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p, uint16_t mode_resp_len,
        uint8_t *mode_resp_p);

/**
 * @brief This function is responsible for analysing and processing received VDM.
 * This function also makes a decision about necessity of response to the received
 * VDM.
 *
 * @param port Port index the function is performed for.
 * @param vdm Pointer to pd packet which contains received VDM.
 * @param vdm_resp_handler VDM handler callback function.
 *
 * @return None
 */
void eval_vdm(uint8_t port, const pd_packet_t *vdm,
        vdm_resp_cbk_t vdm_resp_handler);

/**
 * @brief This function is responsible for getting Discover Mode info from config table .
 *
 * @param port Port index the function is performed for.
 * @param svid SVID which the infofmation is searching for.
 * @param temp_p Temporary pointer to the pointer of PD data object.
 * @param no_of_vdo Pointer to the variable which contains number of VDOs 
 *        in Disc MODE response.
 *
 * @return Returns true if config table contains Disc MODE info for input SVID.
 *         Else returns false.
 */
bool get_modes_vdo_info(uint8_t port, uint16_t svid, pd_do_t **temp_p,
	    uint8_t *no_of_vdo);

#endif /* _VDM_H_ */
/* End of File */

