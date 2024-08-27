/*******************************************************************************
* File Name: VCONN_MON_P1.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VCONN_MON_P1_ALIASES_H) /* Pins VCONN_MON_P1_ALIASES_H */
#define CY_PINS_VCONN_MON_P1_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define VCONN_MON_P1_0			(VCONN_MON_P1__0__PC)
#define VCONN_MON_P1_0_PS		(VCONN_MON_P1__0__PS)
#define VCONN_MON_P1_0_PC		(VCONN_MON_P1__0__PC)
#define VCONN_MON_P1_0_DR		(VCONN_MON_P1__0__DR)
#define VCONN_MON_P1_0_SHIFT	(VCONN_MON_P1__0__SHIFT)
#define VCONN_MON_P1_0_INTR	((uint16)((uint16)0x0003u << (VCONN_MON_P1__0__SHIFT*2u)))

#define VCONN_MON_P1_INTR_ALL	 ((uint16)(VCONN_MON_P1_0_INTR))


#endif /* End Pins VCONN_MON_P1_ALIASES_H */


/* [] END OF FILE */
