/*******************************************************************************
* File Name: VBUS_P_CTRL_P2.h  
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

#if !defined(CY_PINS_VBUS_P_CTRL_P2_ALIASES_H) /* Pins VBUS_P_CTRL_P2_ALIASES_H */
#define CY_PINS_VBUS_P_CTRL_P2_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define VBUS_P_CTRL_P2_0			(VBUS_P_CTRL_P2__0__PC)
#define VBUS_P_CTRL_P2_0_PS		(VBUS_P_CTRL_P2__0__PS)
#define VBUS_P_CTRL_P2_0_PC		(VBUS_P_CTRL_P2__0__PC)
#define VBUS_P_CTRL_P2_0_DR		(VBUS_P_CTRL_P2__0__DR)
#define VBUS_P_CTRL_P2_0_SHIFT	(VBUS_P_CTRL_P2__0__SHIFT)
#define VBUS_P_CTRL_P2_0_INTR	((uint16)((uint16)0x0003u << (VBUS_P_CTRL_P2__0__SHIFT*2u)))

#define VBUS_P_CTRL_P2_INTR_ALL	 ((uint16)(VBUS_P_CTRL_P2_0_INTR))


#endif /* End Pins VBUS_P_CTRL_P2_ALIASES_H */


/* [] END OF FILE */
