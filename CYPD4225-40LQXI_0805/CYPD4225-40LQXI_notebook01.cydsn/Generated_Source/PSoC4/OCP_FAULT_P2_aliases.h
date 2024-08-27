/*******************************************************************************
* File Name: OCP_FAULT_P2.h  
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

#if !defined(CY_PINS_OCP_FAULT_P2_ALIASES_H) /* Pins OCP_FAULT_P2_ALIASES_H */
#define CY_PINS_OCP_FAULT_P2_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define OCP_FAULT_P2_0			(OCP_FAULT_P2__0__PC)
#define OCP_FAULT_P2_0_PS		(OCP_FAULT_P2__0__PS)
#define OCP_FAULT_P2_0_PC		(OCP_FAULT_P2__0__PC)
#define OCP_FAULT_P2_0_DR		(OCP_FAULT_P2__0__DR)
#define OCP_FAULT_P2_0_SHIFT	(OCP_FAULT_P2__0__SHIFT)
#define OCP_FAULT_P2_0_INTR	((uint16)((uint16)0x0003u << (OCP_FAULT_P2__0__SHIFT*2u)))

#define OCP_FAULT_P2_INTR_ALL	 ((uint16)(OCP_FAULT_P2_0_INTR))


#endif /* End Pins OCP_FAULT_P2_ALIASES_H */


/* [] END OF FILE */
