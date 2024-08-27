/*******************************************************************************
* File Name: HPD_P2.h  
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

#if !defined(CY_PINS_HPD_P2_ALIASES_H) /* Pins HPD_P2_ALIASES_H */
#define CY_PINS_HPD_P2_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define HPD_P2_0			(HPD_P2__0__PC)
#define HPD_P2_0_PS		(HPD_P2__0__PS)
#define HPD_P2_0_PC		(HPD_P2__0__PC)
#define HPD_P2_0_DR		(HPD_P2__0__DR)
#define HPD_P2_0_SHIFT	(HPD_P2__0__SHIFT)
#define HPD_P2_0_INTR	((uint16)((uint16)0x0003u << (HPD_P2__0__SHIFT*2u)))

#define HPD_P2_INTR_ALL	 ((uint16)(HPD_P2_0_INTR))


#endif /* End Pins HPD_P2_ALIASES_H */


/* [] END OF FILE */
