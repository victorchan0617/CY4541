/*******************************************************************************
* File Name: VSEL2_P1.h  
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

#if !defined(CY_PINS_VSEL2_P1_ALIASES_H) /* Pins VSEL2_P1_ALIASES_H */
#define CY_PINS_VSEL2_P1_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define VSEL2_P1_0			(VSEL2_P1__0__PC)
#define VSEL2_P1_0_PS		(VSEL2_P1__0__PS)
#define VSEL2_P1_0_PC		(VSEL2_P1__0__PC)
#define VSEL2_P1_0_DR		(VSEL2_P1__0__DR)
#define VSEL2_P1_0_SHIFT	(VSEL2_P1__0__SHIFT)
#define VSEL2_P1_0_INTR	((uint16)((uint16)0x0003u << (VSEL2_P1__0__SHIFT*2u)))

#define VSEL2_P1_INTR_ALL	 ((uint16)(VSEL2_P1_0_INTR))


#endif /* End Pins VSEL2_P1_ALIASES_H */


/* [] END OF FILE */
