/*******************************************************************************
* File Name: HPD_Timer_PM.c
* Version 2.0
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HPD_Timer.h"

static HPD_Timer_BACKUP_STRUCT HPD_Timer_backup;


/*******************************************************************************
* Function Name: HPD_Timer_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void HPD_Timer_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: HPD_Timer_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void HPD_Timer_Sleep(void)
{
    if(0u != (HPD_Timer_BLOCK_CONTROL_REG & HPD_Timer_MASK))
    {
        HPD_Timer_backup.enableState = 1u;
    }
    else
    {
        HPD_Timer_backup.enableState = 0u;
    }

    HPD_Timer_Stop();
    HPD_Timer_SaveConfig();
}


/*******************************************************************************
* Function Name: HPD_Timer_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void HPD_Timer_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: HPD_Timer_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void HPD_Timer_Wakeup(void)
{
    HPD_Timer_RestoreConfig();

    if(0u != HPD_Timer_backup.enableState)
    {
        HPD_Timer_Enable();
    }
}


/* [] END OF FILE */
