/*******************************************************************************
* File Name: MUX_CTRL_PM.c
* Version 3.10
*
* Description:
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "MUX_CTRL.h"
#include "MUX_CTRL_PVT.h"

#if(MUX_CTRL_SCB_MODE_I2C_INC)
    #include "MUX_CTRL_I2C_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_I2C_INC) */

#if(MUX_CTRL_SCB_MODE_EZI2C_INC)
    #include "MUX_CTRL_EZI2C_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_EZI2C_INC) */

#if(MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC)
    #include "MUX_CTRL_SPI_UART_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG || \
   (MUX_CTRL_SCB_MODE_I2C_CONST_CFG   && (!MUX_CTRL_I2C_WAKE_ENABLE_CONST))   || \
   (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG && (!MUX_CTRL_EZI2C_WAKE_ENABLE_CONST)) || \
   (MUX_CTRL_SCB_MODE_SPI_CONST_CFG   && (!MUX_CTRL_SPI_WAKE_ENABLE_CONST))   || \
   (MUX_CTRL_SCB_MODE_UART_CONST_CFG  && (!MUX_CTRL_UART_WAKE_ENABLE_CONST)))

    MUX_CTRL_BACKUP_STRUCT MUX_CTRL_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: MUX_CTRL_Sleep
********************************************************************************
*
* Summary:
*  Prepares the component to enter Deep Sleep.
*  The "Enable wakeup from Sleep Mode" selection has an influence on
*  this function implementation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_Sleep(void)
{
#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)

    if(MUX_CTRL_SCB_WAKE_ENABLE_CHECK)
    {
        if(MUX_CTRL_SCB_MODE_I2C_RUNTM_CFG)
        {
            MUX_CTRL_I2CSaveConfig();
        }
        else if(MUX_CTRL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MUX_CTRL_EzI2CSaveConfig();
        }
    #if(!MUX_CTRL_CY_SCBIP_V1)
        else if(MUX_CTRL_SCB_MODE_SPI_RUNTM_CFG)
        {
            MUX_CTRL_SpiSaveConfig();
        }
        else if(MUX_CTRL_SCB_MODE_UART_RUNTM_CFG)
        {
            MUX_CTRL_UartSaveConfig();
        }
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        MUX_CTRL_backup.enableState = (uint8) MUX_CTRL_GET_CTRL_ENABLED;

        if(0u != MUX_CTRL_backup.enableState)
        {
            MUX_CTRL_Stop();
        }
    }

#else

    #if (MUX_CTRL_SCB_MODE_I2C_CONST_CFG && MUX_CTRL_I2C_WAKE_ENABLE_CONST)
        MUX_CTRL_I2CSaveConfig();

    #elif (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG && MUX_CTRL_EZI2C_WAKE_ENABLE_CONST)
        MUX_CTRL_EzI2CSaveConfig();

    #elif (MUX_CTRL_SCB_MODE_SPI_CONST_CFG && MUX_CTRL_SPI_WAKE_ENABLE_CONST)
        MUX_CTRL_SpiSaveConfig();

    #elif (MUX_CTRL_SCB_MODE_UART_CONST_CFG && MUX_CTRL_UART_WAKE_ENABLE_CONST)
        MUX_CTRL_UartSaveConfig();

    #else

        MUX_CTRL_backup.enableState = (uint8) MUX_CTRL_GET_CTRL_ENABLED;

        if(0u != MUX_CTRL_backup.enableState)
        {
            MUX_CTRL_Stop();
        }

    #endif /* defined (MUX_CTRL_SCB_MODE_I2C_CONST_CFG) && (MUX_CTRL_I2C_WAKE_ENABLE_CONST) */

#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_Wakeup
********************************************************************************
*
* Summary:
*  Prepares the component for the Active mode operation after exiting
*  Deep Sleep. The "Enable wakeup from Sleep Mode" option has an influence
*  on this function implementation.
*  This function should not be called after exiting Sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_Wakeup(void)
{
#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)

    if(MUX_CTRL_SCB_WAKE_ENABLE_CHECK)
    {
        if(MUX_CTRL_SCB_MODE_I2C_RUNTM_CFG)
        {
            MUX_CTRL_I2CRestoreConfig();
        }
        else if(MUX_CTRL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            MUX_CTRL_EzI2CRestoreConfig();
        }
    #if(!MUX_CTRL_CY_SCBIP_V1)
        else if(MUX_CTRL_SCB_MODE_SPI_RUNTM_CFG)
        {
            MUX_CTRL_SpiRestoreConfig();
        }
        else if(MUX_CTRL_SCB_MODE_UART_RUNTM_CFG)
        {
            MUX_CTRL_UartRestoreConfig();
        }
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != MUX_CTRL_backup.enableState)
        {
            MUX_CTRL_Enable();
        }
    }

#else

    #if (MUX_CTRL_SCB_MODE_I2C_CONST_CFG  && MUX_CTRL_I2C_WAKE_ENABLE_CONST)
        MUX_CTRL_I2CRestoreConfig();

    #elif (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG && MUX_CTRL_EZI2C_WAKE_ENABLE_CONST)
        MUX_CTRL_EzI2CRestoreConfig();

    #elif (MUX_CTRL_SCB_MODE_SPI_CONST_CFG && MUX_CTRL_SPI_WAKE_ENABLE_CONST)
        MUX_CTRL_SpiRestoreConfig();

    #elif (MUX_CTRL_SCB_MODE_UART_CONST_CFG && MUX_CTRL_UART_WAKE_ENABLE_CONST)
        MUX_CTRL_UartRestoreConfig();

    #else

        if(0u != MUX_CTRL_backup.enableState)
        {
            MUX_CTRL_Enable();
        }

    #endif /* (MUX_CTRL_I2C_WAKE_ENABLE_CONST) */

#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
