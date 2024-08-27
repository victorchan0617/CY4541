/*******************************************************************************
* File Name: HPI_IF_PM.c
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

#include "HPI_IF.h"
#include "HPI_IF_PVT.h"

#if(HPI_IF_SCB_MODE_I2C_INC)
    #include "HPI_IF_I2C_PVT.h"
#endif /* (HPI_IF_SCB_MODE_I2C_INC) */

#if(HPI_IF_SCB_MODE_EZI2C_INC)
    #include "HPI_IF_EZI2C_PVT.h"
#endif /* (HPI_IF_SCB_MODE_EZI2C_INC) */

#if(HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC)
    #include "HPI_IF_SPI_UART_PVT.h"
#endif /* (HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG || \
   (HPI_IF_SCB_MODE_I2C_CONST_CFG   && (!HPI_IF_I2C_WAKE_ENABLE_CONST))   || \
   (HPI_IF_SCB_MODE_EZI2C_CONST_CFG && (!HPI_IF_EZI2C_WAKE_ENABLE_CONST)) || \
   (HPI_IF_SCB_MODE_SPI_CONST_CFG   && (!HPI_IF_SPI_WAKE_ENABLE_CONST))   || \
   (HPI_IF_SCB_MODE_UART_CONST_CFG  && (!HPI_IF_UART_WAKE_ENABLE_CONST)))

    HPI_IF_BACKUP_STRUCT HPI_IF_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: HPI_IF_Sleep
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
void HPI_IF_Sleep(void)
{
#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)

    if(HPI_IF_SCB_WAKE_ENABLE_CHECK)
    {
        if(HPI_IF_SCB_MODE_I2C_RUNTM_CFG)
        {
            HPI_IF_I2CSaveConfig();
        }
        else if(HPI_IF_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            HPI_IF_EzI2CSaveConfig();
        }
    #if(!HPI_IF_CY_SCBIP_V1)
        else if(HPI_IF_SCB_MODE_SPI_RUNTM_CFG)
        {
            HPI_IF_SpiSaveConfig();
        }
        else if(HPI_IF_SCB_MODE_UART_RUNTM_CFG)
        {
            HPI_IF_UartSaveConfig();
        }
    #endif /* (!HPI_IF_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        HPI_IF_backup.enableState = (uint8) HPI_IF_GET_CTRL_ENABLED;

        if(0u != HPI_IF_backup.enableState)
        {
            HPI_IF_Stop();
        }
    }

#else

    #if (HPI_IF_SCB_MODE_I2C_CONST_CFG && HPI_IF_I2C_WAKE_ENABLE_CONST)
        HPI_IF_I2CSaveConfig();

    #elif (HPI_IF_SCB_MODE_EZI2C_CONST_CFG && HPI_IF_EZI2C_WAKE_ENABLE_CONST)
        HPI_IF_EzI2CSaveConfig();

    #elif (HPI_IF_SCB_MODE_SPI_CONST_CFG && HPI_IF_SPI_WAKE_ENABLE_CONST)
        HPI_IF_SpiSaveConfig();

    #elif (HPI_IF_SCB_MODE_UART_CONST_CFG && HPI_IF_UART_WAKE_ENABLE_CONST)
        HPI_IF_UartSaveConfig();

    #else

        HPI_IF_backup.enableState = (uint8) HPI_IF_GET_CTRL_ENABLED;

        if(0u != HPI_IF_backup.enableState)
        {
            HPI_IF_Stop();
        }

    #endif /* defined (HPI_IF_SCB_MODE_I2C_CONST_CFG) && (HPI_IF_I2C_WAKE_ENABLE_CONST) */

#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: HPI_IF_Wakeup
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
void HPI_IF_Wakeup(void)
{
#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)

    if(HPI_IF_SCB_WAKE_ENABLE_CHECK)
    {
        if(HPI_IF_SCB_MODE_I2C_RUNTM_CFG)
        {
            HPI_IF_I2CRestoreConfig();
        }
        else if(HPI_IF_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            HPI_IF_EzI2CRestoreConfig();
        }
    #if(!HPI_IF_CY_SCBIP_V1)
        else if(HPI_IF_SCB_MODE_SPI_RUNTM_CFG)
        {
            HPI_IF_SpiRestoreConfig();
        }
        else if(HPI_IF_SCB_MODE_UART_RUNTM_CFG)
        {
            HPI_IF_UartRestoreConfig();
        }
    #endif /* (!HPI_IF_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != HPI_IF_backup.enableState)
        {
            HPI_IF_Enable();
        }
    }

#else

    #if (HPI_IF_SCB_MODE_I2C_CONST_CFG  && HPI_IF_I2C_WAKE_ENABLE_CONST)
        HPI_IF_I2CRestoreConfig();

    #elif (HPI_IF_SCB_MODE_EZI2C_CONST_CFG && HPI_IF_EZI2C_WAKE_ENABLE_CONST)
        HPI_IF_EzI2CRestoreConfig();

    #elif (HPI_IF_SCB_MODE_SPI_CONST_CFG && HPI_IF_SPI_WAKE_ENABLE_CONST)
        HPI_IF_SpiRestoreConfig();

    #elif (HPI_IF_SCB_MODE_UART_CONST_CFG && HPI_IF_UART_WAKE_ENABLE_CONST)
        HPI_IF_UartRestoreConfig();

    #else

        if(0u != HPI_IF_backup.enableState)
        {
            HPI_IF_Enable();
        }

    #endif /* (HPI_IF_I2C_WAKE_ENABLE_CONST) */

#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
