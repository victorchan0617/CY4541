/*******************************************************************************
* File Name: .h
* Version 3.10
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_MUX_CTRL_H)
#define CY_SCB_PVT_MUX_CTRL_H

#include "MUX_CTRL.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define MUX_CTRL_SetI2CExtClkInterruptMode(interruptMask) MUX_CTRL_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define MUX_CTRL_ClearI2CExtClkInterruptSource(interruptMask) MUX_CTRL_CLEAR_INTR_I2C_EC(interruptMask)
#define MUX_CTRL_GetI2CExtClkInterruptSource()                (MUX_CTRL_INTR_I2C_EC_REG)
#define MUX_CTRL_GetI2CExtClkInterruptMode()                  (MUX_CTRL_INTR_I2C_EC_MASK_REG)
#define MUX_CTRL_GetI2CExtClkInterruptSourceMasked()          (MUX_CTRL_INTR_I2C_EC_MASKED_REG)

#if (!MUX_CTRL_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define MUX_CTRL_SetSpiExtClkInterruptMode(interruptMask) \
                                                                MUX_CTRL_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define MUX_CTRL_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                MUX_CTRL_CLEAR_INTR_SPI_EC(interruptMask)
    #define MUX_CTRL_GetExtSpiClkInterruptSource()                 (MUX_CTRL_INTR_SPI_EC_REG)
    #define MUX_CTRL_GetExtSpiClkInterruptMode()                   (MUX_CTRL_INTR_SPI_EC_MASK_REG)
    #define MUX_CTRL_GetExtSpiClkInterruptSourceMasked()           (MUX_CTRL_INTR_SPI_EC_MASKED_REG)
#endif /* (!MUX_CTRL_CY_SCBIP_V1) */

#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void MUX_CTRL_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (MUX_CTRL_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER)
    extern cyisraddress MUX_CTRL_customIntrHandler;
#endif /* !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER) */
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */

extern MUX_CTRL_BACKUP_STRUCT MUX_CTRL_backup;

#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 MUX_CTRL_scbMode;
    extern uint8 MUX_CTRL_scbEnableWake;
    extern uint8 MUX_CTRL_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 MUX_CTRL_mode;
    extern uint8 MUX_CTRL_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * MUX_CTRL_rxBuffer;
    extern uint8   MUX_CTRL_rxDataBits;
    extern uint32  MUX_CTRL_rxBufferSize;

    extern volatile uint8 * MUX_CTRL_txBuffer;
    extern uint8   MUX_CTRL_txDataBits;
    extern uint32  MUX_CTRL_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 MUX_CTRL_numberOfAddr;
    extern uint8 MUX_CTRL_subAddrSize;
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*        Conditional Macro
****************************************/

#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define MUX_CTRL_SCB_MODE_I2C_RUNTM_CFG     (MUX_CTRL_SCB_MODE_I2C      == MUX_CTRL_scbMode)
    #define MUX_CTRL_SCB_MODE_SPI_RUNTM_CFG     (MUX_CTRL_SCB_MODE_SPI      == MUX_CTRL_scbMode)
    #define MUX_CTRL_SCB_MODE_UART_RUNTM_CFG    (MUX_CTRL_SCB_MODE_UART     == MUX_CTRL_scbMode)
    #define MUX_CTRL_SCB_MODE_EZI2C_RUNTM_CFG   (MUX_CTRL_SCB_MODE_EZI2C    == MUX_CTRL_scbMode)
    #define MUX_CTRL_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (MUX_CTRL_SCB_MODE_UNCONFIG == MUX_CTRL_scbMode)

    /* Defines wakeup enable */
    #define MUX_CTRL_SCB_WAKE_ENABLE_CHECK       (0u != MUX_CTRL_scbEnableWake)
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!MUX_CTRL_CY_SCBIP_V1)
    #define MUX_CTRL_SCB_PINS_NUMBER    (7u)
#else
    #define MUX_CTRL_SCB_PINS_NUMBER    (2u)
#endif /* (!MUX_CTRL_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_MUX_CTRL_H) */


/* [] END OF FILE */
