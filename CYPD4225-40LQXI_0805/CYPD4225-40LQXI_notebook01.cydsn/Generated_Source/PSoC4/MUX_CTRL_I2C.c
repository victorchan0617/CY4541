/*******************************************************************************
* File Name: MUX_CTRL_I2C.c
* Version 3.10
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  I2C mode.
*
* Note:
*
*******************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "MUX_CTRL_PVT.h"
#include "MUX_CTRL_I2C_PVT.h"


/***************************************
*      I2C Private Vars
***************************************/

volatile uint8 MUX_CTRL_state;  /* Current state of I2C FSM */

#if(MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    /* Constant configuration of I2C */
    const MUX_CTRL_I2C_INIT_STRUCT MUX_CTRL_configI2C =
    {
        MUX_CTRL_I2C_MODE,
        MUX_CTRL_I2C_OVS_FACTOR_LOW,
        MUX_CTRL_I2C_OVS_FACTOR_HIGH,
        MUX_CTRL_I2C_MEDIAN_FILTER_ENABLE,
        MUX_CTRL_I2C_SLAVE_ADDRESS,
        MUX_CTRL_I2C_SLAVE_ADDRESS_MASK,
        MUX_CTRL_I2C_ACCEPT_ADDRESS,
        MUX_CTRL_I2C_WAKE_ENABLE,
        MUX_CTRL_I2C_BYTE_MODE_ENABLE,
        MUX_CTRL_I2C_DATA_RATE,
        MUX_CTRL_I2C_ACCEPT_GENERAL_CALL,
    };

    /*******************************************************************************
    * Function Name: MUX_CTRL_I2CInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for I2C operation.
    *
    * Parameters:
    *  config:  Pointer to a structure that contains the following ordered list of
    *           fields. These fields match the selections available in the
    *           customizer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_I2CInit(const MUX_CTRL_I2C_INIT_STRUCT *config)
    {
        uint32 medianFilter;
        uint32 locEnableWake;

        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Configure pins */
            MUX_CTRL_SetPins(MUX_CTRL_SCB_MODE_I2C, MUX_CTRL_DUMMY_PARAM,
                                     MUX_CTRL_DUMMY_PARAM);

            /* Store internal configuration */
            MUX_CTRL_scbMode       = (uint8) MUX_CTRL_SCB_MODE_I2C;
            MUX_CTRL_scbEnableWake = (uint8) config->enableWake;
            MUX_CTRL_scbEnableIntr = (uint8) MUX_CTRL_SCB_IRQ_INTERNAL;

            MUX_CTRL_mode          = (uint8) config->mode;
            MUX_CTRL_acceptAddr    = (uint8) config->acceptAddr;

        #if (MUX_CTRL_CY_SCBIP_V0)
            /* Adjust SDA filter settings. Ticket ID#150521 */
            MUX_CTRL_SET_I2C_CFG_SDA_FILT_TRIM(MUX_CTRL_EC_AM_I2C_CFG_SDA_FILT_TRIM);
        #endif /* (MUX_CTRL_CY_SCBIP_V0) */

            /* Adjust AF and DF filter settings. Ticket ID#176179 */
            if (((MUX_CTRL_I2C_MODE_SLAVE != config->mode) &&
                 (config->dataRate <= MUX_CTRL_I2C_DATA_RATE_FS_MODE_MAX)) ||
                 (MUX_CTRL_I2C_MODE_SLAVE == config->mode))
            {
                /* AF = 1, DF = 0 */
                MUX_CTRL_I2C_CFG_ANALOG_FITER_ENABLE;
                medianFilter = MUX_CTRL_DIGITAL_FILTER_DISABLE;
            }
            else
            {
                /* AF = 0, DF = 1 */
                MUX_CTRL_I2C_CFG_ANALOG_FITER_DISABLE;
                medianFilter = MUX_CTRL_DIGITAL_FILTER_ENABLE;
            }

        #if (!MUX_CTRL_CY_SCBIP_V0)
            locEnableWake = (MUX_CTRL_I2C_MULTI_MASTER_SLAVE) ? (0u) : (config->enableWake);
        #else
            locEnableWake = config->enableWake;
        #endif /* (!MUX_CTRL_CY_SCBIP_V0) */

            /* Configure I2C interface */
            MUX_CTRL_CTRL_REG     = MUX_CTRL_GET_CTRL_BYTE_MODE  (config->enableByteMode) |
                                            MUX_CTRL_GET_CTRL_ADDR_ACCEPT(config->acceptAddr)     |
                                            MUX_CTRL_GET_CTRL_EC_AM_MODE (locEnableWake);

            MUX_CTRL_I2C_CTRL_REG = MUX_CTRL_GET_I2C_CTRL_HIGH_PHASE_OVS(config->oversampleHigh) |
                    MUX_CTRL_GET_I2C_CTRL_LOW_PHASE_OVS (config->oversampleLow)                          |
                    MUX_CTRL_GET_I2C_CTRL_S_GENERAL_IGNORE((uint32)(0u == config->acceptGeneralAddr))    |
                    MUX_CTRL_GET_I2C_CTRL_SL_MSTR_MODE  (config->mode);

            /* Configure RX direction */
            MUX_CTRL_RX_CTRL_REG      = MUX_CTRL_GET_RX_CTRL_MEDIAN(medianFilter) |
                                                MUX_CTRL_I2C_RX_CTRL;
            MUX_CTRL_RX_FIFO_CTRL_REG = MUX_CTRL_CLEAR_REG;

            /* Set default address and mask */
            MUX_CTRL_RX_MATCH_REG    = ((MUX_CTRL_I2C_SLAVE) ?
                                                (MUX_CTRL_GET_I2C_8BIT_ADDRESS(config->slaveAddr) |
                                                 MUX_CTRL_GET_RX_MATCH_MASK(config->slaveAddrMask)) :
                                                (MUX_CTRL_CLEAR_REG));


            /* Configure TX direction */
            MUX_CTRL_TX_CTRL_REG      = MUX_CTRL_I2C_TX_CTRL;
            MUX_CTRL_TX_FIFO_CTRL_REG = MUX_CTRL_CLEAR_REG;

            /* Configure interrupt with I2C handler but do not enable it */
            CyIntDisable    (MUX_CTRL_ISR_NUMBER);
            CyIntSetPriority(MUX_CTRL_ISR_NUMBER, MUX_CTRL_ISR_PRIORITY);
            (void) CyIntSetVector(MUX_CTRL_ISR_NUMBER, &MUX_CTRL_I2C_ISR);

            /* Configure interrupt sources */
        #if(!MUX_CTRL_CY_SCBIP_V1)
            MUX_CTRL_INTR_SPI_EC_MASK_REG = MUX_CTRL_NO_INTR_SOURCES;
        #endif /* (!MUX_CTRL_CY_SCBIP_V1) */

            MUX_CTRL_INTR_I2C_EC_MASK_REG = MUX_CTRL_NO_INTR_SOURCES;
            MUX_CTRL_INTR_RX_MASK_REG     = MUX_CTRL_NO_INTR_SOURCES;
            MUX_CTRL_INTR_TX_MASK_REG     = MUX_CTRL_NO_INTR_SOURCES;

            MUX_CTRL_INTR_SLAVE_MASK_REG  = ((MUX_CTRL_I2C_SLAVE) ?
                            (MUX_CTRL_GET_INTR_SLAVE_I2C_GENERAL(config->acceptGeneralAddr) |
                             MUX_CTRL_I2C_INTR_SLAVE_MASK) : (MUX_CTRL_CLEAR_REG));

            MUX_CTRL_INTR_MASTER_MASK_REG = ((MUX_CTRL_I2C_MASTER) ?
                                                     (MUX_CTRL_I2C_INTR_MASTER_MASK) :
                                                     (MUX_CTRL_CLEAR_REG));

            /* Configure global variables */
            MUX_CTRL_state = MUX_CTRL_I2C_FSM_IDLE;

            /* Internal slave variables */
            MUX_CTRL_slStatus        = 0u;
            MUX_CTRL_slRdBufIndex    = 0u;
            MUX_CTRL_slWrBufIndex    = 0u;
            MUX_CTRL_slOverFlowCount = 0u;

            /* Internal master variables */
            MUX_CTRL_mstrStatus     = 0u;
            MUX_CTRL_mstrRdBufIndex = 0u;
            MUX_CTRL_mstrWrBufIndex = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: MUX_CTRL_I2CInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for the I2C operation.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_I2CInit(void)
    {
    #if(MUX_CTRL_CY_SCBIP_V0)
        /* Adjust SDA filter settings. Ticket ID#150521 */
        MUX_CTRL_SET_I2C_CFG_SDA_FILT_TRIM(MUX_CTRL_EC_AM_I2C_CFG_SDA_FILT_TRIM);
    #endif /* (MUX_CTRL_CY_SCBIP_V0) */

        /* Adjust AF and DF filter settings. Ticket ID#176179 */
        MUX_CTRL_I2C_CFG_ANALOG_FITER_ENABLE_ADJ;

        /* Configure I2C interface */
        MUX_CTRL_CTRL_REG     = MUX_CTRL_I2C_DEFAULT_CTRL;
        MUX_CTRL_I2C_CTRL_REG = MUX_CTRL_I2C_DEFAULT_I2C_CTRL;

        /* Configure RX direction */
        MUX_CTRL_RX_CTRL_REG      = MUX_CTRL_I2C_DEFAULT_RX_CTRL;
        MUX_CTRL_RX_FIFO_CTRL_REG = MUX_CTRL_I2C_DEFAULT_RX_FIFO_CTRL;

        /* Set default address and mask */
        MUX_CTRL_RX_MATCH_REG     = MUX_CTRL_I2C_DEFAULT_RX_MATCH;

        /* Configure TX direction */
        MUX_CTRL_TX_CTRL_REG      = MUX_CTRL_I2C_DEFAULT_TX_CTRL;
        MUX_CTRL_TX_FIFO_CTRL_REG = MUX_CTRL_I2C_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with I2C handler but do not enable it */
        CyIntDisable    (MUX_CTRL_ISR_NUMBER);
        CyIntSetPriority(MUX_CTRL_ISR_NUMBER, MUX_CTRL_ISR_PRIORITY);
    #if(!MUX_CTRL_I2C_EXTERN_INTR_HANDLER)
        (void) CyIntSetVector(MUX_CTRL_ISR_NUMBER, &MUX_CTRL_I2C_ISR);
    #endif /* (MUX_CTRL_I2C_EXTERN_INTR_HANDLER) */

        /* Configure interrupt sources */
    #if(!MUX_CTRL_CY_SCBIP_V1)
        MUX_CTRL_INTR_SPI_EC_MASK_REG = MUX_CTRL_I2C_DEFAULT_INTR_SPI_EC_MASK;
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */

        MUX_CTRL_INTR_I2C_EC_MASK_REG = MUX_CTRL_I2C_DEFAULT_INTR_I2C_EC_MASK;
        MUX_CTRL_INTR_SLAVE_MASK_REG  = MUX_CTRL_I2C_DEFAULT_INTR_SLAVE_MASK;
        MUX_CTRL_INTR_MASTER_MASK_REG = MUX_CTRL_I2C_DEFAULT_INTR_MASTER_MASK;
        MUX_CTRL_INTR_RX_MASK_REG     = MUX_CTRL_I2C_DEFAULT_INTR_RX_MASK;
        MUX_CTRL_INTR_TX_MASK_REG     = MUX_CTRL_I2C_DEFAULT_INTR_TX_MASK;

        /* Configure global variables */
        MUX_CTRL_state = MUX_CTRL_I2C_FSM_IDLE;

    #if(MUX_CTRL_I2C_SLAVE)
        /* Internal slave variable */
        MUX_CTRL_slStatus        = 0u;
        MUX_CTRL_slRdBufIndex    = 0u;
        MUX_CTRL_slWrBufIndex    = 0u;
        MUX_CTRL_slOverFlowCount = 0u;
    #endif /* (MUX_CTRL_I2C_SLAVE) */

    #if(MUX_CTRL_I2C_MASTER)
    /* Internal master variable */
        MUX_CTRL_mstrStatus     = 0u;
        MUX_CTRL_mstrRdBufIndex = 0u;
        MUX_CTRL_mstrWrBufIndex = 0u;
    #endif /* (MUX_CTRL_I2C_MASTER) */
    }
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: MUX_CTRL_I2CStop
********************************************************************************
*
* Summary:
*  Resets the I2C FSM into the default state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_I2CStop(void)
{
    MUX_CTRL_state = MUX_CTRL_I2C_FSM_IDLE;
}


#if(MUX_CTRL_I2C_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: MUX_CTRL_I2CSaveConfig
    ********************************************************************************
    *
    * Summary:
    *  Enables MUX_CTRL_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_I2CSaveConfig(void)
    {
    #if (!MUX_CTRL_CY_SCBIP_V0)
        #if (MUX_CTRL_I2C_MULTI_MASTER_SLAVE_CONST && MUX_CTRL_I2C_WAKE_ENABLE_CONST)
            /* Enable externally clocked address match if it was not enabled before.
            * This applicable only for Multi-Master-Slave. Ticket ID#192742 */
            if (0u == (MUX_CTRL_CTRL_REG & MUX_CTRL_CTRL_EC_AM_MODE))
            {
                /* Enable external address match logic */
                MUX_CTRL_Stop();
                MUX_CTRL_CTRL_REG |= MUX_CTRL_CTRL_EC_AM_MODE;
                MUX_CTRL_Enable();
            }
        #endif /* (MUX_CTRL_I2C_MULTI_MASTER_SLAVE_CONST) */

        #if (MUX_CTRL_SCB_CLK_INTERNAL)
            /* Disable clock to internal address match logic. Ticket ID#187931 */
            MUX_CTRL_SCBCLK_Stop();
        #endif /* (MUX_CTRL_SCB_CLK_INTERNAL) */
    #endif /* (!MUX_CTRL_CY_SCBIP_V0) */

        MUX_CTRL_SetI2CExtClkInterruptMode(MUX_CTRL_INTR_I2C_EC_WAKE_UP);
    }


    /*******************************************************************************
    * Function Name: MUX_CTRL_I2CRestoreConfig
    ********************************************************************************
    *
    * Summary:
    *  Disables MUX_CTRL_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_I2CRestoreConfig(void)
    {
        /* Disable wakeup interrupt on address match */
        MUX_CTRL_SetI2CExtClkInterruptMode(MUX_CTRL_NO_INTR_SOURCES);

    #if (!MUX_CTRL_CY_SCBIP_V0)
        #if (MUX_CTRL_SCB_CLK_INTERNAL)
            /* Enable clock to internal address match logic. Ticket ID#187931 */
            MUX_CTRL_SCBCLK_Start();
        #endif /* (MUX_CTRL_SCB_CLK_INTERNAL) */
    #endif /* (!MUX_CTRL_CY_SCBIP_V0) */
    }
#endif /* (MUX_CTRL_I2C_WAKE_ENABLE_CONST) */


/* [] END OF FILE */
