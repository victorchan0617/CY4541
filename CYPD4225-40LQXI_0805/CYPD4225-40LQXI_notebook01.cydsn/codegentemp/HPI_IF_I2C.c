/*******************************************************************************
* File Name: HPI_IF_I2C.c
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

#include "HPI_IF_PVT.h"
#include "HPI_IF_I2C_PVT.h"


/***************************************
*      I2C Private Vars
***************************************/

volatile uint8 HPI_IF_state;  /* Current state of I2C FSM */

#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    /* Constant configuration of I2C */
    const HPI_IF_I2C_INIT_STRUCT HPI_IF_configI2C =
    {
        HPI_IF_I2C_MODE,
        HPI_IF_I2C_OVS_FACTOR_LOW,
        HPI_IF_I2C_OVS_FACTOR_HIGH,
        HPI_IF_I2C_MEDIAN_FILTER_ENABLE,
        HPI_IF_I2C_SLAVE_ADDRESS,
        HPI_IF_I2C_SLAVE_ADDRESS_MASK,
        HPI_IF_I2C_ACCEPT_ADDRESS,
        HPI_IF_I2C_WAKE_ENABLE,
        HPI_IF_I2C_BYTE_MODE_ENABLE,
        HPI_IF_I2C_DATA_RATE,
        HPI_IF_I2C_ACCEPT_GENERAL_CALL,
    };

    /*******************************************************************************
    * Function Name: HPI_IF_I2CInit
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
    void HPI_IF_I2CInit(const HPI_IF_I2C_INIT_STRUCT *config)
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
            HPI_IF_SetPins(HPI_IF_SCB_MODE_I2C, HPI_IF_DUMMY_PARAM,
                                     HPI_IF_DUMMY_PARAM);

            /* Store internal configuration */
            HPI_IF_scbMode       = (uint8) HPI_IF_SCB_MODE_I2C;
            HPI_IF_scbEnableWake = (uint8) config->enableWake;
            HPI_IF_scbEnableIntr = (uint8) HPI_IF_SCB_IRQ_INTERNAL;

            HPI_IF_mode          = (uint8) config->mode;
            HPI_IF_acceptAddr    = (uint8) config->acceptAddr;

        #if (HPI_IF_CY_SCBIP_V0)
            /* Adjust SDA filter settings. Ticket ID#150521 */
            HPI_IF_SET_I2C_CFG_SDA_FILT_TRIM(HPI_IF_EC_AM_I2C_CFG_SDA_FILT_TRIM);
        #endif /* (HPI_IF_CY_SCBIP_V0) */

            /* Adjust AF and DF filter settings. Ticket ID#176179 */
            if (((HPI_IF_I2C_MODE_SLAVE != config->mode) &&
                 (config->dataRate <= HPI_IF_I2C_DATA_RATE_FS_MODE_MAX)) ||
                 (HPI_IF_I2C_MODE_SLAVE == config->mode))
            {
                /* AF = 1, DF = 0 */
                HPI_IF_I2C_CFG_ANALOG_FITER_ENABLE;
                medianFilter = HPI_IF_DIGITAL_FILTER_DISABLE;
            }
            else
            {
                /* AF = 0, DF = 1 */
                HPI_IF_I2C_CFG_ANALOG_FITER_DISABLE;
                medianFilter = HPI_IF_DIGITAL_FILTER_ENABLE;
            }

        #if (!HPI_IF_CY_SCBIP_V0)
            locEnableWake = (HPI_IF_I2C_MULTI_MASTER_SLAVE) ? (0u) : (config->enableWake);
        #else
            locEnableWake = config->enableWake;
        #endif /* (!HPI_IF_CY_SCBIP_V0) */

            /* Configure I2C interface */
            HPI_IF_CTRL_REG     = HPI_IF_GET_CTRL_BYTE_MODE  (config->enableByteMode) |
                                            HPI_IF_GET_CTRL_ADDR_ACCEPT(config->acceptAddr)     |
                                            HPI_IF_GET_CTRL_EC_AM_MODE (locEnableWake);

            HPI_IF_I2C_CTRL_REG = HPI_IF_GET_I2C_CTRL_HIGH_PHASE_OVS(config->oversampleHigh) |
                    HPI_IF_GET_I2C_CTRL_LOW_PHASE_OVS (config->oversampleLow)                          |
                    HPI_IF_GET_I2C_CTRL_S_GENERAL_IGNORE((uint32)(0u == config->acceptGeneralAddr))    |
                    HPI_IF_GET_I2C_CTRL_SL_MSTR_MODE  (config->mode);

            /* Configure RX direction */
            HPI_IF_RX_CTRL_REG      = HPI_IF_GET_RX_CTRL_MEDIAN(medianFilter) |
                                                HPI_IF_I2C_RX_CTRL;
            HPI_IF_RX_FIFO_CTRL_REG = HPI_IF_CLEAR_REG;

            /* Set default address and mask */
            HPI_IF_RX_MATCH_REG    = ((HPI_IF_I2C_SLAVE) ?
                                                (HPI_IF_GET_I2C_8BIT_ADDRESS(config->slaveAddr) |
                                                 HPI_IF_GET_RX_MATCH_MASK(config->slaveAddrMask)) :
                                                (HPI_IF_CLEAR_REG));


            /* Configure TX direction */
            HPI_IF_TX_CTRL_REG      = HPI_IF_I2C_TX_CTRL;
            HPI_IF_TX_FIFO_CTRL_REG = HPI_IF_CLEAR_REG;

            /* Configure interrupt with I2C handler but do not enable it */
            CyIntDisable    (HPI_IF_ISR_NUMBER);
            CyIntSetPriority(HPI_IF_ISR_NUMBER, HPI_IF_ISR_PRIORITY);
            (void) CyIntSetVector(HPI_IF_ISR_NUMBER, &HPI_IF_I2C_ISR);

            /* Configure interrupt sources */
        #if(!HPI_IF_CY_SCBIP_V1)
            HPI_IF_INTR_SPI_EC_MASK_REG = HPI_IF_NO_INTR_SOURCES;
        #endif /* (!HPI_IF_CY_SCBIP_V1) */

            HPI_IF_INTR_I2C_EC_MASK_REG = HPI_IF_NO_INTR_SOURCES;
            HPI_IF_INTR_RX_MASK_REG     = HPI_IF_NO_INTR_SOURCES;
            HPI_IF_INTR_TX_MASK_REG     = HPI_IF_NO_INTR_SOURCES;

            HPI_IF_INTR_SLAVE_MASK_REG  = ((HPI_IF_I2C_SLAVE) ?
                            (HPI_IF_GET_INTR_SLAVE_I2C_GENERAL(config->acceptGeneralAddr) |
                             HPI_IF_I2C_INTR_SLAVE_MASK) : (HPI_IF_CLEAR_REG));

            HPI_IF_INTR_MASTER_MASK_REG = ((HPI_IF_I2C_MASTER) ?
                                                     (HPI_IF_I2C_INTR_MASTER_MASK) :
                                                     (HPI_IF_CLEAR_REG));

            /* Configure global variables */
            HPI_IF_state = HPI_IF_I2C_FSM_IDLE;

            /* Internal slave variables */
            HPI_IF_slStatus        = 0u;
            HPI_IF_slRdBufIndex    = 0u;
            HPI_IF_slWrBufIndex    = 0u;
            HPI_IF_slOverFlowCount = 0u;

            /* Internal master variables */
            HPI_IF_mstrStatus     = 0u;
            HPI_IF_mstrRdBufIndex = 0u;
            HPI_IF_mstrWrBufIndex = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: HPI_IF_I2CInit
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
    void HPI_IF_I2CInit(void)
    {
    #if(HPI_IF_CY_SCBIP_V0)
        /* Adjust SDA filter settings. Ticket ID#150521 */
        HPI_IF_SET_I2C_CFG_SDA_FILT_TRIM(HPI_IF_EC_AM_I2C_CFG_SDA_FILT_TRIM);
    #endif /* (HPI_IF_CY_SCBIP_V0) */

        /* Adjust AF and DF filter settings. Ticket ID#176179 */
        HPI_IF_I2C_CFG_ANALOG_FITER_ENABLE_ADJ;

        /* Configure I2C interface */
        HPI_IF_CTRL_REG     = HPI_IF_I2C_DEFAULT_CTRL;
        HPI_IF_I2C_CTRL_REG = HPI_IF_I2C_DEFAULT_I2C_CTRL;

        /* Configure RX direction */
        HPI_IF_RX_CTRL_REG      = HPI_IF_I2C_DEFAULT_RX_CTRL;
        HPI_IF_RX_FIFO_CTRL_REG = HPI_IF_I2C_DEFAULT_RX_FIFO_CTRL;

        /* Set default address and mask */
        HPI_IF_RX_MATCH_REG     = HPI_IF_I2C_DEFAULT_RX_MATCH;

        /* Configure TX direction */
        HPI_IF_TX_CTRL_REG      = HPI_IF_I2C_DEFAULT_TX_CTRL;
        HPI_IF_TX_FIFO_CTRL_REG = HPI_IF_I2C_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with I2C handler but do not enable it */
        CyIntDisable    (HPI_IF_ISR_NUMBER);
        CyIntSetPriority(HPI_IF_ISR_NUMBER, HPI_IF_ISR_PRIORITY);
    #if(!HPI_IF_I2C_EXTERN_INTR_HANDLER)
        (void) CyIntSetVector(HPI_IF_ISR_NUMBER, &HPI_IF_I2C_ISR);
    #endif /* (HPI_IF_I2C_EXTERN_INTR_HANDLER) */

        /* Configure interrupt sources */
    #if(!HPI_IF_CY_SCBIP_V1)
        HPI_IF_INTR_SPI_EC_MASK_REG = HPI_IF_I2C_DEFAULT_INTR_SPI_EC_MASK;
    #endif /* (!HPI_IF_CY_SCBIP_V1) */

        HPI_IF_INTR_I2C_EC_MASK_REG = HPI_IF_I2C_DEFAULT_INTR_I2C_EC_MASK;
        HPI_IF_INTR_SLAVE_MASK_REG  = HPI_IF_I2C_DEFAULT_INTR_SLAVE_MASK;
        HPI_IF_INTR_MASTER_MASK_REG = HPI_IF_I2C_DEFAULT_INTR_MASTER_MASK;
        HPI_IF_INTR_RX_MASK_REG     = HPI_IF_I2C_DEFAULT_INTR_RX_MASK;
        HPI_IF_INTR_TX_MASK_REG     = HPI_IF_I2C_DEFAULT_INTR_TX_MASK;

        /* Configure global variables */
        HPI_IF_state = HPI_IF_I2C_FSM_IDLE;

    #if(HPI_IF_I2C_SLAVE)
        /* Internal slave variable */
        HPI_IF_slStatus        = 0u;
        HPI_IF_slRdBufIndex    = 0u;
        HPI_IF_slWrBufIndex    = 0u;
        HPI_IF_slOverFlowCount = 0u;
    #endif /* (HPI_IF_I2C_SLAVE) */

    #if(HPI_IF_I2C_MASTER)
    /* Internal master variable */
        HPI_IF_mstrStatus     = 0u;
        HPI_IF_mstrRdBufIndex = 0u;
        HPI_IF_mstrWrBufIndex = 0u;
    #endif /* (HPI_IF_I2C_MASTER) */
    }
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: HPI_IF_I2CStop
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
void HPI_IF_I2CStop(void)
{
    HPI_IF_state = HPI_IF_I2C_FSM_IDLE;
}


#if(HPI_IF_I2C_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: HPI_IF_I2CSaveConfig
    ********************************************************************************
    *
    * Summary:
    *  Enables HPI_IF_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void HPI_IF_I2CSaveConfig(void)
    {
    #if (!HPI_IF_CY_SCBIP_V0)
        #if (HPI_IF_I2C_MULTI_MASTER_SLAVE_CONST && HPI_IF_I2C_WAKE_ENABLE_CONST)
            /* Enable externally clocked address match if it was not enabled before.
            * This applicable only for Multi-Master-Slave. Ticket ID#192742 */
            if (0u == (HPI_IF_CTRL_REG & HPI_IF_CTRL_EC_AM_MODE))
            {
                /* Enable external address match logic */
                HPI_IF_Stop();
                HPI_IF_CTRL_REG |= HPI_IF_CTRL_EC_AM_MODE;
                HPI_IF_Enable();
            }
        #endif /* (HPI_IF_I2C_MULTI_MASTER_SLAVE_CONST) */

        #if (HPI_IF_SCB_CLK_INTERNAL)
            /* Disable clock to internal address match logic. Ticket ID#187931 */
            HPI_IF_SCBCLK_Stop();
        #endif /* (HPI_IF_SCB_CLK_INTERNAL) */
    #endif /* (!HPI_IF_CY_SCBIP_V0) */

        HPI_IF_SetI2CExtClkInterruptMode(HPI_IF_INTR_I2C_EC_WAKE_UP);
    }


    /*******************************************************************************
    * Function Name: HPI_IF_I2CRestoreConfig
    ********************************************************************************
    *
    * Summary:
    *  Disables HPI_IF_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void HPI_IF_I2CRestoreConfig(void)
    {
        /* Disable wakeup interrupt on address match */
        HPI_IF_SetI2CExtClkInterruptMode(HPI_IF_NO_INTR_SOURCES);

    #if (!HPI_IF_CY_SCBIP_V0)
        #if (HPI_IF_SCB_CLK_INTERNAL)
            /* Enable clock to internal address match logic. Ticket ID#187931 */
            HPI_IF_SCBCLK_Start();
        #endif /* (HPI_IF_SCB_CLK_INTERNAL) */
    #endif /* (!HPI_IF_CY_SCBIP_V0) */
    }
#endif /* (HPI_IF_I2C_WAKE_ENABLE_CONST) */


/* [] END OF FILE */
