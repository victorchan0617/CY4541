/*******************************************************************************
* File Name: MUX_CTRL.c
* Version 3.10
*
* Description:
*  This file provides the source code to the API for the SCB Component.
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

#if (MUX_CTRL_SCB_MODE_I2C_INC)
    #include "MUX_CTRL_I2C_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_I2C_INC) */

#if (MUX_CTRL_SCB_MODE_EZI2C_INC)
    #include "MUX_CTRL_EZI2C_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_EZI2C_INC) */

#if (MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC)
    #include "MUX_CTRL_SPI_UART_PVT.h"
#endif /* (MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 MUX_CTRL_scbMode = MUX_CTRL_SCB_MODE_UNCONFIG;
    uint8 MUX_CTRL_scbEnableWake;
    uint8 MUX_CTRL_scbEnableIntr;

    /* I2C configuration variables */
    uint8 MUX_CTRL_mode;
    uint8 MUX_CTRL_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * MUX_CTRL_rxBuffer;
    uint8  MUX_CTRL_rxDataBits;
    uint32 MUX_CTRL_rxBufferSize;

    volatile uint8 * MUX_CTRL_txBuffer;
    uint8  MUX_CTRL_txDataBits;
    uint32 MUX_CTRL_txBufferSize;

    /* EZI2C configuration variables */
    uint8 MUX_CTRL_numberOfAddr;
    uint8 MUX_CTRL_subAddrSize;
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 MUX_CTRL_initVar = 0u;

#if (MUX_CTRL_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER)
    void (*MUX_CTRL_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER) */
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void MUX_CTRL_ScbEnableIntr(void);
static void MUX_CTRL_ScbModeStop(void);
static void MUX_CTRL_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: MUX_CTRL_Init
********************************************************************************
*
* Summary:
*  Initializes the SCB component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  SCB_I2CInit, SCB_SpiInit, SCB_UartInit or SCB_EzI2CInit.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_Init(void)
{
#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    if (MUX_CTRL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        MUX_CTRL_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (MUX_CTRL_SCB_MODE_I2C_CONST_CFG)
    MUX_CTRL_I2CInit();

#elif (MUX_CTRL_SCB_MODE_SPI_CONST_CFG)
    MUX_CTRL_SpiInit();

#elif (MUX_CTRL_SCB_MODE_UART_CONST_CFG)
    MUX_CTRL_UartInit();

#elif (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG)
    MUX_CTRL_EzI2CInit();

#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_Enable
********************************************************************************
*
* Summary:
*  Enables the SCB component operation.
*  The SCB configuration should be not changed when the component is enabled.
*  Any configuration changes should be made after disabling the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_Enable(void)
{
#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!MUX_CTRL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        MUX_CTRL_CTRL_REG |= MUX_CTRL_CTRL_ENABLED;

        MUX_CTRL_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        MUX_CTRL_ScbModePostEnable();
    }
#else
    MUX_CTRL_CTRL_REG |= MUX_CTRL_CTRL_ENABLED;

    MUX_CTRL_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    MUX_CTRL_ScbModePostEnable();
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_Start
********************************************************************************
*
* Summary:
*  Invokes SCB_Init() and SCB_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZ I2C. Otherwise this function does not enable the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  MUX_CTRL_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void MUX_CTRL_Start(void)
{
    if (0u == MUX_CTRL_initVar)
    {
        MUX_CTRL_Init();
        MUX_CTRL_initVar = 1u; /* Component was initialized */
    }

    MUX_CTRL_Enable();
}


/*******************************************************************************
* Function Name: MUX_CTRL_Stop
********************************************************************************
*
* Summary:
*  Disables the SCB component and its interrupt.
*  It also disables all TX interrupt sources so as not to cause an unexpected
*  interrupt trigger because after the component is enabled, the TX FIFO 
*  is empty.
*
* Parameters:
*  None
*
* Return:
*  None
* 
*******************************************************************************/
void MUX_CTRL_Stop(void)
{
#if (MUX_CTRL_SCB_IRQ_INTERNAL)
    MUX_CTRL_DisableInt();
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    MUX_CTRL_ScbModeStop();

    /* Disable SCB IP */
    MUX_CTRL_CTRL_REG &= (uint32) ~MUX_CTRL_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger because after the component is enabled, the TX FIFO
    * is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when they are disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    MUX_CTRL_SetTxInterruptMode(MUX_CTRL_NO_INTR_SOURCES);

#if (MUX_CTRL_SCB_IRQ_INTERNAL)
    MUX_CTRL_ClearPendingInt();
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_SetRxFifoLevel
********************************************************************************
*
* Summary:
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
* Parameters:
*  level: Level in the RX FIFO to generate RX level interrupt.
*         The range of valid level values is between 0 and RX FIFO depth - 1.
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = MUX_CTRL_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~MUX_CTRL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (MUX_CTRL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    MUX_CTRL_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: MUX_CTRL_SetTxFifoLevel
********************************************************************************
*
* Summary:
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has more entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
* Parameters:
*  level: Level in the TX FIFO to generate TX level interrupt.
*         The range of valid level values is between 0 and TX FIFO depth - 1.
*
* Return:
*  None
*
*******************************************************************************/
void MUX_CTRL_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = MUX_CTRL_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~MUX_CTRL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (MUX_CTRL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    MUX_CTRL_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (MUX_CTRL_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: MUX_CTRL_SetCustomInterruptHandler
    ********************************************************************************
    *
    * Summary:
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    * Parameters:
    *  func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER)
        MUX_CTRL_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_MUX_CTRL_CUSTOM_INTR_HANDLER) */
    }
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: MUX_CTRL_ScbModeEnableIntr
********************************************************************************
*
* Summary:
*  Enables an interrupt for a specific mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MUX_CTRL_ScbEnableIntr(void)
{
#if (MUX_CTRL_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != MUX_CTRL_scbEnableIntr)
        {
            MUX_CTRL_EnableInt();
        }

    #else
        MUX_CTRL_EnableInt();

    #endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (MUX_CTRL_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_ScbModePostEnable
********************************************************************************
*
* Summary:
*  Calls the PostEnable function for a specific operation mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MUX_CTRL_ScbModePostEnable(void)
{
#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!MUX_CTRL_CY_SCBIP_V1)
    if (MUX_CTRL_SCB_MODE_SPI_RUNTM_CFG)
    {
        MUX_CTRL_SpiPostEnable();
    }
    else if (MUX_CTRL_SCB_MODE_UART_RUNTM_CFG)
    {
        MUX_CTRL_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!MUX_CTRL_CY_SCBIP_V1) */

#elif (MUX_CTRL_SCB_MODE_SPI_CONST_CFG)
    MUX_CTRL_SpiPostEnable();

#elif (MUX_CTRL_SCB_MODE_UART_CONST_CFG)
    MUX_CTRL_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: MUX_CTRL_ScbModeStop
********************************************************************************
*
* Summary:
*  Calls the Stop function for a specific operation mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MUX_CTRL_ScbModeStop(void)
{
#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    if (MUX_CTRL_SCB_MODE_I2C_RUNTM_CFG)
    {
        MUX_CTRL_I2CStop();
    }
    else if (MUX_CTRL_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        MUX_CTRL_EzI2CStop();
    }
#if (!MUX_CTRL_CY_SCBIP_V1)
    else if (MUX_CTRL_SCB_MODE_SPI_RUNTM_CFG)
    {
        MUX_CTRL_SpiStop();
    }
    else if (MUX_CTRL_SCB_MODE_UART_RUNTM_CFG)
    {
        MUX_CTRL_UartStop();
    }
#endif /* (!MUX_CTRL_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (MUX_CTRL_SCB_MODE_I2C_CONST_CFG)
    MUX_CTRL_I2CStop();

#elif (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG)
    MUX_CTRL_EzI2CStop();

#elif (MUX_CTRL_SCB_MODE_SPI_CONST_CFG)
    MUX_CTRL_SpiStop();

#elif (MUX_CTRL_SCB_MODE_UART_CONST_CFG)
    MUX_CTRL_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: MUX_CTRL_SetPins
    ********************************************************************************
    *
    * Summary:
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    * Parameters:
    *  mode:      Mode of SCB operation.
    *  subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 hsiomSel [MUX_CTRL_SCB_PINS_NUMBER];
        uint32 pinsDm   [MUX_CTRL_SCB_PINS_NUMBER];

    #if (!MUX_CTRL_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < MUX_CTRL_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = MUX_CTRL_HSIOM_DEF_SEL;
            pinsDm[i]    = MUX_CTRL_PIN_DM_ALG_HIZ;
        }

        if ((MUX_CTRL_SCB_MODE_I2C   == mode) ||
           (MUX_CTRL_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_HSIOM_I2C_SEL;
            hsiomSel[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_HSIOM_I2C_SEL;

            pinsDm[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_PIN_DM_OD_LO;
            pinsDm[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_PIN_DM_OD_LO;
        }
    #if (!MUX_CTRL_CY_SCBIP_V1)
        else if (MUX_CTRL_SCB_MODE_SPI == mode)
        {
            hsiomSel[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
            hsiomSel[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
            hsiomSel[MUX_CTRL_CTS_SCLK_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;

            if (MUX_CTRL_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;
                pinsDm[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsDm[MUX_CTRL_CTS_SCLK_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;

            #if (MUX_CTRL_RTS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
                pinsDm  [MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;
            #endif /* (MUX_CTRL_RTS_SS0_PIN) */

            #if (MUX_CTRL_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= MUX_CTRL_TX_SDA_MISO_PIN_MASK;
            #endif /* (MUX_CTRL_TX_SDA_MISO_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsDm[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;
                pinsDm[MUX_CTRL_CTS_SCLK_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;

            #if (MUX_CTRL_RTS_SS0_PIN)
                hsiomSel [MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
                pinsDm   [MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsInBuf |= MUX_CTRL_RTS_SS0_PIN_MASK;
            #endif /* (MUX_CTRL_RTS_SS0_PIN) */

            #if (MUX_CTRL_SS1_PIN)
                hsiomSel [MUX_CTRL_SS1_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
                pinsDm   [MUX_CTRL_SS1_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsInBuf |= MUX_CTRL_SS1_PIN_MASK;
            #endif /* (MUX_CTRL_SS1_PIN) */

            #if (MUX_CTRL_SS2_PIN)
                hsiomSel [MUX_CTRL_SS2_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
                pinsDm   [MUX_CTRL_SS2_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsInBuf |= MUX_CTRL_SS2_PIN_MASK;
            #endif /* (MUX_CTRL_SS2_PIN) */

            #if (MUX_CTRL_SS3_PIN)
                hsiomSel [MUX_CTRL_SS3_PIN_INDEX] = MUX_CTRL_HSIOM_SPI_SEL;
                pinsDm   [MUX_CTRL_SS3_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;
                pinsInBuf |= MUX_CTRL_SS3_PIN_MASK;
            #endif /* (MUX_CTRL_SS3_PIN) */

                /* Disable input buffers */
            #if (MUX_CTRL_RX_SCL_MOSI_PIN)
                pinsInBuf |= MUX_CTRL_RX_SCL_MOSI_PIN_MASK;
            #endif /* (MUX_CTRL_RX_SCL_MOSI_PIN) */

             #if (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_MASK;
            #endif /* (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN) */

            #if (MUX_CTRL_CTS_SCLK_PIN)
                pinsInBuf |= MUX_CTRL_CTS_SCLK_PIN_MASK;
            #endif /* (MUX_CTRL_CTS_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (MUX_CTRL_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_HSIOM_UART_SEL;
                pinsDm  [MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if (0u != (MUX_CTRL_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_HSIOM_UART_SEL;
                    pinsDm  [MUX_CTRL_RX_SCL_MOSI_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;
                }

                if (0u != (MUX_CTRL_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_HSIOM_UART_SEL;
                    pinsDm  [MUX_CTRL_TX_SDA_MISO_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;

                #if (MUX_CTRL_TX_SDA_MISO_PIN)
                     pinsInBuf |= MUX_CTRL_TX_SDA_MISO_PIN_MASK;
                #endif /* (MUX_CTRL_TX_SDA_MISO_PIN) */
                }

            #if !(MUX_CTRL_CY_SCBIP_V0 || MUX_CTRL_CY_SCBIP_V1)
                if (MUX_CTRL_UART_MODE_STD == subMode)
                {
                    if (0u != (MUX_CTRL_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                        hsiomSel[MUX_CTRL_CTS_SCLK_PIN_INDEX] = MUX_CTRL_HSIOM_UART_SEL;
                        pinsDm  [MUX_CTRL_CTS_SCLK_PIN_INDEX] = MUX_CTRL_PIN_DM_DIG_HIZ;
                    }

                    if (0u != (MUX_CTRL_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                        hsiomSel[MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_HSIOM_UART_SEL;
                        pinsDm  [MUX_CTRL_RTS_SS0_PIN_INDEX] = MUX_CTRL_PIN_DM_STRONG;

                    #if (MUX_CTRL_RTS_SS0_PIN)
                        /* Disable input buffer */
                        pinsInBuf |= MUX_CTRL_RTS_SS0_PIN_MASK;
                    #endif /* (MUX_CTRL_RTS_SS0_PIN) */
                    }
                }
            #endif /* !(MUX_CTRL_CY_SCBIP_V0 || MUX_CTRL_CY_SCBIP_V1) */
            }
        }
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        MUX_CTRL_SET_INCFG_TYPE(MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        MUX_CTRL_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN) */

    #if (MUX_CTRL_RX_SCL_MOSI_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_RX_SCL_MOSI_HSIOM_REG,
                                       MUX_CTRL_RX_SCL_MOSI_HSIOM_MASK,
                                       MUX_CTRL_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX]);

        MUX_CTRL_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[MUX_CTRL_RX_SCL_MOSI_PIN_INDEX]);

    #if (!MUX_CTRL_CY_SCBIP_V1)
        MUX_CTRL_SET_INP_DIS(MUX_CTRL_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                     MUX_CTRL_uart_rx_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_RX_SCL_MOSI_PIN_MASK)));
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */
    #endif /* (MUX_CTRL_RX_SCL_MOSI_PIN) */

    #if (MUX_CTRL_TX_SDA_MISO_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_TX_SDA_MISO_HSIOM_REG,
                                       MUX_CTRL_TX_SDA_MISO_HSIOM_MASK,
                                       MUX_CTRL_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[MUX_CTRL_TX_SDA_MISO_PIN_INDEX]);

        MUX_CTRL_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[MUX_CTRL_TX_SDA_MISO_PIN_INDEX]);

    #if (!MUX_CTRL_CY_SCBIP_V1)
        MUX_CTRL_SET_INP_DIS(MUX_CTRL_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     MUX_CTRL_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & MUX_CTRL_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!MUX_CTRL_CY_SCBIP_V1) */
    #endif /* (MUX_CTRL_RX_SCL_MOSI_PIN) */

    #if (MUX_CTRL_CTS_SCLK_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_CTS_SCLK_HSIOM_REG,
                                       MUX_CTRL_CTS_SCLK_HSIOM_MASK,
                                       MUX_CTRL_CTS_SCLK_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_CTS_SCLK_PIN_INDEX]);

        MUX_CTRL_uart_cts_spi_sclk_SetDriveMode((uint8) pinsDm[MUX_CTRL_CTS_SCLK_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_uart_cts_spi_sclk_INP_DIS,
                                     MUX_CTRL_uart_cts_spi_sclk_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_CTS_SCLK_PIN_MASK)));
    #endif /* (MUX_CTRL_CTS_SCLK_PIN) */

    #if (MUX_CTRL_RTS_SS0_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_RTS_SS0_HSIOM_REG,
                                       MUX_CTRL_RTS_SS0_HSIOM_MASK,
                                       MUX_CTRL_RTS_SS0_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_RTS_SS0_PIN_INDEX]);

        MUX_CTRL_uart_rts_spi_ss0_SetDriveMode((uint8) pinsDm[MUX_CTRL_RTS_SS0_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_uart_rts_spi_ss0_INP_DIS,
                                     MUX_CTRL_uart_rts_spi_ss0_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_RTS_SS0_PIN_MASK)));
    #endif /* (MUX_CTRL_RTS_SS0_PIN) */

    #if (MUX_CTRL_SS1_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_SS1_HSIOM_REG,
                                       MUX_CTRL_SS1_HSIOM_MASK,
                                       MUX_CTRL_SS1_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_SS1_PIN_INDEX]);

        MUX_CTRL_spi_ss1_SetDriveMode((uint8) pinsDm[MUX_CTRL_SS1_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_spi_ss1_INP_DIS,
                                     MUX_CTRL_spi_ss1_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_SS1_PIN_MASK)));
    #endif /* (MUX_CTRL_SS1_PIN) */

    #if (MUX_CTRL_SS2_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_SS2_HSIOM_REG,
                                       MUX_CTRL_SS2_HSIOM_MASK,
                                       MUX_CTRL_SS2_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_SS2_PIN_INDEX]);

        MUX_CTRL_spi_ss2_SetDriveMode((uint8) pinsDm[MUX_CTRL_SS2_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_spi_ss2_INP_DIS,
                                     MUX_CTRL_spi_ss2_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_SS2_PIN_MASK)));
    #endif /* (MUX_CTRL_SS2_PIN) */

    #if (MUX_CTRL_SS3_PIN)
        MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_SS3_HSIOM_REG,
                                       MUX_CTRL_SS3_HSIOM_MASK,
                                       MUX_CTRL_SS3_HSIOM_POS,
                                       hsiomSel[MUX_CTRL_SS3_PIN_INDEX]);

        MUX_CTRL_spi_ss3_SetDriveMode((uint8) pinsDm[MUX_CTRL_SS3_PIN_INDEX]);

        MUX_CTRL_SET_INP_DIS(MUX_CTRL_spi_ss3_INP_DIS,
                                     MUX_CTRL_spi_ss3_MASK,
                                     (0u != (pinsInBuf & MUX_CTRL_SS3_PIN_MASK)));
    #endif /* (MUX_CTRL_SS3_PIN) */
    }

#endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (MUX_CTRL_CY_SCBIP_V0 || MUX_CTRL_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: MUX_CTRL_I2CSlaveNackGeneration
    ********************************************************************************
    *
    * Summary:
    *  Sets command to generate NACK to the address or data.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void MUX_CTRL_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (MUX_CTRL_CTRL_REG & MUX_CTRL_CTRL_EC_AM_MODE)) &&
            (0u == (MUX_CTRL_I2C_CTRL_REG & MUX_CTRL_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            MUX_CTRL_CTRL_REG &= ~MUX_CTRL_CTRL_EC_AM_MODE;
            MUX_CTRL_CTRL_REG |=  MUX_CTRL_CTRL_EC_AM_MODE;
        }

        MUX_CTRL_I2C_SLAVE_CMD_REG = MUX_CTRL_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (MUX_CTRL_CY_SCBIP_V0 || MUX_CTRL_CY_SCBIP_V1) */


/* [] END OF FILE */
