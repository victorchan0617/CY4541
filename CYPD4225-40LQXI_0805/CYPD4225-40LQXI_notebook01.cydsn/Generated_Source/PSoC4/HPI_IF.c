/*******************************************************************************
* File Name: HPI_IF.c
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

#include "HPI_IF_PVT.h"

#if (HPI_IF_SCB_MODE_I2C_INC)
    #include "HPI_IF_I2C_PVT.h"
#endif /* (HPI_IF_SCB_MODE_I2C_INC) */

#if (HPI_IF_SCB_MODE_EZI2C_INC)
    #include "HPI_IF_EZI2C_PVT.h"
#endif /* (HPI_IF_SCB_MODE_EZI2C_INC) */

#if (HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC)
    #include "HPI_IF_SPI_UART_PVT.h"
#endif /* (HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 HPI_IF_scbMode = HPI_IF_SCB_MODE_UNCONFIG;
    uint8 HPI_IF_scbEnableWake;
    uint8 HPI_IF_scbEnableIntr;

    /* I2C configuration variables */
    uint8 HPI_IF_mode;
    uint8 HPI_IF_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * HPI_IF_rxBuffer;
    uint8  HPI_IF_rxDataBits;
    uint32 HPI_IF_rxBufferSize;

    volatile uint8 * HPI_IF_txBuffer;
    uint8  HPI_IF_txDataBits;
    uint32 HPI_IF_txBufferSize;

    /* EZI2C configuration variables */
    uint8 HPI_IF_numberOfAddr;
    uint8 HPI_IF_subAddrSize;
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 HPI_IF_initVar = 0u;

#if (HPI_IF_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER)
    void (*HPI_IF_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER) */
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void HPI_IF_ScbEnableIntr(void);
static void HPI_IF_ScbModeStop(void);
static void HPI_IF_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: HPI_IF_Init
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
void HPI_IF_Init(void)
{
#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    if (HPI_IF_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        HPI_IF_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (HPI_IF_SCB_MODE_I2C_CONST_CFG)
    HPI_IF_I2CInit();

#elif (HPI_IF_SCB_MODE_SPI_CONST_CFG)
    HPI_IF_SpiInit();

#elif (HPI_IF_SCB_MODE_UART_CONST_CFG)
    HPI_IF_UartInit();

#elif (HPI_IF_SCB_MODE_EZI2C_CONST_CFG)
    HPI_IF_EzI2CInit();

#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: HPI_IF_Enable
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
void HPI_IF_Enable(void)
{
#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!HPI_IF_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        HPI_IF_CTRL_REG |= HPI_IF_CTRL_ENABLED;

        HPI_IF_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        HPI_IF_ScbModePostEnable();
    }
#else
    HPI_IF_CTRL_REG |= HPI_IF_CTRL_ENABLED;

    HPI_IF_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    HPI_IF_ScbModePostEnable();
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: HPI_IF_Start
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
*  HPI_IF_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void HPI_IF_Start(void)
{
    if (0u == HPI_IF_initVar)
    {
        HPI_IF_Init();
        HPI_IF_initVar = 1u; /* Component was initialized */
    }

    HPI_IF_Enable();
}


/*******************************************************************************
* Function Name: HPI_IF_Stop
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
void HPI_IF_Stop(void)
{
#if (HPI_IF_SCB_IRQ_INTERNAL)
    HPI_IF_DisableInt();
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    HPI_IF_ScbModeStop();

    /* Disable SCB IP */
    HPI_IF_CTRL_REG &= (uint32) ~HPI_IF_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger because after the component is enabled, the TX FIFO
    * is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when they are disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    HPI_IF_SetTxInterruptMode(HPI_IF_NO_INTR_SOURCES);

#if (HPI_IF_SCB_IRQ_INTERNAL)
    HPI_IF_ClearPendingInt();
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: HPI_IF_SetRxFifoLevel
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
void HPI_IF_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = HPI_IF_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~HPI_IF_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (HPI_IF_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    HPI_IF_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: HPI_IF_SetTxFifoLevel
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
void HPI_IF_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = HPI_IF_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~HPI_IF_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (HPI_IF_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    HPI_IF_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (HPI_IF_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: HPI_IF_SetCustomInterruptHandler
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
    void HPI_IF_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER)
        HPI_IF_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER) */
    }
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: HPI_IF_ScbModeEnableIntr
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
static void HPI_IF_ScbEnableIntr(void)
{
#if (HPI_IF_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != HPI_IF_scbEnableIntr)
        {
            HPI_IF_EnableInt();
        }

    #else
        HPI_IF_EnableInt();

    #endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: HPI_IF_ScbModePostEnable
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
static void HPI_IF_ScbModePostEnable(void)
{
#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!HPI_IF_CY_SCBIP_V1)
    if (HPI_IF_SCB_MODE_SPI_RUNTM_CFG)
    {
        HPI_IF_SpiPostEnable();
    }
    else if (HPI_IF_SCB_MODE_UART_RUNTM_CFG)
    {
        HPI_IF_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!HPI_IF_CY_SCBIP_V1) */

#elif (HPI_IF_SCB_MODE_SPI_CONST_CFG)
    HPI_IF_SpiPostEnable();

#elif (HPI_IF_SCB_MODE_UART_CONST_CFG)
    HPI_IF_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: HPI_IF_ScbModeStop
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
static void HPI_IF_ScbModeStop(void)
{
#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    if (HPI_IF_SCB_MODE_I2C_RUNTM_CFG)
    {
        HPI_IF_I2CStop();
    }
    else if (HPI_IF_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        HPI_IF_EzI2CStop();
    }
#if (!HPI_IF_CY_SCBIP_V1)
    else if (HPI_IF_SCB_MODE_SPI_RUNTM_CFG)
    {
        HPI_IF_SpiStop();
    }
    else if (HPI_IF_SCB_MODE_UART_RUNTM_CFG)
    {
        HPI_IF_UartStop();
    }
#endif /* (!HPI_IF_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (HPI_IF_SCB_MODE_I2C_CONST_CFG)
    HPI_IF_I2CStop();

#elif (HPI_IF_SCB_MODE_EZI2C_CONST_CFG)
    HPI_IF_EzI2CStop();

#elif (HPI_IF_SCB_MODE_SPI_CONST_CFG)
    HPI_IF_SpiStop();

#elif (HPI_IF_SCB_MODE_UART_CONST_CFG)
    HPI_IF_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: HPI_IF_SetPins
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
    void HPI_IF_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 hsiomSel [HPI_IF_SCB_PINS_NUMBER];
        uint32 pinsDm   [HPI_IF_SCB_PINS_NUMBER];

    #if (!HPI_IF_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!HPI_IF_CY_SCBIP_V1) */

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < HPI_IF_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = HPI_IF_HSIOM_DEF_SEL;
            pinsDm[i]    = HPI_IF_PIN_DM_ALG_HIZ;
        }

        if ((HPI_IF_SCB_MODE_I2C   == mode) ||
           (HPI_IF_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_HSIOM_I2C_SEL;
            hsiomSel[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_HSIOM_I2C_SEL;

            pinsDm[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_PIN_DM_OD_LO;
            pinsDm[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_PIN_DM_OD_LO;
        }
    #if (!HPI_IF_CY_SCBIP_V1)
        else if (HPI_IF_SCB_MODE_SPI == mode)
        {
            hsiomSel[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
            hsiomSel[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
            hsiomSel[HPI_IF_CTS_SCLK_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;

            if (HPI_IF_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;
                pinsDm[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsDm[HPI_IF_CTS_SCLK_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;

            #if (HPI_IF_RTS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
                pinsDm  [HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;
            #endif /* (HPI_IF_RTS_SS0_PIN) */

            #if (HPI_IF_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= HPI_IF_TX_SDA_MISO_PIN_MASK;
            #endif /* (HPI_IF_TX_SDA_MISO_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsDm[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;
                pinsDm[HPI_IF_CTS_SCLK_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;

            #if (HPI_IF_RTS_SS0_PIN)
                hsiomSel [HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
                pinsDm   [HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsInBuf |= HPI_IF_RTS_SS0_PIN_MASK;
            #endif /* (HPI_IF_RTS_SS0_PIN) */

            #if (HPI_IF_SS1_PIN)
                hsiomSel [HPI_IF_SS1_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
                pinsDm   [HPI_IF_SS1_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsInBuf |= HPI_IF_SS1_PIN_MASK;
            #endif /* (HPI_IF_SS1_PIN) */

            #if (HPI_IF_SS2_PIN)
                hsiomSel [HPI_IF_SS2_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
                pinsDm   [HPI_IF_SS2_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsInBuf |= HPI_IF_SS2_PIN_MASK;
            #endif /* (HPI_IF_SS2_PIN) */

            #if (HPI_IF_SS3_PIN)
                hsiomSel [HPI_IF_SS3_PIN_INDEX] = HPI_IF_HSIOM_SPI_SEL;
                pinsDm   [HPI_IF_SS3_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;
                pinsInBuf |= HPI_IF_SS3_PIN_MASK;
            #endif /* (HPI_IF_SS3_PIN) */

                /* Disable input buffers */
            #if (HPI_IF_RX_SCL_MOSI_PIN)
                pinsInBuf |= HPI_IF_RX_SCL_MOSI_PIN_MASK;
            #endif /* (HPI_IF_RX_SCL_MOSI_PIN) */

             #if (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= HPI_IF_RX_WAKE_SCL_MOSI_PIN_MASK;
            #endif /* (HPI_IF_RX_WAKE_SCL_MOSI_PIN) */

            #if (HPI_IF_CTS_SCLK_PIN)
                pinsInBuf |= HPI_IF_CTS_SCLK_PIN_MASK;
            #endif /* (HPI_IF_CTS_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (HPI_IF_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_HSIOM_UART_SEL;
                pinsDm  [HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if (0u != (HPI_IF_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_HSIOM_UART_SEL;
                    pinsDm  [HPI_IF_RX_SCL_MOSI_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;
                }

                if (0u != (HPI_IF_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                    hsiomSel[HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_HSIOM_UART_SEL;
                    pinsDm  [HPI_IF_TX_SDA_MISO_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;

                #if (HPI_IF_TX_SDA_MISO_PIN)
                     pinsInBuf |= HPI_IF_TX_SDA_MISO_PIN_MASK;
                #endif /* (HPI_IF_TX_SDA_MISO_PIN) */
                }

            #if !(HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1)
                if (HPI_IF_UART_MODE_STD == subMode)
                {
                    if (0u != (HPI_IF_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                        hsiomSel[HPI_IF_CTS_SCLK_PIN_INDEX] = HPI_IF_HSIOM_UART_SEL;
                        pinsDm  [HPI_IF_CTS_SCLK_PIN_INDEX] = HPI_IF_PIN_DM_DIG_HIZ;
                    }

                    if (0u != (HPI_IF_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                        hsiomSel[HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_HSIOM_UART_SEL;
                        pinsDm  [HPI_IF_RTS_SS0_PIN_INDEX] = HPI_IF_PIN_DM_STRONG;

                    #if (HPI_IF_RTS_SS0_PIN)
                        /* Disable input buffer */
                        pinsInBuf |= HPI_IF_RTS_SS0_PIN_MASK;
                    #endif /* (HPI_IF_RTS_SS0_PIN) */
                    }
                }
            #endif /* !(HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1) */
            }
        }
    #endif /* (!HPI_IF_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[HPI_IF_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[HPI_IF_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & HPI_IF_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        HPI_IF_SET_INCFG_TYPE(HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        HPI_IF_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (HPI_IF_RX_WAKE_SCL_MOSI_PIN) */

    #if (HPI_IF_RX_SCL_MOSI_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_RX_SCL_MOSI_HSIOM_REG,
                                       HPI_IF_RX_SCL_MOSI_HSIOM_MASK,
                                       HPI_IF_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[HPI_IF_RX_SCL_MOSI_PIN_INDEX]);

        HPI_IF_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[HPI_IF_RX_SCL_MOSI_PIN_INDEX]);

    #if (!HPI_IF_CY_SCBIP_V1)
        HPI_IF_SET_INP_DIS(HPI_IF_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                     HPI_IF_uart_rx_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & HPI_IF_RX_SCL_MOSI_PIN_MASK)));
    #endif /* (!HPI_IF_CY_SCBIP_V1) */
    #endif /* (HPI_IF_RX_SCL_MOSI_PIN) */

    #if (HPI_IF_TX_SDA_MISO_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_TX_SDA_MISO_HSIOM_REG,
                                       HPI_IF_TX_SDA_MISO_HSIOM_MASK,
                                       HPI_IF_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[HPI_IF_TX_SDA_MISO_PIN_INDEX]);

        HPI_IF_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[HPI_IF_TX_SDA_MISO_PIN_INDEX]);

    #if (!HPI_IF_CY_SCBIP_V1)
        HPI_IF_SET_INP_DIS(HPI_IF_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     HPI_IF_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & HPI_IF_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!HPI_IF_CY_SCBIP_V1) */
    #endif /* (HPI_IF_RX_SCL_MOSI_PIN) */

    #if (HPI_IF_CTS_SCLK_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_CTS_SCLK_HSIOM_REG,
                                       HPI_IF_CTS_SCLK_HSIOM_MASK,
                                       HPI_IF_CTS_SCLK_HSIOM_POS,
                                       hsiomSel[HPI_IF_CTS_SCLK_PIN_INDEX]);

        HPI_IF_uart_cts_spi_sclk_SetDriveMode((uint8) pinsDm[HPI_IF_CTS_SCLK_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_uart_cts_spi_sclk_INP_DIS,
                                     HPI_IF_uart_cts_spi_sclk_MASK,
                                     (0u != (pinsInBuf & HPI_IF_CTS_SCLK_PIN_MASK)));
    #endif /* (HPI_IF_CTS_SCLK_PIN) */

    #if (HPI_IF_RTS_SS0_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_RTS_SS0_HSIOM_REG,
                                       HPI_IF_RTS_SS0_HSIOM_MASK,
                                       HPI_IF_RTS_SS0_HSIOM_POS,
                                       hsiomSel[HPI_IF_RTS_SS0_PIN_INDEX]);

        HPI_IF_uart_rts_spi_ss0_SetDriveMode((uint8) pinsDm[HPI_IF_RTS_SS0_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_uart_rts_spi_ss0_INP_DIS,
                                     HPI_IF_uart_rts_spi_ss0_MASK,
                                     (0u != (pinsInBuf & HPI_IF_RTS_SS0_PIN_MASK)));
    #endif /* (HPI_IF_RTS_SS0_PIN) */

    #if (HPI_IF_SS1_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_SS1_HSIOM_REG,
                                       HPI_IF_SS1_HSIOM_MASK,
                                       HPI_IF_SS1_HSIOM_POS,
                                       hsiomSel[HPI_IF_SS1_PIN_INDEX]);

        HPI_IF_spi_ss1_SetDriveMode((uint8) pinsDm[HPI_IF_SS1_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_spi_ss1_INP_DIS,
                                     HPI_IF_spi_ss1_MASK,
                                     (0u != (pinsInBuf & HPI_IF_SS1_PIN_MASK)));
    #endif /* (HPI_IF_SS1_PIN) */

    #if (HPI_IF_SS2_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_SS2_HSIOM_REG,
                                       HPI_IF_SS2_HSIOM_MASK,
                                       HPI_IF_SS2_HSIOM_POS,
                                       hsiomSel[HPI_IF_SS2_PIN_INDEX]);

        HPI_IF_spi_ss2_SetDriveMode((uint8) pinsDm[HPI_IF_SS2_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_spi_ss2_INP_DIS,
                                     HPI_IF_spi_ss2_MASK,
                                     (0u != (pinsInBuf & HPI_IF_SS2_PIN_MASK)));
    #endif /* (HPI_IF_SS2_PIN) */

    #if (HPI_IF_SS3_PIN)
        HPI_IF_SET_HSIOM_SEL(HPI_IF_SS3_HSIOM_REG,
                                       HPI_IF_SS3_HSIOM_MASK,
                                       HPI_IF_SS3_HSIOM_POS,
                                       hsiomSel[HPI_IF_SS3_PIN_INDEX]);

        HPI_IF_spi_ss3_SetDriveMode((uint8) pinsDm[HPI_IF_SS3_PIN_INDEX]);

        HPI_IF_SET_INP_DIS(HPI_IF_spi_ss3_INP_DIS,
                                     HPI_IF_spi_ss3_MASK,
                                     (0u != (pinsInBuf & HPI_IF_SS3_PIN_MASK)));
    #endif /* (HPI_IF_SS3_PIN) */
    }

#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: HPI_IF_I2CSlaveNackGeneration
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
    void HPI_IF_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (HPI_IF_CTRL_REG & HPI_IF_CTRL_EC_AM_MODE)) &&
            (0u == (HPI_IF_I2C_CTRL_REG & HPI_IF_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            HPI_IF_CTRL_REG &= ~HPI_IF_CTRL_EC_AM_MODE;
            HPI_IF_CTRL_REG |=  HPI_IF_CTRL_EC_AM_MODE;
        }

        HPI_IF_I2C_SLAVE_CMD_REG = HPI_IF_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1) */


/* [] END OF FILE */
