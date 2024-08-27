/*******************************************************************************
* File Name: MUX_CTRL_PINS.h
* Version 3.10
*
* Description:
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_MUX_CTRL_H)
#define CY_SCB_PINS_MUX_CTRL_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define MUX_CTRL_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define MUX_CTRL_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define MUX_CTRL_REMOVE_TX_SDA_MISO_PIN      (1u)
#define MUX_CTRL_REMOVE_CTS_SCLK_PIN      (1u)
#define MUX_CTRL_REMOVE_RTS_SS0_PIN      (1u)
#define MUX_CTRL_REMOVE_SS1_PIN                 (1u)
#define MUX_CTRL_REMOVE_SS2_PIN                 (1u)
#define MUX_CTRL_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define MUX_CTRL_REMOVE_I2C_PINS                (0u)
#define MUX_CTRL_REMOVE_SPI_MASTER_PINS         (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define MUX_CTRL_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define MUX_CTRL_REMOVE_SPI_SLAVE_PINS          (1u)
#define MUX_CTRL_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define MUX_CTRL_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define MUX_CTRL_REMOVE_UART_TX_PIN             (1u)
#define MUX_CTRL_REMOVE_UART_RX_TX_PIN          (1u)
#define MUX_CTRL_REMOVE_UART_RX_PIN             (1u)
#define MUX_CTRL_REMOVE_UART_RX_WAKE_PIN        (1u)
#define MUX_CTRL_REMOVE_UART_RTS_PIN            (1u)
#define MUX_CTRL_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define MUX_CTRL_RX_WAKE_SCL_MOSI_PIN (0u == MUX_CTRL_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define MUX_CTRL_RX_SCL_MOSI_PIN     (0u == MUX_CTRL_REMOVE_RX_SCL_MOSI_PIN)
#define MUX_CTRL_TX_SDA_MISO_PIN     (0u == MUX_CTRL_REMOVE_TX_SDA_MISO_PIN)
#define MUX_CTRL_CTS_SCLK_PIN     (0u == MUX_CTRL_REMOVE_CTS_SCLK_PIN)
#define MUX_CTRL_RTS_SS0_PIN     (0u == MUX_CTRL_REMOVE_RTS_SS0_PIN)
#define MUX_CTRL_SS1_PIN                (0u == MUX_CTRL_REMOVE_SS1_PIN)
#define MUX_CTRL_SS2_PIN                (0u == MUX_CTRL_REMOVE_SS2_PIN)
#define MUX_CTRL_SS3_PIN                (0u == MUX_CTRL_REMOVE_SS3_PIN)

/* Mode defined pins */
#define MUX_CTRL_I2C_PINS               (0u == MUX_CTRL_REMOVE_I2C_PINS)
#define MUX_CTRL_SPI_MASTER_PINS        (0u == MUX_CTRL_REMOVE_SPI_MASTER_PINS)
#define MUX_CTRL_SPI_MASTER_SCLK_PIN    (0u == MUX_CTRL_REMOVE_SPI_MASTER_SCLK_PIN)
#define MUX_CTRL_SPI_MASTER_MOSI_PIN    (0u == MUX_CTRL_REMOVE_SPI_MASTER_MOSI_PIN)
#define MUX_CTRL_SPI_MASTER_MISO_PIN    (0u == MUX_CTRL_REMOVE_SPI_MASTER_MISO_PIN)
#define MUX_CTRL_SPI_MASTER_SS0_PIN     (0u == MUX_CTRL_REMOVE_SPI_MASTER_SS0_PIN)
#define MUX_CTRL_SPI_MASTER_SS1_PIN     (0u == MUX_CTRL_REMOVE_SPI_MASTER_SS1_PIN)
#define MUX_CTRL_SPI_MASTER_SS2_PIN     (0u == MUX_CTRL_REMOVE_SPI_MASTER_SS2_PIN)
#define MUX_CTRL_SPI_MASTER_SS3_PIN     (0u == MUX_CTRL_REMOVE_SPI_MASTER_SS3_PIN)
#define MUX_CTRL_SPI_SLAVE_PINS         (0u == MUX_CTRL_REMOVE_SPI_SLAVE_PINS)
#define MUX_CTRL_SPI_SLAVE_MOSI_PIN     (0u == MUX_CTRL_REMOVE_SPI_SLAVE_MOSI_PIN)
#define MUX_CTRL_SPI_SLAVE_MISO_PIN     (0u == MUX_CTRL_REMOVE_SPI_SLAVE_MISO_PIN)
#define MUX_CTRL_UART_TX_PIN            (0u == MUX_CTRL_REMOVE_UART_TX_PIN)
#define MUX_CTRL_UART_RX_TX_PIN         (0u == MUX_CTRL_REMOVE_UART_RX_TX_PIN)
#define MUX_CTRL_UART_RX_PIN            (0u == MUX_CTRL_REMOVE_UART_RX_PIN)
#define MUX_CTRL_UART_RX_WAKE_PIN       (0u == MUX_CTRL_REMOVE_UART_RX_WAKE_PIN)
#define MUX_CTRL_UART_RTS_PIN           (0u == MUX_CTRL_REMOVE_UART_RTS_PIN)
#define MUX_CTRL_UART_CTS_PIN           (0u == MUX_CTRL_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN)
    #include "MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (MUX_CTRL_RX_SCL_MOSI) */

#if (MUX_CTRL_RX_SCL_MOSI_PIN)
    #include "MUX_CTRL_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (MUX_CTRL_RX_SCL_MOSI) */

#if (MUX_CTRL_TX_SDA_MISO_PIN)
    #include "MUX_CTRL_uart_tx_i2c_sda_spi_miso.h"
#endif /* (MUX_CTRL_TX_SDA_MISO) */

#if (MUX_CTRL_CTS_SCLK_PIN)
    #include "MUX_CTRL_uart_cts_spi_sclk.h"
#endif /* (MUX_CTRL_CTS_SCLK) */

#if (MUX_CTRL_RTS_SS0_PIN)
    #include "MUX_CTRL_uart_rts_spi_ss0.h"
#endif /* (MUX_CTRL_RTS_SS0_PIN) */

#if (MUX_CTRL_SS1_PIN)
    #include "MUX_CTRL_spi_ss1.h"
#endif /* (MUX_CTRL_SS1_PIN) */

#if (MUX_CTRL_SS2_PIN)
    #include "MUX_CTRL_spi_ss2.h"
#endif /* (MUX_CTRL_SS2_PIN) */

#if (MUX_CTRL_SS3_PIN)
    #include "MUX_CTRL_spi_ss3.h"
#endif /* (MUX_CTRL_SS3_PIN) */

#if (MUX_CTRL_I2C_PINS)
    #include "MUX_CTRL_scl.h"
    #include "MUX_CTRL_sda.h"
#endif /* (MUX_CTRL_I2C_PINS) */

#if (MUX_CTRL_SPI_MASTER_PINS)
#if (MUX_CTRL_SPI_MASTER_SCLK_PIN)
    #include "MUX_CTRL_sclk_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_SCLK_PIN) */

#if (MUX_CTRL_SPI_MASTER_MOSI_PIN)
    #include "MUX_CTRL_mosi_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_MOSI_PIN) */

#if (MUX_CTRL_SPI_MASTER_MISO_PIN)
    #include "MUX_CTRL_miso_m.h"
#endif /*(MUX_CTRL_SPI_MASTER_MISO_PIN) */
#endif /* (MUX_CTRL_SPI_MASTER_PINS) */

#if (MUX_CTRL_SPI_SLAVE_PINS)
    #include "MUX_CTRL_sclk_s.h"
    #include "MUX_CTRL_ss_s.h"

#if (MUX_CTRL_SPI_SLAVE_MOSI_PIN)
    #include "MUX_CTRL_mosi_s.h"
#endif /* (MUX_CTRL_SPI_SLAVE_MOSI_PIN) */

#if (MUX_CTRL_SPI_SLAVE_MISO_PIN)
    #include "MUX_CTRL_miso_s.h"
#endif /*(MUX_CTRL_SPI_SLAVE_MISO_PIN) */
#endif /* (MUX_CTRL_SPI_SLAVE_PINS) */

#if (MUX_CTRL_SPI_MASTER_SS0_PIN)
    #include "MUX_CTRL_ss0_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_SS0_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS1_PIN)
    #include "MUX_CTRL_ss1_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_SS1_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS2_PIN)
    #include "MUX_CTRL_ss2_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_SS2_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS3_PIN)
    #include "MUX_CTRL_ss3_m.h"
#endif /* (MUX_CTRL_SPI_MASTER_SS3_PIN) */

#if (MUX_CTRL_UART_TX_PIN)
    #include "MUX_CTRL_tx.h"
#endif /* (MUX_CTRL_UART_TX_PIN) */

#if (MUX_CTRL_UART_RX_TX_PIN)
    #include "MUX_CTRL_rx_tx.h"
#endif /* (MUX_CTRL_UART_RX_TX_PIN) */

#if (MUX_CTRL_UART_RX_PIN)
    #include "MUX_CTRL_rx.h"
#endif /* (MUX_CTRL_UART_RX_PIN) */

#if (MUX_CTRL_UART_RX_WAKE_PIN)
    #include "MUX_CTRL_rx_wake.h"
#endif /* (MUX_CTRL_UART_RX_WAKE_PIN) */

#if (MUX_CTRL_UART_RTS_PIN)
    #include "MUX_CTRL_rts.h"
#endif /* (MUX_CTRL_UART_RTS_PIN) */

#if (MUX_CTRL_UART_CTS_PIN)
    #include "MUX_CTRL_cts.h"
#endif /* (MUX_CTRL_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG \
                            (*(reg32 *) MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_PTR \
                            ( (reg32 *) MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_MASK \
                            (MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_POS \
                            (MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)

    #define MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_REG \
                            (*(reg32 *) MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_PTR \
                            ( (reg32 *) MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK \
                            ((uint32) MUX_CTRL_INTCFG_TYPE_MASK << \
                                      MUX_CTRL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#endif /* (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN) */

#if (MUX_CTRL_RX_SCL_MOSI_PIN)
    #define MUX_CTRL_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) MUX_CTRL_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define MUX_CTRL_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) MUX_CTRL_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define MUX_CTRL_RX_SCL_MOSI_HSIOM_MASK  (MUX_CTRL_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define MUX_CTRL_RX_SCL_MOSI_HSIOM_POS   (MUX_CTRL_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_RX_SCL_MOSI_PIN) */

#if (MUX_CTRL_TX_SDA_MISO_PIN)
    #define MUX_CTRL_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) MUX_CTRL_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define MUX_CTRL_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) MUX_CTRL_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define MUX_CTRL_TX_SDA_MISO_HSIOM_MASK  (MUX_CTRL_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define MUX_CTRL_TX_SDA_MISO_HSIOM_POS   (MUX_CTRL_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_TX_SDA_MISO_PIN) */

#if (MUX_CTRL_CTS_SCLK_PIN)
    #define MUX_CTRL_CTS_SCLK_HSIOM_REG   (*(reg32 *) MUX_CTRL_uart_cts_spi_sclk__0__HSIOM)
    #define MUX_CTRL_CTS_SCLK_HSIOM_PTR   ( (reg32 *) MUX_CTRL_uart_cts_spi_sclk__0__HSIOM)
    #define MUX_CTRL_CTS_SCLK_HSIOM_MASK  (MUX_CTRL_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define MUX_CTRL_CTS_SCLK_HSIOM_POS   (MUX_CTRL_uart_cts_spi_sclk__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_CTS_SCLK_PIN) */

#if (MUX_CTRL_RTS_SS0_PIN)
    #define MUX_CTRL_RTS_SS0_HSIOM_REG   (*(reg32 *) MUX_CTRL_uart_rts_spi_ss0__0__HSIOM)
    #define MUX_CTRL_RTS_SS0_HSIOM_PTR   ( (reg32 *) MUX_CTRL_uart_rts_spi_ss0__0__HSIOM)
    #define MUX_CTRL_RTS_SS0_HSIOM_MASK  (MUX_CTRL_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define MUX_CTRL_RTS_SS0_HSIOM_POS   (MUX_CTRL_uart_rts_spi_ss0__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_RTS_SS0_PIN) */

#if (MUX_CTRL_SS1_PIN)
    #define MUX_CTRL_SS1_HSIOM_REG      (*(reg32 *) MUX_CTRL_spi_ss1__0__HSIOM)
    #define MUX_CTRL_SS1_HSIOM_PTR      ( (reg32 *) MUX_CTRL_spi_ss1__0__HSIOM)
    #define MUX_CTRL_SS1_HSIOM_MASK     (MUX_CTRL_spi_ss1__0__HSIOM_MASK)
    #define MUX_CTRL_SS1_HSIOM_POS      (MUX_CTRL_spi_ss1__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SS1_PIN) */

#if (MUX_CTRL_SS2_PIN)
    #define MUX_CTRL_SS2_HSIOM_REG     (*(reg32 *) MUX_CTRL_spi_ss2__0__HSIOM)
    #define MUX_CTRL_SS2_HSIOM_PTR     ( (reg32 *) MUX_CTRL_spi_ss2__0__HSIOM)
    #define MUX_CTRL_SS2_HSIOM_MASK    (MUX_CTRL_spi_ss2__0__HSIOM_MASK)
    #define MUX_CTRL_SS2_HSIOM_POS     (MUX_CTRL_spi_ss2__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SS2_PIN) */

#if (MUX_CTRL_SS3_PIN)
    #define MUX_CTRL_SS3_HSIOM_REG     (*(reg32 *) MUX_CTRL_spi_ss3__0__HSIOM)
    #define MUX_CTRL_SS3_HSIOM_PTR     ( (reg32 *) MUX_CTRL_spi_ss3__0__HSIOM)
    #define MUX_CTRL_SS3_HSIOM_MASK    (MUX_CTRL_spi_ss3__0__HSIOM_MASK)
    #define MUX_CTRL_SS3_HSIOM_POS     (MUX_CTRL_spi_ss3__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SS3_PIN) */

#if (MUX_CTRL_I2C_PINS)
    #define MUX_CTRL_SCL_HSIOM_REG     (*(reg32 *) MUX_CTRL_scl__0__HSIOM)
    #define MUX_CTRL_SCL_HSIOM_PTR     ( (reg32 *) MUX_CTRL_scl__0__HSIOM)
    #define MUX_CTRL_SCL_HSIOM_MASK    (MUX_CTRL_scl__0__HSIOM_MASK)
    #define MUX_CTRL_SCL_HSIOM_POS     (MUX_CTRL_scl__0__HSIOM_SHIFT)

    #define MUX_CTRL_SDA_HSIOM_REG     (*(reg32 *) MUX_CTRL_sda__0__HSIOM)
    #define MUX_CTRL_SDA_HSIOM_PTR     ( (reg32 *) MUX_CTRL_sda__0__HSIOM)
    #define MUX_CTRL_SDA_HSIOM_MASK    (MUX_CTRL_sda__0__HSIOM_MASK)
    #define MUX_CTRL_SDA_HSIOM_POS     (MUX_CTRL_sda__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_I2C_PINS) */

#if (MUX_CTRL_SPI_MASTER_SCLK_PIN)
    #define MUX_CTRL_SCLK_M_HSIOM_REG   (*(reg32 *) MUX_CTRL_sclk_m__0__HSIOM)
    #define MUX_CTRL_SCLK_M_HSIOM_PTR   ( (reg32 *) MUX_CTRL_sclk_m__0__HSIOM)
    #define MUX_CTRL_SCLK_M_HSIOM_MASK  (MUX_CTRL_sclk_m__0__HSIOM_MASK)
    #define MUX_CTRL_SCLK_M_HSIOM_POS   (MUX_CTRL_sclk_m__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SPI_MASTER_SCLK_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS0_PIN)
    #define MUX_CTRL_SS0_M_HSIOM_REG    (*(reg32 *) MUX_CTRL_ss0_m__0__HSIOM)
    #define MUX_CTRL_SS0_M_HSIOM_PTR    ( (reg32 *) MUX_CTRL_ss0_m__0__HSIOM)
    #define MUX_CTRL_SS0_M_HSIOM_MASK   (MUX_CTRL_ss0_m__0__HSIOM_MASK)
    #define MUX_CTRL_SS0_M_HSIOM_POS    (MUX_CTRL_ss0_m__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SPI_MASTER_SS0_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS1_PIN)
    #define MUX_CTRL_SS1_M_HSIOM_REG    (*(reg32 *) MUX_CTRL_ss1_m__0__HSIOM)
    #define MUX_CTRL_SS1_M_HSIOM_PTR    ( (reg32 *) MUX_CTRL_ss1_m__0__HSIOM)
    #define MUX_CTRL_SS1_M_HSIOM_MASK   (MUX_CTRL_ss1_m__0__HSIOM_MASK)
    #define MUX_CTRL_SS1_M_HSIOM_POS    (MUX_CTRL_ss1_m__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SPI_MASTER_SS1_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS2_PIN)
    #define MUX_CTRL_SS2_M_HSIOM_REG    (*(reg32 *) MUX_CTRL_ss2_m__0__HSIOM)
    #define MUX_CTRL_SS2_M_HSIOM_PTR    ( (reg32 *) MUX_CTRL_ss2_m__0__HSIOM)
    #define MUX_CTRL_SS2_M_HSIOM_MASK   (MUX_CTRL_ss2_m__0__HSIOM_MASK)
    #define MUX_CTRL_SS2_M_HSIOM_POS    (MUX_CTRL_ss2_m__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SPI_MASTER_SS2_PIN) */

#if (MUX_CTRL_SPI_MASTER_SS3_PIN)
    #define MUX_CTRL_SS3_M_HSIOM_REG    (*(reg32 *) MUX_CTRL_ss3_m__0__HSIOM)
    #define MUX_CTRL_SS3_M_HSIOM_PTR    ( (reg32 *) MUX_CTRL_ss3_m__0__HSIOM)
    #define MUX_CTRL_SS3_M_HSIOM_MASK   (MUX_CTRL_ss3_m__0__HSIOM_MASK)
    #define MUX_CTRL_SS3_M_HSIOM_POS    (MUX_CTRL_ss3_m__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_SPI_MASTER_SS3_PIN) */

#if (MUX_CTRL_UART_TX_PIN)
    #define MUX_CTRL_TX_HSIOM_REG   (*(reg32 *) MUX_CTRL_tx__0__HSIOM)
    #define MUX_CTRL_TX_HSIOM_PTR   ( (reg32 *) MUX_CTRL_tx_0__HSIOM)
    #define MUX_CTRL_TX_HSIOM_MASK  (MUX_CTRL_tx__0__HSIOM_MASK)
    #define MUX_CTRL_TX_HSIOM_POS   (MUX_CTRL_tx__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_UART_TX_PIN) */

#if (MUX_CTRL_UART_RTS_PIN)
    #define MUX_CTRL_RTS_HSIOM_REG  (*(reg32 *) MUX_CTRL_rts__0__HSIOM)
    #define MUX_CTRL_RTS_HSIOM_PTR  ( (reg32 *) MUX_CTRL_rts__0__HSIOM)
    #define MUX_CTRL_RTS_HSIOM_MASK (MUX_CTRL_rts__0__HSIOM_MASK)
    #define MUX_CTRL_RTS_HSIOM_POS  (MUX_CTRL_rts__0__HSIOM_SHIFT)
#endif /* (MUX_CTRL_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constants */
#define MUX_CTRL_HSIOM_DEF_SEL      (0x00u)
#define MUX_CTRL_HSIOM_GPIO_SEL     (0x00u)
#define MUX_CTRL_HSIOM_UART_SEL     (0x09u)
#define MUX_CTRL_HSIOM_I2C_SEL      (0x0Eu)
#define MUX_CTRL_HSIOM_SPI_SEL      (0x0Fu)

#define MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define MUX_CTRL_RX_SCL_MOSI_PIN_INDEX       (0u)
#define MUX_CTRL_TX_SDA_MISO_PIN_INDEX       (1u)
#define MUX_CTRL_CTS_SCLK_PIN_INDEX       (2u)
#define MUX_CTRL_RTS_SS0_PIN_INDEX       (3u)
#define MUX_CTRL_SS1_PIN_INDEX                  (4u)
#define MUX_CTRL_SS2_PIN_INDEX                  (5u)
#define MUX_CTRL_SS3_PIN_INDEX                  (6u)

#define MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define MUX_CTRL_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << MUX_CTRL_RX_SCL_MOSI_PIN_INDEX)
#define MUX_CTRL_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << MUX_CTRL_TX_SDA_MISO_PIN_INDEX)
#define MUX_CTRL_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << MUX_CTRL_CTS_SCLK_PIN_INDEX)
#define MUX_CTRL_RTS_SS0_PIN_MASK     ((uint32) 0x01u << MUX_CTRL_RTS_SS0_PIN_INDEX)
#define MUX_CTRL_SS1_PIN_MASK                ((uint32) 0x01u << MUX_CTRL_SS1_PIN_INDEX)
#define MUX_CTRL_SS2_PIN_MASK                ((uint32) 0x01u << MUX_CTRL_SS2_PIN_INDEX)
#define MUX_CTRL_SS3_PIN_MASK                ((uint32) 0x01u << MUX_CTRL_SS3_PIN_INDEX)

/* Pin interrupt constants */
#define MUX_CTRL_INTCFG_TYPE_MASK           (0x03u)
#define MUX_CTRL_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants */
#define MUX_CTRL_PIN_DM_ALG_HIZ  (0u)
#define MUX_CTRL_PIN_DM_DIG_HIZ  (1u)
#define MUX_CTRL_PIN_DM_OD_LO    (4u)
#define MUX_CTRL_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define MUX_CTRL_DM_MASK    (0x7u)
#define MUX_CTRL_DM_SIZE    (3)
#define MUX_CTRL_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) MUX_CTRL_DM_MASK << (MUX_CTRL_DM_SIZE * (pos)))) >> \
                                                              (MUX_CTRL_DM_SIZE * (pos)) )

#if (MUX_CTRL_TX_SDA_MISO_PIN)
    #define MUX_CTRL_CHECK_TX_SDA_MISO_PIN_USED \
                (MUX_CTRL_PIN_DM_ALG_HIZ != \
                    MUX_CTRL_GET_P4_PIN_DM(MUX_CTRL_uart_tx_i2c_sda_spi_miso_PC, \
                                                   MUX_CTRL_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (MUX_CTRL_TX_SDA_MISO_PIN) */

#if (MUX_CTRL_RTS_SS0_PIN)
    #define MUX_CTRL_CHECK_RTS_SS0_PIN_USED \
                (MUX_CTRL_PIN_DM_ALG_HIZ != \
                    MUX_CTRL_GET_P4_PIN_DM(MUX_CTRL_uart_rts_spi_ss0_PC, \
                                                   MUX_CTRL_uart_rts_spi_ss0_SHIFT))
#endif /* (MUX_CTRL_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define MUX_CTRL_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define MUX_CTRL_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define MUX_CTRL_SET_HSIOM_SEL(reg, mask, pos, sel) MUX_CTRL_SET_REGISTER_BITS(reg, mask, pos, sel)
#define MUX_CTRL_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        MUX_CTRL_SET_REGISTER_BITS(reg, mask, pos, intType)
#define MUX_CTRL_SET_INP_DIS(reg, mask, val) MUX_CTRL_SET_REGISTER_BIT(reg, mask, val)

/* MUX_CTRL_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  MUX_CTRL_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (MUX_CTRL_CY_SCBIP_V0)
#if (MUX_CTRL_I2C_PINS)
    #define MUX_CTRL_SET_I2C_SCL_DR(val) MUX_CTRL_scl_Write(val)

    #define MUX_CTRL_SET_I2C_SCL_HSIOM_SEL(sel) \
                          MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_SCL_HSIOM_REG,  \
                                                         MUX_CTRL_SCL_HSIOM_MASK, \
                                                         MUX_CTRL_SCL_HSIOM_POS,  \
                                                         (sel))
    #define MUX_CTRL_WAIT_SCL_SET_HIGH  (0u == MUX_CTRL_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN)
    #define MUX_CTRL_SET_I2C_SCL_DR(val) \
                            MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define MUX_CTRL_SET_I2C_SCL_HSIOM_SEL(sel) \
                    MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define MUX_CTRL_WAIT_SCL_SET_HIGH  (0u == MUX_CTRL_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (MUX_CTRL_RX_SCL_MOSI_PIN)
    #define MUX_CTRL_SET_I2C_SCL_DR(val) \
                            MUX_CTRL_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define MUX_CTRL_SET_I2C_SCL_HSIOM_SEL(sel) \
                            MUX_CTRL_SET_HSIOM_SEL(MUX_CTRL_RX_SCL_MOSI_HSIOM_REG,  \
                                                           MUX_CTRL_RX_SCL_MOSI_HSIOM_MASK, \
                                                           MUX_CTRL_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define MUX_CTRL_WAIT_SCL_SET_HIGH  (0u == MUX_CTRL_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define MUX_CTRL_SET_I2C_SCL_DR(val) \
                                                    do{ /* Does nothing */ }while(0)
    #define MUX_CTRL_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                    do{ /* Does nothing */ }while(0)

    #define MUX_CTRL_WAIT_SCL_SET_HIGH  (0u)
#endif /* (MUX_CTRL_I2C_PINS) */

/* SCB I2C: sda signal */
#if (MUX_CTRL_I2C_PINS)
    #define MUX_CTRL_WAIT_SDA_SET_HIGH  (0u == MUX_CTRL_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (MUX_CTRL_TX_SDA_MISO_PIN)
    #define MUX_CTRL_WAIT_SDA_SET_HIGH  (0u == MUX_CTRL_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define MUX_CTRL_WAIT_SDA_SET_HIGH  (0u)
#endif /* (MUX_CTRL_MOSI_SCL_RX_PIN) */
#endif /* (MUX_CTRL_CY_SCBIP_V0) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define MUX_CTRL_REMOVE_MOSI_SCL_RX_WAKE_PIN    MUX_CTRL_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define MUX_CTRL_REMOVE_MOSI_SCL_RX_PIN         MUX_CTRL_REMOVE_RX_SCL_MOSI_PIN
#define MUX_CTRL_REMOVE_MISO_SDA_TX_PIN         MUX_CTRL_REMOVE_TX_SDA_MISO_PIN
#ifndef MUX_CTRL_REMOVE_SCLK_PIN
#define MUX_CTRL_REMOVE_SCLK_PIN                MUX_CTRL_REMOVE_CTS_SCLK_PIN
#endif /* MUX_CTRL_REMOVE_SCLK_PIN */
#ifndef MUX_CTRL_REMOVE_SS0_PIN
#define MUX_CTRL_REMOVE_SS0_PIN                 MUX_CTRL_REMOVE_RTS_SS0_PIN
#endif /* MUX_CTRL_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define MUX_CTRL_MOSI_SCL_RX_WAKE_PIN   MUX_CTRL_RX_WAKE_SCL_MOSI_PIN
#define MUX_CTRL_MOSI_SCL_RX_PIN        MUX_CTRL_RX_SCL_MOSI_PIN
#define MUX_CTRL_MISO_SDA_TX_PIN        MUX_CTRL_TX_SDA_MISO_PIN
#ifndef MUX_CTRL_SCLK_PIN
#define MUX_CTRL_SCLK_PIN               MUX_CTRL_CTS_SCLK_PIN
#endif /* MUX_CTRL_SCLK_PIN */
#ifndef MUX_CTRL_SS0_PIN
#define MUX_CTRL_SS0_PIN                MUX_CTRL_RTS_SS0_PIN
#endif /* MUX_CTRL_SS0_PIN */

#if (MUX_CTRL_MOSI_SCL_RX_WAKE_PIN)
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_HSIOM_REG     MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_HSIOM_PTR     MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_HSIOM_MASK    MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_HSIOM_POS     MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define MUX_CTRL_MOSI_SCL_RX_WAKE_INTCFG_REG    MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_INTCFG_PTR    MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define MUX_CTRL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  MUX_CTRL_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (MUX_CTRL_RX_WAKE_SCL_MOSI_PIN) */

#if (MUX_CTRL_MOSI_SCL_RX_PIN)
    #define MUX_CTRL_MOSI_SCL_RX_HSIOM_REG      MUX_CTRL_RX_SCL_MOSI_HSIOM_REG
    #define MUX_CTRL_MOSI_SCL_RX_HSIOM_PTR      MUX_CTRL_RX_SCL_MOSI_HSIOM_PTR
    #define MUX_CTRL_MOSI_SCL_RX_HSIOM_MASK     MUX_CTRL_RX_SCL_MOSI_HSIOM_MASK
    #define MUX_CTRL_MOSI_SCL_RX_HSIOM_POS      MUX_CTRL_RX_SCL_MOSI_HSIOM_POS
#endif /* (MUX_CTRL_MOSI_SCL_RX_PIN) */

#if (MUX_CTRL_MISO_SDA_TX_PIN)
    #define MUX_CTRL_MISO_SDA_TX_HSIOM_REG      MUX_CTRL_TX_SDA_MISO_HSIOM_REG
    #define MUX_CTRL_MISO_SDA_TX_HSIOM_PTR      MUX_CTRL_TX_SDA_MISO_HSIOM_REG
    #define MUX_CTRL_MISO_SDA_TX_HSIOM_MASK     MUX_CTRL_TX_SDA_MISO_HSIOM_REG
    #define MUX_CTRL_MISO_SDA_TX_HSIOM_POS      MUX_CTRL_TX_SDA_MISO_HSIOM_REG
#endif /* (MUX_CTRL_MISO_SDA_TX_PIN_PIN) */

#if (MUX_CTRL_SCLK_PIN)
    #ifndef MUX_CTRL_SCLK_HSIOM_REG
    #define MUX_CTRL_SCLK_HSIOM_REG     MUX_CTRL_CTS_SCLK_HSIOM_REG
    #define MUX_CTRL_SCLK_HSIOM_PTR     MUX_CTRL_CTS_SCLK_HSIOM_PTR
    #define MUX_CTRL_SCLK_HSIOM_MASK    MUX_CTRL_CTS_SCLK_HSIOM_MASK
    #define MUX_CTRL_SCLK_HSIOM_POS     MUX_CTRL_CTS_SCLK_HSIOM_POS
    #endif /* MUX_CTRL_SCLK_HSIOM_REG */
#endif /* (MUX_CTRL_SCLK_PIN) */

#if (MUX_CTRL_SS0_PIN)
    #ifndef MUX_CTRL_SS0_HSIOM_REG
    #define MUX_CTRL_SS0_HSIOM_REG      MUX_CTRL_RTS_SS0_HSIOM_REG
    #define MUX_CTRL_SS0_HSIOM_PTR      MUX_CTRL_RTS_SS0_HSIOM_PTR
    #define MUX_CTRL_SS0_HSIOM_MASK     MUX_CTRL_RTS_SS0_HSIOM_MASK
    #define MUX_CTRL_SS0_HSIOM_POS      MUX_CTRL_RTS_SS0_HSIOM_POS
    #endif /* MUX_CTRL_SS0_HSIOM_REG */
#endif /* (MUX_CTRL_SS0_PIN) */

#define MUX_CTRL_MOSI_SCL_RX_WAKE_PIN_INDEX MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_INDEX
#define MUX_CTRL_MOSI_SCL_RX_PIN_INDEX      MUX_CTRL_RX_SCL_MOSI_PIN_INDEX
#define MUX_CTRL_MISO_SDA_TX_PIN_INDEX      MUX_CTRL_TX_SDA_MISO_PIN_INDEX
#ifndef MUX_CTRL_SCLK_PIN_INDEX
#define MUX_CTRL_SCLK_PIN_INDEX             MUX_CTRL_CTS_SCLK_PIN_INDEX
#endif /* MUX_CTRL_SCLK_PIN_INDEX */
#ifndef MUX_CTRL_SS0_PIN_INDEX
#define MUX_CTRL_SS0_PIN_INDEX              MUX_CTRL_RTS_SS0_PIN_INDEX
#endif /* MUX_CTRL_SS0_PIN_INDEX */

#define MUX_CTRL_MOSI_SCL_RX_WAKE_PIN_MASK MUX_CTRL_RX_WAKE_SCL_MOSI_PIN_MASK
#define MUX_CTRL_MOSI_SCL_RX_PIN_MASK      MUX_CTRL_RX_SCL_MOSI_PIN_MASK
#define MUX_CTRL_MISO_SDA_TX_PIN_MASK      MUX_CTRL_TX_SDA_MISO_PIN_MASK
#ifndef MUX_CTRL_SCLK_PIN_MASK
#define MUX_CTRL_SCLK_PIN_MASK             MUX_CTRL_CTS_SCLK_PIN_MASK
#endif /* MUX_CTRL_SCLK_PIN_MASK */
#ifndef MUX_CTRL_SS0_PIN_MASK
#define MUX_CTRL_SS0_PIN_MASK              MUX_CTRL_RTS_SS0_PIN_MASK
#endif /* MUX_CTRL_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_MUX_CTRL_H) */


/* [] END OF FILE */
