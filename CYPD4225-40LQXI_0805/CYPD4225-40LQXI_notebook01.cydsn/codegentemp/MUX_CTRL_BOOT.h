/*******************************************************************************
* File Name: MUX_CTRL_BOOT.h
* Version 3.10
*
* Description:
*  This file provides constants and parameter values of the bootloader
*  communication APIs for the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2014-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_MUX_CTRL_H)
#define CY_SCB_BOOT_MUX_CTRL_H

#include "MUX_CTRL_PVT.h"

#if (MUX_CTRL_SCB_MODE_I2C_INC)
    #include "MUX_CTRL_I2C.h"
#endif /* (MUX_CTRL_SCB_MODE_I2C_INC) */

#if (MUX_CTRL_SCB_MODE_EZI2C_INC)
    #include "MUX_CTRL_EZI2C.h"
#endif /* (MUX_CTRL_SCB_MODE_EZI2C_INC) */

#if (MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC)
    #include "MUX_CTRL_SPI_UART.h"
#endif /* (MUX_CTRL_SCB_MODE_SPI_INC || MUX_CTRL_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define MUX_CTRL_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MUX_CTRL) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (MUX_CTRL_SCB_MODE_I2C_INC)
    #define MUX_CTRL_I2C_BTLDR_COMM_ENABLED     (MUX_CTRL_BTLDR_COMM_ENABLED && \
                                                            (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             MUX_CTRL_I2C_SLAVE_CONST))
#else
     #define MUX_CTRL_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (MUX_CTRL_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (MUX_CTRL_SCB_MODE_EZI2C_INC)
    #define MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED   (MUX_CTRL_BTLDR_COMM_ENABLED && \
                                                         MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (MUX_CTRL_SCB_MODE_SPI_INC)
    #define MUX_CTRL_SPI_BTLDR_COMM_ENABLED     (MUX_CTRL_BTLDR_COMM_ENABLED && \
                                                            (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             MUX_CTRL_SPI_SLAVE_CONST))
#else
        #define MUX_CTRL_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (MUX_CTRL_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (MUX_CTRL_SCB_MODE_UART_INC)
       #define MUX_CTRL_UART_BTLDR_COMM_ENABLED    (MUX_CTRL_BTLDR_COMM_ENABLED && \
                                                            (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (MUX_CTRL_UART_RX_DIRECTION && \
                                                              MUX_CTRL_UART_TX_DIRECTION)))
#else
     #define MUX_CTRL_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (MUX_CTRL_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define MUX_CTRL_BTLDR_COMM_MODE_ENABLED    (MUX_CTRL_I2C_BTLDR_COMM_ENABLED   || \
                                                     MUX_CTRL_SPI_BTLDR_COMM_ENABLED   || \
                                                     MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED || \
                                                     MUX_CTRL_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void MUX_CTRL_I2CCyBtldrCommStart(void);
    void MUX_CTRL_I2CCyBtldrCommStop (void);
    void MUX_CTRL_I2CCyBtldrCommReset(void);
    cystatus MUX_CTRL_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MUX_CTRL_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (MUX_CTRL_SCB_MODE_I2C_CONST_CFG)
        #define MUX_CTRL_CyBtldrCommStart   MUX_CTRL_I2CCyBtldrCommStart
        #define MUX_CTRL_CyBtldrCommStop    MUX_CTRL_I2CCyBtldrCommStop
        #define MUX_CTRL_CyBtldrCommReset   MUX_CTRL_I2CCyBtldrCommReset
        #define MUX_CTRL_CyBtldrCommRead    MUX_CTRL_I2CCyBtldrCommRead
        #define MUX_CTRL_CyBtldrCommWrite   MUX_CTRL_I2CCyBtldrCommWrite
    #endif /* (MUX_CTRL_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void MUX_CTRL_EzI2CCyBtldrCommStart(void);
    void MUX_CTRL_EzI2CCyBtldrCommStop (void);
    void MUX_CTRL_EzI2CCyBtldrCommReset(void);
    cystatus MUX_CTRL_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MUX_CTRL_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG)
        #define MUX_CTRL_CyBtldrCommStart   MUX_CTRL_EzI2CCyBtldrCommStart
        #define MUX_CTRL_CyBtldrCommStop    MUX_CTRL_EzI2CCyBtldrCommStop
        #define MUX_CTRL_CyBtldrCommReset   MUX_CTRL_EzI2CCyBtldrCommReset
        #define MUX_CTRL_CyBtldrCommRead    MUX_CTRL_EzI2CCyBtldrCommRead
        #define MUX_CTRL_CyBtldrCommWrite   MUX_CTRL_EzI2CCyBtldrCommWrite
    #endif /* (MUX_CTRL_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void MUX_CTRL_SpiCyBtldrCommStart(void);
    void MUX_CTRL_SpiCyBtldrCommStop (void);
    void MUX_CTRL_SpiCyBtldrCommReset(void);
    cystatus MUX_CTRL_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MUX_CTRL_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (MUX_CTRL_SCB_MODE_SPI_CONST_CFG)
        #define MUX_CTRL_CyBtldrCommStart   MUX_CTRL_SpiCyBtldrCommStart
        #define MUX_CTRL_CyBtldrCommStop    MUX_CTRL_SpiCyBtldrCommStop
        #define MUX_CTRL_CyBtldrCommReset   MUX_CTRL_SpiCyBtldrCommReset
        #define MUX_CTRL_CyBtldrCommRead    MUX_CTRL_SpiCyBtldrCommRead
        #define MUX_CTRL_CyBtldrCommWrite   MUX_CTRL_SpiCyBtldrCommWrite
    #endif /* (MUX_CTRL_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void MUX_CTRL_UartCyBtldrCommStart(void);
    void MUX_CTRL_UartCyBtldrCommStop (void);
    void MUX_CTRL_UartCyBtldrCommReset(void);
    cystatus MUX_CTRL_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus MUX_CTRL_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (MUX_CTRL_SCB_MODE_UART_CONST_CFG)
        #define MUX_CTRL_CyBtldrCommStart   MUX_CTRL_UartCyBtldrCommStart
        #define MUX_CTRL_CyBtldrCommStop    MUX_CTRL_UartCyBtldrCommStop
        #define MUX_CTRL_CyBtldrCommReset   MUX_CTRL_UartCyBtldrCommReset
        #define MUX_CTRL_CyBtldrCommRead    MUX_CTRL_UartCyBtldrCommRead
        #define MUX_CTRL_CyBtldrCommWrite   MUX_CTRL_UartCyBtldrCommWrite
    #endif /* (MUX_CTRL_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_UART_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_BTLDR_COMM_ENABLED)
    #if (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void MUX_CTRL_CyBtldrCommStart(void);
        void MUX_CTRL_CyBtldrCommStop (void);
        void MUX_CTRL_CyBtldrCommReset(void);
        cystatus MUX_CTRL_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus MUX_CTRL_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (MUX_CTRL_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MUX_CTRL)
        #define CyBtldrCommStart    MUX_CTRL_CyBtldrCommStart
        #define CyBtldrCommStop     MUX_CTRL_CyBtldrCommStop
        #define CyBtldrCommReset    MUX_CTRL_CyBtldrCommReset
        #define CyBtldrCommWrite    MUX_CTRL_CyBtldrCommWrite
        #define CyBtldrCommRead     MUX_CTRL_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MUX_CTRL) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MUX_CTRL_BTLDR_COMM_ENABLED) */


/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define MUX_CTRL_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define MUX_CTRL_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define MUX_CTRL_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define MUX_CTRL_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef MUX_CTRL_SPI_BYTE_TO_BYTE
    #define MUX_CTRL_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef MUX_CTRL_UART_BYTE_TO_BYTE
    #define MUX_CTRL_UART_BYTE_TO_BYTE  (2500u)
#endif /* MUX_CTRL_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_MUX_CTRL_H) */


/* [] END OF FILE */
