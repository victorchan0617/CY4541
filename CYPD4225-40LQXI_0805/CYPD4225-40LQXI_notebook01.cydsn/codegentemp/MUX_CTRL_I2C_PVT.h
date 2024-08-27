/*******************************************************************************
* File Name: .h
* Version 3.10
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
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

#if !defined(CY_SCB_I2C_PVT_MUX_CTRL_H)
#define CY_SCB_I2C_PVT_MUX_CTRL_H

#include "MUX_CTRL_I2C.h"


/***************************************
*     Private Global Vars
***************************************/

extern volatile uint8 MUX_CTRL_state; /* Current state of I2C FSM */

#if(MUX_CTRL_I2C_SLAVE_CONST)
    extern volatile uint8 MUX_CTRL_slStatus;          /* Slave Status */

    /* Receive buffer variables */
    extern volatile uint8 * MUX_CTRL_slWrBufPtr;      /* Pointer to Receive buffer  */
    extern volatile uint32  MUX_CTRL_slWrBufSize;     /* Slave Receive buffer size  */
    extern volatile uint32  MUX_CTRL_slWrBufIndex;    /* Slave Receive buffer Index */

    /* Transmit buffer variables */
    extern volatile uint8 * MUX_CTRL_slRdBufPtr;      /* Pointer to Transmit buffer  */
    extern volatile uint32  MUX_CTRL_slRdBufSize;     /* Slave Transmit buffer size  */
    extern volatile uint32  MUX_CTRL_slRdBufIndex;    /* Slave Transmit buffer Index */
    extern volatile uint32  MUX_CTRL_slRdBufIndexTmp; /* Slave Transmit buffer Index Tmp */
    extern volatile uint8   MUX_CTRL_slOverFlowCount; /* Slave Transmit Overflow counter */
#endif /* (MUX_CTRL_I2C_SLAVE_CONST) */

#if(MUX_CTRL_I2C_MASTER_CONST)
    extern volatile uint16 MUX_CTRL_mstrStatus;      /* Master Status byte  */
    extern volatile uint8  MUX_CTRL_mstrControl;     /* Master Control byte */

    /* Receive buffer variables */
    extern volatile uint8 * MUX_CTRL_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint32  MUX_CTRL_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint32  MUX_CTRL_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Transmit buffer variables */
    extern volatile uint8 * MUX_CTRL_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint32  MUX_CTRL_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint32  MUX_CTRL_mstrWrBufIndex; /* Master Write buffer Index      */
    extern volatile uint32  MUX_CTRL_mstrWrBufIndexTmp; /* Master Write buffer Index Tmp */
#endif /* (MUX_CTRL_I2C_MASTER_CONST) */

#if (MUX_CTRL_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
    extern uint32 (*MUX_CTRL_customAddressHandler) (void);
#endif /* (MUX_CTRL_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */

/***************************************
*     Private Function Prototypes
***************************************/

#if(MUX_CTRL_SCB_MODE_I2C_CONST_CFG)
    void MUX_CTRL_I2CInit(void);
#endif /* (MUX_CTRL_SCB_MODE_I2C_CONST_CFG) */

void MUX_CTRL_I2CStop(void);
void MUX_CTRL_I2CSaveConfig(void);
void MUX_CTRL_I2CRestoreConfig(void);

#if(MUX_CTRL_I2C_MASTER_CONST)
    void MUX_CTRL_I2CReStartGeneration(void);
#endif /* (MUX_CTRL_I2C_MASTER_CONST) */

#endif /* (CY_SCB_I2C_PVT_MUX_CTRL_H) */


/* [] END OF FILE */
