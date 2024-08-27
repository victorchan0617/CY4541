/*******************************************************************************
* File Name: VCONN_MON_P1.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VCONN_MON_P1_H) /* Pins VCONN_MON_P1_H */
#define CY_PINS_VCONN_MON_P1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "VCONN_MON_P1_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} VCONN_MON_P1_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   VCONN_MON_P1_Read(void);
void    VCONN_MON_P1_Write(uint8 value);
uint8   VCONN_MON_P1_ReadDataReg(void);
#if defined(VCONN_MON_P1__PC) || (CY_PSOC4_4200L) 
    void    VCONN_MON_P1_SetDriveMode(uint8 mode);
#endif
void    VCONN_MON_P1_SetInterruptMode(uint16 position, uint16 mode);
uint8   VCONN_MON_P1_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void VCONN_MON_P1_Sleep(void); 
void VCONN_MON_P1_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(VCONN_MON_P1__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define VCONN_MON_P1_DRIVE_MODE_BITS        (3)
    #define VCONN_MON_P1_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - VCONN_MON_P1_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the VCONN_MON_P1_SetDriveMode() function.
         *  @{
         */
        #define VCONN_MON_P1_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define VCONN_MON_P1_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define VCONN_MON_P1_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define VCONN_MON_P1_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define VCONN_MON_P1_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define VCONN_MON_P1_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define VCONN_MON_P1_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define VCONN_MON_P1_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define VCONN_MON_P1_MASK               VCONN_MON_P1__MASK
#define VCONN_MON_P1_SHIFT              VCONN_MON_P1__SHIFT
#define VCONN_MON_P1_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VCONN_MON_P1_SetInterruptMode() function.
     *  @{
     */
        #define VCONN_MON_P1_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define VCONN_MON_P1_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define VCONN_MON_P1_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define VCONN_MON_P1_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(VCONN_MON_P1__SIO)
    #define VCONN_MON_P1_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(VCONN_MON_P1__PC) && (CY_PSOC4_4200L)
    #define VCONN_MON_P1_USBIO_ENABLE               ((uint32)0x80000000u)
    #define VCONN_MON_P1_USBIO_DISABLE              ((uint32)(~VCONN_MON_P1_USBIO_ENABLE))
    #define VCONN_MON_P1_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define VCONN_MON_P1_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define VCONN_MON_P1_USBIO_ENTER_SLEEP          ((uint32)((1u << VCONN_MON_P1_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << VCONN_MON_P1_USBIO_SUSPEND_DEL_SHIFT)))
    #define VCONN_MON_P1_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << VCONN_MON_P1_USBIO_SUSPEND_SHIFT)))
    #define VCONN_MON_P1_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << VCONN_MON_P1_USBIO_SUSPEND_DEL_SHIFT)))
    #define VCONN_MON_P1_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(VCONN_MON_P1__PC)
    /* Port Configuration */
    #define VCONN_MON_P1_PC                 (* (reg32 *) VCONN_MON_P1__PC)
#endif
/* Pin State */
#define VCONN_MON_P1_PS                     (* (reg32 *) VCONN_MON_P1__PS)
/* Data Register */
#define VCONN_MON_P1_DR                     (* (reg32 *) VCONN_MON_P1__DR)
/* Input Buffer Disable Override */
#define VCONN_MON_P1_INP_DIS                (* (reg32 *) VCONN_MON_P1__PC2)

/* Interrupt configuration Registers */
#define VCONN_MON_P1_INTCFG                 (* (reg32 *) VCONN_MON_P1__INTCFG)
#define VCONN_MON_P1_INTSTAT                (* (reg32 *) VCONN_MON_P1__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define VCONN_MON_P1_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(VCONN_MON_P1__SIO)
    #define VCONN_MON_P1_SIO_REG            (* (reg32 *) VCONN_MON_P1__SIO)
#endif /* (VCONN_MON_P1__SIO_CFG) */

/* USBIO registers */
#if !defined(VCONN_MON_P1__PC) && (CY_PSOC4_4200L)
    #define VCONN_MON_P1_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define VCONN_MON_P1_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define VCONN_MON_P1_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define VCONN_MON_P1_DRIVE_MODE_SHIFT       (0x00u)
#define VCONN_MON_P1_DRIVE_MODE_MASK        (0x07u << VCONN_MON_P1_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins VCONN_MON_P1_H */


/* [] END OF FILE */
