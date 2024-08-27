/*******************************************************************************
* File Name: HPI_IF_scl.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "HPI_IF_scl.h"

static HPI_IF_scl_BACKUP_STRUCT  HPI_IF_scl_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: HPI_IF_scl_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function must be called for SIO and USBIO
*  pins. It is not essential if using GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet HPI_IF_scl_SUT.c usage_HPI_IF_scl_Sleep_Wakeup
*******************************************************************************/
void HPI_IF_scl_Sleep(void)
{
    #if defined(HPI_IF_scl__PC)
        HPI_IF_scl_backup.pcState = HPI_IF_scl_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            HPI_IF_scl_backup.usbState = HPI_IF_scl_CR1_REG;
            HPI_IF_scl_USB_POWER_REG |= HPI_IF_scl_USBIO_ENTER_SLEEP;
            HPI_IF_scl_CR1_REG &= HPI_IF_scl_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(HPI_IF_scl__SIO)
        HPI_IF_scl_backup.sioState = HPI_IF_scl_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        HPI_IF_scl_SIO_REG &= (uint32)(~HPI_IF_scl_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: HPI_IF_scl_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep().
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to HPI_IF_scl_Sleep() for an example usage.
*******************************************************************************/
void HPI_IF_scl_Wakeup(void)
{
    #if defined(HPI_IF_scl__PC)
        HPI_IF_scl_PC = HPI_IF_scl_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            HPI_IF_scl_USB_POWER_REG &= HPI_IF_scl_USBIO_EXIT_SLEEP_PH1;
            HPI_IF_scl_CR1_REG = HPI_IF_scl_backup.usbState;
            HPI_IF_scl_USB_POWER_REG &= HPI_IF_scl_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(HPI_IF_scl__SIO)
        HPI_IF_scl_SIO_REG = HPI_IF_scl_backup.sioState;
    #endif
}


/* [] END OF FILE */
