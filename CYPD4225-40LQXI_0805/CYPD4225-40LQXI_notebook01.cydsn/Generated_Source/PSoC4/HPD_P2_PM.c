/*******************************************************************************
* File Name: HPD_P2.c  
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
#include "HPD_P2.h"

static HPD_P2_BACKUP_STRUCT  HPD_P2_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: HPD_P2_Sleep
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
*  \snippet HPD_P2_SUT.c usage_HPD_P2_Sleep_Wakeup
*******************************************************************************/
void HPD_P2_Sleep(void)
{
    #if defined(HPD_P2__PC)
        HPD_P2_backup.pcState = HPD_P2_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            HPD_P2_backup.usbState = HPD_P2_CR1_REG;
            HPD_P2_USB_POWER_REG |= HPD_P2_USBIO_ENTER_SLEEP;
            HPD_P2_CR1_REG &= HPD_P2_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(HPD_P2__SIO)
        HPD_P2_backup.sioState = HPD_P2_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        HPD_P2_SIO_REG &= (uint32)(~HPD_P2_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: HPD_P2_Wakeup
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
*  Refer to HPD_P2_Sleep() for an example usage.
*******************************************************************************/
void HPD_P2_Wakeup(void)
{
    #if defined(HPD_P2__PC)
        HPD_P2_PC = HPD_P2_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            HPD_P2_USB_POWER_REG &= HPD_P2_USBIO_EXIT_SLEEP_PH1;
            HPD_P2_CR1_REG = HPD_P2_backup.usbState;
            HPD_P2_USB_POWER_REG &= HPD_P2_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(HPD_P2__SIO)
        HPD_P2_SIO_REG = HPD_P2_backup.sioState;
    #endif
}


/* [] END OF FILE */
