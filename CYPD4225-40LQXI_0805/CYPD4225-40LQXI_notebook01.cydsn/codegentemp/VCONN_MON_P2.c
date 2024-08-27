/*******************************************************************************
* File Name: VCONN_MON_P2.c  
* Version 2.20
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "VCONN_MON_P2.h"


#if defined(VCONN_MON_P2__PC)
    #define VCONN_MON_P2_SetP4PinDriveMode(shift, mode)  \
    do { \
        VCONN_MON_P2_PC =   (VCONN_MON_P2_PC & \
                                (uint32)(~(uint32)(VCONN_MON_P2_DRIVE_MODE_IND_MASK << \
                                (VCONN_MON_P2_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << \
                                (VCONN_MON_P2_DRIVE_MODE_BITS * (shift))); \
    } while (0)
#else
    #if (CY_PSOC4_4200L)
        #define VCONN_MON_P2_SetP4PinDriveMode(shift, mode)  \
        do { \
            VCONN_MON_P2_USBIO_CTRL_REG = (VCONN_MON_P2_USBIO_CTRL_REG & \
                                    (uint32)(~(uint32)(VCONN_MON_P2_DRIVE_MODE_IND_MASK << \
                                    (VCONN_MON_P2_DRIVE_MODE_BITS * (shift))))) | \
                                    (uint32)((uint32)(mode) << \
                                    (VCONN_MON_P2_DRIVE_MODE_BITS * (shift))); \
        } while (0)
    #endif
#endif
  

#if defined(VCONN_MON_P2__PC) || (CY_PSOC4_4200L) 
    /*******************************************************************************
    * Function Name: VCONN_MON_P2_SetDriveMode
    ****************************************************************************//**
    *
    * \brief Sets the drive mode for each of the Pins component's pins.
    * 
    * <b>Note</b> This affects all pins in the Pins component instance. Use the
    * Per-Pin APIs if you wish to control individual pin's drive modes.
    *
    * <b>Note</b> USBIOs have limited drive functionality. Refer to the Drive Mode
    * parameter for more information.
    *
    * \param mode
    *  Mode for the selected signals. Valid options are documented in 
    *  \ref driveMode.
    *
    * \return
    *  None
    *
    * \sideeffect
    *  If you use read-modify-write operations that are not atomic, the ISR can
    *  cause corruption of this function. An ISR that interrupts this function 
    *  and performs writes to the Pins component Drive Mode registers can cause 
    *  corrupted port data. To avoid this issue, you should either use the Per-Pin
    *  APIs (primary method) or disable interrupts around this function.
    *
    * \funcusage
    *  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_SetDriveMode
    *******************************************************************************/
    void VCONN_MON_P2_SetDriveMode(uint8 mode)
    {
		VCONN_MON_P2_SetP4PinDriveMode(VCONN_MON_P2__0__SHIFT, mode);
    }
#endif


/*******************************************************************************
* Function Name: VCONN_MON_P2_Write
****************************************************************************//**
*
* \brief Writes the value to the physical port (data output register), masking
*  and shifting the bits appropriately. 
*
* The data output register controls the signal applied to the physical pin in 
* conjunction with the drive mode parameter. This function avoids changing 
* other bits in the port by using the appropriate method (read-modify-write or
* bit banding).
*
* <b>Note</b> This function should not be used on a hardware digital output pin 
* as it is driven by the hardware signal attached to it.
*
* \param value
*  Value to write to the component instance.
*
* \return 
*  None 
*
* \sideeffect
*  If you use read-modify-write operations that are not atomic; the Interrupt 
*  Service Routines (ISR) can cause corruption of this function. An ISR that 
*  interrupts this function and performs writes to the Pins component data 
*  register can cause corrupted port data. To avoid this issue, you should 
*  either use the Per-Pin APIs (primary method) or disable interrupts around 
*  this function.
*
* \funcusage
*  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_Write
*******************************************************************************/
void VCONN_MON_P2_Write(uint8 value)
{
    uint8 drVal = (uint8)(VCONN_MON_P2_DR & (uint8)(~VCONN_MON_P2_MASK));
    drVal = (drVal | ((uint8)(value << VCONN_MON_P2_SHIFT) & VCONN_MON_P2_MASK));
    VCONN_MON_P2_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: VCONN_MON_P2_Read
****************************************************************************//**
*
* \brief Reads the associated physical port (pin status register) and masks 
*  the required bits according to the width and bit position of the component
*  instance. 
*
* The pin's status register returns the current logic level present on the 
* physical pin.
*
* \return 
*  The current value for the pins in the component as a right justified number.
*
* \funcusage
*  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_Read  
*******************************************************************************/
uint8 VCONN_MON_P2_Read(void)
{
    return (uint8)((VCONN_MON_P2_PS & VCONN_MON_P2_MASK) >> VCONN_MON_P2_SHIFT);
}


/*******************************************************************************
* Function Name: VCONN_MON_P2_ReadDataReg
****************************************************************************//**
*
* \brief Reads the associated physical port's data output register and masks 
*  the correct bits according to the width and bit position of the component 
*  instance. 
*
* The data output register controls the signal applied to the physical pin in 
* conjunction with the drive mode parameter. This is not the same as the 
* preferred VCONN_MON_P2_Read() API because the 
* VCONN_MON_P2_ReadDataReg() reads the data register instead of the status 
* register. For output pins this is a useful function to determine the value 
* just written to the pin.
*
* \return 
*  The current value of the data register masked and shifted into a right 
*  justified number for the component instance.
*
* \funcusage
*  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_ReadDataReg 
*******************************************************************************/
uint8 VCONN_MON_P2_ReadDataReg(void)
{
    return (uint8)((VCONN_MON_P2_DR & VCONN_MON_P2_MASK) >> VCONN_MON_P2_SHIFT);
}


/*******************************************************************************
* Function Name: VCONN_MON_P2_SetInterruptMode
****************************************************************************//**
*
* \brief Configures the interrupt mode for each of the Pins component's
*  pins. Alternatively you may set the interrupt mode for all the pins
*  specified in the Pins component.
*
*  <b>Note</b> The interrupt is port-wide and therefore any enabled pin
*  interrupt may trigger it.
*
* \param position
*  The pin position as listed in the Pins component. You may OR these to be 
*  able to configure the interrupt mode of multiple pins within a Pins 
*  component. Or you may use VCONN_MON_P2_INTR_ALL to configure the
*  interrupt mode of all the pins in the Pins component.       
*  - VCONN_MON_P2_0_INTR       (First pin in the list)
*  - VCONN_MON_P2_1_INTR       (Second pin in the list)
*  - ...
*  - VCONN_MON_P2_INTR_ALL     (All pins in Pins component)
*
* \param mode
*  Interrupt mode for the selected pins. Valid options are documented in
*  \ref intrMode.
*
* \return 
*  None
*  
* \sideeffect
*  It is recommended that the interrupt be disabled before calling this 
*  function to avoid unintended interrupt requests. Note that the interrupt
*  type is port wide, and therefore will trigger for any enabled pin on the 
*  port.
*
* \funcusage
*  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_SetInterruptMode
*******************************************************************************/
void VCONN_MON_P2_SetInterruptMode(uint16 position, uint16 mode)
{
    uint32 intrCfg;
    
    intrCfg =  VCONN_MON_P2_INTCFG & (uint32)(~(uint32)position);
    VCONN_MON_P2_INTCFG = intrCfg | ((uint32)position & (uint32)mode);
}


/*******************************************************************************
* Function Name: VCONN_MON_P2_ClearInterrupt
****************************************************************************//**
*
* \brief Clears any active interrupts attached with the component and returns 
*  the value of the interrupt status register allowing determination of which
*  pins generated an interrupt event.
*
* \return 
*  The right-shifted current value of the interrupt status register. Each pin 
*  has one bit set if it generated an interrupt event. For example, bit 0 is 
*  for pin 0 and bit 1 is for pin 1 of the Pins component.
*  
* \sideeffect
*  Clears all bits of the physical port's interrupt status register, not just
*  those associated with the Pins component.
*
* \funcusage
*  \snippet VCONN_MON_P2_SUT.c usage_VCONN_MON_P2_ClearInterrupt
*******************************************************************************/
uint8 VCONN_MON_P2_ClearInterrupt(void)
{
	uint8 maskedStatus = (uint8)(VCONN_MON_P2_INTSTAT & VCONN_MON_P2_MASK);
	VCONN_MON_P2_INTSTAT = maskedStatus;
    return maskedStatus >> VCONN_MON_P2_SHIFT;
}


/* [] END OF FILE */
