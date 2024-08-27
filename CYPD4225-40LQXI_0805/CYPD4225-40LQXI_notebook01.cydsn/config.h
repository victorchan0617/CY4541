/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG firmware features.
 *
 * This file also provides mapping to the implementation for hardware dependent
 * functions like FET control, voltage selection etc.
 *
 * This current implementation matches the CY4541 EVK from Cypress. This can be
 * updated by users to match their hardware implementation.@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>
#include <stack_params.h>

/*******************************************************************************
 * PSOURCE controls for PD port 1.
 * The current implementation corresponds to the CY4541 kit hardware. Please
 * update this as required to match the target hardware.
 ******************************************************************************/

/* Function/Macro to turn power source for P1 ON. */
#define APP_VBUS_SRC_FET_ON_P1()                    VBUS_P_CTRL_P1_Write(1)

/* Function/Macro to turn power source for P1 OFF. */
#define APP_VBUS_SRC_FET_OFF_P1()                   VBUS_P_CTRL_P1_Write(0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P1()                   VBUS_DISCHARGE_P1_Write(1)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 OFF. */
#define APP_DISCHARGE_FET_OFF_P1()                  VBUS_DISCHARGE_P1_Write(0)

/* Function/Macro to set P1 source voltage to 5V. */
#define APP_VBUS_SET_5V_P1()                        \
{                                                   \
    VSEL1_P1_Write(0);                              \
    VSEL2_P1_Write(0);                              \
}

/* Function/Macro to set P1 source voltage to 9V. */
#define APP_VBUS_SET_9V_P1()                        \
{                                                   \
    VSEL1_P1_Write(1);                              \
    VSEL2_P1_Write(0);                              \
}

/* Function/Macro to set P1 source voltage to 12V. Not supported on CY4541. */
#define APP_VBUS_SET_12V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 13V. Not supported on CY4541. */
#define APP_VBUS_SET_13V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 15V. */
#define APP_VBUS_SET_15V_P1()                       \
{                                                   \
    VSEL1_P1_Write(0);                              \
    VSEL2_P1_Write(1);                              \
}

/* Function/Macro to set P1 source voltage to 19V. Not supported on CY4541. */
#define APP_VBUS_SET_19V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 20V. */
#define APP_VBUS_SET_20V_P1()                       \
{                                                   \
    VSEL1_P1_Write(1);                              \
    VSEL2_P1_Write(1);                              \
}

/*******************************************************************************
 * PSOURCE controls for PD port 2.
 * The current implementation corresponds to the CY4541 kit hardware. Please
 * update this as required to match the target hardware.
 ******************************************************************************/

/* Function/Macro to turn power source for P2 ON. */
#define APP_VBUS_SRC_FET_ON_P2()                    VBUS_P_CTRL_P2_Write(1)

/* Function/Macro to turn power source for P2 OFF. */
#define APP_VBUS_SRC_FET_OFF_P2()                   VBUS_P_CTRL_P2_Write(0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P2 ON. */
#define APP_DISCHARGE_FET_ON_P2()                   VBUS_DISCHARGE_P2_Write(1)

/* Function/Macro to turn VBUS_DISCHARGE FET for P2 OFF. */
#define APP_DISCHARGE_FET_OFF_P2()                  VBUS_DISCHARGE_P2_Write(0)

/* Function/Macro to set P2 source voltage to 5V. */
#define APP_VBUS_SET_5V_P2()                        \
{                                                   \
    VSEL1_P2_Write(0);                              \
    VSEL2_P2_Write(0);                              \
}

/* Function/Macro to set P2 source voltage to 9V. */
#define APP_VBUS_SET_9V_P2()                        \
{                                                   \
    VSEL1_P2_Write(1);                              \
    VSEL2_P2_Write(0);                              \
}

/* Function/Macro to set P2 source voltage to 12V. Not supported on CY4541. */
#define APP_VBUS_SET_12V_P2()                       ((void)0)

/* Function/Macro to set P2 source voltage to 13V. Not supported on CY4541. */
#define APP_VBUS_SET_13V_P2()                       ((void)0)

/* Function/Macro to set P2 source voltage to 15V. */
#define APP_VBUS_SET_15V_P2()                       \
{                                                   \
    VSEL1_P2_Write(0);                              \
    VSEL2_P2_Write(1);                              \
}

/* Function/Macro to set P2 source voltage to 19V. Not supported on CY4541. */
#define APP_VBUS_SET_19V_P2()                       ((void)0)

/* Function/Macro to set P2 source voltage to 20V. */
#define APP_VBUS_SET_20V_P2()                       \
{                                                   \
    VSEL1_P2_Write(1);                              \
    VSEL2_P2_Write(1);                              \
}

/*******************************************************************************
 * Power Sink (PSINK) controls for PD port 1.
 * The current implementation corresponds to the CY4541 kit hardware. Please
 * update this as required to match the target hardware.
 ******************************************************************************/

/* Function/Macro to turn consumer FET for P1 ON. */
#define APP_VBUS_SNK_FET_ON_P1()                    VBUS_C_CTRL_P1_Write(0)

/* Function/Macro to turn consumer FET for P1 OFF. */
#define APP_VBUS_SNK_FET_OFF_P1()                   VBUS_C_CTRL_P1_Write(1)

/*******************************************************************************
 * Power Sink (PSINK) controls for PD port 2.
 * The current implementation corresponds to the CY4541 kit hardware. Please
 * update this as required to match the target hardware.
 ******************************************************************************/

/* Function/Macro to turn consumer FET for P2 ON. */
#define APP_VBUS_SNK_FET_ON_P2()                    VBUS_C_CTRL_P2_Write(0)

/* Function/Macro to turn consumer FET for P2 OFF. */
#define APP_VBUS_SNK_FET_OFF_P2()                   VBUS_C_CTRL_P2_Write(1)

/*******************************************************************************
 * VBus Monitoring Controls for detach detection.
 ******************************************************************************/

/* Division factor applied between VBus and the voltage on VBUS_MON input. */
#define VBUS_MON_DIVIDER                            (11u)

/*******************************************************************************
 * VBus Over-Current Protection Configuration.
 *
 * The VBus OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/

/*
 * VBus OCP feature enable. This can be enabled on CY4541 and on other boards
 * that have the load switch.
 */
#define VBUS_OCP_ENABLE                             (1u)

/*******************************************************************************
 * VBus Over-Voltage Protection Configuration.
 *
 * The VBus OVP feature uses an internal ADC in the CCG to measure the voltage
 * on the VBUS_MON input and uses the ADC output to detect over-voltage
 * conditions.
 *
 * The default implementation of OVP uses firmware ISRs to turn off the FETs
 * when OVP is detected. If quicker response is desired, there is the option of
 * using a direct OVP_TRIP output derived from a hardware comparator associated
 * with the ADC.
 ******************************************************************************/

/* VBus OVP enable setting. */
#define VBUS_OVP_ENABLE                             (1u)

/* Allowed over-voltage margin as percentage of nominal voltage. */
#define VBUS_OVP_MARGIN                             (20)

/* Enable direct OVP_TRIP output from hardware comparator. This control does not exist on CY4541. */
#define VBUS_OVP_TRIP_ENABLE                        (0u)

/*******************************************************************************
 * HPD GPIO enable
 *
 * This feature is enabled as a workaround method to detect fast role swap in
 * **silicon revision since the PD block doesnot have fast role swap capability
 * If this feature is enabled then default HPD feature used in the PD block has to
 * be used to detect fast role signal since other gpio cannot be used to detect
 * the pulse of duration of 150us. So the HPD functionality will be shifted
 * based on the GPIO detection logic
 ******************************************************************************/
#define FRS_WORKAROUND_ENABLE                        (0u)

/*******************************************************************************
 * VConn Over-Current Protection Configuration.
 *
 * The VConn OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/
/* VConn OCP enable setting. */
#define VCONN_OCP_ENABLE                            (0u)

/*******************************************************************************
 * Firmware feature configuration.
 ******************************************************************************/

/*
 * Index of SCB used for HPI interface. This should be set based on
 * the pin selection in the project schematic.
 */
#define HPI_SCB_INDEX                               (0u)

/* Set to 1 if building a debug enabled binary with no boot-loader dependency. */
#define CCG_FIRMWARE_APP_ONLY                       (0u)

/* Enable CCG deep sleep to save power. */
#define SYS_DEEPSLEEP_ENABLE                        (1u)

/* Enable Alternate Mode support when CCG is DFP. */
#define DFP_ALT_MODE_SUPP                           (1u)

/* Enable DisplayPort Source support as DFP. */
#define DP_DFP_SUPP                                 (1u)

/* Enable saving only SVIDs which are supported by CCG. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                           (1u)

/*
 * Select CCG4 GPIO to be used as Activity Indication. This should be set to a
 * valid value if APP_FW_LED_ENABLE is non-zero.
 */
#define FW_LED_GPIO_PORT_PIN                        (GPIO_PORT_1_PIN_1)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                                (0xC0)
/*
 * The LED toggle period.
 */
#define LED_TIMER_PERIOD                            (1000)

/* Timer used to ensure I2C transfers to the MUX complete on time. */
#define MUX_I2C_TIMER                               (0xC1)
/* The MUX transfer timeout is set to 10 ms timeout period. */
#define MUX_I2C_TIMER_PERIOD                        (10u)

#endif /* _CONFIG_H_ */

/* End of file */

