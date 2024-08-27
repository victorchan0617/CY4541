/**
 * @file stack_params.h
 *
 * @brief @{Header file that defines parameters to configure the CCGx Firmware
 * Stack. The current definitions for these values are optimized for the CCG4
 * Notebook Port Controller implementation and should not be changed.
 *
 * Please contact Cypress for details of possible customizations in these
 * settings.@}
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
#ifndef _STACK_PARAMS_H_
#define _STACK_PARAMS_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>

/*******************************************************************************
 * CCG Device Selection.
 ******************************************************************************/

/*
 * Select target device family. This definition is used to implement the PD
 * block driver.
 */
#define CCG4

/* Select target silicon ID for CYPD4225-40LQXI. */
#define CCG_DEV_SILICON_ID                      (0x1800)
#define CCG_DEV_FAMILY_ID                       (0x11A8)

/* No. of USB-PD ports supported. CYPD4225-40LQXI supports two ports. */
#define NO_OF_TYPEC_PORTS                       (2u)

#define TYPEC_PORT_0_IDX                        (0u)
#define TYPEC_PORT_1_IDX                        (1u)

#if (NO_OF_TYPEC_PORTS >= 2)
    /* Set this flag to enable the second PD port. */
    #define CCG_PD_DUALPORT_ENABLE              (1u)
#else
    #define CCG_PD_DUALPORT_ENABLE              (0u)
#endif

/*******************************************************************************
 * High level firmware feature selection.
 ******************************************************************************/

/* Enable HPI support. */
#define CCG_HPI_ENABLE                          (1u)

/* Enable PD policy registers in HPI. */
#define CCG_HPI_PD_ENABLE                       (1u)

/*
 * Disable Pseudo-metadata handling in flashing sequence.
 * This definition should be left enabled for CCG4 solutions.
 */
#define CCG_PSEUDO_METADATA_DISABLE             (1u)

/*******************************************************************************
 * Timer Module Configuration
 ******************************************************************************/

/*
 * The timer module provides software timer support. It provides multiple
 * timers running off a single hardware timer and has a general accuracy of 5%.
 * This module can generate callbacks at a granularity of 1ms. It provides
 * various backend implementations selectable via the following compile time
 * options. The various options can be selected by selecting the required value
 * for implemenation macro TIMER_TYPE.
 */

/*
 * SYS_TICK based timer backend, interrupting the system every 1ms.
 * This implementation requires the SYS_TICK timer to be reserved for the use
 * of this module. This implementation shall not function in DEEP_SLEEP mode
 * and the user should ensure that the timer is shut-off before entering
 * DEEP_SLEEP. To shut off the timer, just ensure that all soft-timers are
 * stopped or have expired.
 */
#define TIMER_TYPE_SYSTICK                      (1)

/*
 * WDT based timer backend.
 * This requires user to reserve both WDT and SYS_TICK timers for the use of this.
 * The WDT timer runs off ILO which is highly inaccurate. The SYS_TICK timer is
 * used to calibrate the WDT to match IMO accuracy. The WDT based
 * implementation works across DEEP_SLEEP.
 */
#define TIMER_TYPE_WDT                          (2)

/*
 * Timer implementation selection.
 * TIMER_TYPE_WDT should be used if deep sleep entry for power saving is
 * being used.
 */
#define TIMER_TYPE                              (TIMER_TYPE_WDT)

/*
 * In addition to the hardware timer options, the module also provides a
 * TICKLESS timer implementation. The TICKLESS implementation is currently
 * available only for WDT based timer backend. The TICKLESS timer interrupts
 * the system only at the timer expiry (instead of every 1ms). Since this
 * involves a more complex algorithm, it requires more code space (about 200
 * bytes more). This implementation allows the same timer to be used in ACTIVE
 * as well as DEEP_SLEEP modes, due to the WDT backend. It also gives maximum
 * power savings as well as faster execution due to less number of interrupts.
 */
#define TIMER_TICKLESS_ENABLE                   (1)

/*
 * Timer module supports multiple software instances based on a single hardware
 * timer. The number of instances is defined based on the PD port count.
 */
#define TIMER_NUM_INSTANCES                     (NO_OF_TYPEC_PORTS)

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD             (250u)

/* Period (in ms) of VBus validity checks after enabling the power source. */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD     (1u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD         (90u)

/* Time (in ms) for which the VBus_Discharge path will be enabled when turning power source OFF. */
#define APP_PSOURCE_DIS_TIMER_PERIOD            (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD    (3u)

/*******************************************************************************
 * VBus monitor configuration.
 ******************************************************************************/

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                     (-20)

/* Allowed VBus valid margin (as percentage of expected voltage) before detach detection is triggered. */
#define VBUS_TURN_OFF_MARGIN                    (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                   (10)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN             (10)

/*******************************************************************************
 * VBus Monitor connection configuration for Port 1.
 ******************************************************************************/

/* CCG IO port to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PORT_NO_P1                 (VBUS_MON_P1__PORT)

/* CCG IO pin to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PIN_NO_P1                  (VBUS_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VBUS_MON_P1 pin. */
//#define APP_VBUS_MON_PORT_PIN_P1                ((VBUS_MON_P1__PORT << 4) | VBUS_MON_P1__SHIFT)

/*
   IO setting to connect VBUS_MON_P1 to an internal comparator. This should be selected from:
   a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
   b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
*/
#define APP_VBUS_MON_AMUX_INPUT_P1              (6)

/*******************************************************************************
 * VBus Monitor connection configuration for Port 2.
 ******************************************************************************/

/* CCG IO port to which the VBUS_MON_P2 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PORT_NO_P2                 (VBUS_MON_P2__PORT)

/* CCG IO pin to which the VBUS_MON_P2 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PIN_NO_P2                  (VBUS_MON_P2__SHIFT)

/* Combined Port+Pin representation for the VBUS_MON_P2 pin. */
#define APP_VBUS_MON_PORT_PIN_P2                ((VBUS_MON_P2__PORT << 4) | VBUS_MON_P2__SHIFT)

/*
 * IO setting to connect VBUS_MON_P2 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VBUS_MON_AMUX_INPUT_P2              (6)

/*******************************************************************************
 * VConn Monitor connection configuration for Port 1.
 * This section is optional as VConn monitoring is not enabled in the stack.
 ******************************************************************************/

/* CCG IO port to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PORT_NO_P1                (VCONN_MON_P1__PORT)

/* CCG IO pin to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PIN_NO_P1                 (VCONN_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VCONN_MON_P1 pin. */
#define APP_VCONN_MON_PORT_PIN_P1               ((VCONN_MON_P1__PORT << 4) | VCONN_MON_P1__SHIFT)

/*
   IO setting to connect VCONN_MON_P1 to an internal comparator. This should be selected from:
   a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
   b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
*/
#define APP_VCONN_MON_AMUX_INPUT_P1             (7)

/*******************************************************************************
 * VConn Monitor connection configuration for Port 2.
 * This section is optional as VConn monitoring is not enabled in the stack.
 ******************************************************************************/

/* CCG IO port to which the VCONN_MON_P2 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PORT_NO_P2                (VCONN_MON_P2__PORT)

/* CCG IO pin to which the VCONN_MON_P2 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PIN_NO_P2                 (VCONN_MON_P2__SHIFT)

/* Combined Port+Pin representation for the VCONN_MON_P2 pin. */
#define APP_VCONN_MON_PORT_PIN_P2               ((VCONN_MON_P2__PORT << 4) | VCONN_MON_P2__SHIFT)

/*
 * IO setting to connect VCONN_MON_P2 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VCONN_MON_AMUX_INPUT_P2  (7)

/*******************************************************************************
 * VBus OCP fault GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PORT_NO_P1           (OCP_FAULT_P1__PORT)

/* CCG pin to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PIN_NO_P1            (OCP_FAULT_P1__SHIFT)

/* Combined Port+Pin representation for the OCP_FAULT_P1 pin. */
#define APP_VBUS_OCP_PORT_PIN_P1                ((OCP_FAULT_P1__PORT << 4) | OCP_FAULT_P1__SHIFT)

/* CCG port to which the OCP_FAULT_P2 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PORT_NO_P2           (OCP_FAULT_P2__PORT)

/* CCG pin to which the OCP_FAULT_P2 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PIN_NO_P2            (OCP_FAULT_P2__SHIFT)

/* Combined Port+Pin representation for the OCP_FAULT_P2 pin. */
#define APP_VBUS_OCP_PORT_PIN_P2                ((OCP_FAULT_P2__PORT << 4) | OCP_FAULT_P2__SHIFT)

/*******************************************************************************
 * VBUS_OVP_TRIP GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PORT_NO_P1            (VBUS_OVP_TRIP_P1__PORT)

/* CCG pin to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PIN_NO_P1             (VBUS_OVP_TRIP_P1__SHIFT)

/* Combined Port+Pin representation of the VBUS_OVP_TRIP_P1 pin. */
#define APP_VBUS_OVP_TRIP_PORT_PIN_P1           ((VBUS_OVP_TRIP_P1__PORT << 4) | VBUS_OVP_TRIP_P1__SHIFT)

/* CCG IO mode corresponding to the VBUS_OVP_TRIP functionality. This should be set to 12. */
#define APP_VBUS_OVP_TRIP_HSIOM_P1              (12)

/* CCG port to which the VBUS_OVP_TRIP_P2 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PORT_NO_P2            (VBUS_OVP_TRIP_P2__PORT)

/* CCG pin to which the VBUS_OVP_TRIP_P2 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PIN_NO_P2             (VBUS_OVP_TRIP_P2__SHIFT)

/* Combined Port+Pin representation of the VBUS_OVP_TRIP_P2 pin. */
#define APP_VBUS_OVP_TRIP_PORT_PIN_P2           ((VBUS_OVP_TRIP_P2__PORT << 4) | VBUS_OVP_TRIP_P2__SHIFT)

/* CCG IO mode corresponding to the VBUS_OVP_TRIP functionality. This should be set to 12. */
#define APP_VBUS_OVP_TRIP_HSIOM_P2              (12)

/*******************************************************************************
 * ADC selection for various functions.
 ******************************************************************************/
#define APP_OVP_ADC_ID                          (PD_ADC_ID_0)
#define APP_VBUS_POLL_ADC_ID                    (PD_ADC_ID_0)
#define APP_OVP_ADC_INPUT                       (PD_ADC_INPUT_AMUX_A)
#define APP_VBUS_POLL_ADC_INPUT                 (PD_ADC_INPUT_AMUX_A)
#define APP_VBUS_DETACH_ADC_ID                  (PD_ADC_ID_1)
#define APP_VBUS_DETACH_ADC_INPUT               (PD_ADC_INPUT_AMUX_A)
#define APP_VCONN_OCP_ADC_INPUT                 (PD_ADC_INPUT_AMUX_B)
#define APP_VCONN_OCP_ADC_ID                    (PD_ADC_ID_0)
#define APP_VCONN_TRIGGER_LEVEL                 (0xFBu)

/*******************************************************************************
 * Types of Data path switches supported.
 ******************************************************************************/

/* CCG controlled switch for DisplayPort and USB lines. */
#define DP_MUX                                  (0u)

/* Data path switching handled by Alpine Ridge. */
#define AR_MUX                                  (1u)

/* This firmware supports only CCG controlled DP/USB switch operation. */
#define MUX_TYPE                                DP_MUX

/*******************************************************************************
 * Firmware workaround enable/disable
 ******************************************************************************/

/*
 * Enable CDT 225123 workaround.
 * This is a workaround to address a CCG4 silicon defect which causes spurious
 * HPD IRQ to be generated by CCG4. The workaround is completely restricted to
 * the PD block driver and there is no customer impact.
 */
#define CCGX_CDT225123_WORKAROUND


#endif /* _STACK_PARAMS_H_ */

/* End of file */

