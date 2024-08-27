/**
 * @file hal_ccgx.c
 *
 * @brief @{PD and Type-C HAL layer for CCG3/CCG4.@}
 */

/*
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

#include "config.h"
#include "hal_ccgx.h"
#include "ccgx_regs.h"
#include "pd.h"
#include "gpio.h"
#include "pdss_hal.h"

#define PDSS_PORT0_PCLK_RX_IDX			(8u)
#define PDSS_PORT0_PCLK_TX_IDX			(9u)
#define PDSS_PORT0_PCLK_SAR_IDX			(10u)

#ifdef CCG3   
    #define PDSS_PORT0_PCLK_SWAP_IDX        (11u)    
#else    
    #define PDSS_PORT0_PCLK_SWAP_IDX        (6u)   
#endif  /* CCG3 */  

#define PDSS_PORT1_PCLK_RX_IDX			(11u)
#define PDSS_PORT1_PCLK_TX_IDX			(12u)
#define PDSS_PORT1_PCLK_SAR_IDX			(13u)
#define PDSS_PORT1_PCLK_SWAP_IDX        (7u)

#if VBUS_OCP_ENABLE
/* OCP callbacks. */
vbus_ocp_cbk_t vbus_ocp_cbk[NO_OF_TYPEC_PORTS];

#ifdef CCG3

#define MA_PER_UNIT                     (10u)
#define VSENSE_MIN                      (13)
#define VSENSE_MAX                      (75)

const uint8_t csa_tab[] =
/* Vsense (mv) (for 10mOhm Rsense), av1 (:3)-bw (:2), vref_sel code (6) */
{                  /* Nominal gain, Vref */
    /* 13, */  0x1F, 0,  /* 100, 1.3   */
    /* 14, */  0x1F, 10, /* 100, 1.4   */
    /* 15, */  0x1F, 20, /* 100, 1.5   */
    /* 16, */  0x1F, 30, /* 100, 1.6   */
    /* 17, */  0x1F, 40, /* 100, 1.7   */
    /* 18, */  0x1F, 50, /* 100, 1.8   */
    /* 19, */  0x1B, 3,  /* 70,  1.33  */
    /* 20, */  0x1B, 10, /* 70,  1.4   */
    /* 21, */  0x1B, 17, /* 70,  1.47  */
    /* 22, */  0x1B, 24, /* 70,  1.54  */
    /* 23, */  0x1B, 31, /* 70,  1.61  */
    /* 24, */  0x1B, 38, /* 70,  1.68  */
    /* 25, */  0x1B, 45, /* 70,  1.75  */
    /* 26, */  0x1B, 52, /* 70,  1.82  */
    /* 27, */  0x1B, 59, /* 70,  1.89  */
    /* 28, */  0x16, 10, /* 50,  1.4   */
    /* 29, */  0x16, 15, /* 50,  1.45  */
    /* 30, */  0x16, 20, /* 50,  1.5   */
    /* 31, */  0x16, 25, /* 50,  1.55  */
    /* 32, */  0x16, 30, /* 50,  1.6   */
    /* 33, */  0x16, 35, /* 50,  1.65  */
    /* 34, */  0x16, 40, /* 50,  1.7   */
    /* 35, */  0x16, 45, /* 50,  1.75  */
    /* 36, */  0x16, 50, /* 50,  1.8   */
    /* 37, */  0x16, 55, /* 50,  1.85  */
    /* 38, */  0x16, 60, /* 50,  1.9   */
    /* 39, */  0x12, 26, /* 40,  1.56  */
    /* 40, */  0x12, 30, /* 40,  1.6   */
    /* 41, */  0x12, 34, /* 40,  1.64  */
    /* 42, */  0x12, 38, /* 40,  1.68  */
    /* 43, */  0x12, 42, /* 40,  1.72  */
    /* 44, */  0x12, 46, /* 40,  1.76  */
    /* 45, */  0x12, 50, /* 40,  1.8   */
    /* 46, */  0x12, 54, /* 40,  1.84  */
    /* 47, */  0x12, 58, /* 40,  1.88  */
    /* 48, */  0x12, 62, /* 40,  1.92  */
    /* 49, */  0x0D, 17, /* 30,  1.47  */
    /* 50, */  0x0D, 20, /* 30,  1.5   */
    /* 51, */  0x0D, 23, /* 30,  1.53  */
    /* 52, */  0x0D, 26, /* 30,  1.56  */
    /* 53, */  0x0D, 29, /* 30,  1.59  */
    /* 54, */  0x0D, 32, /* 30,  1.62  */
    /* 55, */  0x0D, 35, /* 30,  1.65  */
    /* 56, */  0x0D, 38, /* 30,  1.68  */
    /* 57, */  0x0D, 41, /* 30,  1.71  */
    /* 58, */  0x0D, 44, /* 30,  1.74  */
    /* 59, */  0x0D, 47, /* 30,  1.77  */
    /* 60, */  0x0D, 50, /* 30,  1.8   */
    /* 61, */  0x0D, 53, /* 30,  1.83  */
    /* 62, */  0x0D, 56, /* 30,  1.86  */
    /* 63, */  0x0D, 59, /* 30,  1.89  */
    /* 64, */  0x0D, 62, /* 30,  1.92  */
    /* 65, */  0x09, 0,  /* 20,  1.3   */
    /* 66, */  0x09, 2,  /* 20,  1.32  */
    /* 67, */  0x09, 4,  /* 20,  1.34  */
    /* 68, */  0x09, 6,  /* 20,  1.36  */
    /* 69, */  0x09, 8,  /* 20,  1.38  */
    /* 70, */  0x09, 10, /* 20,  1.4   */
    /* 71, */  0x09, 12, /* 20,  1.42  */
    /* 72, */  0x09, 14, /* 20,  1.44  */
    /* 73, */  0x09, 16, /* 20,  1.46  */
    /* 74, */  0x09, 18, /* 20,  1.48  */
    /* 75, */  0x09, 20  /* 20,  1.5   */

/*
 * For an rsense of 10mOhm this is enough.
 * The rest of the table follows.
 */
#if 0
    76,  0x09, 22, /* 20,  1.52  */
    77,  0x09, 24, /* 20,  1.54  */
    78,  0x09, 26, /* 20,  1.56  */
    79,  0x09, 28, /* 20,  1.58  */
    80,  0x09, 30, /* 20,  1.6   */
    81,  0x09, 32, /* 20,  1.62  */
    82,  0x09, 34, /* 20,  1.64  */
    83,  0x09, 36, /* 20,  1.66  */
    84,  0x09, 38, /* 20,  1.68  */
    85,  0x09, 40, /* 20,  1.7   */
    86,  0x09, 42, /* 20,  1.72  */
    87,  0x09, 44, /* 20,  1.74  */
    88,  0x09, 46, /* 20,  1.76  */
    89,  0x09, 48, /* 20,  1.78  */
    90,  0x09, 50, /* 20,  1.8   */
    91,  0x09, 52, /* 20,  1.82  */
    92,  0x09, 54, /* 20,  1.84  */
    93,  0x09, 56, /* 20,  1.86  */
    94,  0x09, 58, /* 20,  1.88  */
    95,  0x09, 60, /* 20,  1.9   */
    96,  0x09, 62, /* 20,  1.92  */
    97,  0x04, 15, /* 15,  1.455 */
    98,  0x04, 17, /* 15,  1.47  */
    99,  0x04, 18, /* 15,  1.485 */
    100, 0x04, 20, /* 15,  1.5   */
    101, 0x04, 21, /* 15,  1.515 */
    102, 0x04, 23, /* 15,  1.53  */
    103, 0x04, 24, /* 15,  1.545 */
    104, 0x04, 26, /* 15,  1.56  */
    105, 0x04, 27, /* 15,  1.575 */
    106, 0x04, 29, /* 15,  1.59  */
    107, 0x04, 30, /* 15,  1.605 */
    108, 0x04, 32, /* 15,  1.62  */
    109, 0x04, 33, /* 15,  1.635 */
    110, 0x04, 35, /* 15,  1.65  */
    111, 0x04, 36, /* 15,  1.665 */
    112, 0x04, 38, /* 15,  1.68  */
    113, 0x04, 39, /* 15,  1.695 */
    114, 0x04, 41, /* 15,  1.71  */
    115, 0x04, 42, /* 15,  1.725 */
    116, 0x04, 44, /* 15,  1.74  */
    117, 0x04, 45, /* 15,  1.755 */
    118, 0x04, 47, /* 15,  1.77  */
    119, 0x04, 48, /* 15,  1.785 */
    120, 0x04, 50, /* 15,  1.8   */
    121, 0x04, 51, /* 15,  1.815 */
    122, 0x04, 53, /* 15,  1.83  */
    123, 0x04, 54, /* 15,  1.845 */
    124, 0x04, 56, /* 15,  1.86  */
    125, 0x04, 57, /* 15,  1.875 */
    126, 0x04, 59, /* 15,  1.89  */
    127, 0x04, 60, /* 15,  1.905 */
    128, 0x04, 62, /* 15,  1.92  */
    129, 0x04, 63, /* 15,  1.935 */
    130, 0x00, 0,  /* 10,  1.3   */
    131, 0x00, 1,  /* 10,  1.31  */
    132, 0x00, 2,  /* 10,  1.32  */
    133, 0x00, 3,  /* 10,  1.33  */
    134, 0x00, 4,  /* 10,  1.34  */
    135, 0x00, 5,  /* 10,  1.35  */
    136, 0x00, 6,  /* 10,  1.36  */
    137, 0x00, 7,  /* 10,  1.37  */
    138, 0x00, 8,  /* 10,  1.38  */
    139, 0x00, 9,  /* 10,  1.39  */
    140, 0x00, 10, /* 10,  1.4   */
    141, 0x00, 11, /* 10,  1.41  */
    142, 0x00, 12, /* 10,  1.42  */
    143, 0x00, 13, /* 10,  1.43  */
    144, 0x00, 14, /* 10,  1.44  */
    145, 0x00, 15, /* 10,  1.45  */
    146, 0x00, 16, /* 10,  1.46  */
    147, 0x00, 17, /* 10,  1.47  */
    148, 0x00, 18, /* 10,  1.48  */
    149, 0x00, 19, /* 10,  1.49  */
    150, 0x00, 20, /* 10,  1.5   */
    151, 0x00, 21, /* 10,  1.51  */
    152, 0x00, 22, /* 10,  1.52  */
    153, 0x00, 23, /* 10,  1.53  */
    154, 0x00, 24, /* 10,  1.54  */
    155, 0x00, 25, /* 10,  1.55  */
    156, 0x00, 26, /* 10,  1.56  */
    157, 0x00, 27, /* 10,  1.57  */
    158, 0x00, 28, /* 10,  1.58  */
    159, 0x00, 29, /* 10,  1.59  */
    160, 0x00, 30, /* 10,  1.6   */
    161, 0x00, 31, /* 10,  1.61  */
    162, 0x00, 32, /* 10,  1.62  */
    163, 0x00, 33, /* 10,  1.63  */
    164, 0x00, 34, /* 10,  1.64  */
    165, 0x00, 35, /* 10,  1.65  */
    166, 0x00, 36, /* 10,  1.66  */
    167, 0x00, 37, /* 10,  1.67  */
    168, 0x00, 38, /* 10,  1.68  */
    169, 0x00, 39, /* 10,  1.69  */
    170, 0x00, 40, /* 10,  1.7   */
    171, 0x00, 41, /* 10,  1.71  */
    172, 0x00, 42, /* 10,  1.72  */
    173, 0x00, 43, /* 10,  1.73  */
    174, 0x00, 44, /* 10,  1.74  */
    175, 0x00, 45, /* 10,  1.75  */
    176, 0x00, 46, /* 10,  1.76  */
    177, 0x00, 47, /* 10,  1.77  */
    178, 0x00, 48, /* 10,  1.78  */
    179, 0x00, 49, /* 10,  1.79  */
    180, 0x00, 50, /* 10,  1.8   */
    181, 0x00, 51, /* 10,  1.81  */
    182, 0x00, 52, /* 10,  1.82  */
    183, 0x00, 53, /* 10,  1.83  */
    184, 0x00, 54, /* 10,  1.84  */
    185, 0x00, 55, /* 10,  1.85  */
    186, 0x00, 56, /* 10,  1.86  */
    187, 0x00, 57, /* 10,  1.87  */
    188, 0x00, 58, /* 10,  1.88  */
    190, 0x00, 60, /* 10,  1.9   */
    191, 0x00, 61, /* 10,  1.91  */
    192, 0x00, 62, /* 10,  1.92  */
    193, 0x00, 63  /* 10,  1.93  */
#endif
};

#endif /* CCG3 */

#ifdef CCG4

static const gpio_port_pin_t hal_ocp_fault_pin[] = {
    APP_VBUS_OCP_PORT_PIN_P1
#if CCG_PD_DUALPORT_ENABLE
    ,
    APP_VBUS_OCP_PORT_PIN_P2
#endif
};

CY_ISR(vbus_ocp_port0_handler)
{
    vbus_ocp_handler(TYPEC_PORT_0_IDX);
}

#if CCG_PD_DUALPORT_ENABLE

CY_ISR(vbus_ocp_port1_handler)
{
    vbus_ocp_handler(TYPEC_PORT_1_IDX);
}

#endif /* CCG_PD_DUALPORT_ENABLE */

#endif /* CCG4 */
#endif /* VBUS_OCP_ENABLE */

void system_init(void)
{
    /* Configure clocks for the PDSS IP block. */
#if (NO_OF_TYPEC_PORTS == 1)
    PERI->pclk_ctl[PDSS_PORT0_PCLK_RX_IDX] = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_TX_IDX] = PDSS_PORT0_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;
    
#if ((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)))
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SWAP_IDX] = PDSS_PORT0_SWAP_CLK_DIV_ID;
#endif /*((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)) */

#elif (NO_OF_TYPEC_PORTS == 2)
    PERI->pclk_ctl[PDSS_PORT0_PCLK_RX_IDX] = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_TX_IDX] = PDSS_PORT0_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;

    PERI->pclk_ctl[PDSS_PORT1_PCLK_RX_IDX] = PDSS_PORT1_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_TX_IDX] = PDSS_PORT1_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SAR_IDX] = PDSS_PORT1_SAR_CLK_DIV_ID;
    
#if ((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)))
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SWAP_IDX] = PDSS_PORT0_SWAP_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SWAP_IDX] = PDSS_PORT1_SWAP_CLK_DIV_ID;
#endif /*((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)) */    
    
#endif /* NO_OF_TYPEC_PORTS */

#if (NO_OF_TYPEC_PORTS == 1)
#if (VBUS_MON_INTERNAL == 0)
    /* Connect port 0 VBUS_MON to corresponding AMUX. */
    hsiom_set_config(APP_VBUS_MON_PORT_PIN_P1, APP_VBUS_MON_AMUX_INPUT_P1);
#endif /* VBUS_MON_INTERNAL */

#ifdef CCG4
#if VBUS_OCP_ENABLE
    uint8_t intr_no;

    /* Port 0 */
    intr_no = GPIO_PORT0_INTR_NO + APP_VBUS_OCP_FAULT_PORT_NO_P1;
    /* Register sync interrupt handler. */
    CyIntDisable(intr_no);
    (void)CyIntSetVector(intr_no, &vbus_ocp_port0_handler);
    CyIntSetPriority(intr_no, 0);
    CyIntEnable(intr_no);
#endif /* VBUS_OCP_ENABLE */

    /* Route OVP comparator output to OVP trip GPIO*/
    system_connect_ovp_trip(TYPEC_PORT_0_IDX);
#endif /* CCG4 */

#elif (NO_OF_TYPEC_PORTS == 2)
#if (VBUS_MON_INTERNAL == 0)
    /* Connect port 0 VBUS_MON to corresponding AMUX. */
    //hsiom_set_config(APP_VBUS_MON_PORT_PIN_P1, APP_VBUS_MON_AMUX_INPUT_P1);
    /* Connect port 1 VBUS_MON to corresponding AMUX. */
    hsiom_set_config(APP_VBUS_MON_PORT_PIN_P2, APP_VBUS_MON_AMUX_INPUT_P2);
#endif /* VBUS_MON_INTERNAL */

#ifdef CCG4
#if VBUS_OCP_ENABLE
    uint8_t intr_no;

    /* Port 0 */
    intr_no = GPIO_PORT0_INTR_NO + APP_VBUS_OCP_FAULT_PORT_NO_P1;
    /* Register sync interrupt handler. */
    CyIntDisable(intr_no);
    (void)CyIntSetVector(intr_no, &vbus_ocp_port0_handler);
    CyIntSetPriority(intr_no, 0);
    CyIntEnable(intr_no);

    /* Port 1 */
    intr_no = GPIO_PORT0_INTR_NO + APP_VBUS_OCP_FAULT_PORT_NO_P2;
    /* Register sync interrupt handler. */
    CyIntDisable(intr_no);
    (void)CyIntSetVector(intr_no, &vbus_ocp_port1_handler);
    CyIntSetPriority(intr_no, 0);
    CyIntEnable(intr_no);
#endif /* VBUS_OCP_ENABLE */

    /* Route OVP comparator output to OVP trip GPIO. */
    system_connect_ovp_trip(TYPEC_PORT_0_IDX);
    system_connect_ovp_trip(TYPEC_PORT_1_IDX);
#endif /* CCG4 */
#endif /* NO_OF_TYPEC_PORTS */
}

#if VBUS_OCP_ENABLE
uint8_t system_vbus_ocp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk)
{
    if (cbk == NULL)
    {
        return false;
    }
    vbus_ocp_cbk[port] = cbk;

    /* Enable VBUS OCP protection. */
#ifdef CCG4
    gpio_int_set_config( hal_ocp_fault_pin[port], GPIO_INTR_FALLING);
#endif /* CCG4 */

#ifdef CCG3
    unsigned int i = 0;
    uint32_t vsense;

    /* Apply the OCP threshold which is mentioned in percentage of rated current. */
    vsense = (cur + (cur * get_pd_port_config(port)->ocp_threshold) / 100);

    /* Convert to mV, accounting for the unit of current which is 10mA. */ 
    vsense = vsense * VBUS_OCP_RSENSE / 100;

    vsense = GET_MIN (vsense, VSENSE_MAX);
    if (vsense < VSENSE_MIN)
        i = 0;
    else
        i = ((vsense - VSENSE_MIN) << 1);

    pd_internal_vbus_ocp_en(csa_tab[i], csa_tab[i + 1], CCG3_SRC_FET, VBUS_OCP_MODE,
            get_pd_port_config(port)->ocp_debounce);
#endif /* CCG3 */

    return true;
}

uint8_t system_vbus_ocp_dis(uint8_t port)
{
#ifdef CCG3
    pd_internal_vbus_ocp_dis(CCG3_SRC_FET);
#endif /* CCG3 */

#ifdef CCG4
    gpio_int_set_config(hal_ocp_fault_pin[port], GPIO_INTR_DISABLE);
#endif /* CCG4 */

    vbus_ocp_cbk[port] = NULL;
    return true;
}

#endif /* VBUS_OCP_ENABLE */ 

void vbus_ocp_handler(uint8_t port)
{
#if VBUS_OCP_ENABLE
#ifdef CCG4
    gpio_port_pin_t pin = hal_ocp_fault_pin[port];

    /* Check if the fault pin interrupt was triggered. */
    if (gpio_get_intr(pin) == true)
    {
        if (vbus_ocp_cbk[port] != NULL)
        {
            vbus_ocp_cbk[port](port);
        }
        /* Clear the interrupt. */
        gpio_clear_intr(pin);
    }
#endif /* CCG4 */

#ifdef CCG3
    if (vbus_ocp_cbk[port] != NULL)
    {
        vbus_ocp_cbk[port](port);
    }
#endif /* CCG3 */
#endif /* VBUS_OCP_ENABLE */
}

void system_disconnect_ovp_trip(uint8_t port)
{
#ifdef CCG4    
#if ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) 
    uint8_t intr_state = CyEnterCriticalSection();
    /*
     * The VBUS OVP trip pin for port1 on the CCG4 EVK can only connect to
     * comparator 0. Configure the HSIOM appropriately. Also certain revision
     * of CCG4 EVK has OVP trip circuit polarity reversed and ovp trip won't work
     * on these boards.
     */
    if(port == TYPEC_PORT_0_IDX)
    {    
        /* Drive GPIO, Actual comparator HW output is opposite of  pd_adc_get_comparator_status() return value*/
        gpio_set_value (APP_VBUS_OVP_TRIP_PORT_PIN_P1, (pd_adc_get_comparator_status (port, APP_OVP_ADC_ID) ^ 0x1));

        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P1, HSIOM_MODE_GPIO);
                
    }
    
    if(port == TYPEC_PORT_1_IDX)
    {
        /* Drive GPIO, Actual comparator HW output is opposite of  pd_adc_get_comparator_status() return value*/
        gpio_set_value (APP_VBUS_OVP_TRIP_PORT_PIN_P2, (pd_adc_get_comparator_status (port, APP_OVP_ADC_ID) ^ 0x1));

        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P2, HSIOM_MODE_GPIO);               
    }
    CyExitCriticalSection(intr_state);
#endif /* ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) */
#endif /* CCG4*/    

}

void system_connect_ovp_trip(uint8_t port)
{
#ifdef CCG4    
#if ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) 
    uint8_t intr_state = CyEnterCriticalSection();
    /*
     * The VBUS OVP trip pin for port1 on the CCG4 EVK can only connect to
     * comparator 0. Configure the HSIOM appropriately. Also certain revision
     * of CCG4 EVK has OVP trip circuit polarity reversed and ovp trip won't work
     * on these boards.
     */
    if(port == TYPEC_PORT_0_IDX)
    {
        /* Set Drive Mode of GPIO. */
        gpio_set_drv_mode (APP_VBUS_OVP_TRIP_PORT_PIN_P1, GPIO_DM_STRONG);
        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P1, APP_VBUS_OVP_TRIP_HSIOM_P1);
                
    }
    
    if(port == TYPEC_PORT_1_IDX)
    {        
        /* Set Drive Mode of GPIO. */
        gpio_set_drv_mode (APP_VBUS_OVP_TRIP_PORT_PIN_P2, GPIO_DM_STRONG);
        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P2, APP_VBUS_OVP_TRIP_HSIOM_P2);        
    }
    CyExitCriticalSection(intr_state);
#endif /* ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) */
#endif /* CCG4*/

}


/* End of file */

