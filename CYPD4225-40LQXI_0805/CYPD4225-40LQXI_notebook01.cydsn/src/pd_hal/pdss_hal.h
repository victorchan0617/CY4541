/**
 * @file pdss_hal.h
 *
 * @brief @{CCG PD PHY driver module header file.@}
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
#ifndef _PDSS_HAL_H_
#define _PDSS_HAL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <ccgx_regs.h>
#include <pd.h>
#include <status.h>

/*******************************************************************************
 * CCG Device Specific Constants.
 * These values should not be modified.
 ******************************************************************************/

/* The TRIM6 and TRIM3 registers needs to be overwritten by firmware. */
#define SILICON_TRIM6_V1P575_TRIM_VALUE         (3u)
#define SILICON_TRIM3_V0P55_TRIM_VALUE          (2u)

/* TRIM0_TX_TRIM register needs to be overwritten by firmware. */
#define TRIM0_TX_TRIM_VALUE_3A                  (2u)
#define TRIM0_TX_TRIM_VALUE_900MA               (0u)

/*
 * Auto GoodCRC message mask configuration must be enabled for all messages
 * except goodcrc itself as per Rev2 spec. If Rev3 spec, then all 32 bits needs
 * to be configured.
 */
#define AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFDu)
#define AUTO_DATA_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFFu)
#define AUTO_EXTD_MESSAGE_GOODCRC_MASK_CFG      (0xFFFFFFFFu)

/* Mask representing header bits that should be matched for expected GoodCRC detection. */
#define EXPECTED_GOOD_CRC_HDR_MASK              (0x7E0Fu)

/*
 * Mask representing header bits that should be matched for expected GoodCRC detection
 * for Rev3 operation. This mask is incremental to base mask
 */
#define EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3    (0x8010u)

/* Mask to clear expected GoodCRC header field. */
#define EXPECTED_GOOD_CRC_CLEAR_MASK            (0xF1FFu)

/* RX CC register config for 12Mhz Rx clock. */
#define RX_CNT_MAX_VAL                          (0xFu)
#define RX_UI_BOUNDARY_DELTA_VAL                (0x2u)
#define RX_CNT_MAX_CFG                          (RX_CNT_MAX_VAL << PDSS_RX_CC_RX_CNT_MAX_POS)
#define RX_UI_BOUNDARY_DELTA_CFG                (RX_UI_BOUNDARY_DELTA_VAL << PDSS_RX_CC_RX_UI_BOUNDARY_DELTA_POS)

#define RX_CC_CFG                               (RX_CNT_MAX_CFG             | \
                                                 RX_UI_BOUNDARY_DELTA_CFG   | \
                                                 PDSS_RX_CC_DELAY_AUTO)

#define RX_SOP_GOOD_CRC_EN_CTRL_CFG             (PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN   | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_PRIME_SOP_GOOD_CRC_EN     | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN)

#define RX_SOP_GOOD_CRC_EN_CTRL_REV3_CFG        (PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN   | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_PRIME_SOP_GOOD_CRC_EN     | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_PRIME_SOP_GOOD_CRC_EN     | \
                                                 PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN)

/* RX_ORDER_SET_CTRL register configuration. */
#define RX_ORDER_SET_CTRL_CFG                   (PDSS_RX_ORDER_SET_CTRL_SOP_CMP_OPT | \
                                                 PDSS_RX_ORDER_SET_CTRL_RST_CMP_OPT | \
                                                 PDSS_RX_ORDER_SET_CTRL_PREAMBLE_SOP_EN | \
                                                 PDSS_RX_ORDER_SET_CTRL_PREAMBLE_RST_EN)

/* CRC_COUNTER register configuration, 1ms. */
#define CRC_COUNTER_CFG                         (300u)

/* INTER_PACKET_COUNTER register configuration. */
#define BUS_IDLE_CNT_VAL                        (10u)
#define IDLE_COUNTER_VAL                        (40u)
#define IFG_COUNTER_VAL                         (10u)

#define BUS_IDLE_CNT_CFG                        (BUS_IDLE_CNT_VAL << PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_POS)
#define IDLE_COUNTER_CFG                        (IDLE_COUNTER_VAL << PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_POS)
#define IFG_COUNTER_CFG                         (IFG_COUNTER_VAL << PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_POS)

#define INTER_PACKET_COUNTER_CFG                (BUS_IDLE_CNT_CFG | IDLE_COUNTER_CFG | IFG_COUNTER_CFG)

/* Extended Header Info register configuration. */
#define EXTENDED_DATA_BIT_LOC_VAL                    (15u)
#define EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_VAL    (16u)
#define EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_VAL     (24u)
#define CHUNK_BIT_LOCATION_VAL                       (15u)
#define HEADER_INFO_CFG                         ((EXTENDED_DATA_BIT_LOC_VAL << 8) | \
                                                 (EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_VAL << 12) | \
                                                 (EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_VAL << 17) | \
                                                 (CHUNK_BIT_LOCATION_VAL << 22))

/* MAX TX FIFO size */
#define PDSS_MAX_TX_MEM_SIZE                    (16u)
#define PDSS_MAX_TX_MEM_HALF_SIZE               (8u)

/* MAX RX FIFO size */
#define PDSS_MAX_RX_MEM_SIZE                    (16u)
#define PDSS_MAX_RX_MEM_HALF_SIZE               (8u)

/* PD block interrupt vector numbers. */
#if defined (CCG4)
#define PD_PORT0_INTR0                          (14u)
#define PD_PORT1_INTR0                          (15u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (7u)

#elif defined (CCG3)
#define PD_PORT0_INTR0                          (13u)
#define PD_PORT1_INTR0                          (255u)

#define PD_PORT0_INTR1                          (6u)
#define PD_PORT1_INTR1                          (255u)

#else
#error "Unsupported CCG device family."
#endif

#define PD_INTR_MASK                            (0x3FF7FBFFu)

#if CCG_PD_REV3_ENABLE
#define RX_INTERRUPTS       (PDSS_INTR0_TX_GOODCRC_MSG_DONE |   /**< GoodCRC message was transmitted. */    \
                             PDSS_INTR0_RX_STATE_IDLE       |   /**< Rx not in progress. */                 \
                             PDSS_INTR0_RCV_RST             |                                               \
                             PDSS_INTR0_RX_SRAM_HALF_END    |                                               \
                             PDSS_INTR0_COLLISION_TYPE3)        /**< Reception detected at GoodCRC transmit. */
#else

#define RX_INTERRUPTS       (PDSS_INTR0_TX_GOODCRC_MSG_DONE |   /**< GoodCRC message was transmitted. */    \
                             PDSS_INTR0_RX_STATE_IDLE       |   /**< Rx not in progress. */                 \
                             PDSS_INTR0_RCV_RST             |                                               \
                             PDSS_INTR0_COLLISION_TYPE3)        /**< Reception detected at GoodCRC transmit. */
#endif /* CCG_PD_REV3_ENABLE */

#define RCV_INTR_MASK       (PDSS_INTR0_RX_STATE_IDLE                   |\
                             PDSS_INTR0_EOP_ERROR                       |\
                             PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE        |\
                             PDSS_INTR0_RCV_BAD_PACKET_COMPLETE         |\
                             PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE        |\
                             PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE   |\
                             PDSS_INTR0_RX_SOP                          |\
                             PDSS_INTR0_RX_SRAM_HALF_END                |\
                             PDSS_INTR0_RX_OVER_RUN)

#define TX_INTERRUPTS       (PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE | /**< Received expected GoodCRC. */               \
                             PDSS_INTR0_CRC_RX_TIMER_EXP         | /**< CRC receive timer has expired. */           \
                             PDSS_INTR0_COLLISION_TYPE1          | /**< Reception detected at message transmit. */  \
                             PDSS_INTR0_TX_PACKET_DONE           |                                                  \
                             PDSS_INTR0_COLLISION_TYPE2)           /**< Reception detected at message retransmit. */

#define RST_TX_INTERRUPTS   (PDSS_INTR0_TX_HARD_RST_DONE         | /**< Hard reset was transmitted. */              \
                             PDSS_INTR0_COLLISION_TYPE4)           /**< Reception detected at reset transmit. */

#define EN_DEFAULT_SOP_DET_VAL                  (1u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_PRIME_SOP_DET_VAL                    (2u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DBL_SOP_DET_VAL                      (4u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DEBUG_PRIME_DET_VAL                  (8u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_DEBUG_DBL_DET_VAL                    (16u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_RX_CABLE_RESET_DET_VAL               (32u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)
#define EN_RX_HARD_RESET_DET_VAL                (64u << PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS)

#define PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG          (7u)

/* CC threshold definitions. */
#define PD_CMP_VSEL_0_2_V                       (0u)
#define PD_CMP_VSEL_0_4_V                       (1u)
#define PD_CMP_VSEL_1_43_V                      (2u)
#define PD_CMP_VSEL_0_655_V                     (3u)
#define PD_CMP_VSEL_0_8_V                       (4u)
#define PD_CMP_VSEL_1_235_V                     (5u)
#define PD_CMP_VSEL_1_77_V                      (6u)
#define PD_CMP_VSEL_2_6_V                       (7u)

#define DN_COMP_IDX                             (0u)
#define UP_COMP_IDX                             (1u)

/* Rp threshold index limits in the thresholds LUT. */
#define LOWER_LIMIT_IDX                         (0u)
#define HIGHER_LIMIT_IDX                        (1u)

/* No of Rd rows and entries per row, in the thresholds LUT. */
#define RD_ROW_NO                               (3u)
#define RD_COL_WIDTH                            (4u)

/* No of entries per Rp row in the thresholds LUT. */
#define RP_COL_WIDTH                            (2u)

/* DEBUG_CC_1 Register Setting */
#define PDSS_TX_STOP_ON_SWAP_MASK               (1u << 26u)

/* Swap CNTRL default settings to 0.52V reference */
#define SWAP_CNTRL0_DFLT_VAL                    (2u << PDSS_SWAP_CNTRL_0_CMP_FS_VSEL_POS)

/* Swap CTRL default settings for FRS receive. This settings are based on 5Mhz clock
 * to the block.
 */
#define FRS_RX_SWAP_CTRL0_SWAPR_SOURCE_SEL_POS  (2u)

#define FRS_RX_SWAP_CTRL1_DFLT_VAL              ((175u << PDSS_SWAP_CTRL1_IRQ_MIN_POS)| \
                                                 (650u << PDSS_SWAP_CTRL1_IRQ_MAX_POS)|\
                                                 (PDSS_SWAP_CTRL1_RESET_SWAP_STATE))

#define FRS_RX_SWAP_CTRL2_DFLT_VAL              ((40u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_LOW_POS)| \
                                                 (1u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_HIGH_POS))

#define FRS_RX_SWAP_CTRL3_DFLT_VAL              ((300u << PDSS_SWAP_CTRL3_STABLE_LOW_POS)|\
                                                 (1200u << PDSS_SWAP_CTRL3_STABLE_HIGH_POS))

#define FRS_RX_SWAP_CTRL4_DFLT_VAL              (10u << PDSS_SWAP_CTRL4_IRQ_SPACING_POS)

#define FRS_RX_SWAP_CTRL5_DFLT_VAL              ((25u << PDSS_SWAP_CTRL5_LONG_HIGH_POS) |\
                                                 (1200u << PDSS_SWAP_CTRL5_LONG_LOW_POS))

/* Swap CTRL settings for FRS TX */
#define FRS_TX_IRQ_WIDTH                        (450u)
#define FRS_TX_SWAP_CTRL1_DFLT_VAL              (FRS_TX_IRQ_WIDTH | PDSS_SWAPT_CTRL1_DEFAULT_LEVEL | \
                                                 (2u << PDSS_SWAPT_CTRL1_COMMAND_POS) | \
                                                 PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE)


/****************************************************************************
 * Constants associated with PD ADC (Comparator) settings on CCG.
 ***************************************************************************/

/* VDDD voltage level in millivolt. */
#define PD_ADC_DEFAULT_VDDD_VOLT_MV             (5000u)

/* Maximum ADC count. */
#define PD_ADC_MAX_VALUE                        (0xFFu)

/* Number of ADC levels. */
#define PD_ADC_NUM_LEVELS                       (PD_ADC_MAX_VALUE + 1u)

/* Bandgap voltage in millivolt. */
#define PD_ADC_BAND_GAP_VOLT_MV                 (1200u)

/*
 * The minimum comparator output shall be limited to this value to guard
 * against ground bounce and other ADC noise.
 */
#define PD_ADC_LEVEL_MIN_THRESHOLD              (4u)

/*
 * The maximum comparator output shall be limited to this value to guard
 * against max level.
 */
#define PD_ADC_LEVEL_MAX_THRESHOLD              (254u)

/* Loop timeout count for ADC. */
#define PD_ADC_TIMEOUT_COUNT                    (200u)

/****************************************************************************
 * GPIO mapping on CCG3/4 for various functions.
 * These mappings are used for the default pins for these functions, which
 * are optimized for the targeted functions.
 ***************************************************************************/

/* HPD Signal Mapping for CCG3/CCG4. */

/*
 * @brief HPD Port and pin macros
 */
#if (defined (CCG3))
#define HPD_P0_PORT_PIN                         (GPIO_PORT_3_PIN_3)
#define HPD_P1_PORT_PIN                         (GPIO_PORT_3_PIN_3)
#define HPD_P0_PORT                             (3)
#define HPD_P1_PORT                             (3)
#define HPD_P0_PIN                              (3)
#define HPD_P1_PIN                              (3)
#define HPD_HSIOM_SETTING                       (12u)
#elif (defined (CCG4))
#define HPD_P0_PORT_PIN                         (GPIO_PORT_2_PIN_3)
#define HPD_P1_PORT_PIN                         (GPIO_PORT_3_PIN_4)
#define HPD_P0_PORT                             (2)
#define HPD_P1_PORT                             (3)
#define HPD_P0_PIN                              (3)
#define HPD_P1_PIN                              (4)
#define HPD_HSIOM_SETTING                       (15u)
#else
#error "Device not supported."
#endif

#ifdef CCG4
/*
 * The FET control lines are fixed and cannot be changed. The following
 * are the selected IO lines for the same. The definitions match silicon
 * and should not be modified.
 */
/* Port 1 PCTRL0 GPIO */
#define VBUS_P_CTRL0_GPIO_P1                     (GPIO_PORT_1_PIN_6)
/* Port 1 PCTRL1 GPIO */
#define VBUS_P_CTRL1_GPIO_P1                     (GPIO_PORT_2_PIN_1)
/* Port 1 CCTRL0 GPIO */
#define VBUS_C_CTRL0_GPIO_P1                     (GPIO_PORT_1_PIN_7)
/* Port 1 CCTRL1 GPIO */
#define VBUS_C_CTRL1_GPIO_P1                     (GPIO_PORT_3_PIN_0)

/* Port 2 PCTRL0 GPIO */
#define VBUS_P_CTRL0_GPIO_P2                     (GPIO_PORT_4_PIN_2)
/* Port 2 PCTRL1 GPIO */
#define VBUS_P_CTRL1_GPIO_P2                     (GPIO_PORT_3_PIN_7)
/* Port 2 CCTRL0 GPIO */
#define VBUS_C_CTRL0_GPIO_P2                     (GPIO_PORT_4_PIN_1)
/* Port 2 CCTRL1 GPIO */
#define VBUS_C_CTRL1_GPIO_P2                     (GPIO_PORT_3_PIN_6)

#endif /* CCG4 */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
 * @typedef PD_ADC_ID_T
 * @brief ADC block IDs on the CCG device.
 */
typedef enum PD_ADC_ID_T
{
    PD_ADC_ID_0,                        /**< ADC-0 associated with the PD block. */
    PD_ADC_ID_1,                        /**< ADC-1 associated with the PD block. */
    PD_ADC_ID_OVUV,                     /**< Dedicated ADC for Over-Voltage/Under-Voltage detection.
                                             Only available in the CCG3 device. */
    PD_ADC_NUM_ADC                      /**< Maximum number of ADCs in the PD block. */
} PD_ADC_ID_T;

/**
 * @typedef PD_ADC_INPUT_T
 * @brief Enumeration of PD ADC input sources. Refer to CCG device datasheet and TRM for
 * more details.
 */
typedef enum PD_ADC_INPUT_T
{
    PD_ADC_INPUT_AMUX_A,                /**< AMUX_A bus. */
    PD_ADC_INPUT_AMUX_B,                /**< AMUX_B bus. */
    PD_ADC_INPUT_BANDGAP,               /**< BANDGAP input. */
    PD_ADC_INPUT_BJT,                   /**< BJT. */
    PD_ADC_NUM_INPUT                    /**< Number of ADC inputs available. */
} PD_ADC_INPUT_T;

/**
 * @typedef PD_ADC_INT_T
 * @brief PD comparator interrupt configuration enumeration.
 * Note: These are the settings for INTR_1_CFG ADC output, not ADC_SAR_CTRL.
 */
typedef enum PD_ADC_INT_T
{
    PD_ADC_INT_DISABLED,                /**< Comparator interrupt disabled. */
    PD_ADC_INT_FALLING,                 /**< Comparator interrupt on falling edge. */
    PD_ADC_INT_RISING,                  /**< Comparator interrrupt on rising edge. */
    PD_ADC_INT_BOTH                     /**< Comparator interrupt on either edge. */
} PD_ADC_INT_T;

/**
 * @typedef pd_phy_evt_t
 * @brief PD PHY state enumeration.
 */
typedef enum
{
    PD_PHY_EVT_TX_MSG_COLLISION,        /**< Bus busy at message transmission. */
    PD_PHY_EVT_TX_MSG_PHY_IDLE,         /**< Bus idle, ready for message transmission. */
    PD_PHY_EVT_TX_MSG_FAILED,           /**< Message transmission was not successful. */
    PD_PHY_EVT_TX_MSG_SUCCESS,          /**< Message transmission was successful. */
    PD_PHY_EVT_TX_RST_COLLISION,        /**< Bus busy just before reset transmission. */
    PD_PHY_EVT_TX_RST_SUCCESS,          /**< Reset transmission was successful. */
    PD_PHY_EVT_RX_MSG,                  /**< Message received. */
    PD_PHY_EVT_RX_MSG_CMPLT,            /**< Message received and GoodCRC sent aka collision type 3. */
    PD_PHY_EVT_RX_RST,                  /**< Reset was received. */
    PD_PHY_EVT_FRS_SIG_RCVD,            /**< FRS signal was received. */
    PD_PHY_EVT_FRS_SIG_SENT             /**< FRS signal was transmitted. */
} pd_phy_evt_t;

/**
 * @typedef vbus_ocp_mode_t
 * @brief CCG OCP modes enumeration.
 */
typedef enum
{
    VBUS_OCP_MODE_EXT,                  /**< OCP through external hardware. */
    VBUS_OCP_MODE_INT,                  /**< Internal OCP without software debounce
                                             or hardware gate control. */
    VBUS_OCP_MODE_INT_AUTOCTRL,         /**< Internal OCP with hardware gate control. */
    VBUS_OCP_MODE_INT_SW_DB             /**< Internal OCP with software debounce. */
} vbus_ocp_mode_t;

/**
 * @typedef vbus_ovp_mode_t
 * @brief CCG OVP modes enumeration.
 */
typedef enum
{
    VBUS_OVP_MODE_ADC,                  /**< OVP using CCG internal ADC. */
    VBUS_OVP_MODE_UVOV,                 /**< OVP using the UVOV block (CCG3 only). */
    VBUS_OVP_MODE_UVOV_AUTOCTRL         /**< OVP using the OVOV block with automatic gate driver control. */
} vbus_ovp_mode_t;

/**
 * @typedef frs_tx_source_t
 * @brief Enum to hold various tx sources of FR-SWAP.
 */
typedef enum
{
    FRS_TX_SOURCE_CPU = 0,
    FRS_TX_SOURCE_GPIO,
    FRS_TX_SOURCE_ADC1,
    FRS_TX_SOURCE_ADC2
} frs_tx_source_t;

/**
 * @typedef frs_vsafe5v_source_t
 * @brief Enum to hold VSAFE5V comparison signal options to the fast role swap block.
 */
typedef enum
{
    FRS_RX_SOURCE_ADC1 = 0,
    FRS_RX_SOURCE_ADC2
}frs_vsafe5v_source_t;

/**
 * @typedef pd_fet_dr_t
 * @brief Enum to hold FET drive modes
 */
typedef enum
{
    PD_FET_DR_ACTIVE_HIGH = 0,
    PD_FET_DR_ACTIVE_LOW,
    PD_FET_DR_N_JN_FET,
    PD_FET_DR_P_JN_FET
} pd_fet_dr_t;

/**
 * @typedef sbu_switch_state_t
 * @brief Enum to hold SBU connection state for CCG3. CCG3 provides internal
 * switch to route SBU 1/2 to AUX 1/2 or isolate.
 */
typedef enum
{
    SBU_NOT_CONNECTED,
    SBU_CONNECT_AUX1,
    SBU_CONNECT_AUX2,
    SBU_MAX_STATE
} sbu_switch_state_t;


/**
 * @typedef aux_resistor_config_t
 * @brief Enum to hold resistor configuration for AUX1 and AUX2. Values assigned
 * are the bit position of corresponding configuration in sbu_ctrl register.
 */
typedef enum
{
    AUX_NO_RESISTOR = 8,                        /**< No resistor. */
    AUX_1_1MEG_PU_RESISTOR = 9,                 /**< AUX1 1M0hm Pullup resistor. */
    AUX_1_100K_PD_RESISTOR = 10,                /**< AUX1 100KOhm Pulldown resistor. */
    AUX_1_470K_PD_RESISTOR = 11,                /**< AUX1 470KOhm Pulldown resistor. */
    AUX_2_100K_PU_RESISTOR = 12,                /**< AUX2 100KOhm Pullup resistor. */
    AUX_2_4P7MEG_PD_RESISTOR = 13,              /**< AUX2 4.7M0hm Pulldown resistor. */
    AUX_2_1MEG_PD_RESISTOR = 14,                /**< AUX2 1M0hm Pulldown resistor. */
    AUX_MAX_RESISTOR_CONFIG,                    /**< Not supported. */
} aux_resistor_config_t;

/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

void change1_gl_pdss_status(uint8_t port);
/**
 * @typedef PD_ADC_CB_T
 * @brief PD ADC comparator interrupt callback type. This callback
 * is called when the desired ADC event/interrupt occurs.
 * Available events:
 * - true : Input voltage is higher than the reference.
 * - false : Input voltage is lower than the reference.
 *
 * @param port PD port on which the ADC event occured.
 * @param comp_out Specifies the type of event.
 */
typedef void (*PD_ADC_CB_T)(uint8_t port, bool comp_out);

/**
 * @typedef pd_phy_cbk_t
 * @brief PD PHY callback prototype. This function will be used to notify
 * the stack about PHY events.
 *
 * @param port PD port on which the PHY event occured.
 * @param event Type of PD PHY event.
 */
typedef void(*pd_phy_cbk_t)(uint8_t port, pd_phy_evt_t event);

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief This function initializes the PDSS IP with necessary clock and interrupt
 * handlers.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t.
 */
ccg_status_t pd_hal_init(uint8_t port);

/**
 * @brief This function configures the PD block for deepsleep entry.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if the port is not busy and has been configured to go to
 * deepsleep, false otherwise. Also returns true if the block was not enabled.
 */
bool pd_phy_deepsleep(uint8_t port);

/**
 * @brief This function configures the PD block on deepsleep exit.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if successful, false otherwise.
 */
bool pd_phy_wakeup(void);

/**
 * @brief This function initializes the PD phy registers.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param cbk The phy event handler callback.
 * @return ccg_status_t.
 */
ccg_status_t pd_phy_init(uint8_t port, pd_phy_cbk_t cbk);

/**
 * @brief This function configures the PD phy as per current port role / data
 * role / contract status of the specified port.
 * This API does not enable the receiver.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_phy_refresh_roles(uint8_t port);

/**
 * @brief This function enables transmission of unchunked extended messages.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_phy_en_unchunked_tx(uint8_t port);

/**
 * @brief This function disables transmission of unchunked extended messages.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_phy_dis_unchunked_tx(uint8_t port);

/**
 * @brief This function loads the PD message in FIFO and configures the
 * necessary registers.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param sop Sop type.
 * @param retries Number of retries.
 * @param dobj_count No of data objects(each 32 bit) in data
 * @param header PD Header in lower 16 bits and optional unchunked extended header in upper 16 bits.
 * @param unchunked Unchunked message if true.
 * @param buf Pointer to message. Message buffer is an array of uint32_t.
 * - buf[0]..buf[n] = Data, if data/extended message.
 * @return Returns true if successful, false otherwise.
 */
bool pd_phy_load_msg(uint8_t port, sop_t sop, uint8_t retries,
        uint8_t dobj_count, uint32_t header, bool unchunked, uint32_t* buf);

/**
 * @brief This function starts the transmission of a message already loaded in
 * FIFO.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if successful, false otherwise.
 */
bool pd_phy_send_msg(uint8_t port);

/**
 * @brief This function starts transmission of BIST Carrier Mode 2.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 */
ccg_status_t pd_phy_send_bist_cm2(uint8_t port);

/**
 * @brief This function stops transmission of BIST Carrier Mode 2.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 */
ccg_status_t pd_phy_abort_bist_cm2(uint8_t port);

/**
 * @brief This function stops transmission of a message.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 */
ccg_status_t pd_phy_abort_tx_msg(uint8_t port);

/**
 * @brief This function starts transmission of a cable reset or a hard reset
 * as requested.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param sop Sop type = Cable Reset or Hard Reset.
 * @return ccg_status_t
 */
ccg_status_t pd_phy_send_reset(uint8_t port, sop_t sop);

/**
 * @brief This function returns the received packet.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Pointer to the received PD packet.
 */
pd_packet_extd_t* pd_phy_get_rx_packet(uint8_t port);

/**
 * @brief This function checks if the PD phy is busy for the specified port.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return Returns true if the phy is busy, false otherwise.
 */
bool pd_phy_is_busy(uint8_t port);

/**
 * @brief This function initializes the Type C registers in the PD block.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t
 */
ccg_status_t pd_typec_init(uint8_t port);

/**
 * @brief This function starts the Type C line comparators. pdss_typec_init()
 * should have been called before calls to this function.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return ccg_status_t.
 */
ccg_status_t pd_typec_start(uint8_t port);

/**
 * @brief: This function stops the Type-C line comparators.
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return: ccg_status_t
 */
ccg_status_t pd_typec_stop(uint8_t port);

/**
 * @brief This function configures and enables Rp.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @param rp_val Rp value.
 * @return None.
 */
void pd_typec_en_rp(uint8_t port, uint8_t channel, rp_term_t rp_val);

/**
 * @brief This function disables Rp.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None.
 */

void pd_typec_dis_rp(uint8_t port, uint8_t channel);

/**
 * @brief This function enables deepsleep Rp
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_typec_en_dpslp_rp(uint8_t port);

/**
 * @brief This function disables deepsleep Rp
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_typec_dis_dpslp_rp(uint8_t port);

/**
 * @brief This function enables Rd.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None.
 */
void pd_typec_en_rd(uint8_t port, uint8_t channel);

/**
 * @brief This function disables Rd.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return None.
 */
void pd_typec_dis_rd(uint8_t port, uint8_t channel);

/*
 * @brief Enable Dead Battery Rd on the specified PD port.
 *
 * This function is used to remove the trimmed Rd and re-enable the
 * dead-battery Rd on the PD port prior to going through a device reset.
 * This method is only used in dead-battery use cases, and allows device
 * flashing while the CCG device is powered through the Type-C connection.
 *
 * @param port Port on which dead-battery Rd is to be enabled.
 * Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_typec_en_deadbat_rd(uint8_t port);

/**
 * @brief This function updates the tx trim settings when in the sink role. It
 * must be called when Rp is changed by the peer.
 * @param port Port index.
 * @return None.
 */
void pd_typec_snk_update_trim(uint8_t port);

/**
 * @brief This function returns current CC status.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return cc_state_t
 */
cc_state_t pd_typec_get_cc_status(uint8_t port);

/**
 * @brief This function sets the CC polarity for the receiver circuit.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void pd_typec_set_polarity(uint8_t port, bool polarity);

/**
 * @brief This function turns on Vconn for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return ccg_status_t
 */
ccg_status_t pd_vconn_enable(uint8_t port, uint8_t channel);

/**
 * @brief This function turns off Vconn for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return ccg_status_t
 */
ccg_status_t pd_vconn_disable(uint8_t port, uint8_t channel);

/**
 * @brief This function gets Vconn status for the specified channel.
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param channel Channel index, where CC1 = 0, CC2 = 1.
 * @return Returns true if Vconn is turned on, false otherwise.
 */
bool pd_is_vconn_present(uint8_t port, uint8_t channel);

/**
 * @brief This function converts the voltage in millivolt to ADC units.
 *
 * It takes a 16-bit voltage value in millivolts and returns the
 * corresponding 8-bit ADC reading. This function does not perform any ADC
 * operations.
 *
 * The minimum value is limited by the PD_ADC_LEVEL_MIN_THRESHOLD value.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param volt Voltage in mV.
 * @return Returns the 8-bit ADC reading.
 */
uint8_t pd_adc_volt_to_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt);

/**
 * @brief This function converts the ADC units to voltage in millivolts.
 *
 * It takes an 8-bit ADC reading and returns the corresponding 16-bit voltage
 * value in millivolts. This function does not perform any ADC operations.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param level The 8-bit ADC reading.
 * @return Returns voltage in mV.
 */
uint16_t pd_adc_level_to_volt(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level);

/**
 * @brief This function configures the ADC for comparator functionality with
 * the requested threshold with no interrupts.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @param level Comparator level.
 * @return ccg_status_t
 */
ccg_status_t  pd_adc_free_run_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level);
/**
 * @brief This function configures the ADC for comparator functionality with
 * the requested threshold.
 *
 * This function configures the ADC block as a comparator. The function takes
 * the input to be configured and the ADC comparator threshold. It also takes a
 * callback. If the callback is not NULL, then the threshold is configured and
 * interrupts are enabled. If the callback is NULL, then the ADC / comparator
 * is set to the low power state and interrupts are disabled.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @param level Comparator level.
 * @param int_cfg Interrupt configuration.
 * @param cb Callback on interrupt.
 * @return None.
 */
void pd_adc_comparator_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level, PD_ADC_INT_T int_cfg, PD_ADC_CB_T cb);

/**
 * @brief This function temporarily configures the comparator as requested and
 * takes a sample.
 *
 * This function restores the state of the comparator after operation. This is
 * useful when the comparator is already configured to function with a certain
 * input and level with interrupt and another reading needs to be done without
 * having to re-configure the block after the sampling.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @return Returns true if voltage > level, false otherwise.
 */
bool pd_adc_comparator_sample(uint8_t port, PD_ADC_ID_T adc_id,
        PD_ADC_INPUT_T input, uint8_t level);

/**
 * @brief This function gets the current comparator status.
 *
 * This function does not configure the ADC / comparator. It just returns the
 * current state of the comparator. If true is returned, then the input voltage
 * is greater than the reference and if false, the input voltage is lower than
 * the reference.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return Returns the comparator output.
 */
bool pd_adc_get_comparator_status(uint8_t port, PD_ADC_ID_T adc_id);

/**
 * @brief This function samples the ADC.
 *
 * This function enables the ADC block to function as an ADC and returns the
 * sample value in ADC units. This function disables any previously configured
 * comparator interrupts / settings before sampling and restores them after the
 * sampling is done. If any interrupt scenario happens across the sampling, the
 * information is lost.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param input ADC input source.
 * @return Returns the ADC sample.
 */
uint8_t pd_adc_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input);

/**
 * @brief This function calibrates the specified ADC for operation.
 *
 * This function calibrates the specified ADC by identifying the VDDD voltage
 * for reference. It should be noted that by calling the function, the
 * previously calculated threshold levels may have to be changed based on the
 * VDDD reading. The VDDD level is calculated based on the bandgap voltage
 * which is expected to be constant.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return None.
 */
void pd_adc_calibrate(uint8_t port, PD_ADC_ID_T adc_id);

/**
 * @brief This function initializes the PD ADC block.
 *
 * This function enables the PD block and the registers required for ADC
 * operation. It then calibrates the ADC to identify the VDDD voltage. This
 * function does not start any ADC operations.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @return ccg_status_t
 */
ccg_status_t pd_adc_init(uint8_t port, PD_ADC_ID_T adc_id);

/**
 * @brief This function gets the ADC level that corresponds to the actual
 * voltage on vbus. It also takes into account the VBus monitor divider.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param adc_id ADC ID.
 * @param volt Voltage in 50mV units.
 * @param per Percentage margin on the voltage.
 * @return Returns the ADC level.
 */
uint8_t pd_get_vbus_adc_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt,
        int8_t per);

#if CCG_PD_REV3_ENABLE
/**
 * @brief This function enables the fast role swap receive functionality. Callback
 * registered in pd_phy_init will be called when fasr role swap signal is received.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 */
bool pd_frs_rx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap receive functionality.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 */
bool pd_frs_rx_disable(uint8_t port);

/**
 * @brief This function enables the fast role swap transmit functionality. Callback
 * registered in pd_phy_init will be called when fast role swap signal is transmitted.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 */
bool pd_frs_tx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap transmit functionality.
 *
 * @param port: Port index. Caller should ensure to provide only valid values.
 * @return Returns true if success otherwise returns false
 */
bool pd_frs_tx_disable(uint8_t port);

#endif /* CCG_PD_REV3_ENABLE */

/**
 * @brief Configures the drive modes for the FETs.
 *
 * This function allows the application to select polarity of drive for CCG4
 * devices and N-FET drive or P-FET drive for CCG3 devices. The configuration
 * should match the hardware implementation on the board.
 *
 * CCG3 devices support N-FET by default for both PCTRL and CCTRL. Override this
 * only for the boards with P-FETs. Standard Cypress reference schematics
 * and kits use N-FETs and changing this shall result in board damage.
 *
 * CCG4 devices support active high polarity for PCTRL (source) and active
 * low polarity for CCTRL (sink) by default. This configuration matches the
 * standard Cypress reference schematics and kits and changing this shall result
 * in board damage.
 *
 * This function is expected to be called once, before the pdss_hal is initialized.
 * Calls during stack operation are not restricted but are unsupported and likely
 * to result in spurious behaviour and / or board damage.
 *
 * @param is_p_jn_fet Set to true if the system uses P-FETs, false otherwise.
 * @return None.
 *
 * @warning Misconfiguration of this parameter will result in board damage.
 */
void pd_hal_set_fet_drive(pd_fet_dr_t pctrl_drive, pd_fet_dr_t cctrl_drive);

/**
 * @brief Configures FET parameters for CCG3 devices.
 *
 * This function configures various FET control parameters for the device.
 * It should be called before the PD HAL is intialized. Also it should be called
 * only once during initialization. Since this is high voltage FET configuration,
 * any wrong configuration shall result in damage of the device and boards. The
 * function should be called only when the default behaviour needs to be
 * overridden.
 *
 * Caller should ensure that the FET access is not done before it is configured
 * and is never configured while the PD stack is in operation.
 *
 * CCG3 devices shall use dual FET configuration with spacing of 10LF cycles
 * by default. If this function is not invoked, then the default configuration
 * shall be used. CCG3 shall always function in dual FET mode as the FET controls
 * are dedicated IOs.
 *
 * @param dual_fet Set to true if the system uses dual FETs for each direction.
 *      Otherwise set to false.
 * @param spacing Spacing in LF-cycles between dual FETs for firmware based
 *      turn-on and turn-off. Auto shut-off and turn-on happens simultaneously.
 *      Valid only for dual_fet configuration. Set to zero for simultaneous
 *      control.
 *
 * @return None.
 *
 * @warning Misconfiguration of this parameter shall result in board damage.
 */
void pd_hal_dual_fet_config(bool dual_fet, uint8_t spacing);

#if ((defined(CCG3)) || (defined(CCG4PD3)))
/*
 * @brief Turn on producer FET for CCG3 and CCG4PD3 devices.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_on_seq This bit selects which FET turns on first.
 *  0: FET 0 turns on first.
 *  1: FET 1 turns on first.
 * @return Void.
 */
void pd_internal_pfet_on(uint8_t port, bool turn_on_seq);

/*
 * @brief Turn off producer FET for CCG3 and CCG4PD3 devices.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_off_seq This bit selects which FET turns off first.
 *  0: FET 0 turns off first.
 *  1: FET 1 turns off first.
 * @return Void.
 */
void pd_internal_pfet_off(uint8_t port, bool turn_off_seq);

/*
 * @brief Turn on consumer FET for CCG3 and CCG4PD3 devices.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_on_seq This bit selects which FET turns on first.
 *  0: FET 0 turns on first.
 *  1: FET 1 turns on first.
 * @return Void.
 */
void pd_internal_cfet_on(uint8_t port, bool turn_on_seq);

/*
 * @brief Turn off consumer FET for CCG3 and CCG4PD3 devices.
 *
 * @param port Port index. Caller should ensure to provide only valid values.
 * @param turn_off_seq This bit selects which FET turns off first.
 *  0: FET 0 turns off first.
 *  1: FET 1 turns off first.
 * @return Void.
 */
void pd_internal_cfet_off(uint8_t port, bool turn_off_seq);

#endif /* #if ((defined(CCG3)) || (defined(CCG4PD3))) */

#ifdef CCG3
/**
 * @brief Enable/disable the internal VBus monitor function in the CCG3 device.
 *
 * @param enable Whether to enable internal VBus Monitor for CCG3.
 * @return None
 */
void pd_hal_enable_internal_vbus_mon(bool enable);

/*
 * @brief Turn on VBus discharge for CCG3 parts.
 *
 * @param Void.
 * @return Void.
 */
void pd_internal_vbus_discharge_on(void);

/*
 * @brief Turn off VBus discharge for CCG3 parts.
 *
 * @param Void.
 * @return Void.
 */
void pd_internal_vbus_discharge_off(void);

#if VBUS_OCP_ENABLE
/*
 * @brief Enable OCP control for CCG3 parts.
 *
 * @param rsense Rsense in mOhms.
 * @param pctrl Flag indicating whether the P_CTRL FETs are in use (as opposed
 * to the C_CTRL FETs).
 * @param mode OCP Mode selection.
 * @param debounce_ms OCP software debounce timeout.
 * @return Void.
 * @warning Implementation only supports operation as a source.
 */
void pd_internal_vbus_ocp_en(uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        vbus_ocp_mode_t mode, uint8_t debounce_ms);

/*
 * @brief Disable OCP control for CCG3 parts.
 *
 * @return Void.
 * @warning Implementation only supports operation as a source.
 */
void pd_internal_vbus_ocp_dis(bool pctrl);

#endif /* VBUS_OCP_ENABLE */

#if VBUS_OVP_ENABLE
/*
 * @brief Enable OVP control for CCG3 parts.
 *
 * @param volt Voltage in 50mV units.
 * @param per Percentage margin on the voltage.
 * @param cb Callback on interrupt.
 * @param pctrl Flag indicating whether the P_CTRL FETs are in use (as opposed
 * to the C_CTRL FETs).
 * @param mode OVP mode selection
 * &param filter_sel The delay in LF Clock cycles between the OVP Interrupt detection
    and trigger.
 * @return Void.
 */
void pd_internal_vbus_ovp_en(uint16_t volt, int8_t per, PD_ADC_CB_T cb, bool pctrl,
        vbus_ovp_mode_t mode, uint8_t filter_sel);

/*
 * @brief Disable OVP control for CCG3 parts.
 *
 * @param pctrl Flag indicating whether the P_CTRL FETs are in use (as opposed
 * to the C_CTRL FETs).
 * @return Void.
 */
void pd_internal_vbus_ovp_dis(bool pctrl);

#endif /* VBUS_OVP_ENABLE */

/*
 * @brief Configure SBU Switch
 *
 * This function CCG3's SBU Switch which can connect SBU1/2 to AUX1/2 or isolate
 * them.
 *
 * @param sbu1_state SBU1 switch state
 * @param sbu2_state SBU2 switch state
 *
 * @return Void
 */
void sbu_switch_configure(sbu_switch_state_t sbu1_state, sbu_switch_state_t sbu2_state);

/*
 * @brief Returns SBU1 switch state
 *
 * @param Void
 * @return SBU1 switch state
 */
sbu_switch_state_t get_sbu1_switch_state(void);

/*
 * @brief Returns SBU2 switch state
 *
 * @param Void
 * @return SBU2 switch state
 */
sbu_switch_state_t get_sbu2_switch_state(void);

/*
 * @brief Configure AUX 1/2 resistor configuration
 *
 * This function controls AUX 1/2 resistor configuration.
 *
 * @param aux1_config AUX1 configuration
 * @param aux2_config AUX2 configuration
 *
 * @return Void
 */
void aux_resistor_configure(aux_resistor_config_t aux1_config,
        aux_resistor_config_t aux2_config);

/*
 * @brief Returns AUX1 resistor configuration
 *
 * @param Void
 * @return AUX1 resistor config
 */
aux_resistor_config_t get_aux1_resistor_config(void);

/*
 * @brief Returns AUX2 resistor configuration
 * @param Void
 * @return AUX2 resistor config
 */
aux_resistor_config_t get_aux2_resistor_config(void);

/**
 * @brief Enable VCONN Comparator for VCONN monitorng.
 *
 * This function enables VCONN Comparator to enable VCONN monitoring.
 *
 * @param None
 *
 * @return None
 */
void pd_enable_vconn_comp(void);

/**
 * @brief Get status of VCONN
 *
 * This function checks if port partner is providing VCONN and returns the status.
 * Expectation is that VCONN comparator is enabled first using pd_enable_vconn_comp
 * fucntion before calling this function. This function is expected to be used
 * by VCONN powered accessories.
 *
 * @param void
 *
 * @return true if VCONN is present, false otherwise
 */
bool pd_get_vconn_status(void);

/**
 * @brief Disconnect Ra from VCONN line
 *
 * This function removes Ra from VCONN. This can be used by AMAs to
 * reduce power consumption.
 *
 * @param None
 * @return None
 */
void pd_disconnect_ra(void);

#endif /* CCG3 */

/**
 * @brief Select the comparator block and input setting used for VBus detach
 * detection. CCG will use the selected settings to detect Type-C disconnection
 * based on removal of the VBus power.
 *
 * @param adc_id Select the comparator (ADC) block to be used.
 * @param adc_inp Select the comparator input to be used.
 * @return None
 */
void pd_hal_set_vbus_detach_params(PD_ADC_ID_T adc_id, PD_ADC_INPUT_T adc_inp);

/**
 * @brief Identify the CCGx ADC that is used for VBus detach detection.
 *
 * @return ID of the ADC block used for VBus detach detection.
 */
PD_ADC_ID_T pd_hal_get_vbus_detach_adc(void);

/**
 * @brief Identify the ADC input that is used for VBus detach detection.
 *
 * @return ADC block input used for VBus detach detection.
 */
PD_ADC_INPUT_T pd_hal_get_vbus_detach_input(void);

/**
 * @brief Specify the voltage division ratio between the voltage applied
 * on VBUS_MON and the actual VBus voltage. The commonly used resistor divider
 * ratio used is 1:10, giving a voltage division ratio of 1/11.
 *
 * @param divider Ratio between VBUS_MON voltage and VBus voltage.
 * @return None
 */
void pd_hal_set_vbus_mon_divider(uint8_t divider);

#endif /* _PDSS_HAL_H_ */

/* End of file */

