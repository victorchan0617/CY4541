/**
 * @file datamux_ctrl.c
 *
 * @brief @{Source file containing the data MUX control functions for the
 * CCG firmware application.
 *
 * The current implementation is for the PS8740B Type-C Redriving switch
 * from Parade. The implementation can be updated to support other MUX
 * and switch parts as required.@}
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

#include <pd.h>
#include <alt_modes_mngr.h>
#include <app.h>
#include <timer.h>

/* Polarity state offset in the current mux state. */
#define MUX_STATE_POLARITY_OFFSET               (7u)

/* PS8740B MUX Configuration Registers and Field Definitions. */
#define PS8740_MODE_SEL_IDX                     (0x0u)  /* Chip configuration and Mode selection Register offset. */
#define PS8740_PWR_DN_SHIFT                     (0x7u)  /* [7] Chip Power Down. */
#define PS8740_PWR_DN_MASK                      (0x1u << PS8740_PWR_DN_SHIFT)
#define PS8740_PWR_DN_NORMAL                    (0x0u << PS8740_PWR_DN_SHIFT)
#define PS8740_PWR_DN                           (0x1u << PS8740_PWR_DN_SHIFT)
#define PS8740_OP_MODE_SEL_SHIFT                (0x4u)   /* [6:4]   Operation Mode Selection. */
#define PS8740_OP_MODE_SEL_MASK                 (0x7u << PS8740_OP_MODE_SEL_SHIFT)
#define PS8740_OP_MODE_PWR_DN_                  (0x0u << PS8740_OP_MODE_SEL_SHIFT)
#define PS8740_OP_MODE_PWR_DN__                 (0x1u << PS8740_OP_MODE_SEL_SHIFT)
#define PS8740_USB_ONLY_UNFLP                   (0x2u<< PS8740_OP_MODE_SEL_SHIFT)
#define PS8740_USB_ONLY_FLP                     (0x3u<< PS8740_OP_MODE_SEL_SHIFT)
#define PS8740_DP_C_E_UNFLP                     (0x4u<< PS8740_OP_MODE_SEL_SHIFT) /* 4-lane DP. */
#define PS8740_DP_C_E_FLP                       (0x5u<< PS8740_OP_MODE_SEL_SHIFT) /* 4-lane DP. */
#define PS8740_DP_D_F_UNFLP                     (0x6u<< PS8740_OP_MODE_SEL_SHIFT) /* 2-lane DP plus USB. */
#define PS8740_DP_D_F_FLP                       (0x7u<< PS8740_OP_MODE_SEL_SHIFT) /* 2-lane DP plus USB. */
#define PS8740_HPD_SETTING_EN_SHIFT             (0x3u) /* [3] */
#define PS8740_HPD_SETTING_EN_MASK              (0x1u << PS8740_HPD_SETTING_EN_SHIFT)
#define PS8740_HPD_IN_HPD_PIN                   (0x0u << PS8740_HPD_SETTING_EN_SHIFT)
#define PS8740_HPD_REGISTERS                    (0x1u << PS8740_HPD_SETTING_EN_SHIFT)
#define PS8740_HPD_ASSERTION_SHIFT              (0x2u) /* [2] */
#define PS8740_HPD_ASSERTION_MASK               (0x1u << PS8740_HPD_ASSERTION_SHIFT)
#define PS8740_HPD_DEASSERTED                   (0x0u << PS8740_HPD_ASSERTION_SHIFT)
#define PS8740_HPD_ASSERTED                     (0x1u << PS8740_HPD_ASSERTION_SHIFT)
#define PS8740_USB_STANDBY_EN_SHIFT             (0x1u) /* [1] */
#define PS8740_USB_STANDBY_EN_MASK              (0x1u << PS8740_USB_STANDBY_EN_SHIFT)
#define PS8740_USB_NORMAL                       (0x0u << PS8740_USB_STANDBY_EN_SHIFT)
#define PS8740_USB_STANDBY                      (0x1u << PS8740_USB_STANDBY_EN_SHIFT)
#define PS8740_DP_STANDBY_EN_SHIFT              (0x0u) /* [0] */
#define PS8740_DP_STANDBY_EN_MASK               (0x1u << PS8740_DP_STANDBY_EN_SHIFT)
#define PS8740_DP_NORMAL                        (0x0u << PS8740_DP_STANDBY_EN_SHIFT)
#define PS8740_DP_STANDBY                       (0x1u << PS8740_DP_STANDBY_EN_SHIFT)

#define PS8740_DP_AUTO_TEST_IDX                 (0x1u)
#define PS8740_DP_AUTO_TEST_SHIFT               (0x0u) /* [0] */
#define PS8740_DP_AUTO_TEST_MASK                (0x1u<<PS8740_DP_AUTO_TEST_SHIFT)
#define PS8740_DP_AUTO_TEST_DISABLED            (0x0u<<PS8740_DP_AUTO_TEST_SHIFT)
#define PS8740_DP_AUTO_TEST_ENABLED             (0x1u<<PS8740_DP_AUTO_TEST_SHIFT)

#define PS8740_DP_TERM_ADJ_IDX                  (0x3u)
#define PS8740_DP_INPUT_ADJ_SHIFT               (0x6u) /* [6] */
#define PS8740_DP_INPUT_ADJ_MASK                (0x1u<<PS8740_DP_INPUT_ADJ_SHIFT)
#define PS8740_DP_INPUT_ADJ_100_OHM             (0x0u<<PS8740_DP_INPUT_ADJ_SHIFT)
#define PS8740_DP_INPUT_ADJ_85_OHM              (0x1u<<PS8740_DP_INPUT_ADJ_SHIFT)
#define PS8740_DP_OUTPUT_ADJ_SHIFT              (0x5u) /* [5] */
#define PS8740_DP_OUTPUT_ADJ_MASK               (0x1u<<PS8740_DP_OUTPUT_ADJ_SHIFT)
#define PS8740_DP_OUTPUT_ADJ_100_OHM            (0x0u<<PS8740_DP_OUTPUT_ADJ_SHIFT)
#define PS8740_DP_OUTPUT_ADJ_85_OHM             (0x1u<<PS8740_DP_OUTPUT_ADJ_SHIFT)
#define PS8740_DP_LINK_TRAIN_SHIFT              (0x4u) /* [4] */
#define PS8740_DP_LINK_TRAIN_MASK               (0x1u<<PS8740_DP_LINK_TRAIN_SHIFT)
#define PS8740_DP_LINK_TRAIN_ENABLED            (0x0u<<PS8740_DP_LINK_TRAIN_SHIFT)
#define PS8740_DP_LINK_TRAIN_DISABLED           (0x1u<<PS8740_DP_LINK_TRAIN_SHIFT)

#define PS8740_DP_AUX_INTERCEPT_IDX             (0x5u)
#define PS8740_DP_AUX_INTERCEPT_SHIFT           (0x7u) /* [7] */
#define PS8740_DP_AUX_INTERCEPT_MASK            (0x1u<<PS8740_DP_AUX_INTERCEPT_SHIFT)
#define PS8740_DP_AUX_INTERCEPT_ENABLED         (0x0u<<PS8740_DP_AUX_INTERCEPT_SHIFT)
#define PS8740_DP_AUX_INTERCEPT_DISABLED        (0x1u<<PS8740_DP_AUX_INTERCEPT_SHIFT)

#define PS8740_DP_EQ_IDX                        (0x6u)
#define PS8740_DP_EQ_AUTO_SHIFT                 (0x7u) /* [7] */
#define PS8740_DP_EQ_AUTO_MASK                  (0x1u<<PS8740_DP_EQ_AUTO_SHIFT)
#define PS8740_DP_EQ_LEVEL_SHIFT                (0x3u) /* [5:3] */
#define PS8740_DP_EQ_LEVEL_MASK                 (0x7u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_AUTO_DISABLE               (0x1u<<PS8740_DP_EQ_AUTO_SHIFT)
#define PS8740_DP_EQ_AUTO_ENABLE                (0x0u<<PS8740_DP_EQ_AUTO_SHIFT)
#define PS8740_DP_EQ_LEVEL_10_1_DB              (0x0u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_14_3_DB              (0x1u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_8_5_DB               (0x2u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_6_5_DB               (0x3u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_11_5_DB              (0x4u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_14_3__DB             (0x5u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_9_5_DB               (0x6u<<PS8740_DP_EQ_LEVEL_SHIFT)
#define PS8740_DP_EQ_LEVEL_7_5_DB               (0x7u<<PS8740_DP_EQ_LEVEL_SHIFT)

#define PS8740_SSTX_TX_IDX                      (0x31u)
#define PS8740_SSTX_TX_CH_DISABLE_SHIFT         (0x7u) /* [7] */
#define PS8740_SSTX_TX_CH_DISABLE_MASK          (0x1u<<PS8740_SSTX_TX_CH_DISABLE_SHIFT)
#define PS8740_SSTX_TX_CH_DISABLE               (0x01u<<PS8740_SSTX_TX_CH_DISABLE_SHIFT)
#define PS8740_SSTX_TX_CH_ENABLE                (0x00u)
#define PS8740_SSTX_TX_RXDET_SHIFT              (0x6u) /* [6] */
#define PS8740_SSTX_TX_RXDET_MASK               (0x1u<<PS8740_SSTX_TX_RXDET_SHIFT)
#define PS8740_SSTX_TX_RXDET_ENABLE             (0x0u)
#define PS8740_SSTX_TX_RXDET_DISABLE            (0x1u<<PS8740_SSTX_TX_RXDET_SHIFT)
#define PS8740_SSTX_TX_TEST_SHIFT               (0x3u) /* [3] */
#define PS8740_SSTX_TX_TEST_MASK                (0x1u<<PS8740_SSTX_TX_TEST_SHIFT)
#define PS8740_SSTX_TX_TEST_ENABLE              (0x1u<<PS8740_SSTX_TX_TEST_SHIFT)
#define PS8740_SSTX_TX_TEST_DISABLE             (0x0u)

#define PS8740_USBHOST_EQ_SETTING_IDX           (0x32u)
#define PS8740_USBHOST_EQ_SETTING_SHIFT         (0x5u) /* [7:5] */
#define PS8740_USBHOST_EQ_SETTING_MASK          (0x7u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_10_1_DB       (0x0u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_14_3_DB       (0x1u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_8_5_DB        (0x2u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_6_5_DB        (0x3u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_11_5_DB       (0x4u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_14_3__DB      (0x5u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_9_5_DB        (0x6u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_SETTING_7_5_DB        (0x7u<<PS8740_USBHOST_EQ_SETTING_SHIFT)
#define PS8740_USBHOST_EQ_INPUT_SHIFT           (0x2u) /* [2] */
#define PS8740_USBHOST_EQ_INPUT_MASK            (0x1u<<PS8740_USBHOST_EQ_INPUT_SHIFT)
#define PS8740_USBHOST_EQ_INPUT_100_OHM         (0x0u<<PS8740_USBHOST_EQ_INPUT_SHIFT)
#define PS8740_USBHOST_EQ_INPUT_85_OHM          (0x1u<<PS8740_USBHOST_EQ_INPUT_SHIFT)

#define PS8740_TYPEC_USB_DE_IDX                 (0x35u)
#define PS8740_TYPEC_USB_DE_LEVEL_SHIFT         (0x6u) /* [7:6] */
#define PS8740_TYPEC_USB_DE_LEVEL_MASK          (0x3u<<PS8740_TYPEC_USB_DE_LEVEL_SHIFT)
#define PS8740_TYPEC_USB_DE_OUTPUT_SHIFT        (0x0u) /* [0] */
#define PS8740_TYPEC_USB_DE_OUTPUT_MASK         (0x1u<<PS8740_TYPEC_USB_DE_OUTPUT_SHIFT)
#define PS8740_TYPEC_USB_DE_LEVEL_3_5_DB        (0x0u<<PS8740_TYPEC_USB_DE_LEVEL_SHIFT)
#define PS8740_TYPEC_USB_DE_LEVEL_0_DB          (0x1u<<PS8740_TYPEC_USB_DE_LEVEL_SHIFT)
#define PS8740_TYPEC_USB_DE_LEVEL_6_DB          (0x2u<<PS8740_TYPEC_USB_DE_LEVEL_SHIFT)
#define PS8740_TYPEC_USB_DE_OUTPUT_100_OHM      (0x0u<<PS8740_TYPEC_USB_DE_OUTPUT_SHIFT)
#define PS8740_TYPEC_USB_DE_OUTPUT_85_OHM       (0x1u<<PS8740_TYPEC_USB_DE_OUTPUT_SHIFT)

#define PS8740_TX_OUTPUT_SWING_ADJUST_IDX       (0x36u)
#define PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT     (0x6u) /* [7:6] */
#define PS8740_TX_OUTPUT_SWING_ADJUST_MASK      (0x3u<<PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_TX_OUTPUT_SWING_ADJUST_DEF       (0x0u<<PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_TX_OUTPUT_SWING_ADJUST_MIN20     (0x1u<<PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_TX_OUTPUT_SWING_ADJUST_MIN15     (0x2u<<PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_TX_OUTPUT_SWING_ADJUST_15        (0x3u<<PS8740_TX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_TX_LFPS_TD_SHIFT                 (0x3u) /* [3] */
#define PS8740_TX_LFPS_TD_MASK                  (0x3u<<PS8740_TX_LFPS_TD_SHIFT)
#define PS8740_TX_LFPS_SWING_DEFAULT            (0x0u<<PS8740_TX_LFPS_TD_SHIFT)
#define PS8740_TX_LFPS_SWING_TURNED_DOWN        (0x1u<<PS8740_TX_LFPS_TD_SHIFT)

#define PS8740_RX_SSRX_IDX                      (0x39u)
#define PS8740_RX_SSRX_CH_DISABLE_SHIFT         (0x7u) /* [7] */
#define PS8740_RX_SSRX_CH_DISABLE_MASK          (0x1u<<PS8740_RX_SSRX_CH_DISABLE_SHIFT)
#define PS8740_RX_SSRX_CH_ENABLE                (0x00u)
#define PS8740_RX_SSRX_CH_DISABLE               (0x01u<<PS8740_RX_SSRX_CH_DISABLE_SHIFT)
#define PS8740_RX_SSRX_RXDET_SHIFT              (0x6u) /* [6] */
#define PS8740_RX_SSRX_RXDET_MASK               (0x1u<<PS8740_RX_SSRX_RXDET_SHIFT)
#define PS8740_RX_SSRX_RXDET_ENABLE             (0x0u)
#define PS8740_RX_SSRX_RXDET_DISABLE            (0x1u<<PS8740_RX_SSRX_RXDET_SHIFT)
#define PS8740_RX_SSRX_TEST_SHIFT               (0x3u) /* [3] */
#define PS8740_RX_SSRX_TEST_MASK                (0x1u<<PS8740_RX_SSRX_TEST_SHIFT)
#define PS8740_RX_SSRX_TEST_ENABLE              (0x1u<<PS8740_RX_SSRX_TEST_SHIFT)
#define PS8740_RX_SSRX_TEST_DISABLE             (0x0u)

#define PS8740_USB_INPUT_ADJUST_IDX             (0x3Au)
#define PS8740_USB_INPUT_ADJUST_SHIFT           (0x4u) /* [4] */
#define PS8740_USB_INPUT_ADJUST_MASK            (0x1u<<PS8740_USB_INPUT_ADJUST_SHIFT)
#define PS8740_USB_INPUT_ADJUST_100_OHM         (0x0u<<PS8740_USB_INPUT_ADJUST_SHIFT)
#define PS8740_USB_INPUT_ADJUST_85_OHM          (0x1u<<PPS8740_USB_INPUT_ADJUST_SHIFT)

#define PS8740_TYPEC_EQ_SETTING_IDX             (0x3Bu)
#define PS8740_TYPEC_EQ_SETTING_SHIFT           (0x4u) /* [7:4] */
#define PS8740_TYPEC_EQ_SETTING_MASK            (0xFu<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_4_4_DB          (0x0u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_7_DB            (0x1u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_8_2_DB          (0x2u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_9_4_DB          (0x3u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_10_2_DB         (0x4u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_11_4_DB         (0x5u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_14_3_DB         (0x6u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_14_8_DB         (0x7u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_15_2_DB         (0x8u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_15_5_DB         (0x9u<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_16_2_DB         (0xAu<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_17_3_DB         (0xBu<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_18_4_DB         (0xCu<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_20_1_DB         (0xDu<<PS8740_TYPEC_EQ_SETTING_SHIFT)
#define PS8740_TYPEC_EQ_SETTING_21_3_DB         (0xEu<<PS8740_TYPEC_EQ_SETTING_SHIFT)

#define PS8740_TYPEC_HS_THRESHOLD_IDX           (0x3Cu)
#define PS8740_TYPEC_HS_THRESHOLD_LEVEL_2       (0x80u)

#define PS8740_SSRX_USB_DE_IDX                  (0x3Eu)
#define PS8740_SSRX_USB_DE_LEVEL_SHIFT          (0x6u) /* [7:6] */
#define PS8740_SSRX_USB_DE_LEVEL_MASK           (0x3u<<PS8740_SSRX_USB_DE_LEVEL_SHIFT)
#define PS8740_SSRX_USB_DE_OUTPUT_SHIFT         (0x0u) /* [0] */
#define PS8740_SSRX_USB_DE_OUTPUT_MASK          (0x1u<<PS8740_SSRX_USB_DE_OUTPUT_SHIFT)
#define PS8740_SSRX_USB_DE_LEVEL_3_5_DB         (0x0u<<PS8740_SSRX_USB_DE_LEVEL_SHIFT)
#define PS8740_SSRX_USB_DE_LEVEL_0_DB           (0x1u<<PS8740_SSRX_USB_DE_LEVEL_SHIFT)
#define PS8740_SSRX_USB_DE_LEVEL_1_5_DB         (0x2u<<PS8740_SSRX_USB_DE_LEVEL_SHIFT)
#define PS8740_SSRX_USB_DE_LEVEL_6_DB           (0x3u<<PS8740_SSRX_USB_DE_LEVEL_SHIFT)
#define PS8740_SSRX_USB_DE_OUTPUT_100_OHM       (0x0u<<PS8740_SSRX_USB_DE_OUTPUT_SHIFT)
#define PS8740_SSRX_USB_DE_OUTPUT_85_OHM        (0x1u<<PS8740_SSRX_USB_DE_OUTPUT_SHIFT)

#define PS8740_SSRX_OUTPUT_SWING_ADJUST_IDX     (0x3Fu)
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT   (0x6u) /* [7:6] */
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_MASK    (0x3u<<PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_100     (0x0u<<PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_MIN15   (0x1u<<PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_15      (0x2u<<PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT)
#define PS8740_SSRX_OUTPUT_SWING_ADJUST_30      (0x3u<<PS8740_SSRX_OUTPUT_SWING_ADJUST_SHIFT)

#define PS8740_STATUS_IDX                       (0x9u)
#define PS8740_HPD_STATUS_SHIFT                 (0x7u) /* [7] */
#define PS8740_HPD_STATUS_MASK                  (0x1u<<PS8740_HPD_STATUS_SHIFT)
#define PS8740_DP_MODE_STATUS_SHIFT             (0x4u) /* [4] */
#define PS8740_DP_MODE_STATUS_MASK              (0x1u<<PS8740_DP_MODE_STATUS_SHIFT)
#define PS8740_USB_MODE_STATUS_SHIFT            (0x3u) /* [3] */
#define PS8740_USB_MODE_STATUS_MASK             (0x1u<<PS8740_USB_MODE_STATUS_SHIFT)
#define PS8740_FLIP_MODE_STATUS_SHIFT           (0x2u) /* [2] */
#define PS8740_FLIP_MODE_STATUS_MASK            (0x1u<<PS8740_FLIP_MODE_STATUS_SHIFT)

#define GET_HPD_STATUS(status)                  (((status) >> PS8740_HPD_STATUS_SHIFT) & 0x1)
#define GET_DP_MODE_STATUS(status)              (((status) >> PS8740_DP_MODE_STATUS_SHIFT) & 0x1)
#define GET_USB_MODE_STATUS(status)             (((status) >> PS8740_USB_MODE_STATUS_SHIFT) & 0x1)
#define GET_FLIP_MODE_STATUS(status)            (((status) >> PS8740_FLIP_MODE_STATUS_SHIFT) & 0x1)

#define PS8740_CHANNEL_MODE_IDX                 (0x42u)
#define PS8740_SSTX_TX_SUSPEND_EN_SHIFT         (0x6u)   /* [6]  SSTX to Type C TX channel suspend mode. */
#define PS8740_SSTX_TX_SUSPEND_EN_MASK          (0x1u<<PS8740_SSTX_TX_SUSPEND_EN_SHIFT)
#define PS8740_SSTX_TX_POWERSAVING_EN_SHIFT     (0x5u)   /* [5]  SSTX to Type C TX channel poversaving mode. */
#define PS8740_SSTX_TX_POWERSAVING_EN_MASK      (0x1u<<PS8740_SSTX_TX_POWERSAVING_EN_SHIFT)
#define PS8740_SSTX_TX_NORMAL_EN_SHIFT          (0x4u)   /* [4]  SSTX to Type C TX channel normal mode. */
#define PS8740_SSTX_TX_NORMAL_EN_MASK           (0x1u<<PS8740_SSTX_TX_NORMAL_EN_SHIFT)
#define PS8740_RX_SSRX_SUSPEND_EN_SHIFT         (0x2u)   /* [2] */
#define PS8740_RX_SSRX_SUSPEND_EN_MASK          (0x1u<<PS8740_RX_SSRX_SUSPEND_EN_SHIFT)
#define PS8740_RX_SSRX_POWERSAVING_EN_SHIFT     (0x1u)   /* [1] */
#define PS8740_RX_SSRX_POWERSAVING_EN_MASK      (0x1u<<PS8740_RX_SSRX_POWERSAVING_EN_SHIFT)
#define PS8740_RX_SSRX_NORMAL_EN_SHIFT          (0x0u)   /* [0] */
#define PS8740_RX_SSRX_NORMAL_EN_MASK           (0x1u<<PS8740_RX_SSRX_NORMAL_EN_SHIFT)

#define PS8740_REV_ID_BYTE1_IDX                 (0xF0u) /* Revision ID byte 2. */
#define PS8740_REV_ID_BYTE2_IDX                 (0xF1u) /* Revision ID byte 2. */
#define PS8740_CHIP_ID_BYTE1_IDX                (0xF2u) /* Chip ID byte 1. */
#define PS8740_CHIP_ID_BYTE2_IDX                (0xF3u) /* Chip ID byte 2. */

/* I2C slave address assigned to PS8740B data mux for each PD port. */
static const uint8_t mux_ps8740_address[NO_OF_TYPEC_PORTS] =
{
    0x10
#if CCG_PD_DUALPORT_ENABLE
    ,
    0x11
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* Current mux state. */
static uint8_t mux_state[NO_OF_TYPEC_PORTS] =
{
    MUX_CONFIG_ISOLATE
#if CCG_PD_DUALPORT_ENABLE
    ,
    MUX_CONFIG_ISOLATE
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* MUX access timeout indication. */
static volatile bool mux_xfer_timeout = false;

/* Init settings to be applied to the PS8740 MUX for USB signal integrity. */
static uint8_t PS8740_redriver_config[][2] =
{
    {PS8740_USBHOST_EQ_SETTING_IDX, PS8740_USBHOST_EQ_SETTING_6_5_DB},
    {PS8740_TYPEC_USB_DE_IDX,       0x00},
    {PS8740_TYPEC_EQ_SETTING_IDX,   0x80},
    {PS8740_SSRX_USB_DE_IDX,        0x00},
    {PS8740_DP_EQ_IDX,              PS8740_DP_EQ_LEVEL_6_5_DB},
    {PS8740_TYPEC_HS_THRESHOLD_IDX, PS8740_TYPEC_HS_THRESHOLD_LEVEL_2}
};

/* Timer callback used for I2C transactions to the MUX. */
static void mux_xfer_timer_cb(uint8_t port, timer_id_t id)
{
    (void)port;
    (void)id;

    /*
     * I2C transmission to MUX continues longer than timeout. Slave doesn't
     * respond.
     */
    mux_xfer_timeout = true;
}

static bool PS8740_I2C_Write( uint8_t addr, uint8_t *buffer, uint32_t count)
{
    uint32_t i;
    uint8_t  status = false;

    /* Clear the timeout flag and start a timer. */
    mux_xfer_timeout = false;
    timer_start (0, MUX_I2C_TIMER, MUX_I2C_TIMER_PERIOD, mux_xfer_timer_cb);

    /* If the bus is free, generate a Start condition. */
    if ((MUX_CTRL_I2C_STATUS_REG & MUX_CTRL_I2C_STATUS_BUS_BUSY) == 0)
    {
        /* Assume operation passed for now. */
        status = true;

        /* TX and RX FIFO have to be EMPTY. */
        MUX_CTRL_TX_FIFO_WR_REG = (uint32_t)(addr << 1); /* Put address in TX FIFO. */
        MUX_CTRL_ClearMasterInterruptSource (MUX_CTRL_INTR_MASTER_ALL);
        MUX_CTRL_I2C_MASTER_GENERATE_START;

        /* Wait for an ACK from the MUX. */
        while (!MUX_CTRL_CHECK_INTR_MASTER(MUX_CTRL_INTR_MASTER_I2C_BUS_ERROR |
                                           MUX_CTRL_INTR_MASTER_I2C_ACK |
                    MUX_CTRL_INTR_MASTER_I2C_NACK |
                                           MUX_CTRL_INTR_MASTER_I2C_ARB_LOST) &&
               (!mux_xfer_timeout));

        /* Transfer the remaining data out to the MUX. */
        for (i = 0; ((!mux_xfer_timeout) && (i < count)); i++)
        {
            MUX_CTRL_TX_FIFO_WR_REG = buffer[i];
            while (!MUX_CTRL_CHECK_INTR_MASTER(MUX_CTRL_INTR_MASTER_I2C_BUS_ERROR|
                                               MUX_CTRL_INTR_MASTER_I2C_ACK |
                        MUX_CTRL_INTR_MASTER_I2C_NACK |
                                               MUX_CTRL_INTR_MASTER_I2C_ARB_LOST) &&
                   (!mux_xfer_timeout));
        }
    }

    /* Send a STOP to the slave. */
    MUX_CTRL_I2C_MASTER_GENERATE_STOP;
    while (!MUX_CTRL_CHECK_INTR_MASTER(MUX_CTRL_INTR_MASTER_I2C_STOP |
                MUX_CTRL_INTR_MASTER_I2C_ARB_LOST |
                                       MUX_CTRL_INTR_MASTER_I2C_BUS_ERROR) &&
           (!mux_xfer_timeout));

    /* Check the results of the address phase. */
    if (MUX_CTRL_CHECK_INTR_MASTER (MUX_CTRL_INTR_MASTER_I2C_NACK |
                MUX_CTRL_INTR_MASTER_I2C_ARB_LOST |
                                   MUX_CTRL_INTR_MASTER_I2C_BUS_ERROR) ||
        (mux_xfer_timeout))
    {
        /* Transaction failed. Reset the SCB block and return error. */
        MUX_CTRL_CTRL_REG &= ((uint32) ~MUX_CTRL_CTRL_ENABLED);
        MUX_CTRL_CTRL_REG |= ((uint32)  MUX_CTRL_CTRL_ENABLED);
        status = false;
    }

    timer_stop (0, MUX_I2C_TIMER);
    return (status);
}

/* Update the data mux settings as required. */
bool mux_ctrl_set_cfg(uint8_t port, mux_select_t cfg, uint8_t polarity)
{
    uint8_t tmp_buf[2u];
    bool    status = false;

    /*
     * Compare  new and recent mux states. Write to the MUX only if the config
     * is changing, or if the desired configuration is ISOLATE (to handle the
     * start-up case).
     */
    if ((port < NO_OF_TYPEC_PORTS) &&
        ((cfg == MUX_CONFIG_ISOLATE) || (((polarity << MUX_STATE_POLARITY_OFFSET) | cfg) != mux_state[port])))
    {
        tmp_buf[0] = PS8740_MODE_SEL_IDX;
        tmp_buf[1] = (polarity << PS8740_OP_MODE_SEL_SHIFT);

        switch(cfg)
        {
            case MUX_CONFIG_ISOLATE:
                /* Set mux to powerdown state. */
                tmp_buf[1] |= PS8740_DP_STANDBY | PS8740_USB_STANDBY;
                status = PS8740_I2C_Write (mux_ps8740_address[port], tmp_buf, sizeof(tmp_buf));
                break;

            case MUX_CONFIG_SS_ONLY:
                /* Set mux to USB-only mode state. */
                tmp_buf[1] |= PS8740_USB_ONLY_UNFLP;
                status = PS8740_I2C_Write (mux_ps8740_address[port], tmp_buf, sizeof(tmp_buf));
                break;

            case MUX_CONFIG_DP_4_LANE:
                /* Set mux to 4-lane DP mode state. */
                tmp_buf[1] |= PS8740_DP_C_E_UNFLP;
                status = PS8740_I2C_Write (mux_ps8740_address[port], tmp_buf, sizeof(tmp_buf));
                break;

            case MUX_CONFIG_DP_2_LANE:
                /* Set mux to to 2-lane DP plus USB-only mode state. */
                tmp_buf[1] |= PS8740_DP_D_F_UNFLP;
                status = PS8740_I2C_Write (mux_ps8740_address[port], tmp_buf, sizeof(tmp_buf));
                break;

            default:
                break;
        }

        /* Update current mux state if the operation succeeded. */
        if (status)
            mux_state[port] = ((polarity << MUX_STATE_POLARITY_OFFSET) | cfg);
    }

    return (status);
}

/* Applies recommended redriver settings for better signal quality through the PS8740 Mux. */
static bool ps8740_init(uint8_t port)
{
    uint8_t wr_buf[2];
    bool    status = true;
    int     index, count;

    count = sizeof (PS8740_redriver_config) / (2 * sizeof (uint8_t));
    for (index = 0; index < count; index++)
    {
        /* Write data in index 1 to address in index 0. */
        wr_buf[0] = PS8740_redriver_config[index][0];
        wr_buf[1] = PS8740_redriver_config[index][1];

        status = PS8740_I2C_Write (mux_ps8740_address[port], wr_buf, 2);
        if (!status)
            return status;
    }

    return status;
}

/* Initialize the MUX control SCB block. */
bool mux_ctrl_init(uint8_t port)
{
    bool status = true;

    /* Configure the I2C interface. */
    MUX_CTRL_CTRL_REG     = MUX_CTRL_I2C_DEFAULT_CTRL;
    MUX_CTRL_I2C_CTRL_REG = MUX_CTRL_I2C_DEFAULT_I2C_CTRL;

    /* Configure TX direction. */
    MUX_CTRL_TX_CTRL_REG      = MUX_CTRL_I2C_DEFAULT_TX_CTRL;
    MUX_CTRL_TX_FIFO_CTRL_REG = MUX_CTRL_I2C_DEFAULT_TX_FIFO_CTRL;

    MUX_CTRL_CTRL_REG |= MUX_CTRL_CTRL_ENABLED;

    /* Force the MUX into ISOLATE state. */
    status = mux_ctrl_set_cfg (port, MUX_CONFIG_ISOLATE, 0);
    if (!status)
        return status;

        status = ps8740_init (port);
    return status;
}

/* End of file */

