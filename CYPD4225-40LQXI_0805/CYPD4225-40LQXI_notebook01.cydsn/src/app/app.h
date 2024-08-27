/**
 * @file app.h
 *
 * @brief @{PD application handler header file.@}
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

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <pd.h>
#include <pdss_hal.h>
#include <alt_mode_hw.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* Note: Application Timers Must have timer id  >=30 and < 128 */

/* Note: Psource timers IDs must not be changed and must be consecutive */

/**
   @brief Power source enable timer ID.
 */
#define APP_PSOURCE_EN_TIMER                            (APP_TIMERS_START_ID)
/**
   @brief Power source monitor enable timer ID.
 */
#define APP_PSOURCE_EN_MONITOR_TIMER                    (31u)
/**
   @brief Power source enable hysteresis timer ID.
 */
#define APP_PSOURCE_EN_HYS_TIMER                        (32u)
/**
   @brief Power source disable timer ID.
 */
#define APP_PSOURCE_DIS_TIMER                           (33u)
/**
   @brief Power source disable monitor timer ID.
 */
#define APP_PSOURCE_DIS_MONITOR_TIMER                   (34u)
    
/**
   @brief Power source disable extra discharge timer ID.
 */
#define APP_PSOURCE_DIS_EXT_DIS_TIMER                   (35u)
#define APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD            (2u)
    
/**
   @brief Dead battery Sink Fet disable delay timer
 */
#define APP_DB_SNK_FET_DIS_DELAY_TIMER                  (36u)

/**
   @brief Power sink discharge disable timer ID.
 */
#define APP_PSINK_DIS_TIMER                             (37u)
#define APP_PSINK_DIS_TIMER_PERIOD                      (250u)

/**
   @brief Power sink disable monitor timer ID.
 */
#define APP_PSINK_DIS_MONITOR_TIMER                     (38u)
#define APP_PSINK_DIS_MONITOR_TIMER_PERIOD              (1u)

/**
   @brief VDM busy timer ID.
 */
#define APP_VDM_BUSY_TIMER                              (40u)
/**
   @brief tAME timer ID.
 */
#define APP_AME_TIMEOUT_TIMER                           (41u)
/**
    @brief VBUS OCP OFF Timer ID.
 */
#define APP_VBUS_OCP_OFF_TIMER                          (42u)
/**
   @brief VDM busy timer period (in ms).
 */
#define APP_VDM_BUSY_TIMER_PERIOD                       (50u)
/**
   @brief VDM retry (on failure) timer period in ms.
 */
#define APP_VDM_FAIL_RETRY_PERIOD                       (100u)
/**
   @brief tAME timer period (in ms).
 */
#define APP_AME_TIMEOUT_TIMER_PERIOD                    (1000u)
/**
   @brief Dead battery Sink Fet disable delay timer period.
 */
#define APP_DB_SNK_FET_DIS_DELAY_TIMER_PERIOD           (50u)

/**
 * @brief Billboard enumeration ON delay timer. This timer is used to delay
 * back to back turn on. This allows USB host to detect disconnection correctly.
 * This timer is used only for devices with internal USB block (CCG3).
 */
#define APP_BB_ON_TIMER                                 (60u)
/**
 * @brief Billboard ON delay timer period. If billboard needs to re-started,
 * delay start by specified time to ensure that the host can detect the device
 * going away correctly.
 */
#define APP_BB_ON_TIMER_PERIOD                          (250u)
/**
 * @brief Billboard OFF delay timer. This timer is used to remove the billboard
 * enumeration to save power. The USB block shall be disabled after the a
 * specified timeout. The timeout value is expected to be specified via the
 * configuration table.
 */
#define APP_BB_OFF_TIMER                                (61u)

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/
/**
   @struct app_status_t
   @brief This structure hold all variables related to application layer functionality.
 */
typedef struct
{
    pwr_ready_cbk_t pwr_ready_cbk;        /**< Registered Power source callback. */
    sink_discharge_off_cbk_t snk_dis_cbk; /**< Registered Power sink callback. */
    app_resp_t app_resp;                  /**< Buffer for APP responses. */
    vdm_resp_t vdm_resp;                  /**< Buffer for VDM responses. */
    uint16_t psrc_volt;                   /**< Current Psource voltage. */
    uint16_t psrc_volt_old;               /**< Old Psource voltage. */
    uint8_t vdm_task_en;                  /**< Flag to indicate is vdm task manager enabled. */
    uint8_t cbl_disc_id_finished;         /**< Flag to indicate that cable disc id finished. */
    uint8_t vdm_version;                  /**< Live VDM version. */
    bool alt_mode_trigger;                /**< Flag to indicate whether alt mode trigger is enabled by EC. */
    bool alt_mode_entered;                /**< Flag to indicate is alternate modes currently entered. */
    bool vdm_prcs_failed;                 /**< Flag to indicate is vdm process failed. */
    bool is_vbus_on;                      /**< Is supplying VBUS flag. */
    bool is_vconn_on;                     /**< Is supplying VCONN flag. */
    bool vdm_retry_pending;               /**< Whether VDM retry on timeout is pending. */
    bool psrc_rising;                     /**< Voltage ramp up/down. */
}app_status_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Application level init function.
 *
 * This function performs any Application level initialization required
 * for the CCG solution. This should be called before calling the
 * dpm_init function.
 *
 * @return None.
 *
 */
void app_init(void);

/**
 * @brief Solution handler for PD events reported from the stack.
 *
 * The function provides all PD events to the solution. For a solution
 * supporting HPI, the solution function should re-direct the calls to
 * hpi_pd_event_handler. If no HPI is supported, the function can be a simple
 * dummy function.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that
 * needs to be de-referenced based on event type.
 *
 * @return None
 */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

/**
 * @brief Handler for application level asynchronous tasks.
 * @param port USB-PD port for which tasks are to be handled.
 * @return 1 in case of success, 0 in case of task handling error.
 */
uint8_t app_task(uint8_t port);

/**
 * @brief This function return the App callback structure pointer
 * @param port port index
 * @return  Application callback structure pointer
 */
app_cbk_t* app_get_callback_ptr(uint8_t port);

/**
 * @brief Handler for event notifications from the PD stack.
 * @param port Port on which events are to be handled.
 * @param evt Type of event to be handled.
 * @param dat Data associated with the event.
 * @return None
 */
void app_event_handler (uint8_t port, app_evt_t evt, const void* dat);

/**
 * @brief Get a handle to the application provide PD command response buffer.
 * @param port PD port corresponding to the command and response.
 * @return Pointer to the response buffer.
 */
app_resp_t* app_get_resp_buf(uint8_t port);

/**
 * @brief Get handle to structure containing information about the system status for a PD port.
 * @param port PD port to be queried.
 * @return Pointer to the system information structure.
 */
app_status_t* app_get_status(uint8_t port);

/**
 * @brief Check whether the APP handlers are ready to allow device deep sleep.
 * @return true if APP handler is idle, false otherwise.
 */
bool app_sleep (void);

/**
 * @brief Restore the APP handler state after CCG device wakes from deep-sleep.
 * @return None
 */
void app_wakeup (void);

/**
 * @brief Check whether cable discovery has been completed.
 *
 * This function is used by the alternate modes manager to check whether
 * cable discovery has been completed.
 *
 * @param port Port for which the check is to be performed.
 * @return true if cable discovery is complete (or disabled), false if not.
 */
bool app_is_cbl_disc_done(uint8_t port);

/**
 * @brief Function to place CCG device in power saving mode if possible.
 *
 * This function places the CCG device in power saving deep sleep mode
 * if possible. The function checks for each interface (PD, HPI etc.)
 * being idle and then enters sleep mode with the appropriate wake-up
 * triggers. If the device enters sleep mode, the function will only
 * return after the device has woken up.
 *
 * @return true if the device went into sleep, false otherwise.
 */
bool system_sleep(void);

/*****************************************************************************
  Functions related to power
 *****************************************************************************/

/**
 * @brief This function enables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_enable(uint8_t port, uint8_t channel);

/**
 * @brief This function disables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_disable(uint8_t port, uint8_t channel);

/**
 * @brief This function checks if power is present on VConn
 *
 * @param port Port index the function is performed for.
 *
 * @return true if power is present on VConn, else returns false
 */
bool vconn_is_present(uint8_t port);

/**
 * @brief This function checks if power is present on VBus
 *
 * @param port Port index the function is performed for.
 * @param volt Voltage in 50mV units.
 * @param per  Threshold margin.
 *
 * @return true if power is present on VBus, else returns false
 */
bool vbus_is_present(uint8_t port, uint16_t volt, int8_t per);

/**
 * @brief This function turns on dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_on(uint8_t port);

/**
 * @brief This function turns off dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_off(uint8_t port);

/**
 * @brief This function enable vconn ocp
 * @param port Port index
 * @param cbk OCP callback
 * @return  Returns 1 if params are ok else return 0
 */
uint8_t system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk);

/**
 * @brief This function disable vconn ocp
 * @param port Port index
 * @return  Returns 1 if params are ok else return 0
 */
uint8_t system_vconn_ocp_dis(uint8_t port);

/**
 * @brief Enable and configure the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param volt_50mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param ovp_cb Callback function to be triggered when there is an OV event.
 * @return None
 */
void app_ovp_enable(uint8_t port, uint16_t volt_50mV, bool pfet, PD_ADC_CB_T ovp_cb);

/**
 * @brief Disable the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_ovp_disable(uint8_t port, bool pfet);

/*****************************************************************************
  Functions to be provided at the solution level.
 *****************************************************************************/

/**
 * @brief Initialize the Type-C Data Mux for a specific PD port.
 *
 * @param port USB-PD port for which the MUX is to be initialized.
 * @return Returns true if the MUX is initialized successfully, false otherwise.
 */
bool mux_ctrl_init(uint8_t port);

/**
 * @brief Set the Type-C MUX to the desired configuration.
 * @param port PD port on which MUX is to be configured.
 * @param cfg Desired MUX configuration.
 * @param polarity Polarity of the Type-C connection.
 * @return Returns true if the operation is successful, false otherwise.
 */
bool mux_ctrl_set_cfg(uint8_t port, mux_select_t cfg, uint8_t polarity);

#endif /* _APP_H_ */

/* End of File */

