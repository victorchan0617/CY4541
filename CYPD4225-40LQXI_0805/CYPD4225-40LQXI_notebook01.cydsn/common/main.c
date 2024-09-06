/**
 * @file main.c
 *
 * @brief @{Main source file for CCG firmware implementation.@}
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

#include <project.h>
#include <flash_config.h>
#include <system.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <swap.h>
#include <vdm.h>
#include <pdo.h>
#include <app.h>
#include <hal_ccgx.h>
#include <timer.h>
#include <hpi.h>
#include <hpd.h>
#include <boot.h>
#include <flash.h>
#include <status.h>
#include <ccgx_version.h>
#include <app_version.h>
#include <utils.h>
#include <gpio.h>
#include <pdss_hal.h>
#include <pd_protocol.h>
#include <pd_policy_engine.h>
#include <pd.h>

#if AR_SLAVE_IF_ENABLE
#include <ar_slave.h>
#endif /* AR_SLAVE_IF_ENABLE */

#define PORT_START_IDX   (TYPEC_PORT_0_IDX)

/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
__attribute__ ((section(".customer_region"), used))
const uint32_t customer_info[8] = {0x00};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[5] = {0};

#if CCG_FIRMWARE_APP_ONLY
/*
   Provide a variable required by the HPI library. The real variable only
   exists if the boot-loadable component is included.
 */
volatile uint32_t cyBtldrRunType = 0;
#endif

/*該函式提供所有 PD 事件給解決方案。對於支援 HPI 的解決方案，解決方案函式應將呼叫重定向至 hpi_pd_event_handler。如果不支援 HPI，則該函式可以是簡單的假函式。*/
/* Solution PD event handler */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
#if (CCG_HPI_PD_ENABLE == 1)
    /* Pass the callback to HPI */
    hpi_pd_event_handler(port, evt, data);
#else /* (CCG_HPI_PD_ENABLE == 0) */
    (void)port;
    (void)evt;
    (void)data;
    /* Do nothing. */
#endif /* (CCG_HPI_PD_ENABLE) */
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const app_cbk_t app_callback =
{
    app_event_handler,
    psrc_set_voltage,
    psrc_set_current,
    psrc_enable,
    psrc_disable,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_rdo,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm
};



app_cbk_t* app_get_callback_ptr(uint8_t port)
{
    (void)port;
    /* Solution callback pointer is same for all ports */
    return ((app_cbk_t *)(&app_callback));
}

#if CCG_HPI_ENABLE
static void
update_hpi_regs (
        void)
{
    
/* Set to 1 if building a debug enabled binary with no boot-loader dependency. */
#if CCG_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint8_t ver_invalid[8] = {0};

    /* Flash access is not allowed. */
    flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08;              /* FW2 is not valid. */
    hpi_set_mode_regs (mode, reason);
    hpi_update_versions (ver_invalid, (uint8_t *)&base_version, ver_invalid);
    hpi_update_fw_locations (0, CCG_LAST_FLASH_ROW_NUM);

#else /* !CCG_FIRMWARE_APP_ONLY */

    uint8_t mode, reason = 0x00;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0};

     /* 根據使用中的韌體設定模式變數和快閃記憶體存取限制。*/
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        mode = 0x81 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW2 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x08;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_IMG1_LAST_FLASH_ROW_NUM + 1, CCG_IMG2_LAST_FLASH_ROW_NUM,
                CCG_IMG2_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }
    else
    {
        mode = 0x82 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW1 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x04;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, gl_img2_fw_metadata->boot_last_row,
                CCG_IMG1_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }

    hpi_set_mode_regs (mode, reason);

    /* 從韌體的詮釋資料計算版本位置 */
    if ((reason & 0x04) == 0)
    {
        fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
        fw1_ver = (((uint32_t)fw1_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->boot_last_row + 1;
    }
    else
    {
        fw1_ver = (uint32_t)ver_invalid;
        fw1_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    if ((reason & 0x08) == 0)
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = (((uint32_t)fw2_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->boot_last_row + 1;
    }
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    /* Update version information in the HPI registers. */
    hpi_update_versions (
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

    /* Update firmware location registers. */
    hpi_update_fw_locations (fw1_loc, fw2_loc);

#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* CCG_HPI_ENABLE */

#if APP_FW_LED_ENABLE

/* Blink the LED every LED_TIMER_PERIOD ms. This serves as an indication of the firmware running. */
void led_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    (void)port;
    (void)id;

    gpio_set_value (FW_LED_GPIO_PORT_PIN, !(gpio_read_value (FW_LED_GPIO_PORT_PIN)));
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);
}

#endif /* APP_FW_LED_ENABLE */

/*CY_ISR( Pin_SW_Handler )
{
    if(Pin_SW_Read()==0){
        Pin_1_Write(Pin_SW_Read());
        dpm_update_src_cap_mask (1, 0x03);
        dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
    }
    else if(Pin_SW_Read()!=0){
        Pin_1_Write(Pin_SW_Read());
        dpm_update_src_cap_mask (1, 0x0C);
        dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
    }
    Pin_SW_ClearInterrupt();
}*/

void choose_status(ccg_status_t ccg){
    if(ccg == CCG_STAT_NO_RESPONSE){
       UART_PutString("CCG_STAT_NO_RESPONSE");
    }
    else if(ccg == CCG_STAT_SUCCESS){
       UART_PutString("CCG_STAT_SUCCESS");
    }
    else if(ccg == CCG_STAT_FLASH_DATA_AVAILABLE){
       UART_PutString("CCG_STAT_FLASH_DATA_AVAILABLE");
    }
    else if(ccg == CCG_STAT_BAD_PARAM){
       UART_PutString("CCG_STAT_BAD_PARAM");
    }
    else if(ccg == CCG_STAT_INVALID_COMMAND){
       UART_PutString("CCG_STAT_INVALID_COMMAND");
    }
    else if(ccg == CCG_STAT_FLASH_UPDATE_FAILED){
       UART_PutString("CCG_STAT_FLASH_UPDATE_FAILED");
    }
    else if(ccg == CCG_STAT_INVALID_FW){
       UART_PutString("CCG_STAT_INVALID_FW");
    }
    else if(ccg == CCG_STAT_INVALID_ARGUMENT){
       UART_PutString("CCG_STAT_INVALID_ARGUMENT");
    }
    else if(ccg == CCG_STAT_NOT_SUPPORTED){
       UART_PutString("CCG_STAT_NOT_SUPPORTED");
    }
    else if(ccg == CCG_STAT_INVALID_SIGNATURE){
       UART_PutString("CCG_STAT_INVALID_SIGNATURE");
    }
    else if(ccg == CCG_STAT_TRANS_FAILURE){
       UART_PutString("CCG_STAT_TRANS_FAILURE");
    }
    else if(ccg == CCG_STAT_CMD_FAILURE){
       UART_PutString("CCG_STAT_CMD_FAILURE");
    }
    else if(ccg == CCG_STAT_FAILURE){
       UART_PutString("CCG_STAT_FAILURE");
    }
    else if(ccg == CCG_STAT_READ_DATA){
       UART_PutString("CCG_STAT_READ_DATA");
    }
    else if(ccg == CCG_STAT_BUSY){
       UART_PutString("CCG_STAT_BUSY");
    }
    else if(ccg == CCG_STAT_TIMEOUT){
       UART_PutString("CCG_STAT_TIMEOUT");
    }
}

    
void my_pd_phy_callback(uint8_t port, pd_phy_evt_t event) {
    if(port == 0){
        UART_PutString("000");
        UART_PutCRLF();
    }
    else if(port == 1){
        UART_PutString("111");
        UART_PutCRLF();
    }
    if(event == PD_PHY_EVT_TX_MSG_COLLISION)
    {
        UART_PutString("oPD_PHY_EVT_TX_MSG_COLLISION_event");
        UART_PutCRLF();
    }
    else{
        UART_PutString("other_event");
        UART_PutCRLF();
    }
    UART_PutString("------------------");
    UART_PutCRLF();
}

void my_pd_callback(uint8_t port,uint32_t event){
    if(port == 0){
        UART_PutString("000");
    }
    else if(port == 1){
        UART_PutString("111");
    }
    if(event == PD_PHY_EVT_TX_MSG_COLLISION)
    {
        UART_PutString("ok_prot_event");
    }
    UART_PutString("++++++++++++++++++");
}

int main()
{
    uint32_t conf_addr;
    uint8_t  port;
    pd_do_t SRC_PDO[4];
    ccg_status_t cc;
    pd_do_t dobj[3];
    uint32_t buf[3];
    
    //uint32_t receivedChar;
    //char c ='0';
    pd_packet_extd_t *Pk;
    //pd_cbk_t cbk;
    //pd_do_t *tst;
    //pd_phy_cbk_t *TTT;
    UART_Start();
    
#if CCG_HPI_ENABLE
    uint8_t i;
#endif /* CCG_HPI_ENABLE */

    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x= 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

/* 如果建立啟用 debug 的二進位檔案，且不依存於 boot-loader，則設為 1。*/
#if CCG_FIRMWARE_APP_ONLY 
    sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
#else /* !CCG_FIRMWARE_APP_ONLY */
    if ((uint32_t)&base_version < CCG_FW1_CONFTABLE_MAX_ADDR)
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
    }
    else
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_2);

/*Disable Pseudo-metadata handling in flashing sequence.*/
#ifndef CCG_PSEUDO_METADATA_DISABLE
        /* Determine pseudo metadata table address for Image 1 from the BOOT LAST row
         * parameter of Image 2. */
        gl_img1_fw_pseudo_metadata = (sys_fw_metadata_t *)((gl_img2_fw_metadata->boot_last_row
                << CCG_FLASH_ROW_SHIFT_NUM) + (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE));
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    }
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Validate the signature and checksum of the configuration table. */
    /*驗證組態表的簽章和校驗和。*/
    conf_addr = (uint32_t)get_pd_config ();
    if (boot_validate_configtable ((uint8_t *)conf_addr) != CCG_STAT_SUCCESS)
    {
#if CCG_FIRMWARE_APP_ONLY
        /* Can't do anything if config table is not valid. */
        while (1);
#else /* !CCG_FIRMWARE_APP_ONLY */
        /* Erase the firmware metadata so that this binary stops loading. */
        if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
        {
            flash_row_clear (CCG_IMG1_METADATA_ROW_NUM);
        }
        else
        {
            flash_row_clear (CCG_IMG2_METADATA_ROW_NUM);
        }

        /* Now reset the device. */
        CySoftwareReset ();
#endif /* CCG_FIRMWARE_APP_ONLY */
    }

#ifndef CCG_PSEUDO_METADATA_DISABLE
    /* Check if Other FW image is pending to be validated. If other image is valid,
     * device will undergo reset and then jump to the other image. */
    boot_check_for_valid_fw ();
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    /*
     * Configure the FET type usage model. This should be modified only if
     * the board supports a different configuration. Wrong configuration
     * shall result in damage of the boards and attached devices. Leave it
     * to default configuration for working with standard cypress design.
     *
     * For CCG4, the default for PCTRL is active high and CCTRL is active low.
     */
    pd_hal_set_fet_drive(PD_FET_DR_ACTIVE_HIGH, PD_FET_DR_ACTIVE_LOW);

    system_init();

    /* Timer INIT has to be done first. */
    timer_init();

    /* Enable global interrupts */
    CyGlobalIntEnable;/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */


#if AR_SLAVE_IF_ENABLE
    /* Initialize the Alpine-Ridge slave interface. */
    ar_slave_init (AR_SLAVE_SCB_INDEX, 0);
#endif /* AR_SLAVE_IF_ENABLE */

#if CCG_HPI_ENABLE

#if CCG_FIRMWARE_APP_ONLY
    /* Configure HPI for no-boot operation. */
    hpi_set_no_boot_mode (true);
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Initialize the HPI interface. */
    hpi_init (HPI_SCB_INDEX);

    /* Update HPI registers with mode and version information. */
    update_hpi_regs ();

#endif /* CCG_HPI_ENABLE */

    /* Perform application level initialization. */
    app_init();

    /* Initialize the Device Policy Manager for each PD port on the device. */
    for(port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {   
       
        
        pe_init(port);
        cc = pd_prot_init(port,my_pd_callback);
        choose_status(cc);
        
        UART_PutChar(' ');
        cc = pd_phy_init(port,my_pd_phy_callback);
        choose_status(cc);
 
        UART_PutCRLF();
        
        
        dpm_init(port, app_get_callback_ptr(port));
        
    }
        test_cbk(0);
#if CCG_HPI_ENABLE

    /* Send a reset complete event to the EC. */
    hpi_send_fw_ready_event ();

    /* Wait until EC ready event has been received or 100 ms has elapsed. */
    for (i = 0; i < 100; i ++)
    {
        /*此功能透過 HPI 暫存器處理來自 EC 的指令。來自 EC 的 HPI 寫入會在中斷情境中處理，任何相關的工作都會排隊由此函式處理。
hpi_task 應定期從韌體應用程式的主任務循環呼叫。*/
        hpi_task ();
        if (hpi_is_ec_ready ())
            break;

        CyDelay (1);
    }

#endif /* CCG_HPI_ENABLE */

    for (port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_HPI_ENABLE
        /* 只有在 HPI 層級啟用 DPM 時，才啟動連接埠的 DPM。有可能在我們到這裡之前，連接埠已被停用。

         */
        if ((hpi_get_port_enable () & (1 << port)) != 0)
#endif /* CCG_HPI_ENABLE */
        {
            pe_start(port);
            pd_prot_start(port);
            dpm_start(port);
        }
    }

#if APP_FW_LED_ENABLE

    /* Configure the LED control GPIO as an output. */
    gpio_hsiom_set_config (FW_LED_GPIO_PORT_PIN, HSIOM_MODE_GPIO, GPIO_DM_STRONG, true);
    /* Start a timer that will blink the FW ACTIVE LED, if required. */
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);

#endif /* APP_FW_LED_ENABLE */
    

    //Pin_SW_Int_StartEx(Pin_SW_Handler);
    
    //int first_time = 0;

    SRC_PDO[0].val = 0x2601912C; //5v
    SRC_PDO[0].fixed_src.usb_comm_cap = 1;
    SRC_PDO[1].val = 0x0002D12C; //9v
    SRC_PDO[2].val = 0x0004B12C; //15V
    SRC_PDO[2].fixed_src.usb_comm_cap = 1;
    SRC_PDO[3].val = 0x0006412C; //20V
    dpm_update_src_cap(1, 4, SRC_PDO);
    dpm_update_src_cap_mask (1, 0x07);
    dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
    
    
    //cc = pd_prot_send_data_msg(0,SOP_PRIME,DATA_MSG_SRC_CAP,4,tst);
    //pd_phy_cbk_t  my_callback_ptr;
    //my_callback_ptr = my_pd_phy_callback;
    
    //pd_prot_send_ctrl_msg(1,)
    change1_gl_pdss_status(1);
    
    
    /*Pk = pd_prot_get_rx_packet(1);
    if(Pk->sop == SOP){
        UART_PutString("yes");
    }
    else{
        UART_PutString("no");
    }*/
    
    buf[0] = 0x12345678;
    buf[1] = 0xabcdef12;
    buf[2] = 0x98765432;
    
    pd_phy_load_msg(1,SOP_PRIME,10,3,0x33A1,true,buf);
    pd_phy_send_msg(1);
    
    dobj[0].fixed_src.max_current = 300; // 3.0 A
    dobj[0].fixed_src.voltage = 100;    // 5.0 V
    dobj[0].fixed_src.supply_type = 0;  // Fixed Supply

    dobj[1].fixed_src.max_current = 300; // 3.0 A
    dobj[1].fixed_src.voltage = 180;    // 9.0 V
    dobj[1].fixed_src.usb_comm_cap = 0x0000;
    dobj[1].fixed_src.supply_type = 0;  // Fixed Supply
    
    dobj[2].fixed_src.max_current = 300; // 3.0 A
    dobj[2].fixed_src.voltage = 300;    // 15 V
    dobj[2].fixed_src.supply_type = 0;  // Fixed Supply
    
    
    
    cc = pd_prot_send_data_msg(1,SOP_PRIME,DATA_MSG_SRC_CAP,3,dobj);
    
    
    
    //cc = pd_prot_send_ctrl_msg(1,SOP,CTRL_MSG_FR_SWAP);
    //pd_phy_load_msg(1,SOP_P_DEBUG,3,0,385,true,NULL);
    //pd_phy_send_msg(1);
    //cc = pd_prot_send_ctrl_msg(1,SOP_DPRIME,CTRL_MSG_PING);
    //pd_phy_send_msg(0);
    
    //uint8 first = Pin_SW_Read();
    while(1)
    {
        //UART_PutHexByte(0x2b);
        
        /*if(Pin_SW_Read()==0 && first_time == 0){
            //dpm_update_src_cap(1, 4, SRC_PDO);
            dpm_update_src_cap_mask (1, 0x03);
            dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
            first_time = 1;
        }
        else if(Pin_SW_Read()!=0 && first_time == 1){
            //dpm_update_src_cap(1, 4, SRC_PDO);
            dpm_update_src_cap_mask (1, 0x0C);
            dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
            first_time = 0;
        }*/
        
        Pk = pd_prot_get_rx_packet(1);
        //UART_PutHexInt(Pk->hdr.hdr.msg_id);
        UART_PutHexInt(Pk->dat->fixed_src.usb_comm_cap);

        /*if(Pk->sop == SOP){
            UART_PutHexInt(Pk->sop);
        }
        else{
            UART_PutString("no");
        }*/
        
        /* Handle the device policy tasks for each PD port. */
        for(port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            //pe_fsm(port);
            /*此功能會執行指定連接埠的裝置管理員工作。*/
            dpm_task(port);
            app_task(port);

#if CCG_HPI_ENABLE
            /* Handle any pending HPI commands. */
            hpi_task ();
#endif /* CCG_HPI_ENABLE */
            
        }
        
       /* if(Pin_SW_Read()!=0){
            dpm_update_src_cap_mask (1, 0x03);
            dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
        }
        else{
            dpm_update_src_cap_mask (1, 0x0C);
            dpm_pd_command (1, DPM_CMD_SRC_CAP_CHNG, NULL,NULL);
        }*/
        
#if SYS_DEEPSLEEP_ENABLE

        /* If possible, enter sleep mode for power saving. */
        system_sleep();

#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* End of file */
