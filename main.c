#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util.h"
#include "app_error.h"

#include "app_button.h"

#include "device_manager.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_bas.h"
#include "ble_dis.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_config.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"

#include "pstorage.h"

#include "softdevice_handler.h"
#include "sensorsim.h"

#include "SEGGER_RTT.h"
#include "our_service.h"


#define WAKEUP_BUTTON_PIN								BUTTON_0                                    
#define LEDBUTTON_BUTTON_PIN_NO					BUTTON_1                                    
#define SEND_INTEGER_BUTTON_PIN_NR			BUTTON_2                                   

#define CENTRAL_LINK_COUNT							1                               /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT						0                               /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define UART_TX_BUF_SIZE								256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE								256                             /**< UART RX buffer size. */
                                                                
#define NUS_SERVICE_UUID_TYPE						BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */
                                                                
#define APP_TIMER_PRESCALER							256                          		/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_CONVERTOR             128
#define APP_TIMER_OP_QUEUE_SIZE					4                               /**< Size of timer operation queues. */
#define APP_TIMER_MAX_TIMERS						1     													// Maximum number of timers in this application.

#define DEVICE_NAME											"pvbadge"            		/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Panvista" 

#define APP_ADV_INTERVAL								160                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS			0                       /**< The advertising timeout (in units of seconds). */

#define SCAN_INTERVAL										0x00C8                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW											0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE											1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE									0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT										0x01A4                          

#define BUTTON_DETECTION_DELAY					APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define PSTORAGE_BLOCK_SIZE							1024
#define PSTORAGE_BLOCK_NUMBER						1

#define NUMBER_OF_BEACONS               100												/**< Number of rows in Beacon Table>**/

#define UPLOAD_APP_TIME									17												/**< Every 17 seconds, upload current tick on pstorage >**/
#define UPLOAD_BEACON_DATA_TIME					240												/**< Every 240 seconds, upload beacon data >**/

static ble_nus_c_t											m_ble_nus_c;
static ble_db_discovery_t								m_ble_db_discovery;

uint8_t pstorage_wait_flag							= 0;
pstorage_block_t pstorage_wait_handle		= 0;
bool payload_flag												= false;

uint32_t current_app_tick, 
				 payload_tick;

uint8_t upload_tick[4];

pstorage_handle_t handle;
pstorage_handle_t block_0_handle;
pstorage_module_param_t param;

uint8_t device_info[9] = {0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, NULL};
uint8_t dest_data_0[(NUMBER_OF_BEACONS * 8) + 8];
uint8_t beacon_table[NUMBER_OF_BEACONS][8] = {{NULL,},};
uint8_t raw_beacon_table[NUMBER_OF_BEACONS * 8] = {NULL,};
uint8_t adv_read[7];

static const ble_gap_scan_params_t m_scan_params = 
{
	.active      = SCAN_ACTIVE,
	.selective   = SCAN_SELECTIVE,
	.p_whitelist = NULL,
	.interval    = SCAN_INTERVAL,
	.window      = SCAN_WINDOW,
	//.timeout     = SCAN_TIMEOUT
};

static const ble_uuid_t m_nus_uuid = 
{
	.uuid = BLE_UUID_NUS_SERVICE,
	.type = NUS_SERVICE_UUID_TYPE	
};

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void scan_start(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_scan_start(&m_scan_params);
		APP_ERROR_CHECK(err_code);
		
    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(err_code);
}

static void scan_stop()
{
		SEGGER_RTT_WriteString(0,"\nScan stopped\r\n");
		sd_ble_gap_scan_stop();
}

static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t            err_code;
    ble_advdata_t       advdata;    // Struct containing advertising parameters
    uint8_t             flags       = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; 					// Defines how your device is seen (or not seen) by other devices
		ble_uuid_t          adv_uuids[] = {BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN};   // Random example User Unique Identifier
   
		SEGGER_RTT_WriteString(0,"1\r\n");
		
    // Populate the advertisement packet
    // Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&advdata, 0, sizeof(advdata));
    
		SEGGER_RTT_WriteString(0,"2\r\n");
		
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
		advdata.include_appearance      = true;		
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;                                     // Must be included, but not discussed in this tutorial
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]); // Must be included, but not discussed in this tutorial
    advdata.uuids_complete.p_uuids  = adv_uuids;                                // Must be included, but not discussed in this tutorial

		SEGGER_RTT_WriteString(0,"3\r\n");
		
    // Set the advertisement and scan response packet. 
    err_code = ble_advdata_set(&advdata, NULL);
		
		SEGGER_RTT_WriteString(0,"4\r\n");
		
    APP_ERROR_CHECK(err_code);// Check for errors
		
		SEGGER_RTT_WriteString(0,"5\r\n");
}

static void advertise_connectable_start(void){
    uint32_t            err_code;
    ble_advdata_t       advdata;    // Struct containing advertising parameters
    uint8_t             flags       = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_uuid_t          adv_uuids[] = {{0x1234, BLE_UUID_TYPE_BLE}};   

    // Populate the advertising packet
    // Always initialize all fields in structs to zero or you might get unexpected behaviours
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;                   // Use full name
    advdata.flags                   = flags;                                    // Set some flags. This is a topic for another tutorial
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]); // Create a list of UUIDs. This is a topic for another tutorial
    advdata.uuids_complete.p_uuids  = adv_uuids;                                // Create a list of UUIDs. This is a topic for another tutorial

    // Set the advertisement and scan response packet. 
    err_code = ble_advdata_set(&advdata, NULL); 
    APP_ERROR_CHECK(err_code);                  // Check for errors	
}

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */ 
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                while (ble_nus_c_string_send(&m_ble_nus_c, data_array, index) != NRF_SUCCESS)			
                {
                    // repeat until sent
                }
                index = 0;
            }
            break;
        /**@snippet [Handling data from UART] */ 
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud38400
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}

static void pstorage_handler(pstorage_handle_t  * handle,
															 uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
		if(handle->block_id == pstorage_wait_handle) { pstorage_wait_flag = 0;}
		
		switch(op_code)
		{
			case PSTORAGE_LOAD_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
						 SEGGER_RTT_printf(0,"pstorage LOAD callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_ALERT_0);				 
				 }
				 else
				 {
						 SEGGER_RTT_printf(0,"pstorage LOAD ERROR callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_RCV_ERROR);
				 }
				 break;
			case PSTORAGE_STORE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
						 SEGGER_RTT_printf(0,"pstorage STORE callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_ALERT_1);
				 }
				 else
				 {
					   SEGGER_RTT_printf(0,"pstorage STORE ERROR callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_RCV_ERROR);
				 }
				 break;				 
			case PSTORAGE_UPDATE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
						 SEGGER_RTT_printf(0,"pstorage UPDATE callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_ALERT_2);
				 }
				 else
				 {
						 SEGGER_RTT_printf(0,"pstorage UPDATE ERROR callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_RCV_ERROR);
				 }
				 break;
			case PSTORAGE_CLEAR_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
						 SEGGER_RTT_printf(0,"pstorage CLEAR callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_ALERT_3);
				 }
				 else
				 {
					   SEGGER_RTT_printf(0,"pstorage CLEAR ERROR callback received \r\n");
						 bsp_indication_set(BSP_INDICATE_RCV_ERROR);
				 }
				 break;		 
		}			
}

void pstorage_initialization()
{
			pstorage_init();

			param.block_size  = PSTORAGE_BLOCK_SIZE;
			param.block_count = PSTORAGE_BLOCK_NUMBER;
			param.cb          = pstorage_handler;
			pstorage_register(&param, &handle);
}

void pstorage_clear_block()
{
		pstorage_clear(&block_0_handle, PSTORAGE_BLOCK_SIZE);
}

void pstorage_store_block()
{
    pstorage_block_identifier_get(&handle, 0, &block_0_handle);
		pstorage_clear_block();
	
		pstorage_wait_handle = block_0_handle.block_id;
		pstorage_wait_flag = 1;

		pstorage_store(&block_0_handle, device_info, 8, 0);
		while(pstorage_wait_flag) { power_manage(); }
}

void pstorage_load_block()
{
			pstorage_block_identifier_get(&handle, 0, &block_0_handle);
			
			pstorage_wait_handle = block_0_handle.block_id;
			pstorage_wait_flag = 1;

			pstorage_load(dest_data_0, &block_0_handle, sizeof(dest_data_0), 0);
			while(pstorage_wait_flag) { power_manage(); }
			printf("pstorage load block 0 :\r\n");
			
			int i;
			for (i = 0; i < sizeof(dest_data_0); i ++)
			{
					if (i > 7 && i < (sizeof(dest_data_0) - 4) 
						 && dest_data_0[i] == 0x00 && dest_data_0[i+1] == 0x00 
						 && dest_data_0[i+2] == 0x00 && dest_data_0[i+3] == 0x00
						 ) 
					{ break; }
					
					if (i % 8 == 0) { printf("\n"); }
					printf("%02x ", dest_data_0[i]);
					nrf_delay_ms(5);
			} printf("\n\n");
}

void pstorage_update_block_device_info(uint32_t source_data)
{
    device_info[4] = (source_data >> 24) & 0xFF;
		device_info[5] = (source_data >> 16) & 0xFF;
		device_info[6] = (source_data >> 8) & 0xFF;
		device_info[7] = (source_data >> 0) & 0xFF;
																	 
		pstorage_block_identifier_get(&handle, 0, &block_0_handle);
    pstorage_wait_handle = block_0_handle.block_id;
		
		pstorage_wait_flag = 1;
    pstorage_update(&block_0_handle, device_info, 8, 0);
		while(pstorage_wait_flag) { power_manage(); }
}

void pstorage_update_block_beacon_data()
{
		pstorage_block_identifier_get(&handle, 0, &block_0_handle);
    pstorage_wait_handle = block_0_handle.block_id;
    pstorage_wait_flag = 1;
    pstorage_update(&block_0_handle, raw_beacon_table, sizeof(raw_beacon_table), 8);
		while(pstorage_wait_flag) { power_manage(); }
}

static void printf_beacon_data_ram()
{
		uint32_t number_of_rows = sizeof(beacon_table)/8;
		uint8_t i;
	
		for (i = 0; i < number_of_rows; i++ ) 
		{
				if(beacon_table[i][0] == 0x00 && beacon_table[i][1] == 0x00 
						 && beacon_table[i][2] == 0x00 &&  beacon_table[i][3] == 0x00
					)
				{
						break;
				}

				SEGGER_RTT_printf(0, "[%02d]:[%02x%02x %02x%02x %02x%02x %02x%02x]\n",
												 i, beacon_table[i][0],beacon_table[i][1],
												 beacon_table[i][2],beacon_table[i][3],
												 beacon_table[i][4],beacon_table[i][5],
												 beacon_table[i][6],beacon_table[i][7]);
		}
}

int find_beacon_data_ram()
{
		uint32_t number_of_rows = sizeof(beacon_table)/8;
		uint8_t i;
	
		for (i = 0; i < number_of_rows; i++ ) 
		{
				if(adv_read[2] == beacon_table[i][0] && adv_read[3] == beacon_table[i][1] 
					 && adv_read[4] == beacon_table[i][2] && adv_read[5] == beacon_table[i][3]
					)
				{
						return i;
				}
		}
		
		return -1;
}

int find_end_beacon_data_ram()
{
		uint32_t number_of_rows = sizeof(beacon_table)/8;
		uint8_t i;
	
		for (i = 0; i < number_of_rows; i++ ) 
		{
				if(beacon_table[i][0] == 0x00 && beacon_table[i][1] == 0x00 
					 && beacon_table[i][2] == 0x00 &&  beacon_table[i][3] == 0x00
					)
				{
						return i;
				}
		}
		
		return -1;
}

static void store_beacon_data_ram()
{
		int store_index = find_beacon_data_ram();
	
		if(adv_read[2] == 0x00 && adv_read[3] == 0x00 && adv_read[4] == 0x00 && adv_read[5] == 0x00) { return; }
		
		if (store_index == -1)
		{
				int end_index = find_end_beacon_data_ram();
				
				if (end_index == -1) { return; }
				
				beacon_table[end_index][0] = adv_read[2];
				beacon_table[end_index][1] = adv_read[3];
				beacon_table[end_index][2] = adv_read[4];
				beacon_table[end_index][3] = adv_read[5];
				beacon_table[end_index][4] = adv_read[0];
				beacon_table[end_index][5] = adv_read[1];
				beacon_table[end_index][6] = adv_read[0];
				beacon_table[end_index][7] = adv_read[1];
		}
		else if(store_index >= 0)
		{
				beacon_table[store_index][6] = adv_read[0];
				beacon_table[store_index][7] = adv_read[1];
		}
}

static void clear_beacon_data_ram()
{
		uint32_t number_of_rows = sizeof(beacon_table)/8;
		int i, j;
	
		for (i = 0; i < number_of_rows; i++ ) 
		{
				for (j = 0; j < 8; j++)
				{
						beacon_table[i][j] = NULL;
						raw_beacon_table[(i*8)+j] = NULL;
				}
		}
}

static void transform_beacon_table()
{
		uint32_t number_of_rows = sizeof(beacon_table)/8;
		int i, j, k = 0;
	
		for (i = 0; i < number_of_rows; i++ ) 
		{
				for (j = 0; j < 8; j++)  
				{
						raw_beacon_table[k] = beacon_table[i][j];
						k++;
				}
		}
}

static void record_data_into_pstorage()
{
		transform_beacon_table();
		pstorage_update_block_device_info(current_app_tick/APP_TIMER_CONVERTOR);
		pstorage_update_block_beacon_data();
		pstorage_load_block();
}

static void print_beacon_adv()
{
		SEGGER_RTT_printf(0,"[%d][%02x%02x%02x%02x]:",
											current_app_tick/APP_TIMER_CONVERTOR,
											upload_tick[0], upload_tick[1], upload_tick[2], upload_tick[3]);
	
		SEGGER_RTT_printf(0,"[%03d] Time: %02x%02x Major: %02x%02x, Minor: %02x%02x\n", 
											payload_tick, adv_read[0], adv_read[1], 
																		adv_read[2], adv_read[3], 
																		adv_read[4], adv_read[5]);
}

static bool is_uuid_present(const ble_uuid_t *p_target_uuid, 
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t 	index 	= 0;
    uint8_t 	*p_data = (uint8_t *)p_adv_report->data;
	
    while (index < p_adv_report->dlen)
    {
        uint8_t 	field_length	= p_data[index],
									beacon_type 	= 7;

				if (p_adv_report->rssi > -50) {
						app_timer_cnt_get(&current_app_tick);
						payload_tick = (current_app_tick/APP_TIMER_CONVERTOR) % 240;

						upload_tick[0] = ((current_app_tick/APP_TIMER_CONVERTOR) >> 24) & 0xFF;
						upload_tick[1] = ((current_app_tick/APP_TIMER_CONVERTOR) >> 16) & 0xFF;
						upload_tick[2] = ((current_app_tick/APP_TIMER_CONVERTOR) >> 8) & 0xFF;
						upload_tick[3] = ((current_app_tick/APP_TIMER_CONVERTOR) >> 0) & 0xFF;
		
						adv_read[0] = (payload_tick >> 8) & 0xFF;
						adv_read[1] = (payload_tick >> 0) & 0xFF;	
					
						if (p_adv_report->data[beacon_type] == 0x02 
								&& p_adv_report->data[beacon_type+1] == 0x15
							 )
						{
								adv_read[2] = p_adv_report->data[25];
								adv_read[3] = p_adv_report->data[26];
								adv_read[4] = p_adv_report->data[27];
								adv_read[5] = p_adv_report->data[28];
								adv_read[6] = NULL;
						}
						else if (p_adv_report->data[beacon_type] == 0x00 
										 && p_adv_report->data[beacon_type+1] == 0x99
										)
						{																					
								adv_read[2] = p_adv_report->data[26];
								adv_read[3] = p_adv_report->data[27];
								adv_read[4] = p_adv_report->data[28];
								adv_read[5] = p_adv_report->data[29];
								adv_read[6] = NULL;
						}
						
						store_beacon_data_ram();
						print_beacon_adv();
				}
        index += field_length + 1;
    }
    return false;
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;
            is_uuid_present(&m_nus_uuid, p_adv_report);   				
            break;
        }
        default:
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
		switch (sys_evt)
    {
				case NRF_EVT_FLASH_OPERATION_SUCCESS:
				case NRF_EVT_FLASH_OPERATION_ERROR:
						pstorage_sys_event_handler(sys_evt);
						break;
				default:
						break;
    }
}

static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case 13:
          SEGGER_RTT_printf(0,"\nButton 1 Pushed: %d\r\n", event);
					printf("\nButton 1 Pushed: %d\r\n", event);
					scan_start();
          break;

        case 14:
					SEGGER_RTT_printf(0,"\nButton 2 Pushed: %d\r\n", event);
					printf("\nButton 2 Pushed: %d\r\n", event);
					scan_stop();
          break;

				case 15:
					SEGGER_RTT_printf(0,"\nButton 3 Pushed: %d\r\n", event);
					printf_beacon_data_ram();
					break;
				
				case 16:
					SEGGER_RTT_printf(0,"\nButton 4 Pushed: %d\r\n", event);
					break;
				
        default:
					SEGGER_RTT_printf(0,"\nSome other event happened: %d\r\n", event);
          break;
    }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

int main(void)
{
		uint32_t err_code;
		bool erase_bonds;

		SEGGER_RTT_WriteString(0,"\nMain started\r\n");
		SEGGER_RTT_WriteString(0,"A\r\n");
		
		app_trace_init();
		timers_init();

		SEGGER_RTT_WriteString(0,"B\r\n");
		
		buttons_leds_init(&erase_bonds);
	  uart_init();
    ble_stack_init();
		ble_db_discovery_init();
	
		SEGGER_RTT_WriteString(0,"C\r\n");

		//advertising_init();

		pstorage_initialization();
		pstorage_store_block();
		nrf_delay_ms(100);
		pstorage_load_block();
	
		NRF_RTC1->TASKS_START = 1;
		scan_start();
		
    for (;;)
    {
				app_timer_cnt_get(&current_app_tick);
				if((current_app_tick/APP_TIMER_CONVERTOR) % UPLOAD_BEACON_DATA_TIME == 0 
					 &&(current_app_tick/APP_TIMER_CONVERTOR) / UPLOAD_BEACON_DATA_TIME > 0
					 )
				{
						clear_beacon_data_ram();
						SEGGER_RTT_printf(0,"::: Cleaned Beacon table\n");
						printf("::: Cleaned Beacon table\n");
						nrf_delay_ms(999);
				}
				else if((current_app_tick/APP_TIMER_CONVERTOR) % UPLOAD_APP_TIME == 0 
								&&(current_app_tick/APP_TIMER_CONVERTOR) / UPLOAD_APP_TIME > 0
								) 
				{
					printf("update at : %d\n", current_app_tick/APP_TIMER_CONVERTOR);
						record_data_into_pstorage();
						nrf_delay_ms(999);
				}
				
        power_manage();
    }
}
