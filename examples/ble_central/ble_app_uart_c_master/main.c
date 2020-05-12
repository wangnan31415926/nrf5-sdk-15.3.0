/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//wm
#include "ble_conn_state.h"
#include "app_scheduler.h"

unsigned char DeviceName[]="YASNodic_Uart";
unsigned char Mac_temp[6];
unsigned char uuid_temp[16];

//
extern unsigned char ADDR_wn_[NRF_SDH_BLE_CENTRAL_LINK_COUNT][6];//待过滤mac地址存储位置
extern unsigned char ADDR_counter;

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */
APP_TIMER_DEF(m_test_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_SCAN_timer_id);

BLE_NUS_C_DEF(m_ble_wangnan_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_c,NRF_SDH_BLE_CENTRAL_LINK_COUNT);  //BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_SCAN_DEF(m_scan_6050);                                               /**< Scanning Module instance. */
NRF_BLE_SCAN_DEF(m_scan_GHG);                                               /**< Scanning Module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

unsigned char uartbuf[128];//串口发送缓冲区
unsigned char length=0;   //串口需要发送的数据长度

unsigned char mac_6_buf[6];//mac地址过滤缓冲区

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE_BMA250,
    .type = NUS_SERVICE_UUID_TYPE
};

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid_6050 =
{
    .uuid = BLE_UUID_NUS_SERVICE_MPU6050,
    .type = NUS_SERVICE_UUID_TYPE
};

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid_GHG =
{
    .uuid = BLE_UUID_NUS_SERVICE_GHG,
    .type = NUS_SERVICE_UUID_TYPE
};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             printf("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
				     scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);
		
	  err_code = nrf_ble_scan_filter_set(&m_scan_6050, SCAN_UUID_FILTER, &m_nus_uuid_6050);
	  APP_ERROR_CHECK(err_code);

		err_code = nrf_ble_scan_filter_set(&m_scan_GHG, SCAN_UUID_FILTER, &m_nus_uuid_GHG);
	  APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
	///////////////////////////////////////////////////////////////////////////////////////

}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);//wn
}

void Bluetooth_ReciveANDSend(void * p_event_data, uint16_t event_size)
{
	    for (uint32_t i = 0; i < length; i++)
    {
        app_uart_put(uartbuf[i]);
    }
		length=0;
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 *///蓝牙接收串口发送函数
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len,uint16_t conn_handle)//wn
{
    ret_code_t ret_val;

//    NRF_LOG_DEBUG("Receiving data.");
//    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

	  for(unsigned char i=0;i<16;i++)//准备发送的uuid
	  {
		  uartbuf[i]=m_ble_nus_c[conn_handle].uuid[15-i];
	  }
		length=16;

		memcpy(&uartbuf[length],&m_ble_nus_c[conn_handle].mac[0],6);   //mac地址存入串口发送缓冲区
		length=length+6; 	
	
		memcpy(&uartbuf[length],p_data,data_len);                      //将蓝牙接收的数据存入串口发送缓冲区
		length=length+data_len;                                        //串口数据长度赋值
		
		app_sched_event_put(NULL,NULL, Bluetooth_ReciveANDSend);       //串口发送

}

uint16_t ble_index = 0;
uint8_t  uart_rdata_array[BLE_NUS_MAX_DATA_LEN];//串口接收缓冲区，20字节
unsigned char flag_new_mac=0;                   //有新mac数据需要更新标志位
unsigned char check_mac_id=0;                   //查找到的mac存储位置
//计算校验值
unsigned char crc8_(unsigned char* buf,unsigned char l)
{
	unsigned char i=0;
	unsigned char crc=0;
	for(i=0;i<l;i++)
	{
		crc=crc^buf[i];
	}
	return crc;
}

static unsigned char check_mac_save(unsigned char* buf)
{
  unsigned char i;
  for(i=0;i<ADDR_counter;i++)
  {
    if(memcmp(buf,&ADDR_wn_[i][0],6)==0)
    {
        check_mac_id=i;
        return 0;
    }//已经存有mac地址
  }

  return 1;//没有相同mac地址的存储
}
//存储数据永远是从低到高的地址依次存储
//设置mac数据，此函数只进行缓存数据的处理
unsigned char set_mac(unsigned char* buf)
{
  if(ADDR_counter>NRF_SDH_BLE_CENTRAL_LINK_COUNT||ADDR_counter==NRF_SDH_BLE_CENTRAL_LINK_COUNT)//判断是否还有存储mac的空间
	{
		return 0x67;//没有空间
	}
	
	if(check_mac_save(buf))//查找mac地址是否已经存储
	{//没有存储，进行存储
		memcpy(&ADDR_wn_[ADDR_counter][0],buf,6);
		ADDR_counter++;
	}
	
	return 0x96;//ok
}


//存储数据永远是从低到高的地址依次存储
//删除mac数据，此函数只进行缓存数据的处理
unsigned char clean_mac(unsigned char* buf)
{
	unsigned char i;
	unsigned char data0[]={0,0,0,0,0,0};
	
	if(!check_mac_save(buf))//查找mac地址是否已经存储
	{
	 //有存储，进行删除动作
	 //根据查找到的位置进行删除
	 memset(&ADDR_wn_[check_mac_id][0],0,6);
	 if(memcmp(&ADDR_wn_[check_mac_id+1][0],data0,6)!=0)
	 {
	   memcpy(&ADDR_wn_[check_mac_id][0],&ADDR_wn_[check_mac_id+1][0],6*(ADDR_counter-check_mac_id-1));
	 }
	 ADDR_counter--;		 
	}

	
	return 0x96;//ok
}

/**@brief  串口接收中断
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 *收满20字节进行一次处理解析
 *头 + 指令 + 长度 + 数据 + crc（1字节）
 *a55a  01     6     mac      xx             -----------设置需要过滤的mac地址
 *a55a  02     6     mac      xx             -----------删除指定mac地址
 *a55a  03     \      \       \              -----------获取当前所有过滤mac地址 mac的个数+所有的mac+crc(长度+mac)，如果没有返回 0
 *a55a  04     \      \       \              -----------开启扫描
 *a55a  05     \      \       \              -----------复位设备，可进行初始化配置，所有设置必须在没有进行扫描工作时进行配置
 *                          crc=crc[长度+mac]
 *ack 96 ok/ 67 fail
 *
 *
*/
void uart_event_handle(app_uart_evt_t * p_event)
{
	  unsigned char peer_index=0;
	  unsigned char ok=0x96;
	  unsigned char fail=0x67;

    // uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_rdata_array[ble_index]));//接收1字节串口数据
            ble_index++;
				
				    if(ble_index==1&&uart_rdata_array[0]!=0xa5)                 //判断是否接收到数据头
							{	                                                        //收到乱码，不做处理						
						  ble_index=0;
							break;
						}
				    
				    if (ble_index >= (m_ble_nus_max_data_len))                  //数据是否接收满
						{ 
							ble_index=0;
							if(uart_rdata_array[0]==0xa5&&uart_rdata_array[1]==0x5a)
							{
							  switch(uart_rdata_array[2])
								{
									case 0x01://设置需要过滤的mac地址
												if(1)//(crc8_(&uart_rdata_array[3],7)==uart_rdata_array[10])
												{
                                                    app_uart_put(set_mac(&uart_rdata_array[4]));//校验正确,进行mac设置
												}
												else
												{
													app_uart_put(fail);
												}
									break;
									case 0x02:
												if(1)//(crc8_(&uart_rdata_array[3],7)==uart_rdata_array[10])
												{
													//校验正确,进行mac删除
													app_uart_put(clean_mac(&uart_rdata_array[4]));
												}
												else
												{
													app_uart_put(fail);
												}
									break;
									case 0x03:
                                                uartbuf[0]=ADDR_counter;                                        //多少个mac地址需要过滤
                                                if(ADDR_counter!=0)
										        {
										        	memcpy(&uartbuf[1],ADDR_wn_,6*ADDR_counter);                //mac地址存入串口发送缓冲区
		                                            length=6*ADDR_counter+1;
                                                    uartbuf[length]=crc8_(uartbuf,length);                        //计算校验值
                                                    length++;                                                   //串口数据长度赋值
                                                    app_sched_event_put(NULL,NULL, Bluetooth_ReciveANDSend);    //串口发送//获取当前所有mac地址
                                                }
                                                else
                                                {
                                                    length=1;
                                                    app_sched_event_put(NULL,NULL, Bluetooth_ReciveANDSend); 
                                                }
									break;
									case 0x04://开启扫描工作
										        app_uart_put(ok);
									          //将flash中的mac地址复制到缓存中
									            scan_start();
									break;
									case 0x05://复位
			                                    NVIC_SystemReset();
									break;
								}
							
							break;
							}
								
						
							app_uart_put(fail); 
						}

            break;

//        /**@snippet [Handling data from UART] */
//        case APP_UART_COMMUNICATION_ERROR:
//            NRF_LOG_ERROR("Communication error occurred while handling UART.");
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

        default:
            break;
    }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len,p_ble_nus_c->conn_handle);//wn
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ble_nus_c_init_t init;//wn20190621
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle, NULL);//wn
            APP_ERROR_CHECK(err_code);
            //wn
				    memcpy(m_ble_nus_c[p_ble_evt->evt.gap_evt.conn_handle].mac,Mac_temp,sizeof(Mac_temp));
				    memcpy(m_ble_nus_c[p_ble_evt->evt.gap_evt.conn_handle].uuid,uuid_temp,sizeof(uuid_temp));
						//
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
				    //wn
				    if(ble_conn_state_central_conn_count()!=NRF_SDH_BLE_CENTRAL_LINK_COUNT)
						{
							err_code = app_timer_start(m_SCAN_timer_id, APP_TIMER_TICKS(1000), NULL);
              APP_ERROR_CHECK(err_code);
						}
						//
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
             init.evt_handler = ble_nus_c_evt_handler;

			        err_code = ble_nus_c_init(&m_ble_nus_c[p_gap_evt->conn_handle], &init);
              APP_ERROR_CHECK(err_code);
				
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT://wn
//            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
            break;

        default:
            break;
    }
}

//查看串口有没有争取收到数据
void test_timeout_handler(void * p_context)
{
	 UNUSED_PARAMETER(p_context);
   

}


void SCAN_timeout_handler(void * p_context)
{
		 UNUSED_PARAMETER(p_context);
	   scan_start();
}


/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;
//wn
	  for(unsigned long i=0;i<NRF_SDH_BLE_CENTRAL_LINK_COUNT;i++)
    {
			err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
      APP_ERROR_CHECK(err_code);
		}
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
		   
   	err_code = app_timer_create(&m_test_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                test_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	  err_code = app_timer_create(&m_SCAN_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SCAN_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_test_timer_id, APP_TIMER_TICKS(30), NULL);
    APP_ERROR_CHECK(err_code);

}
/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, NULL, 8);
    APP_ERROR_CHECK(err_code);
//	  ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_db_disc , 8);
//    APP_ERROR_CHECK(err_code);
}/**@brief Function for changing the tx power.
 */
//static void tx_power_set(void)
//{
//    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8);
//    APP_ERROR_CHECK(err_code);
//}
int main(void)
{
	unsigned char address_buf[6]={0x18,0xb3,0x38,0x3b,0x57,0xd6};//{0xd6,0x57,0x3b,0x38,0xb3,0x18};
    // Initialize.
    log_init();
    timer_init();
    uart_init();
	  APP_SCHED_INIT(50,30);  //wn
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    scan_init();

    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    tx_power_set();
//    scan_start();
//    application_timers_start();
    // Enter main loop.
	
   memset(ADDR_wn_,0,sizeof(ADDR_wn_));
	//从flash获取mac相关过滤数据
//	memcpy(ADDR_wn_,address_buf,6);//过滤mac付初值

    for (;;)
    {
			  app_sched_execute();
        idle_state_handle();
    }
}
