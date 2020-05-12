/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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
#include "nrfx_spim.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define NRFX_SPIM_SCK_PIN  26//3
#define NRFX_SPIM_MOSI_PIN 29//4
#define NRFX_SPIM_MISO_PIN 30//28
#define NRFX_SPIM_SS_PIN   31//29
#define NRFX_SPIM_DCX_PIN  8//30


#define delay_ms nrf_delay_ms
#define u8 unsigned char
#define u16 unsigned short
	
#define X_MAX_PIXEL 128
#define Y_MAX_PIXEL 128

#define USE_HORIZONTAL 1

#define u32 unsigned int
	
//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F //////////////////////////// 
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800///////////////////////////////
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0/////////////////////////////
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

#define LCD_W 240
#define LCD_H 240

#define res_pin 7
#define dc_pin 8

#define LCD_SDA_SET nrf_gpio_pin_set(SPI_MOSI_PIN)
#define LCD_SDA_CLR nrf_gpio_pin_clear(SPI_MOSI_PIN)

#define LCD_SCL_CLR nrf_gpio_pin_clear(SPI_SCK_PIN)
#define LCD_SCL_SET nrf_gpio_pin_set(SPI_SCK_PIN)

#define LCD_CS_CLR nrf_gpio_pin_clear(SPI_SS_PIN)
#define LCD_CS_SET nrf_gpio_pin_set(SPI_SS_PIN)

#define LCD_RS_CLR nrf_gpio_pin_clear(dc_pin)
#define LCD_RS_SET nrf_gpio_pin_set(dc_pin)

#define LCD_RST_CLR nrf_gpio_pin_clear(res_pin)
#define LCD_RST_SET nrf_gpio_pin_set(res_pin)


#define OLED_CS_Clr() LCD_CS_CLR
#define OLED_CS_Set() LCD_CS_SET
#define OLED_SCLK_Clr() LCD_SCL_CLR
#define OLED_SCLK_Set() LCD_SCL_SET
#define OLED_SDIN_Set() LCD_SDA_SET
#define OLED_SDIN_Clr() LCD_SDA_CLR
#define OLED_DC_Set() LCD_RS_SET
#define OLED_DC_Clr() LCD_RS_CLR
#define OLED_RST_Clr() LCD_RST_CLR
#define OLED_RST_Set() LCD_RST_SET

//////////////////////////////////////////////////////////////////////////////////////////////////////

#define SPI_INSTANCE  3                                           /**< SPI instance index. */
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic123456789012345678901234567890"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];  /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */






unsigned char display_data[240*240*2];


//向SPI总线传输一个8位数据
/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
unsigned char buf_send[2];
void LCD_Writ_Bus(u8 dat) 
{	
	buf_send[0]=dat;
	NRFX_SPIM_XFER_TX( buf_send, 1);
		//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf_send, 1, buf_send, 0));
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	OLED_DC_Set();//写数据
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	OLED_DC_Set();//写数据
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	OLED_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	  LCD_WR_REG(0x2a);//列地址设置

		LCD_WR_DATA(x1);

		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置

		LCD_WR_DATA(y1);

		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
}


/******************************************************************************
      函数说明：LCD初始化函数
      入口数据：无
      返回值：  无
******************************************************************************/
#define Write_Register LCD_WR_REG
#define Write_Parameter LCD_WR_DATA8
#define Delay delay_ms
void Lcd_Init(void)
{
	OLED_RST_Clr();
	delay_ms(500);
	OLED_RST_Set();
	delay_ms(500);
	
Write_Register(0x36); 
Write_Parameter(0x00);

Write_Register(0x3A); 
Write_Parameter(0x05);

Write_Register(0xB2);
Write_Parameter(0x0C);
Write_Parameter(0x0C);
Write_Parameter(0x00);
Write_Parameter(0x33);
Write_Parameter(0x33);

Write_Register(0xB7); 
Write_Parameter(0x35);  

Write_Register(0xBB);
Write_Parameter(0x19);

Write_Register(0xC0);
Write_Parameter(0x2C);

Write_Register(0xC2);
Write_Parameter(0x01);

Write_Register(0xC3);
Write_Parameter(0x12);   

Write_Register(0xC4);
Write_Parameter(0x20);  

Write_Register(0xC6); 
Write_Parameter(0x0F);    

Write_Register(0xD0); 
Write_Parameter(0xA4);
Write_Parameter(0xA1);

Write_Register(0xE0);
Write_Parameter(0xD0);
Write_Parameter(0x04);
Write_Parameter(0x0D);
Write_Parameter(0x11);
Write_Parameter(0x13);
Write_Parameter(0x2B);
Write_Parameter(0x3F);
Write_Parameter(0x54);
Write_Parameter(0x4C);
Write_Parameter(0x18);
Write_Parameter(0x0D);
Write_Parameter(0x0B);
Write_Parameter(0x1F);
Write_Parameter(0x23);

Write_Register(0xE1);
Write_Parameter(0xD0);
Write_Parameter(0x04);
Write_Parameter(0x0C);
Write_Parameter(0x11);
Write_Parameter(0x13);
Write_Parameter(0x2C);
Write_Parameter(0x3F);
Write_Parameter(0x44);
Write_Parameter(0x51);
Write_Parameter(0x2F);
Write_Parameter(0x1F);
Write_Parameter(0x1F);
Write_Parameter(0x20);
Write_Parameter(0x23);

Write_Register(0x21); 

Write_Register(0x11); 
Delay (120); 

Write_Register(0x29); 
	
}

/******************************************************************************
      函数说明：LCD清屏函数
      入口数据：无
      返回值：  无
******************************************************************************/
void LCD_Clear(u16 Color)
{
	u16 i,j;  	
	  LCD_Address_Set(0,0,LCD_W-1,LCD_H-1);

		buf_send[0]=(unsigned char)(Color>>8);
	buf_send[1]=(unsigned char)Color;
	
	for(i=0;i<240*240;i++)
	{
		display_data[2*i]=(unsigned char)(Color>>8);
		display_data[2*i+1]=(unsigned char)(Color);
	}	
	
	OLED_DC_Set();//写数据

//	for(i=0;i<240*2;i++)
//        	{
//					  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*240], 240, buf_send, 0));	
//					
//					}		 
	
	
	for(i=0;i<240*240*2/65500;i++)
        	{
					  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*65500], 65500, buf_send, 0));	
					
					}		
					APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*65500], (240*240*2)%65500, buf_send, 0));	
}









void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, m_length, m_rx_buf, m_length);

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_32M;
    spi_config.ss_pin         = NRFX_SPIM_SS_PIN;
    spi_config.miso_pin       = NRFX_SPIM_MISO_PIN;
    spi_config.mosi_pin       = NRFX_SPIM_MOSI_PIN;
    spi_config.sck_pin        = NRFX_SPIM_SCK_PIN;
    //spi_config.dcx_pin        = NRFX_SPIM_DCX_PIN;
    spi_config.use_hw_ss      = true;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));

    NRF_LOG_INFO("NRFX SPIM example started.");

    while (1)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrfx_spim_xfer_dcx(&spi, &xfer_desc, 0, 15));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
