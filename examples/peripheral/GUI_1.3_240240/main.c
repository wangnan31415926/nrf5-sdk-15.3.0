/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "UGUI.h"




#define LCD_W 240
#define LCD_H 240
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

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

#define delay_ms nrf_delay_ms
#define u8 unsigned char
#define u16 unsigned short
	
#define X_MAX_PIXEL 128
#define Y_MAX_PIXEL 128

#define USE_HORIZONTAL 1

#define u32 unsigned int
	
//������ɫ
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
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)



































unsigned char display_data[240*240*2];


//��SPI���ߴ���һ��8λ����
/******************************************************************************
      ����˵����LCD��������д�뺯��
      ������ݣ�dat  Ҫд��Ĵ�������
      ����ֵ��  ��
******************************************************************************/
unsigned char buf_send[2];
void LCD_Writ_Bus(u8 dat) 
{	
	buf_send[0]=dat;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf_send, 1, buf_send, 0));
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	OLED_DC_Set();//д����
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	OLED_DC_Set();//д����
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	OLED_DC_Clr();//д����
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	  LCD_WR_REG(0x2a);//�е�ַ����

		LCD_WR_DATA(x1);

		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����

		LCD_WR_DATA(y1);

		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
}


/******************************************************************************
      ����˵����LCD��ʼ������
      ������ݣ���
      ����ֵ��  ��
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
      ����˵����LCD��������
      ������ݣ���
      ����ֵ��  ��
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
	
	OLED_DC_Set();//д����

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


/******************************************************************************
      ����˵����LCD��ʾ����
      ������ݣ�x,y   ��ʼ����
                index ���ֵ����
                size1  �ֺ�
      ����ֵ��  ��
******************************************************************************/
//void LCD_ShowChinese(u16 x,u16 y,u8 index,u8 size1,u16 color)	
//{  

//}

void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//���ù��λ�� 
	LCD_WR_DATA(color);
} 

void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
/******************************************************************************
      ����˵����LCD��һ����ĵ�
      ������ݣ�x,y   ��ʼ����
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawPoint_big(u16 x,u16 y,u16 color)
{
	LCD_Fill(x-1,y-1,x+1,y+1,color);
} 


/******************************************************************************
      ����˵������ָ�����������ɫ
      ������ݣ�xsta,ysta   ��ʼ����
                xend,yend   ��ֹ����
      ����ֵ��  ��
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend,yend);      //���ù��λ�� 
	OLED_DC_Set();//д����
	for(i=0;i<((yend-ysta)*(xend-xsta));i++)
        	{
					  display_data[2*i]=(unsigned char)(color>>8);
            display_data[2*i+1]=(unsigned char)(color&0x00ff);
					}
	for(i=0;i<((yend-ysta)*(xend-xsta)*2)/240;i++)
	{APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*240],240, buf_send, 0));	}
	if(((yend-ysta)*(xend-xsta)*2)%240!=0)
	{
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi,&display_data[i*240],((yend-ysta)*(xend-xsta)*2)%240, buf_send, 0));
	}
	
	
}


/******************************************************************************
      ����˵��������
      ������ݣ�x1,y1   ��ʼ����
                x2,y2   ��ֹ����
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1;
	uRow=x1;//�����������
	uCol=y1;
	if(delta_x>0)incx=1; //���õ������� 
	else if (delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//ˮƽ�� 
	else {incy=-1;delta_y=-delta_x;}
	if(delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//����
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      ����˵����������
      ������ݣ�x1,y1   ��ʼ����
                x2,y2   ��ֹ����
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      ����˵������Բ
      ������ݣ�x0,y0   Բ������
                r       �뾶
      ����ֵ��  ��
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//�ж�Ҫ���ĵ��Ƿ��Զ
		{
			b--;
		}
	}
}

	u8 wn[]={0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00,0x00};/*"8",0*//*"8",0*/
/******************************************************************************
      ����˵������ʾ�ַ� 8*16 ����ɨ�裬��λ��ǰ
      ������ݣ�x,y    �������
                num    Ҫ��ʾ���ַ�
                mode   1���ӷ�ʽ  0�ǵ��ӷ�ʽ
      ����ֵ��  ��
******************************************************************************/
#define char_w 22
#define char_h 36
void LCD_ShowChar(u16 x,u16 y,u8* data,u8 mode,u16 color)
{
    u8 temp;
    u8 pos,t;
	  u16 x0=x;    
	


	LCD_Address_Set(x,y,x+char_w-1,y+char_h-1);      //���ù��λ�� 

	if(!mode) //�ǵ��ӷ�ʽ
	{
		for(pos=0;pos<char_h;pos++)
		{ 
			temp=(unsigned char)data[pos];		 //����1608����
			for(t=0;t<char_w;t++)
		    {                 
		        if(temp&0x01)LCD_WR_DATA(color);
				else LCD_WR_DATA(BLACK);
				temp>>=1; 
//				x++;
		    }
//			x=x0;
//			y++;
		}	
	}else//���ӷ�ʽ
	{
		for(pos=0;pos<char_h;pos++)
		{
		    temp=(unsigned char)data[pos];		 //����1608����
			for(t=0;t<char_w;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos,color);//��һ����     
		        temp>>=1; 
		    }
		}
	}    	   	 	  
}


/******************************************************************************
      ����˵������ʾ�ַ���
      ������ݣ�x,y    �������
                *p     �ַ�����ʼ��ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 color)
{         
//    while(*p!='\0')
//    {       
//        if(x>LCD_W-16){x=0;y+=16;}
//        if(y>LCD_H-16){y=x=0;LCD_Clear(RED);}
//        LCD_ShowChar(x,y,*p,0,color);
//        x+=8;
//        p++;
//    }  
}


/******************************************************************************
      ����˵������ʾ����
      ������ݣ�m������nָ��
      ����ֵ��  ��
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}


///******************************************************************************
//      ����˵������ʾ����
//      ������ݣ�x,y    �������
//                num    Ҫ��ʾ������
//                len    Ҫ��ʾ�����ָ���
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowNum(u16 x,u16 y,u16 num,u8 len,u16 color)
//{         	
//	u8 t,temp;
//	u8 enshow=0;
//	for(t=0;t<len;t++)
//	{
//		temp=(num/mypow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				LCD_ShowChar(x+8*t,y,' ',0,color);
//				continue;
//			}else enshow=1; 
//		 	 
//		}
//	 	LCD_ShowChar(x+8*t,y,temp+48,0,color); 
//	}
//} 


///******************************************************************************
//      ����˵������ʾС��
//      ������ݣ�x,y    �������
//                num    Ҫ��ʾ��С��
//                len    Ҫ��ʾ�����ָ���
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowNum1(u16 x,u16 y,float num,u8 len,u16 color)
//{         	
//	u8 t,temp;
//	u8 enshow=0;
//	u16 num1;
//	num1=num*100;
//	for(t=0;t<len;t++)
//	{
//		temp=(num1/mypow(10,len-t-1))%10;
//		if(t==(len-2))
//		{
//			LCD_ShowChar(x+8*(len-2),y,'.',0,color);
//			t++;
//			len+=1;
//		}
//	 	LCD_ShowChar(x+8*t,y,temp+48,0,color);
//	}
//}


/******************************************************************************
      ����˵������ʾ40x40ͼƬ
      ������ݣ�x,y    �������
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowPicture(u16 x1,u16 y1,u16 x2,u16 y2,const unsigned char*image) 
{
	int i=0;
	  LCD_Address_Set(0,0,x2,y2);
 
	memcpy(display_data,image,128*128*2);
	
	OLED_DC_Set();//д����

		for(i=0;i<128*2;i++)
        	{APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*128], 128, buf_send, 0));	}		

}

void LCD_ShowPicture_240(u16 x1,u16 y1,u16 x2,u16 y2,const unsigned char*image,unsigned char xy) 
{
	int i=0;
	  LCD_Address_Set(0,0,x2,y2);
 
	memcpy(display_data,image,xy*xy*2);
	
	OLED_DC_Set();//д����

		for(i=0;i<xy*2;i++)
        	{APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &display_data[i*xy], xy, buf_send, 0));	}		

}


#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111















//����λ�û�һ����
void UserPixelSetFunction( UG_S16 x, UG_S16 y,UG_COLOR c)
{
	
		unsigned char buf_send[1];

		OLED_DC_Clr();//д����
		buf_send[0]=0x2a;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf_send, 1, buf_send, 0));


		OLED_DC_Set();//д����
	  LCD_Writ_Bus(x>>8);
	  LCD_Writ_Bus(x);

	  LCD_Writ_Bus(x>>8);
	  LCD_Writ_Bus(x);

	
		OLED_DC_Clr();//д����
		buf_send[0]=0x2b;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf_send, 1, buf_send, 0));

		OLED_DC_Set();//д����
	  LCD_Writ_Bus(y>>8);
	  LCD_Writ_Bus(y);

	  LCD_Writ_Bus(y>>8);
	  LCD_Writ_Bus(y);
		
	  OLED_DC_Clr();//д����
		buf_send[0]=0x2c;
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf_send, 1, buf_send, 0));
	
	

	   OLED_DC_Set();//д����
	   LCD_Writ_Bus(c>>8);
	   LCD_Writ_Bus(c);
	
	
	
	
	
	
	
	
	
//// Your code....
//	LCD_Address_Set(x,y,x,y);//���ù��λ�� 
//	LCD_WR_DATA(c);

}



extern const unsigned char font_22x36[256][108];
UG_GUI gui; // Global GUI structure
int main(void)
{
	bsp_board_init(BSP_INIT_LEDS);
//gpio_output_voltage_setup();
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
	  spi_config.mode=NRF_DRV_SPI_MODE_3;
	
	spi_config.frequency=NRF_SPIM_FREQ_8M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));

  	nrf_gpio_cfg_output(dc_pin);//8
	  nrf_gpio_pin_set(dc_pin);
		nrf_gpio_cfg_output(res_pin);//7
	  nrf_gpio_pin_clear(res_pin);

	  Lcd_Init();			     //��ʼ��OLED  
	
	 LCD_Clear(RED);
	  UG_Init(&gui , UserPixelSetFunction , 240 , 240) ;
//	  UG_FillScreen(BLUE);
	UG_DrawCircle(128,128,50,RED);
	UG_FontSelect( FONT_22X36 ) ;
UG_SetBackcolor(BLACK ) ;
UG_SetForecolor (WHITE ) ;
delay_ms(1000);
UG_PutString(120,120,"abcde") ;
	delay_ms(1000);
	
	for(char i=0;i<8;i++)
	{LCD_ShowChar(20+i*22,200,(unsigned char*)&font_22x36[0x7f][0],0,WHITE);}

//	  LCD_Clear(RED);
//	Draw_Circle(128,128,50,GREEN);
//	Draw_Circle(128,128,49,GREEN);
//	Draw_Circle(128,128,48,GREEN);
//	Draw_Circle(128,128,47,GREEN);
//	Draw_Circle(128,128,46,GREEN);
//	LCD_ShowChar(120,120,wn,0,WHITE);
    while (1)
    {	
			
			
			delay_ms(1000);
			
				LCD_Clear(BLUE);
			delay_ms(1000);
				LCD_Clear(RED);
			
			
			
			
			
//			delay_ms(1000);
//			LCD_Fill(28,122,120,230,RED);
//			LCD_DrawRectangle(28, 122, 120, 230,RED);
//			Draw_Circle(128,128,50,RED);
//           LCD_DrawRectangle(28, 28, 120, 120,BLUE);
//			
//			delay_ms(1000);
//			LCD_DrawRectangle(28, 28, 120, 120,RED);
//			Draw_Circle(128,128,50,GREEN);
//			delay_ms(1000);
//			LCD_Fill(28,122,120,230,GREEN);
//			LCD_DrawRectangle(28, 122, 120, 230,BLACK);
//			delay_ms(1000);
//			LCD_DrawPoint_big(0,120,YELLOW);
    }
}
