#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "tm7711.h"

#define CH1_10HZ 0x01
#define CH1_40HZ 0x02
#define CH2_TEMP 0x03

#define CH1_10HZ_CLK 25
#define CH2_40HZ_CLK 27
#define CH2_TEMP_CLK 26

//#define DOUT_7711   30//sbit DOUT_7711=P2^1;
//#define PD_SCK_7711 28//sbit PD_SCK_7711=P2^0;

 
 
// void nrf_delay_us(unsigned int n) //n=1时约1us
// {
//    while(--n);
// }

void GPIO_tm7711_init(void)
{
 nrf_gpio_cfg_output(PD_SCK_7711);
 nrf_gpio_cfg_input(DOUT_7711,NRF_GPIO_PIN_NOPULL);//NRF_GPIO_PIN_PULLUP);    	
}

unsigned long Read_TM7711(unsigned char next_select)
{
   unsigned char i=0;
   unsigned long data_temp=0;
   for(i=0;i<24;i++)
   {
      nrf_gpio_pin_set(PD_SCK_7711);                        //PD_SCK_7711=1;
      data_temp<<=1;
	    nrf_delay_us(1);
	    if(nrf_gpio_pin_read(DOUT_7711))//(DOUT_7711==1)
	    {
	     data_temp=data_temp+1;
	    }
   	   
	   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0;
	   nrf_delay_us(8);
   }
	 switch(next_select)
	 {
	    case CH1_10HZ:
		     nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
			   nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0;
			  
			 break;
		case CH1_40HZ:
		     nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
			   nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0; 
	 		   nrf_delay_us(1);
			   nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
			   nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0; 
	 		   nrf_delay_us(1);
			   nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
		     nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0;
			 break; 
	    case CH2_TEMP:
		     nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
			   nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0; 
	 		   nrf_delay_us(1);
			   nrf_gpio_pin_set(PD_SCK_7711);//PD_SCK_7711=1;
		     nrf_delay_us(1);
			   nrf_gpio_pin_clear(PD_SCK_7711);//PD_SCK_7711=0;
			 break;
	   default:
	         break;		  
	 
	 }
	 return data_temp;
} 	



//void main()				  //main.c
//{

//	unsigned long a1;
//	unsigned char a,b,c;
//	nrf_delay_us(100);
//	UART_init();
//	Read_TM7711(0x01);
//	 	
// while(1)
// {
//  	 
//	 while(DOUT_7711==1);
//  	    
//         a1=Read_TM7711(0x01);
//		 
//	 a=a1/65536;		//高8位
//	
//	 b=a1/256%256;
//	
//	 c=a1%256;
//		
//	  if(a/16>=10)
//	  send_char(a/16+55) ;
//	  else
//	  send_char(a/16+48);

//	  if(a%16>=10)
//	  send_char(a%16+55) ;
//	  else
//	  send_char(a%16+48) ;

//	  if(b/16>=10)
//	  send_char(b/16+55) ;
//	  else
//	  send_char(b/16+48);

//	  if(b%16>=10)
//	  send_char(b%16+55) ;
//	  else
//	  send_char(b%16+48) ;

//	  if(c/16>=10)
//	  send_char(c/16+55) ;
//	  else
//	  send_char(c/16+48);

//	  if(c%16>=10)
//	  send_char(c%16+55) ;
//	  else
//	  send_char(c%16+48) ;	 
//   	
//	  send_char(' ');  //空格
//	  send_char(' ');
//	 
//   
//   }
// 
// }	 
//	 		

//	  



