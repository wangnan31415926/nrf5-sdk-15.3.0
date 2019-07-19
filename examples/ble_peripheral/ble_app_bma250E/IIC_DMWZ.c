// <0=> 0 (P0.0) 
// <1=> 1 (P0.1) 
// <2=> 2 (P0.2) 
// <3=> 3 (P0.3) 
// <4=> 4 (P0.4) 
// <5=> 5 (P0.5) 
// <6=> 6 (P0.6) 
// <7=> 7 (P0.7) 
// <8=> 8 (P0.8) 
// <9=> 9 (P0.9) 
// <10=> 10 (P0.10) 
// <11=> 11 (P0.11) 
// <12=> 12 (P0.12) 
// <13=> 13 (P0.13) 
// <14=> 14 (P0.14) 
// <15=> 15 (P0.15) 
// <16=> 16 (P0.16) 
// <17=> 17 (P0.17) 
// <18=> 18 (P0.18) 
// <19=> 19 (P0.19) 
// <20=> 20 (P0.20) 
// <21=> 21 (P0.21) 
// <22=> 22 (P0.22) 
// <23=> 23 (P0.23) 
// <24=> 24 (P0.24) 
// <25=> 25 (P0.25) 
// <26=> 26 (P0.26) 
// <27=> 27 (P0.27) 
// <28=> 28 (P0.28) 
// <29=> 29 (P0.29) 
// <30=> 30 (P0.30) 
// <31=> 31 (P0.31) 
// <32=> 32 (P1.0) 
// <33=> 33 (P1.1) 
// <34=> 34 (P1.2) 
// <35=> 35 (P1.3) 
// <36=> 36 (P1.4) 
// <37=> 37 (P1.5) 
// <38=> 38 (P1.6) 
// <39=> 39 (P1.7) 
// <40=> 40 (P1.8) 
// <41=> 41 (P1.9) 
// <42=> 42 (P1.10) 
// <43=> 43 (P1.11) 
// <44=> 44 (P1.12) 
// <45=> 45 (P1.13) 
// <46=> 46 (P1.14) 
// <47=> 47 (P1.15) 
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "IIC_DMWZ.h"

#define SDA 12
#define SCL 13
#define DELAY 10
#define DELAY_1_2 5

unsigned char send_byte;

void SCL_(void)
{
	unsigned char i=200;
	 nrf_gpio_cfg_output(SCL); 
	 nrf_gpio_cfg_output(SDA);
	 nrf_gpio_pin_set(SDA);//gpio_direction_output(SDA, 1); 
	while(i)
	{
		nrf_gpio_pin_toggle(SCL);
		nrf_delay_us(DELAY);
		i--;
		
	}
}

void i2c_start(void)  
{       
  
    nrf_gpio_cfg_output(SCL); 
    nrf_gpio_cfg_output(SDA);
	  nrf_gpio_pin_set(SDA);//gpio_direction_output(SDA, 1); 
    nrf_gpio_pin_set(SCL);//gpio_direction_output(SCL, 1);	
    nrf_delay_us(DELAY);
  
      
    nrf_gpio_pin_clear(SDA);//gpio_set_value(SDA, 0);  
    nrf_delay_us(DELAY);      
            
    nrf_gpio_pin_clear(SCL);//gpio_set_value(SCL, 0);  
    nrf_delay_us(DELAY);  
	
	
}  
void i2c_stop(void)  
{   
    nrf_gpio_pin_clear(SCL);//gpio_set_value(SCL, 0);  
    nrf_gpio_pin_clear(SDA);//gpio_set_value(SDA, 0);  
    nrf_delay_us(DELAY);  
      
    nrf_gpio_pin_set(SCL);//gpio_set_value(SCL, 1);  
    nrf_delay_us(DELAY);  
    nrf_gpio_pin_set(SDA);//gpio_set_value(SDA, 1);  
    nrf_delay_us(DELAY);  
}  
void i2c_send_ack(unsigned char ack)  
{  
	  nrf_gpio_cfg_output(SDA);
    if(ack)
		{nrf_gpio_pin_set(SDA);}			
    else  
		{nrf_gpio_pin_clear(SDA);}			
    nrf_delay_us(DELAY);  
      
    nrf_gpio_pin_set(SCL);  
    nrf_delay_us(DELAY);  
      
    nrf_gpio_pin_clear(SCL);  
    nrf_delay_us(DELAY);    
}  
unsigned char i2c_receive_ack(void)  
{  
    unsigned char rc = 0;  
 
    nrf_gpio_cfg_input(SDA, NRF_GPIO_PIN_NOPULL); //set SDA input   
    nrf_gpio_pin_set(SCL);                        //set SCL output 1
    nrf_delay_us(DELAY);  
      
    if(nrf_gpio_pin_read(SDA)) {                  //read SDA for ack
        rc = 1;  
    }  
    nrf_gpio_pin_clear(SCL);   //set SCL output 0
    nrf_gpio_cfg_output(SDA);  
//    nrf_gpio_pin_set(SDA);		 //set SDA output 1
    return rc;  
}  
unsigned char i2c_send_byte(void)  
{  
    unsigned char rc = 0;  
    unsigned char out_mask = 0x80;  
    unsigned char value;  
    unsigned char count = 8;  
    out_mask = 0x80;
    while(count > 0) {                  
        value = ((send_byte & out_mask) ? 1 : 0);     
        if (value == 1) {                                     
            nrf_gpio_pin_set(SDA);   //set SDA output 1      
        }      
        else {                                    
            nrf_gpio_pin_clear(SDA); //set SDA output 0  
        }      
        nrf_delay_us(DELAY_1_2);  
                                    
        nrf_gpio_pin_set(SCL);       //set SCL output 1                 
        nrf_delay_us(DELAY);  
        nrf_gpio_pin_clear(SCL);     //set SCL output 0   
        nrf_delay_us(DELAY_1_2);  
                    
        out_mask >>= 1;        
        count--;         
    }  
		nrf_gpio_pin_set(SDA); 
    nrf_delay_us(DELAY_1_2);
     
    rc = i2c_receive_ack();  
    return rc;  
}

void i2c_read_byte(unsigned char *buffer, unsigned char ack)  
{  
    unsigned char count = 0x08;  
    unsigned char data = 0x00;  
    unsigned char temp = 0;  
      
    nrf_gpio_cfg_input(SDA, NRF_GPIO_PIN_NOPULL);//SET SDA input    
    while(count > 0) {  
        nrf_gpio_pin_set(SCL);             //SCL 1
        nrf_delay_us(DELAY);  
        temp = nrf_gpio_pin_read(SDA);     //read SDA   
        data <<= 1;  
        if (temp)  
            data |= 0x01;  
        nrf_gpio_pin_clear(SCL);           //SCL 0
        nrf_delay_us(DELAY);  
        count--;  
    }  
    i2c_send_ack(ack);//0 = ACK    1 = NACK   
    *buffer = data;           
}  

//to client register send datas  
unsigned char i2c_write(unsigned char device_id, unsigned char reg_address, unsigned char* data, unsigned char len)  
{  
    unsigned char rc = 0;  
    unsigned char i;  
    i2c_start();    
   send_byte=(device_id << 1) | 0x00;
     i2c_send_byte();  //rc |= i2c_send_byte( (device_id << 1) | 0x00 );  
	send_byte=reg_address;
     i2c_send_byte();  //rc |= i2c_send_byte(reg_address);  
    if(0==len) {  
        i2c_stop();  
        return rc;  
    }  
          
    for(i=0; i<len; i++) { 
send_byte=*data;			
        i2c_send_byte();//rc |= i2c_send_byte(*data);  
        data++;  
    }  
      
    i2c_stop();       
    if(rc) {  
//        printk("ERROR!  ssd2531_i2c_write failed/n");  
    }  
    return rc;    
}  
    unsigned char rc_n = 0;  
//from register read len datas save to buffer 
unsigned char i2c_read(unsigned char device_id, unsigned char reg_address, unsigned char *buffer, unsigned char len)  
{  
    unsigned char rc = 1;  
    unsigned char i;  

    i2c_start(); 
     send_byte=(device_id << 1) | 0x00;	
     i2c_send_byte();//rc_n |= i2c_send_byte( (device_id << 1) | 0x01 );
     nrf_delay_us(DELAY); 

	
	  send_byte=reg_address;
    i2c_send_byte();//rc_n |= i2c_send_byte(reg_address); 
	
    i2c_start();//restart I2C 
     send_byte=(device_id << 1) | 0x01;	
     i2c_send_byte();//rc_n |= i2c_send_byte( (device_id << 1) | 0x01 );
	
//    rc_n |= i2c_send_byte( (device_id << 1) | 0x01 );  
      
    for(i=0;i<len;i++) {  
        i2c_read_byte(buffer++, !(len-i-1));//when read one byte need to ack ,but when read the last data need an NACK  
    }  
      
    i2c_stop();   
    if(rc_n) {  
//        printk("ERROR!   ssd2531_i2c_read failed/n");  
        return rc_n;  
    }  
    return rc_n;    
}