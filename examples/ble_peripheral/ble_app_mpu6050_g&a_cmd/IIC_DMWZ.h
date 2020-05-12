#ifndef _IIC_DMWZ_H_
#define _IIC_DMWZ_H_

//from register read len datas save to buffer 
unsigned char i2c_read(unsigned char device_id, unsigned char reg_address, unsigned char *buffer, unsigned char len);
//to client register send datas  
unsigned char i2c_write(unsigned char device_id, unsigned char reg_address, unsigned char* data, unsigned char len);

#endif
