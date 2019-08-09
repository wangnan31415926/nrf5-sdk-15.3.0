#include "IIC_DMWZ.h"
#include "bma.h"
#include "nrf_delay.h"

bma255_t bma255;
bma255_t * p_bma255;
#define deviceaddress 0x18


unsigned char V_BMA255FIFOData_U8R[6] = {0,0,0,0,0,0}; //传感器数据存储
/*******************************************************************************
* Function Name  : LIS3DH_WriteReg
* Description  : Generic Writing function. It must be fullfilled with either
*   : I2C or I2C writing function
* Input   : Register Address, Data to be written
* Output  : None
* Return  : 0 success,1 fail
*******************************************************************************/
unsigned char I2CMasterBuffer[1];
unsigned char BMA250E_WriteReg(unsigned char WriteAddr,unsigned char  Data)
{
    unsigned char i=5;
    unsigned char I2CMasterBuffer[1];
    I2CMasterBuffer[0] = Data;
	  
	  while(i)
		{
			if(0==i2c_write(deviceaddress,WriteAddr,I2CMasterBuffer,1))
			{
			  i=0;
			  return 0;
			}
			else
			{i--;}
		}
		
		return 1;
//		i2c_write(deviceaddress,WriteAddr,I2CMasterBuffer,1);
//	  while((i2c_write(deviceaddress,WriteAddr,I2CMasterBuffer,1)==0)&&(i--));
//    return 0;//i2c_write(deviceaddress,WriteAddr,I2CMasterBuffer,1);
}
/*******************************************************************************
* Function Name  : LIS3DH_ReadReg
* Description  : Generic Reading function. It must be fullfilled with either
*   : I2C or SPI reading functions     
* Input   : Register Address
* Output  : Data REad
* Return  : 0 success,1 fail
*******************************************************************************/
unsigned char BMA250E_ReadReg(unsigned char Reg, unsigned char* Data)
{
	  unsigned char i=5;
	   while(i)
		{
			if(0==i2c_read(deviceaddress,Reg,Data,1))
			{ i=0;
			  return 0;
			}
			else
			{i--;}
		}
//	i2c_read(deviceaddress,Reg,Data,1);
    return 1;//i2c_read(deviceaddress,Reg,Data,1);
}

/*******************************************************************************
 * Description: *//**\brief
 *                      This function is used for the soft reset
 *     The soft reset register will be written with 0xB6.
 *
 *
 *
 *  \param None
 *
 *
 *
 *  \return Communication results.
 *
 *
 ******************************************************************************/
void bma250_soft_reset(void) 
   {

     BMA250E_WriteReg(BMA255_RESET_REG,0xB6);/* To reset the sensor 0xB6 value will be written */

   }
/*******************************************************************************
 * Description: *//**\brief 
 *        This function initialises the structure pointer and assigns the I2C address.
 *                              
 *
 *
 *
 *
 *  \param  bma255_t *bma255 structure pointer.
 *
 *
 *
 *  \return communication results.
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma255_init(bma255_t *bma255)
   {
   int comres= C_Zero_BMA_U8X ;
//   unsigned char data;
      
   p_bma255 = bma255;  
   	                                                                           /* assign bma255 ptr */
//   comres = BMA250E_ReadReg(BMA255_CHIP_ID__REG,&data);

//   
//   p_bma255->chip_id = data ;                                          /* get bitslice */
//   
//   comres += BMA250E_ReadReg(BMA255_VERSION_REG,&data);

//   p_bma255->version = BMA255_GET_BITSLICE(data, BMA255_VERSION);     /* get the revision id */
   return comres;
   }

/*******************************************************************************
 * Description: *//**\brief This API is used to set Bandwidth of the sensor
 *
 *
 *
 *
 *  \param unsigned char BW
 *                       8 -> 7.81HZ
 *                       9 -> 15.63HZ
 *                      10 -> 31.25HZ
 *                      11 -> 62.50HZ
 *                      12 -> 125HZ
 *                      13 -> 250HZ
 *                      14 -> 500HZ
 *                      15 -> 1000HZ 
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma255_set_bandwidth(unsigned char BW)
   {
   int comres = C_Zero_BMA_U8X ;
   unsigned char data;
   int Bandwidth ;
   if (p_bma255 == C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (p_bma255->chip_id == 0xFB)
         {
         if (BW > C_Seven_BMA_U8X && BW < C_Fifteen_BMA_U8X)
            {
            switch (BW)
               {
               case BMA255_BW_7_81HZ:
                  Bandwidth = BMA255_BW_7_81HZ;
                  
                  /*  7.81 Hz      64000 uS   */
                  break;
               case BMA255_BW_15_63HZ:
                  Bandwidth = BMA255_BW_15_63HZ;
                  
                  /*  15.63 Hz     32000 uS   */
                  break;
               case BMA255_BW_31_25HZ:
                  Bandwidth = BMA255_BW_31_25HZ;
                  
                  /*  31.25 Hz     16000 uS   */
                  break;
               case BMA255_BW_62_50HZ:
                  Bandwidth = BMA255_BW_62_50HZ;
                  
                  /*  62.50 Hz     8000 uS   */
                  break;
               case BMA255_BW_125HZ:
                  Bandwidth = BMA255_BW_125HZ;
                  
                  /*  125 Hz       4000 uS   */
                  break;
               case BMA255_BW_250HZ:
                  Bandwidth = BMA255_BW_250HZ;
                  
                  /*  250 Hz       2000 uS   */
                  break;
               case BMA255_BW_500HZ:
                  Bandwidth = BMA255_BW_500HZ;
                  
                  /*  500 Hz       1000 uS   */
                  break;
               default:
                  break;
               }
            comres = BMA250E_ReadReg(BMA255_BANDWIDTH__REG,&data);
							 //p_bma255->BMA255_BUS_READ_FUNC(p_bma255->dev_addr, BMA255_BANDWIDTH__REG, &data, C_One_BMA_U8X );
            data = BMA255_SET_BITSLICE(data, BMA255_BANDWIDTH, Bandwidth );
            comres += BMA250E_WriteReg(BMA255_BANDWIDTH__REG,data);
							 //p_bma255->BMA255_BUS_WRITE_FUNC(p_bma255->dev_addr, BMA255_BANDWIDTH__REG, &data, C_One_BMA_U8X );
            }
         else
            {
            comres = E_OUT_OF_RANGE ;
            }
         }
      else
         {
         if (BW > C_Seven_BMA_U8X && BW < C_Sixteen_BMA_U8X)
            {
            switch (BW)
               {
               case BMA255_BW_7_81HZ:
                  Bandwidth = BMA255_BW_7_81HZ;
                  
                  /*  7.81 Hz      64000 uS   */
                  break;
               case BMA255_BW_15_63HZ:
                  Bandwidth = BMA255_BW_15_63HZ;
                  
                  /*  15.63 Hz     32000 uS   */
                  break;
               case BMA255_BW_31_25HZ:
                  Bandwidth = BMA255_BW_31_25HZ;
                  
                  /*  31.25 Hz     16000 uS   */
                  break;
               case BMA255_BW_62_50HZ:
                  Bandwidth = BMA255_BW_62_50HZ;
                  
                  /*  62.50 Hz     8000 uS   */
                  break;
               case BMA255_BW_125HZ:
                  Bandwidth = BMA255_BW_125HZ;
                  
                  /*  125 Hz       4000 uS   */
                  break;
               case BMA255_BW_250HZ:
                  Bandwidth = BMA255_BW_250HZ;
                  
                  /*  250 Hz       2000 uS   */
                  break;
               case BMA255_BW_500HZ:
                  Bandwidth = BMA255_BW_500HZ;
                  
                  /*  500 Hz       1000 uS   */
                  break;
               case BMA255_BW_1000HZ:
                  Bandwidth = BMA255_BW_1000HZ;
                  
                  /*  1000 Hz      500 uS   */
                  break;
               default:
                  break;
               }
            comres = BMA250E_ReadReg(BMA255_BANDWIDTH__REG,&data);
							 //p_bma255->BMA255_BUS_READ_FUNC(p_bma255->dev_addr, BMA255_BANDWIDTH__REG, &data, C_One_BMA_U8X );
            data = BMA255_SET_BITSLICE(data, BMA255_BANDWIDTH, Bandwidth );
            comres += BMA250E_WriteReg(BMA255_BANDWIDTH__REG,data);
							 //p_bma255->BMA255_BUS_WRITE_FUNC(p_bma255->dev_addr, BMA255_BANDWIDTH__REG, &data, C_One_BMA_U8X );
            }
         else
            {
            comres = E_OUT_OF_RANGE ;
            }
         }
      }
   return comres;
   }
	 
	 
	 
/***************************************************************************************
 * Description: *//**\brief This API is used to set the operating Modes of the sensor
 *
 *
 *
 *
 *  \param unsigned char Mode
 *                       0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 *  \return communication results
 *
 *
 ***************************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
unsigned char bma255_set_mode(unsigned char Mode)
   {
   int comres=C_Zero_BMA_U8X ;
   unsigned char data1,data2;
   if (p_bma255 == C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (Mode < C_Six_BMA_U8X)
         {
         BMA250E_ReadReg (BMA255_MODE_CTRL_REG, &data1);
				 
         BMA250E_ReadReg(BMA255_LOW_NOISE_CTRL_REG, &data2);
         switch (Mode)
            {
            case BMA255_MODE_NORMAL:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_Zero_BMA_U8X);                       
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_Zero_BMA_U8X);
               break;
            case BMA255_MODE_LOWPOWER1:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_Two_BMA_U8X);
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_Zero_BMA_U8X);
               break;
            case BMA255_MODE_SUSPEND:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_Four_BMA_U8X);
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_Zero_BMA_U8X);
               break;
            case BMA255_MODE_DEEP_SUSPEND:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_One_BMA_U8X);
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_One_BMA_U8X);
               break;
            case BMA255_MODE_LOWPOWER2:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_Two_BMA_U8X);
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_One_BMA_U8X);
               break;
            case BMA255_MODE_STANDBY:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_MODE_CTRL, C_Four_BMA_U8X);
               data2  = BMA255_SET_BITSLICE(data2, BMA255_LOW_POWER_MODE, C_One_BMA_U8X);
               break;
            default:
               break;
            }
         BMA250E_WriteReg(BMA255_MODE_CTRL_REG, data1);
		nrf_delay_ms(10);////A minimum delay of atleast 450us is required for the low power modes, as per the data sheet.
         BMA250E_WriteReg(BMA255_LOW_NOISE_CTRL_REG, data2);
         p_bma255->mode = Mode;
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }	 
/***********************************************************************************
 * Description: *//**\brief This API is used to set Ranges(g value) of the sensor
 *
 *
 *
 *
 *  \param unsigned char Range
 *                        3 -> 2G
 *                        5 -> 4G
 *                        8 -> 8G
 *                       12 -> 16G
 *
 *  \return communication results
 *
 *
 ************************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma255_set_range(unsigned char Range)
   {
   int comres=C_Zero_BMA_U8X ;
   unsigned char data1;
   if (p_bma255 == C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if ((Range == C_Three_BMA_U8X) || (Range == C_Five_BMA_U8X) || (Range == C_Eight_BMA_U8X) || (Range == C_Twelve_BMA_U8X))
         {
         comres = BMA250E_ReadReg(BMA255_RANGE_SEL_REG,&data1);
					 //p_bma255->BMA255_BUS_READ_FUNC(p_bma255->dev_addr, BMA255_RANGE_SEL_REG, &data1, C_One_BMA_U8X );
         switch (Range)
            {
            case BMA255_RANGE_2G:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_RANGE_SEL, C_Three_BMA_U8X);
               break;
            case BMA255_RANGE_4G:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_RANGE_SEL, C_Five_BMA_U8X);
               break;
            case BMA255_RANGE_8G:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_RANGE_SEL, C_Eight_BMA_U8X);
               break;
            case BMA255_RANGE_16G:
               data1  = BMA255_SET_BITSLICE(data1, BMA255_RANGE_SEL, C_Twelve_BMA_U8X);
               break;
            default:
               break;
            }
         comres += BMA250E_WriteReg(BMA255_RANGE_SEL_REG, data1);
						//p_bma255->BMA255_BUS_WRITE_FUNC(p_bma255->dev_addr, BMA255_RANGE_SEL_REG, &data1, C_One_BMA_U8X);
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

/************************************************************************************
 * Description: *//**\brief This API is used to set Sleep Duration of the sensor
 *
 *
 *
 *
 *  \param unsigned char sleep_dur
 *                       5 -> 0.5MS
 *                       6 -> 1MS
 *                       7 -> 2MS
 *                       8 -> 4MS
 *                       9 -> 6MS
 *                      10 -> 10MS
 *                      11 -> 25MS
 *                      12 -> 50MS
 *                      13 -> 100MS
 *                      14 -> 500MS 
 *                      15 -> 1S  
 *
 *
 *  \return communication results
 *
 *
 **************************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA255_RETURN_FUNCTION_TYPE bma255_set_sleep_dur (unsigned char sleep_dur)
   {
   unsigned char data;
   int comres = C_Zero_BMA_U8X ;
   int sleep_duration=0; 
   if (p_bma255 == C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (sleep_dur > C_Four_BMA_U8X && sleep_dur < C_Sixteen_BMA_U8X)
         {
         switch (sleep_dur)
            {
            case BMA255_SLEEP_DUR_0_5MS:
               sleep_duration = BMA255_SLEEP_DUR_0_5MS;
               
               /*  0.5 MS   */
               break;
            case BMA255_SLEEP_DUR_1MS:
               sleep_duration = BMA255_SLEEP_DUR_1MS;
               
               /*  1 MS  */
               break;
            case BMA255_SLEEP_DUR_2MS:
               sleep_duration = BMA255_SLEEP_DUR_2MS;
               
               /*  2 MS  */
               break;
            case BMA255_SLEEP_DUR_4MS:
               sleep_duration = BMA255_SLEEP_DUR_4MS;
               
               /*  4 MS   */
               break;
            case BMA255_SLEEP_DUR_6MS:
               sleep_duration = BMA255_SLEEP_DUR_6MS;
               
               /*  6 MS  */
               break;
            case BMA255_SLEEP_DUR_10MS:
               sleep_duration = BMA255_SLEEP_DUR_10MS;
               
               /*  10 MS  */
               break;
            case BMA255_SLEEP_DUR_25MS:
               sleep_duration = BMA255_SLEEP_DUR_25MS;
               
               /*  25 MS  */
               break;
            case BMA255_SLEEP_DUR_50MS:
               sleep_duration = BMA255_SLEEP_DUR_50MS;
               
               /*  50 MS   */
               break;
            case BMA255_SLEEP_DUR_100MS:
               sleep_duration = BMA255_SLEEP_DUR_100MS;
               
               /*  100 MS  */
               break;
            case BMA255_SLEEP_DUR_500MS:
               sleep_duration = BMA255_SLEEP_DUR_500MS;
               
               /*  500 MS   */
               break;
            case BMA255_SLEEP_DUR_1S:
               sleep_duration = BMA255_SLEEP_DUR_1S;
               
               /*  1 SECS   */
               break;
            default:
               break;
            }
         //SLEEP DURATION
         comres =BMA250E_ReadReg(BMA255_SLEEP_DUR__REG, &data);
         data = BMA255_SET_BITSLICE(data, BMA255_SLEEP_DUR, sleep_duration);
         comres =BMA250E_WriteReg(BMA255_SLEEP_DUR__REG, data);
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }
	 
	/*******************************************************************************
 * Description: *//**\brief This API Reads interrupt status 2 register byte 
 *                          from location 0Ah
 *
 *
 *
 *
 *  \param unsigned char * status_2 : Address of status 2 register
 *
 *
 *
 *  \return Result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma255_get_interrupt_status2(unsigned char * status_2)
   {
   int comres = C_Zero_BMA_U8X;
   if (p_bma255 == C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres =BMA250E_ReadReg(BMA255_STATUS2_REG, status_2);
      }
   return comres;
   }
	 
/****************************************************************************************
 * Description: *//**\brief This API is used to set interrupt enable bits of the sensor
 *
 *
 *
 *
 *  \param unsigned char InterruptType , unsigned char value
 *                        0 -> Low_G_Interrupt
 *                        1 -> High_G_X_Interrupt
 *                        2 -> High_G_Y_Interrupt
 *                        3 -> High_G_Z_Interrupt
 *                        4 -> DATA_EN
 *                        5 -> Slope_X_Interrupt
 *                        6 -> Slope_Y_Interrupt 
 *                        7 -> Slope_Z_Interrupt
 *                        8 -> Single_Tap_Interrupt
 *                        9 -> Double_Tap_Interrupt
 *                       10 -> Orient_Interrupt
 *                       11 -> Flat_Interrupt
 *                       
 *                       
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma255_set_Int_Enable(unsigned char InterruptType , unsigned char value )
   {
   int comres=C_Zero_BMA_U8X;
   unsigned char data1,data2;
   if (p_bma255==C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres =BMA250E_ReadReg(BMA255_INT_ENABLE1_REG, &data1);
      comres =BMA250E_ReadReg(BMA255_INT_ENABLE2_REG, &data2);
      
      value = value & C_One_BMA_U8X;
      switch (InterruptType)
         {
         case BMA255_Low_G_Interrupt:
            /* Low G Interrupt  */
            data2 = BMA255_SET_BITSLICE(data2, BMA255_EN_LOWG_INT, value );
            break;
         case BMA255_High_G_X_Interrupt:
            /* High G X Interrupt */
            /* EasyCASE - */
            data2 = BMA255_SET_BITSLICE(data2, BMA255_EN_HIGHG_X_INT, value );
            break;
         case BMA255_High_G_Y_Interrupt:
            /* High G Y Interrupt */
            /* EasyCASE - */
            data2 = BMA255_SET_BITSLICE(data2, BMA255_EN_HIGHG_Y_INT, value );
            break;
         case BMA255_High_G_Z_Interrupt:
            /* High G Z Interrupt */
            /* EasyCASE - */
            data2 = BMA255_SET_BITSLICE(data2, BMA255_EN_HIGHG_Z_INT, value );
            break;
         case BMA255_DATA_EN:
            /*Data En Interrupt  */
            /* EasyCASE - */
            data2 = BMA255_SET_BITSLICE(data2, BMA255_EN_NEW_DATA_INT, value );
            break;
         case BMA255_Slope_X_Interrupt:
            /* Slope X Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_X_INT, value );
            break;
         case BMA255_Slope_Y_Interrupt:
            /* Slope Y Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Y_INT, value );
            break;
         case BMA255_Slope_Z_Interrupt:
            /* Slope Z Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Z_INT, value );
            break;
         case BMA255_Single_Tap_Interrupt:
            /* Single Tap Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SINGLE_TAP_INT, value );
            break;
         case BMA255_Double_Tap_Interrupt:
            /* Double Tap Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_DOUBLE_TAP_INT, value );
            break;
         case BMA255_Orient_Interrupt:
            /* Orient Interrupt  */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_ORIENT_INT, value );
            break;
         case BMA255_Flat_Interrupt:
            /* Flat Interrupt */
            /* EasyCASE - */
            data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_FLAT_INT, value );
            break;
         default:
            comres = E_BMA255_OUT_OF_RANGE;
            break;
         }
      comres =BMA250E_WriteReg(BMA255_INT_ENABLE1_REG, data1);
      comres =BMA250E_WriteReg(BMA255_INT_ENABLE2_REG, data2);
      }
   return comres;
   }


unsigned char BMA250E_ReadReg_DMWZ(unsigned char Reg, unsigned char* Data,unsigned char len)
{
	return i2c_read(deviceaddress,Reg,Data,len);
}
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_data_out_reg(unsigned char *out_reg)
   {
   unsigned char data;
   int comres = C_Zero_BMA_U8X;
   if (p_bma255==C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      //GET FIFO DATA OUTPUT REGISTER
      comres =BMA250E_ReadReg_DMWZ(BMA255_FIFO_DATA_OUTPUT_REG,&data,6);
      *out_reg = data;
      }
   return comres;
   }
	 
/*******************************************************************************
 * Description: *//**\brief This API is used to get the status of offset
 *
 *
 *
 *
 *  \param unsigned char channel,unsigned char *offset
 *                         Channel ->
 *                   BMA255_X_AXIS     ->      0                           
 *                   BMA255_Y_AXIS     ->      1                            
 *                   BMA255_Z_AXIS     ->      2 
 *                   offset -> Any valid value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA255_RETURN_FUNCTION_TYPE bma255_get_offset(unsigned char channel,unsigned char *offset)
   {
   unsigned char data;
   int comres = C_Zero_BMA_U8X;
   if (p_bma255==C_Zero_BMA_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (channel)
         {
         case BMA255_X_AXIS:
            comres =BMA250E_ReadReg(BMA255_OFFSET_X_AXIS_REG,&data);
            *offset = data;
            break;
         case BMA255_Y_AXIS:
            comres =BMA250E_ReadReg(BMA255_OFFSET_Y_AXIS_REG,&data);
            *offset = data;
            break;
         case BMA255_Z_AXIS:
            comres =BMA250E_ReadReg(BMA255_OFFSET_Z_AXIS_REG,&data);
            *offset = data;
            break;
         default:
            comres = E_BMA255_OUT_OF_RANGE;
            break;
         }
      }
   return comres;
   }
	 
/************************************************
函数名称:获取传感器数据,直接读取X,Y,Z寄存器
**************************************************/
void BMA253_Timer_Handler(void)
{
	unsigned char accel_data[6] = {0x00};
	unsigned char j = 0;
//	memset(V_BMA255FIFOData_U8R,0,6);

	for(j=0; j<6; j++)
	{
		BMA250E_ReadReg(BMA255_ACC_X12_LSB__REG+j, &V_BMA255FIFOData_U8R[j]);	//read x,y,z accel
	}

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_mode (unsigned char *fifo_mode)
   {
   int comres;
   unsigned char data;
   if (p_bma255==C_Zero_BMA_U8X)
      {
      comres =E_SMB_NULL_PTR ;
      }
   else
      {
      comres =BMA250E_ReadReg(BMA255_FIFO_MODE__REG,&data);
      *fifo_mode = BMA255_GET_BITSLICE(data, BMA255_FIFO_MODE);
      }
   return comres;
   }
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_data_sel (unsigned char *data_sel)
   {
   int comres;
   unsigned char data;
   if (p_bma255==C_Zero_BMA_U8X)
      {
      comres =E_SMB_NULL_PTR ;
      }
   else
      {
      comres = BMA250E_ReadReg(BMA255_FIFO_DATA_SELECT__REG,&data);
      *data_sel = BMA255_GET_BITSLICE(data, BMA255_FIFO_DATA_SELECT);
      }
   return comres;
   }	 
BMA255_RETURN_FUNCTION_TYPE bma255_burst_read(unsigned char addr, unsigned char *data, unsigned int len)
   {
   
   
   BMA255_RETURN_FUNCTION_TYPE comres;
   
   if (p_bma255 == BMA255_NULL)
      {
      comres = E_BMA255_NULL_PTR;
      }
   else
      {
      comres =BMA250E_ReadReg_DMWZ(addr, data, len);
      }
   return comres;
   }
	 

/************************************************
函数名称:获取传感器数据,读取FIFO寄存器
**************************************************/
unsigned int BMA255_FIFODAQ(void)
{

   unsigned char v_noofbytestoread_u8r = 0;
 	 v_noofbytestoread_u8r = 6;
   //Read FIFO Data
   bma255_burst_read(0x3F, V_BMA255FIFOData_U8R, v_noofbytestoread_u8r);
}


void bma_init(void)
{
	unsigned char buf[4];
	bma255_init(&bma255);
	bma250_soft_reset();
  SCL_();
	bma255_set_range(BMA255_RANGE_2G);	
//	bma255_set_range(BMA255_RANGE_8G);
//	bma255_set_range(BMA255_RANGE_4G);
//	bma255_set_range(BMA255_RANGE_2G);
//	bma255_set_bandwidth(BMA255_BW_15_63HZ);
//	bma255_set_Int_Enable(BMA255_DATA_EN,1);
//	bma255_set_mode(BMA255_MODE_LOWPOWER1);
//	BMA250E_WriteReg(BMA255_FIFO_MODE_REG, 0x80);
//	bma255_set_sleep_dur(BMA255_SLEEP_DUR_25MS);
//	BMA250E_ReadReg_DMWZ(0x00,buf,1);
}





