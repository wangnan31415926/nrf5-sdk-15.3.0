#ifndef BMA_H
#define BMA_H
/* EasyCASE - */
/*
 ***************************************************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 **************************************************************************************************/
/*  $Date: 2011/07/03
 *  $Revision: 1.1 $
 *
 */

/**************************************************************************************************
* Copyright (C) 2007 Bosch Sensortec GmbH
*
* bma255.h
*
* Usage:        BMA255 Sensor Driver Support Header File
*
* Author:       VenkataKrishnan.Venkataraman@in.bosch.com
**************************************************************************************************/
/* EasyCASE ( 72
   Disclaimer */
/*************************************************************************************************/
/*  Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry. They may only be used
* within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
* provided with the express understanding that there is no warranty of fitness for a particular purpose.
* They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
* Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
* The resale and/or use of products are at the purchaser抯 own risk and his own responsibility. The
* examination of fitness for the intended use is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
* incidental, or consequential damages, arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
* Sensortec for all costs in connection with such claims.
*
* The purchaser must monitor the market for the purchased products, particularly with regard to
* product safety and inform Bosch Sensortec without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
* technical specifications of the product series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal client testing. The testing of an
* engineering sample may in no way replace the testing of a product series. Bosch Sensortec
* assumes no liability for the use of engineering samples. By accepting the engineering samples, the
* Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information on application-sheets
* (hereinafter called "Information") is provided free of charge for the sole purpose to support your
* application work. The Software and Information is subject to the following terms and conditions:
*
* The Software is specifically designed for the exclusive use for Bosch Sensortec products by
* personnel who have special experience and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed or implied warranties,
* including without limitation, the implied warranties of merchantability and fitness for a particular
* purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
* of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
* representatives and agents shall not be liable for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
* responsibility for the consequences of use of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use. No license is granted by implication or
* otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software to any third party without permission of
* Bosch Sensortec.
*/
/*************************************************************************************************/
/* EasyCASE ) */
/* EasyCASE ( 913
   File Name For Doxy */
/*! \file bma255.h
    \brief BMA255 Sensor Driver Support Header File */
/* EasyCASE ) */
/* EasyCASE ( 73
   Includes */
/* EasyCASE ( 912
   Standard includes */
/* EasyCASE ) */
/* EasyCASE ( 914
   Module includes */
/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ( 75
   #Define Constants */
/* user defined code to be added here ... */

//Example....
//#define YOUR_H_DEFINE  /**< <Doxy Comment for YOUR_H_DEFINE> */
/* EasyCASE ( 916
   bma255 Macro for read and write commincation */
/** Define the calling convention of YOUR bus communication routine.
        \note This includes types of parameters. This example shows the configuration for an SPI bus link.

    If your communication function looks like this:

    write_my_bus_xy(unsigned char device_addr, unsigned char register_addr, unsigned char * data, unsigned char length);

    The BMA255_WR_FUNC_PTR would equal:

    #define     BMA255_WR_FUNC_PTR char (* bus_write)(unsigned char, unsigned char, unsigned char *, unsigned char)

    Parameters can be mixed as needed refer to the \ref BMA255_BUS_WRITE_FUNC  macro.


*/
#define BMA255_WR_FUNC_PTR char (* bus_write)(unsigned char, unsigned char , unsigned char *, unsigned char)



/** link makro between API function calls and bus write function
        \note The bus write function can change since this is a system dependant issue.

    If the bus_write parameter calling order is like: reg_addr, reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication function like I2C need to know the device address,
    you can change this macro accordingly.


    define BMA255_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_write(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a way that equals your definition in the
    \ref BMA255_WR_FUNC_PTR definition.



*/
#define BMA255_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
           bus_write(dev_addr, reg_addr, reg_data, wr_len)


/** Define the calling convention of YOUR bus communication routine.
        \note This includes types of parameters. This example shows the configuration for an SPI bus link.

    If your communication function looks like this:

    read_my_bus_xy(unsigned char device_addr, unsigned char register_addr, unsigned char * data, unsigned char length);

    The BMA255_RD_FUNC_PTR would equal:

    #define     BMA255_RD_FUNC_PTR char (* bus_read)(unsigned char, unsigned char, unsigned char *, unsigned char)

        Parameters can be mixed as needed refer to the \ref BMA255_BUS_READ_FUNC  macro.


*/

#define BMA255_SPI_RD_MASK 0x80   /* for spi read transactions on SPI the MSB has to be set */
#define BMA255_RD_FUNC_PTR char (* bus_read)( unsigned char, unsigned char , unsigned char *,unsigned char)
#define BMA255_BRD_FUNC_PTR char (* burst_read)(unsigned char, unsigned char, unsigned char *, unsigned char)


/** link makro between API function calls and bus read function
        \note The bus write function can change since this is a system dependant issue.

    If the bus_read parameter calling order is like: reg_addr, reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication function like I2C need to know the device address,
    you can change this macro accordingly.


        define BMA255_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
           bus_read(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a way that equals your definition in the
    \ref BMA255_WR_FUNC_PTR definition.

        \note: this macro also includes the "MSB='1'" for reading BMA255 addresses.

*/
#define BMA255_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
           bus_read(dev_addr, reg_addr, reg_data, r_len)
#define BMA255_BURST_READ_FUNC(device_addr, register_addr, register_data, rd_len )\
           burst_read(device_addr, register_addr, register_data, rd_len )
/* EasyCASE ) */
/** bma255 I2C Address
*/

#define BMA255_I2C_ADDR1                0x10
#define BMA255_I2C_ADDR2                0x19
#define BMA255_I2C_ADDR                 BMA255_I2C_ADDR1


/*
        SMB380 API error codes
*/

#define E_SMB_NULL_PTR          (char)-127
#define E_COMM_RES              (char)-1
#define E_OUT_OF_RANGE          (char)-2
#define E_EEPROM_BUSY           (char)-3

#define BMA255_RETURN_FUNCTION_TYPE        int            /**< This refers BMA255 return type as char */

/*
 *
 *      register definitions
 *
 */


#define BMA255_EEP_OFFSET                       0x16
#define BMA255_IMAGE_BASE                       0x38
#define BMA255_IMAGE_LEN                        22


#define BMA255_CHIP_ID_REG                      0x00
#define BMA255_VERSION_REG                      0x01
#define BMA255_X_AXIS_LSB_REG                   0x02
#define BMA255_X_AXIS_MSB_REG                   0x03
#define BMA255_Y_AXIS_LSB_REG                   0x04
#define BMA255_Y_AXIS_MSB_REG                   0x05
#define BMA255_Z_AXIS_LSB_REG                   0x06
#define BMA255_Z_AXIS_MSB_REG                   0x07
#define BMA255_TEMP_RD_REG                      0x08
#define BMA255_STATUS1_REG                      0x09
#define BMA255_STATUS2_REG                      0x0A
#define BMA255_STATUS_TAP_SLOPE_REG             0x0B
#define BMA255_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA255_STATUS_FIFO_REG                  0x0E       //ADDED
#define BMA255_RANGE_SEL_REG                    0x0F
#define BMA255_BW_SEL_REG                       0x10
#define BMA255_MODE_CTRL_REG                    0x11
#define BMA255_LOW_NOISE_CTRL_REG               0x12
#define BMA255_DATA_CTRL_REG                    0x13
#define BMA255_RESET_REG                        0x14
#define BMA255_INT_ENABLE1_REG                  0x16       //设置中断使能
#define BMA255_INT_ENABLE2_REG                  0x17
#define BMA255_INT_SLO_NO_MOT_REG               0x18      //ADDED
#define BMA255_INT1_PAD_SEL_REG                 0x19        //设置中断引脚int1
#define BMA255_INT_DATA_SEL_REG                 0x1A
#define BMA255_INT2_PAD_SEL_REG                 0x1B
#define BMA255_INT_SRC_REG                      0x1E
#define BMA255_INT_SET_REG                      0x20
#define BMA255_INT_CTRL_REG                     0x21
#define BMA255_LOW_DURN_REG                     0x22
#define BMA255_LOW_THRES_REG                    0x23
#define BMA255_LOW_HIGH_HYST_REG                0x24
#define BMA255_HIGH_DURN_REG                    0x25
#define BMA255_HIGH_THRES_REG                   0x26
#define BMA255_SLOPE_DURN_REG                   0x27
#define BMA255_SLOPE_THRES_REG                  0x28
#define BMA255_SLO_NO_MOT_THRES_REG             0x29    //ADDED
#define BMA255_TAP_PARAM_REG                    0x2A
#define BMA255_TAP_THRES_REG                    0x2B
#define BMA255_ORIENT_PARAM_REG                 0x2C
#define BMA255_THETA_BLOCK_REG                  0x2D
#define BMA255_THETA_FLAT_REG                   0x2E
#define BMA255_FLAT_HOLD_TIME_REG               0x2F
#define BMA255_FIFO_WML_TRIG                    0x30   //ADDED
#define BMA255_SELF_TEST_REG                    0x32
#define BMA255_EEPROM_CTRL_REG                  0x33
#define BMA255_SERIAL_CTRL_REG                  0x34
#define BMA255_EXTMODE_CTRL_REG                 0x35
#define BMA255_OFFSET_CTRL_REG                  0x36
#define BMA255_OFFSET_PARAMS_REG                0x37
#define BMA255_OFFSET_X_AXIS_REG                0x38
#define BMA255_OFFSET_Y_AXIS_REG                0x39
#define BMA255_OFFSET_Z_AXIS_REG                0x3A
#define BMA255_GP0_REG                          0x3B    //ADDED
#define BMA255_GP1_REG                          0x3C    //ADDED
#define BMA255_FIFO_MODE_REG                    0x3E    //ADDED
#define BMA255_FIFO_DATA_OUTPUT_REG             0x3F    //ADDED

#define E_BMA255_NULL_PTR                       (char)-127
#define BMA255_NULL                                                     0

#define 	C_Null_BMA_U8X				0
#define		C_Zero_BMA_U8X				0
#define 	C_One_BMA_U8X				1
#define		C_Two_BMA_U8X				2
#define         C_Three_BMA_U8X                             3
#define         C_Four_BMA_U8X                              4
#define         C_Five_BMA_U8X                              5
#define         C_Six_BMA_U8X                               6
#define			C_Seven_BMA_U8X								7
#define         C_Eight_BMA_U8X                             8
#define         C_Nine_BMA_U8X               			    9
#define         C_Ten_BMA_U8X 			                 	10
#define     	C_Eleven_BMA_U8X                           	11
#define         C_Twelve_BMA_U8X                            12
#define         C_Thirteen_BMA_U8X                          13
#define 		C_Fifteen_BMA_U8X							15
#define		    C_Sixteen_BMA_U8X                			16
#define         C_TwentyTwo_BMA_U8X 	                 	22
#define         C_TwentyThree_BMA_U8X 	                 	23
#define         C_TwentyFour_BMA_U8X 	                 	24
#define         C_TwentyFive_BMA_U8X 	                 	25
#define         C_ThirtyTwo_BMA_U8X                    		32
#define		    C_Hundred_BMA_U8X		                 	100
#define   		C_OneTwentySeven_BMA_U8X	                127
#define   		C_OneTwentyEight_BMA_U8X	                128
#define  		C_TwoFiftyFive_BMA_U8X	                 	255
#define		    C_TwoFiftySix_BMA_U16X              		256

/* register write and read delays */

#define BMA255_MDELAY_DATA_TYPE                 unsigned char
#define BMA255_EE_W_DELAY                       28                    /* delay after EEP write is 28 msec */
/* EasyCASE ( 919
   bma255acc_t */
/* EasyCASE C */
/** bma255 acceleration data
        \brief Structure containing acceleration values for x,y and z-axis in signed short

*/

typedef struct
   {
   short x, /**< holds x-axis acceleration data sign extended. Range -512 to 511. */
         y, /**< holds y-axis acceleration data sign extended. Range -512 to 511. */
         z; /**< holds z-axis acceleration data sign extended. Range -512 to 511. */
   } bma255acc_t;
/* EasyCASE E */
/* EasyCASE ) */
/* EasyCASE ( 920
   bma255regs_t */
/* EasyCASE C */
/** bma255 image registers data structure
        \brief Register type that contains all bma255 image registers from address 0x38 to 0x4D
        This structure can hold the complete image data of bma255

*/
typedef struct
   {
   unsigned char
   offset_filt_x ,                 /**<  image address 0x38:  */
   offset_filt_y ,                 /**<  image address 0x39:  */
   offset_filt_z ,                 /**<  image address 0x3A:  */
   offset_unfilt_x ,               /**<  image address 0x3B:  */
   offset_unfilt_y ,               /**<  image address 0x3C:  */
   offset_unfilt_z ,               /**<  image address 0x3D:  */
   spare_0 ,                       /**<  image address 0x3E:  */
   spare_1 ,                       /**<  image address 0x3F:  */
   crc ,                           /**<  image address 0x40:  */
   i2c_addr ,                      /**<  image address 0x41:  */
   dev_config ,                    /**<  image address 0x42:  */
   trim_offset_t ,                 /**<  image address 0x43:  */
   gain_x ,                        /**<  image address 0x44:  */
   offset_x ,                      /**<  image address 0x45:  */
   gain_y ,                        /**<  image address 0x46:  */
   offset_y ,                      /**<  image address 0x47:  */
   gain_z ,                        /**<  image address 0x48:  */
   offset_z ,                      /**<  image address 0x49:  */
   trim1 ,                         /**<  image address 0x4A:  */
   trim2 ,                         /**<  image address 0x4B:  */
   trim3 ,                         /**<  image address 0x4C:  */
   trim4 ;                          /**<  image address 0x4D:  */
   } bma255regs_t;
/* EasyCASE E */
/* EasyCASE ) */
/* EasyCASE ( 921
   bma255_t */
/* EasyCASE C */
/** bma255 typedef structure
        \brief This structure holds all relevant information about bma255 and links communication to the
*/

typedef struct
   {
   bma255regs_t * image;   /**< pointer to bma255regs_t structure not mandatory */
   unsigned char mode;     /**< save current bma255 operation mode */
   unsigned char chip_id,  /**< save bma255's chip id which has to be 0x02 after calling bma255_init() */
                          version; /**< holds the bma255 version number */
   unsigned char dev_addr;   /**< initializes bma255's I2C device address 0x38 */
   unsigned char int_mask;   /**< stores the current bma255 API generated interrupt mask */
   BMA255_WR_FUNC_PTR;               /**< function pointer to the SPI/I2C write function */
   BMA255_RD_FUNC_PTR;               /**< function pointer to the SPI/I2C read function */
   BMA255_BRD_FUNC_PTR;
   void (*delay_msec)( BMA255_MDELAY_DATA_TYPE ); /**< function pointer to a pause in mili seconds function */
   } bma255_t;
/* EasyCASE E */
/* EasyCASE ) */
/* EasyCASE ( 922
   BIT'S & BYTE'S */
#define BMA255_CHIP_ID__POS             0
#define BMA255_CHIP_ID__MSK             0xFF
#define BMA255_CHIP_ID__LEN             8
#define BMA255_CHIP_ID__REG             BMA255_CHIP_ID_REG
/* EasyCASE - */
#define BMA255_VERSION__POS          0
#define BMA255_VERSION__LEN          8
#define BMA255_VERSION__MSK          0xFF
#define BMA255_VERSION__REG          BMA255_VERSION_REG
/* EasyCASE - */
/* DATA REGISTERS */

#define BMA255_NEW_DATA_X__POS          0
#define BMA255_NEW_DATA_X__LEN          1
#define BMA255_NEW_DATA_X__MSK          0x01
#define BMA255_NEW_DATA_X__REG          BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X14_LSB__POS           2
#define BMA255_ACC_X14_LSB__LEN           6
#define BMA255_ACC_X14_LSB__MSK           0xFC
#define BMA255_ACC_X14_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X12_LSB__POS           4
#define BMA255_ACC_X12_LSB__LEN           4
#define BMA255_ACC_X12_LSB__MSK           0xF0
#define BMA255_ACC_X12_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X10_LSB__POS           6
#define BMA255_ACC_X10_LSB__LEN           2
#define BMA255_ACC_X10_LSB__MSK           0xC0
#define BMA255_ACC_X10_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X8_LSB__POS           0
#define BMA255_ACC_X8_LSB__LEN           0
#define BMA255_ACC_X8_LSB__MSK           0x00
#define BMA255_ACC_X8_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X_MSB__POS           0
#define BMA255_ACC_X_MSB__LEN           8
#define BMA255_ACC_X_MSB__MSK           0xFF
#define BMA255_ACC_X_MSB__REG           BMA255_X_AXIS_MSB_REG

#define BMA255_NEW_DATA_Y__POS          0
#define BMA255_NEW_DATA_Y__LEN          1
#define BMA255_NEW_DATA_Y__MSK          0x01
#define BMA255_NEW_DATA_Y__REG          BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y14_LSB__POS           2
#define BMA255_ACC_Y14_LSB__LEN           6
#define BMA255_ACC_Y14_LSB__MSK           0xFC
#define BMA255_ACC_Y14_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y12_LSB__POS           4
#define BMA255_ACC_Y12_LSB__LEN           4
#define BMA255_ACC_Y12_LSB__MSK           0xF0
#define BMA255_ACC_Y12_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y10_LSB__POS           6
#define BMA255_ACC_Y10_LSB__LEN           2
#define BMA255_ACC_Y10_LSB__MSK           0xC0
#define BMA255_ACC_Y10_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y8_LSB__POS           0
#define BMA255_ACC_Y8_LSB__LEN           0
#define BMA255_ACC_Y8_LSB__MSK           0x00
#define BMA255_ACC_Y8_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y_MSB__POS           0
#define BMA255_ACC_Y_MSB__LEN           8
#define BMA255_ACC_Y_MSB__MSK           0xFF
#define BMA255_ACC_Y_MSB__REG           BMA255_Y_AXIS_MSB_REG

#define BMA255_NEW_DATA_Z__POS          0
#define BMA255_NEW_DATA_Z__LEN          1
#define BMA255_NEW_DATA_Z__MSK          0x01
#define BMA255_NEW_DATA_Z__REG          BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z14_LSB__POS           2
#define BMA255_ACC_Z14_LSB__LEN           6
#define BMA255_ACC_Z14_LSB__MSK           0xFC
#define BMA255_ACC_Z14_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z12_LSB__POS           4
#define BMA255_ACC_Z12_LSB__LEN           4
#define BMA255_ACC_Z12_LSB__MSK           0xF0
#define BMA255_ACC_Z12_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z10_LSB__POS           6
#define BMA255_ACC_Z10_LSB__LEN           2
#define BMA255_ACC_Z10_LSB__MSK           0xC0
#define BMA255_ACC_Z10_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z8_LSB__POS           0
#define BMA255_ACC_Z8_LSB__LEN           0
#define BMA255_ACC_Z8_LSB__MSK           0x00
#define BMA255_ACC_Z8_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z_MSB__POS           0
#define BMA255_ACC_Z_MSB__LEN           8
#define BMA255_ACC_Z_MSB__MSK           0xFF
#define BMA255_ACC_Z_MSB__REG           BMA255_Z_AXIS_MSB_REG
/* EasyCASE - */
#define BMA255_TEMPERATURE__POS         0
#define BMA255_TEMPERATURE__LEN         8
#define BMA255_TEMPERATURE__MSK         0xFF
#define BMA255_TEMPERATURE__REG         BMA255_TEMP_RD_REG
/* EasyCASE - */
/*  INTERRUPT STATUS BITS  */

#define BMA255_LOWG_INT_S__POS          0
#define BMA255_LOWG_INT_S__LEN          1
#define BMA255_LOWG_INT_S__MSK          0x01
#define BMA255_LOWG_INT_S__REG          BMA255_STATUS1_REG

#define BMA255_HIGHG_INT_S__POS          1
#define BMA255_HIGHG_INT_S__LEN          1
#define BMA255_HIGHG_INT_S__MSK          0x02
#define BMA255_HIGHG_INT_S__REG          BMA255_STATUS1_REG

#define BMA255_SLOPE_INT_S__POS          2
#define BMA255_SLOPE_INT_S__LEN          1
#define BMA255_SLOPE_INT_S__MSK          0x04
#define BMA255_SLOPE_INT_S__REG          BMA255_STATUS1_REG

//ADDED
#define BMA255_SLO_NO_MOT_INT_S__POS          3
#define BMA255_SLO_NO_MOT_INT_S__LEN          1
#define BMA255_SLO_NO_MOT_INT_S__MSK          0x08
#define BMA255_SLO_NO_MOT_INT_S__REG          BMA255_STATUS1_REG

#define BMA255_DOUBLE_TAP_INT_S__POS     4
#define BMA255_DOUBLE_TAP_INT_S__LEN     1
#define BMA255_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA255_DOUBLE_TAP_INT_S__REG     BMA255_STATUS1_REG

#define BMA255_SINGLE_TAP_INT_S__POS     5
#define BMA255_SINGLE_TAP_INT_S__LEN     1
#define BMA255_SINGLE_TAP_INT_S__MSK     0x20
#define BMA255_SINGLE_TAP_INT_S__REG     BMA255_STATUS1_REG

#define BMA255_ORIENT_INT_S__POS         6
#define BMA255_ORIENT_INT_S__LEN         1
#define BMA255_ORIENT_INT_S__MSK         0x40
#define BMA255_ORIENT_INT_S__REG         BMA255_STATUS1_REG

#define BMA255_FLAT_INT_S__POS           7
#define BMA255_FLAT_INT_S__LEN           1
#define BMA255_FLAT_INT_S__MSK           0x80
#define BMA255_FLAT_INT_S__REG           BMA255_STATUS1_REG

#define BMA255_FIFO_FULL_INT_S__POS           5
#define BMA255_FIFO_FULL_INT_S__LEN           1
#define BMA255_FIFO_FULL_INT_S__MSK           0x20
#define BMA255_FIFO_FULL_INT_S__REG           BMA255_STATUS2_REG

#define BMA255_FIFO_WM_INT_S__POS           6
#define BMA255_FIFO_WM_INT_S__LEN           1
#define BMA255_FIFO_WM_INT_S__MSK           0x40
#define BMA255_FIFO_WM_INT_S__REG           BMA255_STATUS2_REG

#define BMA255_DATA_INT_S__POS           7
#define BMA255_DATA_INT_S__LEN           1
#define BMA255_DATA_INT_S__MSK           0x80
#define BMA255_DATA_INT_S__REG           BMA255_STATUS2_REG
/* EasyCASE - */
#define BMA255_SLOPE_FIRST_X__POS        0
#define BMA255_SLOPE_FIRST_X__LEN        1
#define BMA255_SLOPE_FIRST_X__MSK        0x01
#define BMA255_SLOPE_FIRST_X__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_FIRST_Y__POS        1
#define BMA255_SLOPE_FIRST_Y__LEN        1
#define BMA255_SLOPE_FIRST_Y__MSK        0x02
#define BMA255_SLOPE_FIRST_Y__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_FIRST_Z__POS        2
#define BMA255_SLOPE_FIRST_Z__LEN        1
#define BMA255_SLOPE_FIRST_Z__MSK        0x04
#define BMA255_SLOPE_FIRST_Z__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_SIGN_S__POS         3
#define BMA255_SLOPE_SIGN_S__LEN         1
#define BMA255_SLOPE_SIGN_S__MSK         0x08
#define BMA255_SLOPE_SIGN_S__REG         BMA255_STATUS_TAP_SLOPE_REG
/* EasyCASE - */
#define BMA255_TAP_FIRST_X__POS        4
#define BMA255_TAP_FIRST_X__LEN        1
#define BMA255_TAP_FIRST_X__MSK        0x10
#define BMA255_TAP_FIRST_X__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_TAP_FIRST_Y__POS        5
#define BMA255_TAP_FIRST_Y__LEN        1
#define BMA255_TAP_FIRST_Y__MSK        0x20
#define BMA255_TAP_FIRST_Y__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_TAP_FIRST_Z__POS        6
#define BMA255_TAP_FIRST_Z__LEN        1
#define BMA255_TAP_FIRST_Z__MSK        0x40
#define BMA255_TAP_FIRST_Z__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_TAP_SIGN_S__POS         7
#define BMA255_TAP_SIGN_S__LEN         1
#define BMA255_TAP_SIGN_S__MSK         0x80
#define BMA255_TAP_SIGN_S__REG         BMA255_STATUS_TAP_SLOPE_REG
/* EasyCASE - */
#define BMA255_HIGHG_FIRST_X__POS        0
#define BMA255_HIGHG_FIRST_X__LEN        1
#define BMA255_HIGHG_FIRST_X__MSK        0x01
#define BMA255_HIGHG_FIRST_X__REG        BMA255_STATUS_ORIENT_HIGH_REG

#define BMA255_HIGHG_FIRST_Y__POS        1
#define BMA255_HIGHG_FIRST_Y__LEN        1
#define BMA255_HIGHG_FIRST_Y__MSK        0x02
#define BMA255_HIGHG_FIRST_Y__REG        BMA255_STATUS_ORIENT_HIGH_REG

#define BMA255_HIGHG_FIRST_Z__POS        2
#define BMA255_HIGHG_FIRST_Z__LEN        1
#define BMA255_HIGHG_FIRST_Z__MSK        0x04
#define BMA255_HIGHG_FIRST_Z__REG        BMA255_STATUS_ORIENT_HIGH_REG

#define BMA255_HIGHG_SIGN_S__POS         3
#define BMA255_HIGHG_SIGN_S__LEN         1
#define BMA255_HIGHG_SIGN_S__MSK         0x08
#define BMA255_HIGHG_SIGN_S__REG         BMA255_STATUS_ORIENT_HIGH_REG
/* EasyCASE - */
#define BMA255_ORIENT_S__POS             4
#define BMA255_ORIENT_S__LEN             3
#define BMA255_ORIENT_S__MSK             0x70
#define BMA255_ORIENT_S__REG             BMA255_STATUS_ORIENT_HIGH_REG

#define BMA255_FLAT_S__POS               7
#define BMA255_FLAT_S__LEN               1
#define BMA255_FLAT_S__MSK               0x80
#define BMA255_FLAT_S__REG               BMA255_STATUS_ORIENT_HIGH_REG
/* EasyCASE - */
//FIFO_STATUS

#define BMA255_FIFO_FRAME_COUNTER_S__POS             0
#define BMA255_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA255_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA255_FIFO_FRAME_COUNTER_S__REG             BMA255_STATUS_FIFO_REG

#define BMA255_FIFO_OVERRUN_S__POS             7
#define BMA255_FIFO_OVERRUN_S__LEN             1
#define BMA255_FIFO_OVERRUN_S__MSK             0x80
#define BMA255_FIFO_OVERRUN_S__REG             BMA255_STATUS_FIFO_REG
/* EasyCASE - */
#define BMA255_RANGE_SEL__POS             0
#define BMA255_RANGE_SEL__LEN             4
#define BMA255_RANGE_SEL__MSK             0x0F
#define BMA255_RANGE_SEL__REG             BMA255_RANGE_SEL_REG
/* EasyCASE - */
#define BMA255_BANDWIDTH__POS             0
#define BMA255_BANDWIDTH__LEN             5
#define BMA255_BANDWIDTH__MSK             0x1F
#define BMA255_BANDWIDTH__REG             BMA255_BW_SEL_REG
/* EasyCASE - */
#define BMA255_SLEEP_DUR__POS             1
#define BMA255_SLEEP_DUR__LEN             4
#define BMA255_SLEEP_DUR__MSK             0x1E
#define BMA255_SLEEP_DUR__REG             BMA255_MODE_CTRL_REG

#define BMA255_MODE_CTRL__POS             5
#define BMA255_MODE_CTRL__LEN             3
#define BMA255_MODE_CTRL__MSK             0xE0
#define BMA255_MODE_CTRL__REG             BMA255_MODE_CTRL_REG
/* EasyCASE - */
//ADDED

#define BMA255_DEEP_SUSPEND__POS          5
#define BMA255_DEEP_SUSPEND__LEN          1
#define BMA255_DEEP_SUSPEND__MSK          0x20
#define BMA255_DEEP_SUSPEND__REG          BMA255_MODE_CTRL_REG
/* EasyCASE - */
#define BMA255_EN_LOW_POWER__POS          6
#define BMA255_EN_LOW_POWER__LEN          1
#define BMA255_EN_LOW_POWER__MSK          0x40
#define BMA255_EN_LOW_POWER__REG          BMA255_MODE_CTRL_REG
/* EasyCASE - */
#define BMA255_EN_SUSPEND__POS            7
#define BMA255_EN_SUSPEND__LEN            1
#define BMA255_EN_SUSPEND__MSK            0x80
#define BMA255_EN_SUSPEND__REG            BMA255_MODE_CTRL_REG
/* EasyCASE - */
//ADDED

#define BMA255_SLEEP_TIMER__POS          5
#define BMA255_SLEEP_TIMER__LEN          1
#define BMA255_SLEEP_TIMER__MSK          0x20
#define BMA255_SLEEP_TIMER__REG          BMA255_LOW_NOISE_CTRL_REG

//ADDED
#define BMA255_LOW_POWER_MODE__POS          6
#define BMA255_LOW_POWER_MODE__LEN          1
#define BMA255_LOW_POWER_MODE__MSK          0x40
#define BMA255_LOW_POWER_MODE__REG          BMA255_LOW_NOISE_CTRL_REG

#define BMA255_EN_LOW_NOISE__POS          7
#define BMA255_EN_LOW_NOISE__LEN          1
#define BMA255_EN_LOW_NOISE__MSK          0x80
#define BMA255_EN_LOW_NOISE__REG          BMA255_LOW_NOISE_CTRL_REG
/* EasyCASE - */
/**     DISABLE MSB SHADOWING PROCEDURE          **/

#define BMA255_DIS_SHADOW_PROC__POS       6
#define BMA255_DIS_SHADOW_PROC__LEN       1
#define BMA255_DIS_SHADOW_PROC__MSK       0x40
#define BMA255_DIS_SHADOW_PROC__REG       BMA255_DATA_CTRL_REG

/**     FILTERED OR UNFILTERED ACCELERATION DATA  **/

#define BMA255_EN_DATA_HIGH_BW__POS         7
#define BMA255_EN_DATA_HIGH_BW__LEN         1
#define BMA255_EN_DATA_HIGH_BW__MSK         0x80
#define BMA255_EN_DATA_HIGH_BW__REG         BMA255_DATA_CTRL_REG
/* EasyCASE - */
/**     RESET REGISTERS                         **/

#define BMA255_EN_SOFT_RESET__POS         0
#define BMA255_EN_SOFT_RESET__LEN         8
#define BMA255_EN_SOFT_RESET__MSK         0xFF
#define BMA255_EN_SOFT_RESET__REG         BMA255_RESET_REG

#define BMA255_EN_SOFT_RESET_VALUE        0xB6
/* EasyCASE - */
/**     INTERRUPT ENABLE REGISTER              **/


#define BMA255_EN_SLOPE_X_INT__POS         0
#define BMA255_EN_SLOPE_X_INT__LEN         1
#define BMA255_EN_SLOPE_X_INT__MSK         0x01
#define BMA255_EN_SLOPE_X_INT__REG         BMA255_INT_ENABLE1_REG

#define BMA255_EN_SLOPE_Y_INT__POS         1
#define BMA255_EN_SLOPE_Y_INT__LEN         1
#define BMA255_EN_SLOPE_Y_INT__MSK         0x02
#define BMA255_EN_SLOPE_Y_INT__REG         BMA255_INT_ENABLE1_REG

#define BMA255_EN_SLOPE_Z_INT__POS         2
#define BMA255_EN_SLOPE_Z_INT__LEN         1
#define BMA255_EN_SLOPE_Z_INT__MSK         0x04
#define BMA255_EN_SLOPE_Z_INT__REG         BMA255_INT_ENABLE1_REG

#define BMA255_EN_DOUBLE_TAP_INT__POS      4
#define BMA255_EN_DOUBLE_TAP_INT__LEN      1
#define BMA255_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA255_EN_DOUBLE_TAP_INT__REG      BMA255_INT_ENABLE1_REG

#define BMA255_EN_SINGLE_TAP_INT__POS      5
#define BMA255_EN_SINGLE_TAP_INT__LEN      1
#define BMA255_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA255_EN_SINGLE_TAP_INT__REG      BMA255_INT_ENABLE1_REG

#define BMA255_EN_ORIENT_INT__POS          6
#define BMA255_EN_ORIENT_INT__LEN          1
#define BMA255_EN_ORIENT_INT__MSK          0x40
#define BMA255_EN_ORIENT_INT__REG          BMA255_INT_ENABLE1_REG

#define BMA255_EN_FLAT_INT__POS            7
#define BMA255_EN_FLAT_INT__LEN            1
#define BMA255_EN_FLAT_INT__MSK            0x80
#define BMA255_EN_FLAT_INT__REG            BMA255_INT_ENABLE1_REG
/* EasyCASE - */
/**     INTERRUPT ENABLE REGISTER              **/

#define BMA255_EN_HIGHG_X_INT__POS         0
#define BMA255_EN_HIGHG_X_INT__LEN         1
#define BMA255_EN_HIGHG_X_INT__MSK         0x01
#define BMA255_EN_HIGHG_X_INT__REG         BMA255_INT_ENABLE2_REG

#define BMA255_EN_HIGHG_Y_INT__POS         1
#define BMA255_EN_HIGHG_Y_INT__LEN         1
#define BMA255_EN_HIGHG_Y_INT__MSK         0x02
#define BMA255_EN_HIGHG_Y_INT__REG         BMA255_INT_ENABLE2_REG

#define BMA255_EN_HIGHG_Z_INT__POS         2
#define BMA255_EN_HIGHG_Z_INT__LEN         1
#define BMA255_EN_HIGHG_Z_INT__MSK         0x04
#define BMA255_EN_HIGHG_Z_INT__REG         BMA255_INT_ENABLE2_REG

#define BMA255_EN_LOWG_INT__POS            3
#define BMA255_EN_LOWG_INT__LEN            1
#define BMA255_EN_LOWG_INT__MSK            0x08
#define BMA255_EN_LOWG_INT__REG            BMA255_INT_ENABLE2_REG

#define BMA255_EN_NEW_DATA_INT__POS        4
#define BMA255_EN_NEW_DATA_INT__LEN        1
#define BMA255_EN_NEW_DATA_INT__MSK        0x10
#define BMA255_EN_NEW_DATA_INT__REG        BMA255_INT_ENABLE2_REG

//ADDED
#define BMA255_INT_FFULL_EN_INT__POS        5
#define BMA255_INT_FFULL_EN_INT__LEN        1
#define BMA255_INT_FFULL_EN_INT__MSK        0x20
#define BMA255_INT_FFULL_EN_INT__REG        BMA255_INT_ENABLE2_REG

#define BMA255_INT_FWM_EN_INT__POS        6
#define BMA255_INT_FWM_EN_INT__LEN        1
#define BMA255_INT_FWM_EN_INT__MSK        0x40
#define BMA255_INT_FWM_EN_INT__REG        BMA255_INT_ENABLE2_REG
/* EasyCASE - */
//INT SLO NO MOT

#define BMA255_INT_SLO_NO_MOT_EN_X_INT__POS        0
#define BMA255_INT_SLO_NO_MOT_EN_X_INT__LEN        1
#define BMA255_INT_SLO_NO_MOT_EN_X_INT__MSK        0x01
#define BMA255_INT_SLO_NO_MOT_EN_X_INT__REG        BMA255_INT_SLO_NO_MOT_REG

#define BMA255_INT_SLO_NO_MOT_EN_Y_INT__POS        1
#define BMA255_INT_SLO_NO_MOT_EN_Y_INT__LEN        1
#define BMA255_INT_SLO_NO_MOT_EN_Y_INT__MSK        0x02
#define BMA255_INT_SLO_NO_MOT_EN_Y_INT__REG        BMA255_INT_SLO_NO_MOT_REG

#define BMA255_INT_SLO_NO_MOT_EN_Z_INT__POS        2
#define BMA255_INT_SLO_NO_MOT_EN_Z_INT__LEN        1
#define BMA255_INT_SLO_NO_MOT_EN_Z_INT__MSK        0x04
#define BMA255_INT_SLO_NO_MOT_EN_Z_INT__REG        BMA255_INT_SLO_NO_MOT_REG

#define BMA255_INT_SLO_NO_MOT_EN_SEL_INT__POS        3
#define BMA255_INT_SLO_NO_MOT_EN_SEL_INT__LEN        1
#define BMA255_INT_SLO_NO_MOT_EN_SEL_INT__MSK        0x08
#define BMA255_INT_SLO_NO_MOT_EN_SEL_INT__REG        BMA255_INT_SLO_NO_MOT_REG
/* EasyCASE - */
#define BMA255_EN_INT1_PAD_LOWG__POS        0
#define BMA255_EN_INT1_PAD_LOWG__LEN        1
#define BMA255_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA255_EN_INT1_PAD_LOWG__REG        BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_HIGHG__POS       1
#define BMA255_EN_INT1_PAD_HIGHG__LEN       1
#define BMA255_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA255_EN_INT1_PAD_HIGHG__REG       BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_SLOPE__POS       2
#define BMA255_EN_INT1_PAD_SLOPE__LEN       1
#define BMA255_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA255_EN_INT1_PAD_SLOPE__REG       BMA255_INT1_PAD_SEL_REG

//ADDED
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__REG        BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_DB_TAP__POS      4
#define BMA255_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA255_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA255_EN_INT1_PAD_DB_TAP__REG      BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA255_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA255_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA255_EN_INT1_PAD_SNG_TAP__REG     BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_ORIENT__POS      6
#define BMA255_EN_INT1_PAD_ORIENT__LEN      1
#define BMA255_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA255_EN_INT1_PAD_ORIENT__REG      BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_FLAT__POS        7
#define BMA255_EN_INT1_PAD_FLAT__LEN        1
#define BMA255_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA255_EN_INT1_PAD_FLAT__REG        BMA255_INT1_PAD_SEL_REG
/* EasyCASE - */
#define BMA255_EN_INT2_PAD_LOWG__POS        0
#define BMA255_EN_INT2_PAD_LOWG__LEN        1
#define BMA255_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA255_EN_INT2_PAD_LOWG__REG        BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_HIGHG__POS       1
#define BMA255_EN_INT2_PAD_HIGHG__LEN       1
#define BMA255_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA255_EN_INT2_PAD_HIGHG__REG       BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_SLOPE__POS       2
#define BMA255_EN_INT2_PAD_SLOPE__LEN       1
#define BMA255_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA255_EN_INT2_PAD_SLOPE__REG       BMA255_INT2_PAD_SEL_REG

//ADDED

#define BMA255_EN_INT2_PAD_SLO_NO_MOT__POS        3
#define BMA255_EN_INT2_PAD_SLO_NO_MOT__LEN        1
#define BMA255_EN_INT2_PAD_SLO_NO_MOT__MSK        0x08
#define BMA255_EN_INT2_PAD_SLO_NO_MOT__REG        BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_DB_TAP__POS      4
#define BMA255_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA255_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA255_EN_INT2_PAD_DB_TAP__REG      BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA255_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA255_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA255_EN_INT2_PAD_SNG_TAP__REG     BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_ORIENT__POS      6
#define BMA255_EN_INT2_PAD_ORIENT__LEN      1
#define BMA255_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA255_EN_INT2_PAD_ORIENT__REG      BMA255_INT2_PAD_SEL_REG

#define BMA255_EN_INT2_PAD_FLAT__POS        7
#define BMA255_EN_INT2_PAD_FLAT__LEN        1
#define BMA255_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA255_EN_INT2_PAD_FLAT__REG        BMA255_INT2_PAD_SEL_REG
/* EasyCASE - */
#define BMA255_EN_INT1_PAD_NEWDATA__POS     0
#define BMA255_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA255_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA255_EN_INT1_PAD_NEWDATA__REG     BMA255_INT_DATA_SEL_REG

//ADDED
#define BMA255_EN_INT1_PAD_FWM__POS     1
#define BMA255_EN_INT1_PAD_FWM__LEN     1
#define BMA255_EN_INT1_PAD_FWM__MSK     0x02
#define BMA255_EN_INT1_PAD_FWM__REG     BMA255_INT_DATA_SEL_REG

#define BMA255_EN_INT1_PAD_FFULL__POS     2
#define BMA255_EN_INT1_PAD_FFULL__LEN     1
#define BMA255_EN_INT1_PAD_FFULL__MSK     0x04
#define BMA255_EN_INT1_PAD_FFULL__REG     BMA255_INT_DATA_SEL_REG

#define BMA255_EN_INT2_PAD_FFULL__POS     5
#define BMA255_EN_INT2_PAD_FFULL__LEN     1
#define BMA255_EN_INT2_PAD_FFULL__MSK     0x20
#define BMA255_EN_INT2_PAD_FFULL__REG     BMA255_INT_DATA_SEL_REG

#define BMA255_EN_INT2_PAD_FWM__POS     6
#define BMA255_EN_INT2_PAD_FWM__LEN     1
#define BMA255_EN_INT2_PAD_FWM__MSK     0x40
#define BMA255_EN_INT2_PAD_FWM__REG     BMA255_INT_DATA_SEL_REG

#define BMA255_EN_INT2_PAD_NEWDATA__POS     7
#define BMA255_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA255_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA255_EN_INT2_PAD_NEWDATA__REG     BMA255_INT_DATA_SEL_REG
/* EasyCASE - */
/*****          INTERRUPT SOURCE SELECTION                      *****/

#define BMA255_UNFILT_INT_SRC_LOWG__POS        0
#define BMA255_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA255_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA255_UNFILT_INT_SRC_LOWG__REG        BMA255_INT_SRC_REG

#define BMA255_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA255_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA255_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA255_UNFILT_INT_SRC_HIGHG__REG       BMA255_INT_SRC_REG

#define BMA255_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA255_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA255_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA255_UNFILT_INT_SRC_SLOPE__REG       BMA255_INT_SRC_REG

//ADDED
#define BMA255_UNFILT_INT_SRC_SLO_NO_MOT__POS        3
#define BMA255_UNFILT_INT_SRC_SLO_NO_MOT__LEN        1
#define BMA255_UNFILT_INT_SRC_SLO_NO_MOT__MSK        0x08
#define BMA255_UNFILT_INT_SRC_SLO_NO_MOT__REG        BMA255_INT_SRC_REG

#define BMA255_UNFILT_INT_SRC_TAP__POS         4
#define BMA255_UNFILT_INT_SRC_TAP__LEN         1
#define BMA255_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA255_UNFILT_INT_SRC_TAP__REG         BMA255_INT_SRC_REG

#define BMA255_UNFILT_INT_SRC_DATA__POS        5
#define BMA255_UNFILT_INT_SRC_DATA__LEN        1
#define BMA255_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA255_UNFILT_INT_SRC_DATA__REG        BMA255_INT_SRC_REG
/* EasyCASE - */
/*****  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE       *****/

#define BMA255_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA255_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA255_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA255_INT1_PAD_ACTIVE_LEVEL__REG       BMA255_INT_SET_REG

#define BMA255_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA255_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA255_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA255_INT2_PAD_ACTIVE_LEVEL__REG       BMA255_INT_SET_REG


/*****  OUTPUT TYPE IF SET TO 1 IS : OPEN DRIVE , IF NOT SET
        IT IS PUSH-PULL                                  *****/


#define BMA255_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA255_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA255_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA255_INT1_PAD_OUTPUT_TYPE__REG        BMA255_INT_SET_REG

#define BMA255_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA255_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA255_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA255_INT2_PAD_OUTPUT_TYPE__REG        BMA255_INT_SET_REG
/* EasyCASE - */
/*****               INTERRUPT MODE SELECTION              ******/

#define BMA255_LATCH_INT__POS                0
#define BMA255_LATCH_INT__LEN                4
#define BMA255_LATCH_INT__MSK                0x0F
#define BMA255_LATCH_INT__REG                BMA255_INT_CTRL_REG

/*****               LATCHED INTERRUPT RESET               ******/

#define BMA255_RESET_INT__POS           7
#define BMA255_RESET_INT__LEN           1
#define BMA255_RESET_INT__MSK           0x80
#define BMA255_RESET_INT__REG           BMA255_INT_CTRL_REG
/* EasyCASE - */
/*****               LOW-G DURATION                        ******/

#define BMA255_LOWG_DUR__POS                    0
#define BMA255_LOWG_DUR__LEN                    8
#define BMA255_LOWG_DUR__MSK                    0xFF
#define BMA255_LOWG_DUR__REG                    BMA255_LOW_DURN_REG

/*****               LOW-G THRESHOLD                       ******/

#define BMA255_LOWG_THRES__POS                  0
#define BMA255_LOWG_THRES__LEN                  8
#define BMA255_LOWG_THRES__MSK                  0xFF
#define BMA255_LOWG_THRES__REG                  BMA255_LOW_THRES_REG

/*****               LOW-G HYSTERESIS                       ******/

#define BMA255_LOWG_HYST__POS                   0
#define BMA255_LOWG_HYST__LEN                   2
#define BMA255_LOWG_HYST__MSK                   0x03
#define BMA255_LOWG_HYST__REG                   BMA255_LOW_HIGH_HYST_REG

/*****               LOW-G INTERRUPT MODE                   ******/
/*****       IF 1 -- SUM MODE , 0 -- SINGLE MODE            ******/
#define BMA255_LOWG_INT_MODE__POS               2
#define BMA255_LOWG_INT_MODE__LEN               1
#define BMA255_LOWG_INT_MODE__MSK               0x04
#define BMA255_LOWG_INT_MODE__REG               BMA255_LOW_HIGH_HYST_REG
/* EasyCASE - */
/*****               HIGH-G DURATION                        ******/

#define BMA255_HIGHG_DUR__POS                    0
#define BMA255_HIGHG_DUR__LEN                    8
#define BMA255_HIGHG_DUR__MSK                    0xFF
#define BMA255_HIGHG_DUR__REG                    BMA255_HIGH_DURN_REG

/*****               HIGH-G THRESHOLD                       ******/

#define BMA255_HIGHG_THRES__POS                  0
#define BMA255_HIGHG_THRES__LEN                  8
#define BMA255_HIGHG_THRES__MSK                  0xFF
#define BMA255_HIGHG_THRES__REG                  BMA255_HIGH_THRES_REG


/*****               HIGH-G HYSTERESIS                       ******/

#define BMA255_HIGHG_HYST__POS                  6
#define BMA255_HIGHG_HYST__LEN                  2
#define BMA255_HIGHG_HYST__MSK                  0xC0
#define BMA255_HIGHG_HYST__REG                  BMA255_LOW_HIGH_HYST_REG
/* EasyCASE - */
/*****               SLOPE DURATION                        ******/

#define BMA255_SLOPE_DUR__POS                    0
#define BMA255_SLOPE_DUR__LEN                    2
#define BMA255_SLOPE_DUR__MSK                    0x03
#define BMA255_SLOPE_DUR__REG                    BMA255_SLOPE_DURN_REG

//SLO_NO_MOT_DUR ADDED
#define BMA255_SLO_NO_MOT_DUR__POS                    2
#define BMA255_SLO_NO_MOT_DUR__LEN                    6
#define BMA255_SLO_NO_MOT_DUR__MSK                    0xFC
#define BMA255_SLO_NO_MOT_DUR__REG                    BMA255_SLOPE_DURN_REG
/* EasyCASE - */
/*****               SLOPE THRESHOLD                       ******/

#define BMA255_SLOPE_THRES__POS                  0
#define BMA255_SLOPE_THRES__LEN                  8
#define BMA255_SLOPE_THRES__MSK                  0xFF
#define BMA255_SLOPE_THRES__REG                  BMA255_SLOPE_THRES_REG
/* EasyCASE - */
/*****               SLO_NO_MOT THRESHOLD                       ******/
//ADDED

#define BMA255_SLO_NO_MOT_THRES__POS                  0
#define BMA255_SLO_NO_MOT_THRES__LEN                  8
#define BMA255_SLO_NO_MOT_THRES__MSK                  0xFF
#define BMA255_SLO_NO_MOT_THRES__REG                  BMA255_SLO_NO_MOT_THRES_REG
/* EasyCASE - */
/*****               TAP DURATION                        ******/

#define BMA255_TAP_DUR__POS                    0
#define BMA255_TAP_DUR__LEN                    3
#define BMA255_TAP_DUR__MSK                    0x07
#define BMA255_TAP_DUR__REG                    BMA255_TAP_PARAM_REG

/*****               TAP SHOCK DURATION                 ******/

#define BMA255_TAP_SHOCK_DURN__POS             6
#define BMA255_TAP_SHOCK_DURN__LEN             1
#define BMA255_TAP_SHOCK_DURN__MSK             0x40
#define BMA255_TAP_SHOCK_DURN__REG             BMA255_TAP_PARAM_REG

/*****               ADV TAP INT                        ******/

#define BMA255_ADV_TAP_INT__POS                5
#define BMA255_ADV_TAP_INT__LEN                1
#define BMA255_ADV_TAP_INT__MSK                0x20
#define BMA255_ADV_TAP_INT__REG                BMA255_TAP_PARAM_REG

/*****               TAP QUIET DURATION                 ******/

#define BMA255_TAP_QUIET_DURN__POS             7
#define BMA255_TAP_QUIET_DURN__LEN             1
#define BMA255_TAP_QUIET_DURN__MSK             0x80
#define BMA255_TAP_QUIET_DURN__REG             BMA255_TAP_PARAM_REG
/* EasyCASE - */
/*****               TAP THRESHOLD                       ******/

#define BMA255_TAP_THRES__POS                  0
#define BMA255_TAP_THRES__LEN                  5
#define BMA255_TAP_THRES__MSK                  0x1F
#define BMA255_TAP_THRES__REG                  BMA255_TAP_THRES_REG

/*****               TAP SAMPLES                         ******/

#define BMA255_TAP_SAMPLES__POS                6
#define BMA255_TAP_SAMPLES__LEN                2
#define BMA255_TAP_SAMPLES__MSK                0xC0
#define BMA255_TAP_SAMPLES__REG                BMA255_TAP_THRES_REG
/* EasyCASE - */
/*****       ORIENTATION MODE                        ******/

#define BMA255_ORIENT_MODE__POS                  0
#define BMA255_ORIENT_MODE__LEN                  2
#define BMA255_ORIENT_MODE__MSK                  0x03
#define BMA255_ORIENT_MODE__REG                  BMA255_ORIENT_PARAM_REG

/*****       ORIENTATION BLOCKING                    ******/

#define BMA255_ORIENT_BLOCK__POS                 2
#define BMA255_ORIENT_BLOCK__LEN                 2
#define BMA255_ORIENT_BLOCK__MSK                 0x0C
#define BMA255_ORIENT_BLOCK__REG                 BMA255_ORIENT_PARAM_REG

/*****       ORIENTATION HYSTERESIS                  ******/

#define BMA255_ORIENT_HYST__POS                  4
#define BMA255_ORIENT_HYST__LEN                  3
#define BMA255_ORIENT_HYST__MSK                  0x70
#define BMA255_ORIENT_HYST__REG                  BMA255_ORIENT_PARAM_REG
/* EasyCASE - */
/*****       ORIENTATION AXIS SELECTION              ******/
/***** IF SET TO 1 -- X AND Z ARE SWAPPED , Y IS INVERTED */

#define BMA255_ORIENT_AXIS__POS                  7
#define BMA255_ORIENT_AXIS__LEN                  1
#define BMA255_ORIENT_AXIS__MSK                  0x80
#define BMA255_ORIENT_AXIS__REG                  BMA255_THETA_BLOCK_REG

//ADDED
#define BMA255_ORIENT_UD_EN__POS                  6
#define BMA255_ORIENT_UD_EN__LEN                  1
#define BMA255_ORIENT_UD_EN__MSK                  0x40
#define BMA255_ORIENT_UD_EN__REG                  BMA255_THETA_BLOCK_REG

/*****       THETA BLOCKING                    ******/

#define BMA255_THETA_BLOCK__POS                  0
#define BMA255_THETA_BLOCK__LEN                  6
#define BMA255_THETA_BLOCK__MSK                  0x3F
#define BMA255_THETA_BLOCK__REG                  BMA255_THETA_BLOCK_REG
/* EasyCASE - */
/*****       THETA FLAT                        ******/

#define BMA255_THETA_FLAT__POS                  0
#define BMA255_THETA_FLAT__LEN                  6
#define BMA255_THETA_FLAT__MSK                  0x3F
#define BMA255_THETA_FLAT__REG                  BMA255_THETA_FLAT_REG
/* EasyCASE - */
/*****      FLAT HOLD TIME                     ******/

#define BMA255_FLAT_HOLD_TIME__POS              4
#define BMA255_FLAT_HOLD_TIME__LEN              2
#define BMA255_FLAT_HOLD_TIME__MSK              0x30
#define BMA255_FLAT_HOLD_TIME__REG              BMA255_FLAT_HOLD_TIME_REG
/* EasyCASE - */
/*****      FLAT HYS                           ******/

#define BMA255_FLAT_HYS__POS                   0
#define BMA255_FLAT_HYS__LEN                   3
#define BMA255_FLAT_HYS__MSK                   0x07
#define BMA255_FLAT_HYS__REG                   BMA255_FLAT_HOLD_TIME_REG
/* EasyCASE - */
/*****      FIFO WATER MARK LEVEL TRIGGER RETAIN                        ******/
//ADDED
#define BMA255_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA255_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA255_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA255_FIFO_WML_TRIG_RETAIN__REG                   BMA255_FIFO_WML_TRIG
/* EasyCASE - */
/*****      ACTIVATE SELF TEST                 ******/

#define BMA255_EN_SELF_TEST__POS                0
#define BMA255_EN_SELF_TEST__LEN                2
#define BMA255_EN_SELF_TEST__MSK                0x03
#define BMA255_EN_SELF_TEST__REG                BMA255_SELF_TEST_REG

/*****     SELF TEST -- NEGATIVE               ******/

#define BMA255_NEG_SELF_TEST__POS               2
#define BMA255_NEG_SELF_TEST__LEN               1
#define BMA255_NEG_SELF_TEST__MSK               0x04
#define BMA255_NEG_SELF_TEST__REG               BMA255_SELF_TEST_REG

/*****     SELF TEST AMPLITUDE                 ******/

#define BMA255_SELF_TEST_AMP__POS               4
#define BMA255_SELF_TEST_AMP__LEN               3
#define BMA255_SELF_TEST_AMP__MSK               0x70
#define BMA255_SELF_TEST_AMP__REG               BMA255_SELF_TEST_REG
/* EasyCASE - */
/*****     EEPROM CONTROL                      ******/

/* SETTING THIS BIT  UNLOCK'S WRITING SETTING REGISTERS TO EEPROM */

#define BMA255_UNLOCK_EE_PROG_MODE__POS     0
#define BMA255_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA255_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA255_UNLOCK_EE_PROG_MODE__REG     BMA255_EEPROM_CTRL_REG


/* SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */

#define BMA255_START_EE_PROG_TRIG__POS      1
#define BMA255_START_EE_PROG_TRIG__LEN      1
#define BMA255_START_EE_PROG_TRIG__MSK      0x02
#define BMA255_START_EE_PROG_TRIG__REG      BMA255_EEPROM_CTRL_REG


/* STATUS OF WRITING TO EEPROM */

#define BMA255_EE_PROG_READY__POS          2
#define BMA255_EE_PROG_READY__LEN          1
#define BMA255_EE_PROG_READY__MSK          0x04
#define BMA255_EE_PROG_READY__REG          BMA255_EEPROM_CTRL_REG


/* UPDATE IMAGE REGISTERS WRITING TO EEPROM */

#define BMA255_UPDATE_IMAGE__POS                3
#define BMA255_UPDATE_IMAGE__LEN                1
#define BMA255_UPDATE_IMAGE__MSK                0x08
#define BMA255_UPDATE_IMAGE__REG                BMA255_EEPROM_CTRL_REG

/* ADDED */

#define BMA255_EE_REMAIN__POS                4
#define BMA255_EE_REMAIN__LEN                4
#define BMA255_EE_REMAIN__MSK                0xF0
#define BMA255_EE_REMAIN__REG                BMA255_EEPROM_CTRL_REG
/* EasyCASE - */
/* SPI INTERFACE MODE SELECTION */

#define BMA255_EN_SPI_MODE_3__POS              0
#define BMA255_EN_SPI_MODE_3__LEN              1
#define BMA255_EN_SPI_MODE_3__MSK              0x01
#define BMA255_EN_SPI_MODE_3__REG              BMA255_SERIAL_CTRL_REG

/* I2C WATCHDOG PERIOD SELECTION */

#define BMA255_I2C_WATCHDOG_PERIOD__POS        1
#define BMA255_I2C_WATCHDOG_PERIOD__LEN        1
#define BMA255_I2C_WATCHDOG_PERIOD__MSK        0x02
#define BMA255_I2C_WATCHDOG_PERIOD__REG        BMA255_SERIAL_CTRL_REG

/* I2C WATCHDOG ENABLE */

#define BMA255_EN_I2C_WATCHDOG__POS            2
#define BMA255_EN_I2C_WATCHDOG__LEN            1
#define BMA255_EN_I2C_WATCHDOG__MSK            0x04
#define BMA255_EN_I2C_WATCHDOG__REG            BMA255_SERIAL_CTRL_REG
/* EasyCASE - */
/* SPI INTERFACE MODE SELECTION */
//ADDED
#define BMA255_EXT_MODE__POS              7
#define BMA255_EXT_MODE__LEN              1
#define BMA255_EXT_MODE__MSK              0x80
#define BMA255_EXT_MODE__REG              BMA255_EXTMODE_CTRL_REG

#define BMA255_ALLOW_UPPER__POS        6
#define BMA255_ALLOW_UPPER__LEN        1
#define BMA255_ALLOW_UPPER__MSK        0x40
#define BMA255_ALLOW_UPPER__REG        BMA255_EXTMODE_CTRL_REG

#define BMA255_MAP_2_LOWER__POS            5
#define BMA255_MAP_2_LOWER__LEN            1
#define BMA255_MAP_2_LOWER__MSK            0x20
#define BMA255_MAP_2_LOWER__REG            BMA255_EXTMODE_CTRL_REG

#define BMA255_MAGIC_NUMBER__POS            0
#define BMA255_MAGIC_NUMBER__LEN            5
#define BMA255_MAGIC_NUMBER__MSK            0x1F
#define BMA255_MAGIC_NUMBER__REG            BMA255_EXTMODE_CTRL_REG
/* EasyCASE - */
/* SETTING THIS BIT  UNLOCK'S WRITING TRIMMING REGISTERS TO EEPROM */

#define BMA255_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA255_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA255_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA255_UNLOCK_EE_WRITE_TRIM__REG        BMA255_CTRL_UNLOCK_REG
/* EasyCASE - */
/**    OFFSET  COMPENSATION     **/

/**    SLOW COMPENSATION FOR X,Y,Z AXIS      **/

#define BMA255_EN_SLOW_COMP_X__POS              0
#define BMA255_EN_SLOW_COMP_X__LEN              1
#define BMA255_EN_SLOW_COMP_X__MSK              0x01
#define BMA255_EN_SLOW_COMP_X__REG              BMA255_OFFSET_CTRL_REG

#define BMA255_EN_SLOW_COMP_Y__POS              1
#define BMA255_EN_SLOW_COMP_Y__LEN              1
#define BMA255_EN_SLOW_COMP_Y__MSK              0x02
#define BMA255_EN_SLOW_COMP_Y__REG              BMA255_OFFSET_CTRL_REG

#define BMA255_EN_SLOW_COMP_Z__POS              2
#define BMA255_EN_SLOW_COMP_Z__LEN              1
#define BMA255_EN_SLOW_COMP_Z__MSK              0x04
#define BMA255_EN_SLOW_COMP_Z__REG              BMA255_OFFSET_CTRL_REG

/**    FAST COMPENSATION READY FLAG          **/
//ADDED
#define BMA255_FAST_CAL_RDY_S__POS             4
#define BMA255_FAST_CAL_RDY_S__LEN             1
#define BMA255_FAST_CAL_RDY_S__MSK             0x10
#define BMA255_FAST_CAL_RDY_S__REG             BMA255_OFFSET_CTRL_REG

/**    FAST COMPENSATION FOR X,Y,Z AXIS      **/

#define BMA255_CAL_TRIGGER__POS                5
#define BMA255_CAL_TRIGGER__LEN                2
#define BMA255_CAL_TRIGGER__MSK                0x60
#define BMA255_CAL_TRIGGER__REG                BMA255_OFFSET_CTRL_REG

/**    RESET OFFSET REGISTERS                **/

#define BMA255_RESET_OFFSET_REGS__POS           7
#define BMA255_RESET_OFFSET_REGS__LEN           1
#define BMA255_RESET_OFFSET_REGS__MSK           0x80
#define BMA255_RESET_OFFSET_REGS__REG           BMA255_OFFSET_CTRL_REG
/* EasyCASE - */
/**     SLOW COMPENSATION  CUTOFF               **/

#define BMA255_COMP_CUTOFF__POS                 0
#define BMA255_COMP_CUTOFF__LEN                 1
#define BMA255_COMP_CUTOFF__MSK                 0x01
#define BMA255_COMP_CUTOFF__REG                 BMA255_OFFSET_PARAMS_REG

/**     COMPENSATION TARGET                  **/

#define BMA255_COMP_TARGET_OFFSET_X__POS        1
#define BMA255_COMP_TARGET_OFFSET_X__LEN        2
#define BMA255_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA255_COMP_TARGET_OFFSET_X__REG        BMA255_OFFSET_PARAMS_REG

#define BMA255_COMP_TARGET_OFFSET_Y__POS        3
#define BMA255_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA255_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA255_COMP_TARGET_OFFSET_Y__REG        BMA255_OFFSET_PARAMS_REG

#define BMA255_COMP_TARGET_OFFSET_Z__POS        5
#define BMA255_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA255_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA255_COMP_TARGET_OFFSET_Z__REG        BMA255_OFFSET_PARAMS_REG
/* EasyCASE - */
/**     FIFO DATA SELECT              **/
//ADDED 

#define BMA255_FIFO_DATA_SELECT__POS                 0
#define BMA255_FIFO_DATA_SELECT__LEN                 2
#define BMA255_FIFO_DATA_SELECT__MSK                 0x03
#define BMA255_FIFO_DATA_SELECT__REG                 BMA255_FIFO_MODE_REG

//FIFO TRIGGER SOURCE

#define BMA255_FIFO_TRIGGER_SOURCE__POS                 2
#define BMA255_FIFO_TRIGGER_SOURCE__LEN                 2
#define BMA255_FIFO_TRIGGER_SOURCE__MSK                 0x0C
#define BMA255_FIFO_TRIGGER_SOURCE__REG                 BMA255_FIFO_MODE_REG

//FIFO TRIGGER ACTION

#define BMA255_FIFO_TRIGGER_ACTION__POS                 4
#define BMA255_FIFO_TRIGGER_ACTION__LEN                 2
#define BMA255_FIFO_TRIGGER_ACTION__MSK                 0x30
#define BMA255_FIFO_TRIGGER_ACTION__REG                 BMA255_FIFO_MODE_REG

//FIFO MODE

#define BMA255_FIFO_MODE__POS                 6
#define BMA255_FIFO_MODE__LEN                 2
#define BMA255_FIFO_MODE__MSK                 0xC0
#define BMA255_FIFO_MODE__REG                 BMA255_FIFO_MODE_REG
/* EasyCASE ) */
#define BMA255_GET_BITSLICE(regvar, bitname)\
                        (regvar & bitname##__MSK) >> bitname##__POS


#define BMA255_SET_BITSLICE(regvar, bitname, val)\
                  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)


/** \endcond */


/* CONSTANTS */

#define BMA255_STATUS1                             0               /**< It refers BMA255 STATUS_INT1 */
#define BMA255_STATUS2                             1               /**< It refers BMA255 STATUS_INT2 */
#define BMA255_STATUS3                             2               /**< It refers BMA255 STATUS_INT_TAP */
#define BMA255_STATUS4                             3               /**< It refers BMA255 STATUS_INT_ORIENT */
#define BMA255_STATUS5                             4               /**< It refers BMA255 STATUS_INT_FIFO */


#define E_BMA255_OUT_OF_RANGE                      (char)-2


/* range and bandwidth */

#define BMA255_RANGE_2G                 3 /**< sets range to +/- 2G mode \see BMA255_set_range() */
#define BMA255_RANGE_4G                 5 /**< sets range to +/- 4G mode \see BMA255_set_range() */
#define BMA255_RANGE_8G                 8 /**< sets range to +/- 8G mode \see BMA255_set_range() */
#define BMA255_RANGE_16G                12/**< sets range to +/- 16G mode \see BMA255_set_range() */


#define BMA255_BW_7_81HZ        0x08       /**< sets bandwidth to LowPass 7.81  HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_15_63HZ       0x09       /**< sets bandwidth to LowPass 15.63 HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_31_25HZ       0x0A       /**< sets bandwidth to LowPass 31.25 HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_62_50HZ       0x0B       /**< sets bandwidth to LowPass 62.50 HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_125HZ         0x0C       /**< sets bandwidth to LowPass 125HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_250HZ         0x0D       /**< sets bandwidth to LowPass 250HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_500HZ         0x0E       /**< sets bandwidth to LowPass 500HZ \see BMA255_set_bandwidth() */
#define BMA255_BW_1000HZ        0x0F       /**< sets bandwidth to LowPass 1000HZ \see BMA255_set_bandwidth() */

/*        SLEEP DURATION              */

#define BMA255_SLEEP_DUR_0_5MS        0x05       /* sets sleep duration to 0.5 ms  */
#define BMA255_SLEEP_DUR_1MS          0x06       /* sets sleep duration to 1 ms */
#define BMA255_SLEEP_DUR_2MS          0x07       /* sets sleep duration to 2 ms */
#define BMA255_SLEEP_DUR_4MS          0x08       /* sets sleep duration to 4 ms */
#define BMA255_SLEEP_DUR_6MS          0x09       /* sets sleep duration to 6 ms*/
#define BMA255_SLEEP_DUR_10MS         0x0A       /* sets sleep duration to 10 ms */
#define BMA255_SLEEP_DUR_25MS         0x0B       /* sets sleep duration to 25 ms */
#define BMA255_SLEEP_DUR_50MS         0x0C       /* sets sleep duration to 50 ms */
#define BMA255_SLEEP_DUR_100MS        0x0D       /* sets sleep duration to 100 ms */
#define BMA255_SLEEP_DUR_500MS        0x0E        /* sets sleep duration to 500 ms */
#define BMA255_SLEEP_DUR_1S           0x0F        /* sets sleep duration to 1 s */

/*        LATCH DURATION              */

#define BMA255_LATCH_DUR_NON_LATCH    0x00       /* sets LATCH duration to NON LATCH  */
#define BMA255_LATCH_DUR_250MS        0x01       /* sets LATCH duration to 250 ms */
#define BMA255_LATCH_DUR_500MS        0x02       /* sets LATCH duration to 500 ms */
#define BMA255_LATCH_DUR_1S           0x03       /* sets LATCH duration to 1 s */
#define BMA255_LATCH_DUR_2S           0x04       /* sets LATCH duration to 2 s*/
#define BMA255_LATCH_DUR_4S           0x05       /* sets LATCH duration to 4 s */
#define BMA255_LATCH_DUR_8S           0x06       /* sets LATCH duration to 8 s */
#define BMA255_LATCH_DUR_LATCH        0x07       /* sets LATCH duration to LATCH */
#define BMA255_LATCH_DUR_NON_LATCH1   0x08       /* sets LATCH duration to NON LATCH1 */
#define BMA255_LATCH_DUR_250US        0x09        /* sets LATCH duration to 250 Us */
#define BMA255_LATCH_DUR_500US        0x0A        /* sets LATCH duration to 500 Us */
#define BMA255_LATCH_DUR_1MS          0x0B        /* sets LATCH duration to 1 Ms */
#define BMA255_LATCH_DUR_12_5MS       0x0C        /* sets LATCH duration to 12.5 Ms */
#define BMA255_LATCH_DUR_25MS         0x0D        /* sets LATCH duration to 25 Ms */
#define BMA255_LATCH_DUR_50MS         0x0E        /* sets LATCH duration to 50 Ms */
#define BMA255_LATCH_DUR_LATCH1       0x0F        /* sets LATCH duration to LATCH*/

/* mode settings */

#define BMA255_MODE_NORMAL             0
#define BMA255_MODE_LOWPOWER1          1
#define BMA255_MODE_SUSPEND            2
#define BMA255_MODE_DEEP_SUSPEND       3
#define BMA255_MODE_LOWPOWER2          4
#define BMA255_MODE_STANDBY            5

/* BMA255 AXIS      */

#define BMA255_X_AXIS           0                            /**< It refers BMA255 X-axis */
#define BMA255_Y_AXIS           1                            /**< It refers BMA255 Y-axis */
#define BMA255_Z_AXIS           2                            /**< It refers BMA255 Z-axis */

/*  INTERRUPT TYPES    */

#define BMA255_Low_G_Interrupt       0
#define BMA255_High_G_X_Interrupt    1
#define BMA255_High_G_Y_Interrupt    2
#define BMA255_High_G_Z_Interrupt    3
#define BMA255_DATA_EN               4
#define BMA255_Slope_X_Interrupt     5
#define BMA255_Slope_Y_Interrupt     6
#define BMA255_Slope_Z_Interrupt     7
#define BMA255_Single_Tap_Interrupt  8
#define BMA255_Double_Tap_Interrupt  9
#define BMA255_Orient_Interrupt      10
#define BMA255_Flat_Interrupt        11
#define BMA255_FFULL_INTERRUPT       12
#define BMA255_FWM_INTERRUPT         13

/*  INTERRUPTS PADS  */

#define BMA255_INT1_LOWG         0
#define BMA255_INT2_LOWG         1
#define BMA255_INT1_HIGHG        0
#define BMA255_INT2_HIGHG        1
#define BMA255_INT1_SLOPE        0
#define BMA255_INT2_SLOPE        1
#define BMA255_INT1_SLO_NO_MOT   0
#define BMA255_INT2_SLO_NO_MOT   1
#define BMA255_INT1_DTAP         0
#define BMA255_INT2_DTAP         1
#define BMA255_INT1_STAP         0
#define BMA255_INT2_STAP         1
#define BMA255_INT1_ORIENT       0
#define BMA255_INT2_ORIENT       1
#define BMA255_INT1_FLAT         0
#define BMA255_INT2_FLAT         1
#define BMA255_INT1_NDATA        0
#define BMA255_INT2_NDATA        1
#define BMA255_INT1_FWM          0
#define BMA255_INT2_FWM          1
#define BMA255_INT1_FFULL        0
#define BMA255_INT2_FFULL        1

/*       SOURCE REGISTER        */

#define BMA255_SRC_LOWG         0
#define BMA255_SRC_HIGHG        1
#define BMA255_SRC_SLOPE        2
#define BMA255_SRC_SLO_NO_MOT   3
#define BMA255_SRC_TAP          4
#define BMA255_SRC_DATA         5

#define BMA255_INT1_OUTPUT      0
#define BMA255_INT2_OUTPUT      1
#define BMA255_INT1_LEVEL       0
#define BMA255_INT2_LEVEL       1

/*    DURATION         */

#define BMA255_LOW_DURATION            0
#define BMA255_HIGH_DURATION           1
#define BMA255_SLOPE_DURATION          2
#define BMA255_SLO_NO_MOT_DURATION     3

/*      THRESHOLD        */

#define BMA255_LOW_THRESHOLD            0
#define BMA255_HIGH_THRESHOLD           1
#define BMA255_SLOPE_THRESHOLD          2
#define BMA255_SLO_NO_MOT_THRESHOLD     3


#define BMA255_LOWG_HYST                0
#define BMA255_HIGHG_HYST               1

#define BMA255_ORIENT_THETA             0
#define BMA255_FLAT_THETA               1

#define BMA255_I2C_SELECT               0
#define BMA255_I2C_EN                   1

/*    COMPENSATION           */

#define BMA255_SLOW_COMP_X              0
#define BMA255_SLOW_COMP_Y              1
#define BMA255_SLOW_COMP_Z              2

/*       OFFSET TRIGGER          */

#define BMA255_CUT_OFF                  0
#define BMA255_OFFSET_TRIGGER_X         1
#define BMA255_OFFSET_TRIGGER_Y         2
#define BMA255_OFFSET_TRIGGER_Z         3

/*       GP REGISTERS           */

#define BMA255_GP0                      0
#define BMA255_GP1                      1

/*    SLO NO MOT REGISTER          */

#define BMA255_SLO_NO_MOT_EN_X          0
#define BMA255_SLO_NO_MOT_EN_Y          1
#define BMA255_SLO_NO_MOT_EN_Z          2
#define BMA255_SLO_NO_MOT_EN_SEL        3


/* wake up */

#define BMA255_WAKE_UP_DUR_20MS         0
#define BMA255_WAKE_UP_DUR_80MS         1
#define BMA255_WAKE_UP_DUR_320MS                2
#define BMA255_WAKE_UP_DUR_2560MS               3


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define BMA255_SELF_TEST0_ON            1
#define BMA255_SELF_TEST1_ON            2

#define BMA255_EE_W_OFF                 0
#define BMA255_EE_W_ON                  1

/* Resolution Settings */
#define BMA255_RESOLUTION_12_BIT        0
#define BMA255_RESOLUTION_10_BIT        1
#define BMA255_RESOLUTION_8_BIT         2
#define BMA255_RESOLUTION_14_BIT        3

#define BMA255           0x16 
#define BMA280           0x17
#define BMA222E          0x01 
#define BMA250E          0x21
#define BMA254           0x41
/* EasyCASE ( 925
   MACRO's to convert g values to register values */
/** Macro to convert floating point low-g-thresholds in G to 8-bit register values.<br>
  * Example: BMA255_LOW_TH_IN_G( 0.3, 2.0) generates the register value for 0.3G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA255_LOW_TH_IN_G( gthres, range)                      ((256 * gthres ) / range)

/** Macro to convert floating point high-g-thresholds in G to 8-bit register values.<br>
  * Example: BMA255_HIGH_TH_IN_G( 1.4, 2.0) generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA255_HIGH_TH_IN_G(gthres, range)                              ((256 * gthres ) / range)

/** Macro to convert floating point low-g-hysteresis in G to 8-bit register values.<br>
  * Example: BMA255_LOW_HY_IN_G( 0.2, 2.0) generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA255_LOW_HY_IN_G( ghyst, range )                              ((32 * ghyst) / range)

/** Macro to convert floating point high-g-hysteresis in G to 8-bit register values.<br>
  * Example: BMA255_HIGH_HY_IN_G( 0.2, 2.0) generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA255_HIGH_HY_IN_G( ghyst, range )                             ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds to 8-bit register values<br>
  * Example: BMA255_SLOPE_TH_IN_G( 1.2, 2.0) generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define BMA255_SLOPE_TH_IN_G( gthres, range)    ((128 * gthres ) / range)
/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ( 76
   ENUM and struct Definitions */
/*user defined Enums*/
/* EasyCASE - */
/* EasyCASE < */
//Example..
//enum {
//E_YOURDATA1, /**< <DOXY Comment for E_YOURDATA1> */
//E_YOURDATA2  /**< <DOXY Comment for E_YOURDATA2> */
//};
/* EasyCASE > */
/* EasyCASE - */
/* EasyCASE < */
//Example...
//struct DUMMY_STRUCT {
//data1, /**< <DOXY Comment for data1> */
//data2  /**< <DOXY Comment for data1> */
//};
/* EasyCASE > */
/* EasyCASE ) */
/* EasyCASE ( 79
   Public API Declarations */
/* EasyCASE ( 1601
   bma255_burst_read */
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *  
 *
 *
 *
 *  \param unsigned char addr, unsigned char *data
 *                       addr -> Address of the register
 *                       data -> address of the variable, read value will be kept
 *  \return  results of bus communication function
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
BMA255_RETURN_FUNCTION_TYPE bma255_burst_read(unsigned char addr, unsigned char *data, unsigned int len);
/* EasyCASE ) */
/* EasyCASE ( 928
   bma255_init */
int bma255_init(bma255_t *bma255) ;
/* EasyCASE ) */
/* EasyCASE ( 938
   bma255_write_reg */
int bma255_write_reg(unsigned char addr, unsigned char *data, unsigned char len);
/* EasyCASE ) */
/* EasyCASE ( 939
   bma255_read_reg */
int bma255_read_reg(unsigned char addr, unsigned char *data, unsigned char len);
/* EasyCASE ) */
/* EasyCASE ( 942
   bma255_read_accel_x */
int bma255_read_accel_x(short *a_x);
/* EasyCASE ) */
/* EasyCASE ( 943
   bma255_read_accel_y */
int bma255_read_accel_y(short *a_y);
/* EasyCASE ) */
/* EasyCASE ( 944
   bma255_read_accel_z */
int bma255_read_accel_z(short *a_z);
/* EasyCASE ) */
/* EasyCASE ( 941
   bma255_read_accel_xyz */
int bma255_read_accel_xyz(bma255acc_t * acc);
/* EasyCASE ) */
/* EasyCASE ( 1483
   bma255_get_temp */
int bma255_get_temp(unsigned char *Temp );
/* EasyCASE ) */
/* EasyCASE ( 946
   bma255_get_interrupt_status1 */
int bma255_get_interrupt_status1(unsigned char *status_1);
/* EasyCASE ) */
/* EasyCASE ( 1487
   bma255_get_interrupt_status2 */
int bma255_get_interrupt_status2(unsigned char *status_2);
/* EasyCASE ) */
/* EasyCASE ( 1491
   bma255_get_int_tap_status */
int bma255_get_int_tap_status(unsigned char * status_tap);
/* EasyCASE ) */
/* EasyCASE ( 1493
   bma255_get_int_orient_status */
int bma255_get_int_orient_status(unsigned char *status_orient);
/* EasyCASE ) */
/* EasyCASE ( 1495
   bma255_get_fifo_status */
int bma255_get_fifo_status(unsigned char *status_fifo);
/* EasyCASE ) */
/* EasyCASE ( 1497
   bma255_get_fifo_framecount */
int bma255_get_fifo_framecount(unsigned char *framecount);
/* EasyCASE ) */
/* EasyCASE ( 1499
   bma255_get_fifo_overrun */
int bma255_get_fifo_overrun(unsigned char *overrun);
/* EasyCASE ) */
/* EasyCASE ( 1549
   bma255_get_interrupt_status */
int bma255_get_interrupt_status(unsigned char *status);
/* EasyCASE ) */
/* EasyCASE ( 935
   bma255_get_range */
int bma255_get_range(unsigned char * Range );
/* EasyCASE ) */
/* EasyCASE ( 934
   bma255_set_range */
int bma255_set_range(unsigned char Range);
/* EasyCASE ) */
/* EasyCASE ( 937
   bma255_get_bandwidth */
int bma255_get_bandwidth(unsigned char * bw);
/* EasyCASE ) */
/* EasyCASE ( 936
   bma255_set_bandwidth */
int bma255_set_bandwidth(unsigned char bw);
/* EasyCASE ) */
/* EasyCASE ( 1553
   bma255_get_mode */
int bma255_get_mode(unsigned char * Mode );
/* EasyCASE ) */
/* EasyCASE ( 1557
   bma255_set_mode */
unsigned char bma255_set_mode(unsigned char Mode);
/* EasyCASE ) */
/* EasyCASE ( 933
   bma255_get_suspend */
BMA255_RETURN_FUNCTION_TYPE bma255_get_suspend(unsigned char *suspend);
/* EasyCASE ) */
/* EasyCASE ( 932
   bma255_set_suspend */
BMA255_RETURN_FUNCTION_TYPE bma255_set_suspend (unsigned char suspend);
/* EasyCASE ) */
/* EasyCASE ( 1503
   bma255_get_lowpower_en */
BMA255_RETURN_FUNCTION_TYPE bma255_get_lowpower_en(unsigned char *lowpow);
/* EasyCASE ) */
/* EasyCASE ( 1505
   bma255_set_lowpower_en */
BMA255_RETURN_FUNCTION_TYPE bma255_set_lowpower_en (unsigned char lowpow);
/* EasyCASE ) */
/* EasyCASE ( 1509
   bma255_get_deep_suspend */
BMA255_RETURN_FUNCTION_TYPE bma255_get_deep_suspend(unsigned char *dsuspend);
/* EasyCASE ) */
/* EasyCASE ( 1513
   bma255_set_deep_suspend */
BMA255_RETURN_FUNCTION_TYPE bma255_set_deep_suspend (unsigned char dsuspend);
/* EasyCASE ) */
/* EasyCASE ( 1517
   bma255_get_sleep_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_get_sleep_dur(unsigned char *sleep_dur);
/* EasyCASE ) */
/* EasyCASE ( 1521
   bma255_set_sleep_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_set_sleep_dur (unsigned char sleep_dur);
/* EasyCASE ) */
/* EasyCASE ( 1525
   bma255_get_lownoise_en */
BMA255_RETURN_FUNCTION_TYPE bma255_get_lownoise_en(unsigned char *lownoise);
/* EasyCASE ) */
/* EasyCASE ( 1529
   bma255_set_lownoise_en */
BMA255_RETURN_FUNCTION_TYPE bma255_set_lownoise_en (unsigned char lownoise);
/* EasyCASE ) */
/* EasyCASE ( 1533
   bma255_get_lowpower_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_lowpower_mode(unsigned char *lowpower_mode);
/* EasyCASE ) */
/* EasyCASE ( 1537
   bma255_set_lowpower_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_set_lowpower_mode (unsigned char lowpower_mode);
/* EasyCASE ) */
/* EasyCASE ( 1545
   bma255_get_sleeptmr_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_sleeptmr_mode(unsigned char *sleep_tmr);
/* EasyCASE ) */
/* EasyCASE ( 1541
   bma255_set_sleeptmr_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_set_sleeptmr_mode (unsigned char sleep_tmr);
/* EasyCASE ) */
/* EasyCASE ( 985
   bma255_get_high_bw */
BMA255_RETURN_FUNCTION_TYPE bma255_get_high_bw (unsigned char *high_bw);
/* EasyCASE ) */
/* EasyCASE ( 993
   bma255_set_high_bw */
BMA255_RETURN_FUNCTION_TYPE bma255_set_high_bw (unsigned char high_bw);
/* EasyCASE ) */
/* EasyCASE ( 1001
   bma255_get_shadow_dis */
BMA255_RETURN_FUNCTION_TYPE bma255_get_shadow_dis (unsigned char *shadow_dis);
/* EasyCASE ) */
/* EasyCASE ( 1009
   bma255_set_shadow_dis */
BMA255_RETURN_FUNCTION_TYPE bma255_set_shadow_dis (unsigned char shadow_dis);
/* EasyCASE ) */
/* EasyCASE ( 927
   bma255_soft_reset */
int bma255_soft_reset(void);
/* EasyCASE ) */
/* EasyCASE ( 948
   bma255_update_image */
int bma255_update_image(void);
/* EasyCASE ) */
/* EasyCASE ( 951
   bma255_set_Int_Enable */
int bma255_set_Int_Enable( unsigned char InterruptType , unsigned char value );
/* EasyCASE ) */
/* EasyCASE ( 1587
   bma255_get_int_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_ffull (unsigned char *ffull);
/* EasyCASE ) */
/* EasyCASE ( 1591
   bma255_set_int_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_ffull (unsigned char ffull);
/* EasyCASE ) */
/* EasyCASE ( 1595
   bma255_get_int_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_fwm (unsigned char *fwm);
/* EasyCASE ) */
/* EasyCASE ( 1599
   bma255_set_int_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_fwm (unsigned char fwm);
/* EasyCASE ) */
/* EasyCASE ( 1029
   bma255_get_slo_no_mot */
BMA255_RETURN_FUNCTION_TYPE bma255_get_slo_no_mot (unsigned char channel,unsigned char *slo_data);
/* EasyCASE ) */
/* EasyCASE ( 1033
   bma255_set_slo_no_mot */
BMA255_RETURN_FUNCTION_TYPE bma255_set_slo_no_mot (unsigned char channel,unsigned char slo_data);
/* EasyCASE ) */
/* EasyCASE ( 1037
   bma255_get_int_low */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_low (unsigned char channel,unsigned char *int_low);
/* EasyCASE ) */
/* EasyCASE ( 1041
   bma255_set_int_low */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_low (unsigned char channel,unsigned char int_low);
/* EasyCASE ) */
/* EasyCASE ( 1045
   bma255_get_int_high */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_high(unsigned char channel,unsigned char *int_high);
/* EasyCASE ) */
/* EasyCASE ( 1049
   bma255_set_int_high */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_high(unsigned char channel,unsigned char int_high);
/* EasyCASE ) */
/* EasyCASE ( 1053
   bma255_get_int_slope */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_slope(unsigned char channel,unsigned char *int_slope);
/* EasyCASE ) */
/* EasyCASE ( 1057
   bma255_set_int_slope */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_slope(unsigned char channel,unsigned char int_slope);
/* EasyCASE ) */
/* EasyCASE ( 1077
   bma255_get_int_slo_no_mot */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_slo_no_mot (unsigned char channel,unsigned char *int_slo_no_mot);
/* EasyCASE ) */
/* EasyCASE ( 1081
   bma255_set_int_slo_no_mot */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_slo_no_mot (unsigned char channel,unsigned char int_slo_no_mot);
/* EasyCASE ) */
/* EasyCASE ( 1087
   bma255_get_int_d_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_d_tap (unsigned char channel,unsigned char *int_d_tap);
/* EasyCASE ) */
/* EasyCASE ( 1091
   bma255_set_int_d_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_d_tap (unsigned char channel,unsigned char int_d_tap);
/* EasyCASE ) */
/* EasyCASE ( 1097
   bma255_get_int_s_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_s_tap (unsigned char channel,unsigned char *int_s_tap);
/* EasyCASE ) */
/* EasyCASE ( 1101
   bma255_set_int_s_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_s_tap (unsigned char channel,unsigned char int_s_tap);
/* EasyCASE ) */
/* EasyCASE ( 1105
   bma255_get_int_orient */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_orient (unsigned char channel,unsigned char *int_orient);
/* EasyCASE ) */
/* EasyCASE ( 1109
   bma255_set_int_orient */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_orient (unsigned char channel,unsigned char int_orient);
/* EasyCASE ) */
/* EasyCASE ( 1113
   bma255_get_int_flat */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_flat (unsigned char channel,unsigned char *int_flat);
/* EasyCASE ) */
/* EasyCASE ( 1119
   bma255_set_int_flat */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_flat (unsigned char channel,unsigned char int_flat);
/* EasyCASE ) */
/* EasyCASE ( 1123
   bma255_get_newdata */
BMA255_RETURN_FUNCTION_TYPE bma255_get_newdata (unsigned char channel,unsigned char *int_newdata);
/* EasyCASE ) */
/* EasyCASE ( 1127
   bma255_set_newdata */
BMA255_RETURN_FUNCTION_TYPE bma255_set_newdata (unsigned char channel,unsigned char int_newdata);
/* EasyCASE ) */
/* EasyCASE ( 1131
   bma255_get_int1_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int1_fwm (unsigned char *int1_fwm);
/* EasyCASE ) */
/* EasyCASE ( 1135
   bma255_set_int1_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int1_fwm (unsigned char int1_fwm);
/* EasyCASE ) */
/* EasyCASE ( 1569
   bma255_get_int2_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int2_fwm (unsigned char *int2_fwm);
/* EasyCASE ) */
/* EasyCASE ( 1573
   bma255_set_int2_fwm */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int2_fwm (unsigned char int2_fwm);
/* EasyCASE ) */
/* EasyCASE ( 1139
   bma255_get_int1_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int1_ffull (unsigned char *int1_ffull);
/* EasyCASE ) */
/* EasyCASE ( 1143
   bma255_set_int1_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int1_ffull (unsigned char int1_ffull);
/* EasyCASE ) */
/* EasyCASE ( 1577
   bma255_get_int2_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int2_ffull (unsigned char *int2_ffull);
/* EasyCASE ) */
/* EasyCASE ( 1583
   bma255_set_int2_ffull */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int2_ffull (unsigned char int2_ffull);
/* EasyCASE ) */
/* EasyCASE ( 1147
   bma255_get_source */
BMA255_RETURN_FUNCTION_TYPE bma255_get_source (unsigned char channel,unsigned char *int_source);
/* EasyCASE ) */
/* EasyCASE ( 1151
   bma255_set_source */
BMA255_RETURN_FUNCTION_TYPE bma255_set_source (unsigned char channel,unsigned char int_source);
/* EasyCASE ) */
/* EasyCASE ( 1155
   bma255_get_int_od */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_od (unsigned char channel,unsigned char *int_od);
/* EasyCASE ) */
/* EasyCASE ( 1163
   bma255_set_int_od */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_od (unsigned char channel,unsigned char int_od);
/* EasyCASE ) */
/* EasyCASE ( 1167
   bma255_get_int_lvl */
BMA255_RETURN_FUNCTION_TYPE bma255_get_int_lvl (unsigned char channel,unsigned char *int_lvl);
/* EasyCASE ) */
/* EasyCASE ( 1171
   bma255_set_int_lvl */
BMA255_RETURN_FUNCTION_TYPE bma255_set_int_lvl (unsigned char channel,unsigned char int_lvl);
/* EasyCASE ) */
/* EasyCASE ( 1175
   bma255_reset_interrupt */
BMA255_RETURN_FUNCTION_TYPE bma255_reset_interrupt (unsigned char reset_int);
/* EasyCASE ) */
/* EasyCASE ( 1179
   bma255_get_latch_int */
BMA255_RETURN_FUNCTION_TYPE bma255_get_latch_int (unsigned char *latch_int);
/* EasyCASE ) */
/* EasyCASE ( 1183
   bma255_set_latch_int */
BMA255_RETURN_FUNCTION_TYPE bma255_set_latch_int (unsigned char latch_int);
/* EasyCASE ) */
/* EasyCASE ( 1187
   bma255_get_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_get_dur(unsigned char channel,unsigned char *dur);
/* EasyCASE ) */
/* EasyCASE ( 1191
   bma255_set_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_set_dur (unsigned char channel,unsigned char dur);
/* EasyCASE ) */
/* EasyCASE ( 1195
   bma255_get_thr */
BMA255_RETURN_FUNCTION_TYPE bma255_get_thr(unsigned char channel,unsigned char *thr);
/* EasyCASE ) */
/* EasyCASE ( 1199
   bma255_set_thr */
BMA255_RETURN_FUNCTION_TYPE bma255_set_thr (unsigned char channel,unsigned char thr);
/* EasyCASE ) */
/* EasyCASE ( 1203
   bma255_get_low_high_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_get_low_high_hyst(unsigned char channel,unsigned char *hyst);
/* EasyCASE ) */
/* EasyCASE ( 1207
   bma255_set_low_high_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_set_low_high_hyst (unsigned char channel,unsigned char hyst);
/* EasyCASE ) */
/* EasyCASE ( 1211
   bma255_get_low_high_hyst_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_low_high_hyst_mode(unsigned char *mode);
/* EasyCASE ) */
/* EasyCASE ( 1215
   bma255_set_low_high_hyst_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_set_low_high_hyst_mode (unsigned char mode);
/* EasyCASE ) */
/* EasyCASE ( 1219
   bma255_get_tap_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_get_tap_dur (unsigned char *tap_dur);
/* EasyCASE ) */
/* EasyCASE ( 1223
   bma255_set_tap_dur */
BMA255_RETURN_FUNCTION_TYPE bma255_set_tap_dur (unsigned char tap_dur);
/* EasyCASE ) */
/* EasyCASE ( 1227
   bma255_get_adv_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_get_adv_tap_int (unsigned char *adv_tap);
/* EasyCASE ) */
/* EasyCASE ( 1231
   bma255_set_adv_tap */
BMA255_RETURN_FUNCTION_TYPE bma255_set_adv_tap_int (unsigned char adv_tap);
/* EasyCASE ) */
/* EasyCASE ( 1235
   bma255_get_tap_shock */
BMA255_RETURN_FUNCTION_TYPE bma255_get_tap_shock (unsigned char *tap_shock);
/* EasyCASE ) */
/* EasyCASE ( 1239
   bma255_set_tap_shock */
BMA255_RETURN_FUNCTION_TYPE bma255_set_tap_shock (unsigned char tap_shock);
/* EasyCASE ) */
/* EasyCASE ( 1243
   bma255_get_tap_quiet */
BMA255_RETURN_FUNCTION_TYPE bma255_get_tap_quiet (unsigned char *tap_quiet);
/* EasyCASE ) */
/* EasyCASE ( 1247
   bma255_set_tap_quiet */
BMA255_RETURN_FUNCTION_TYPE bma255_set_tap_quiet (unsigned char tap_quiet);
/* EasyCASE ) */
/* EasyCASE ( 1255
   bma255_get_tap_thr */
BMA255_RETURN_FUNCTION_TYPE bma255_get_tap_thr (unsigned char *tap_thr);
/* EasyCASE ) */
/* EasyCASE ( 1251
   bma255_set_tap_thr */
BMA255_RETURN_FUNCTION_TYPE bma255_set_tap_thr (unsigned char tap_thr);
/* EasyCASE ) */
/* EasyCASE ( 1259
   bma255_get_tap_sample */
BMA255_RETURN_FUNCTION_TYPE bma255_get_tap_sample (unsigned char *tap_sample);
/* EasyCASE ) */
/* EasyCASE ( 1263
   bma255_set_tap_sample */
BMA255_RETURN_FUNCTION_TYPE bma255_set_tap_sample (unsigned char tap_sample);
/* EasyCASE ) */
/* EasyCASE ( 1267
   bma255_get_orient_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_orient_mode (unsigned char *orient_mode);
/* EasyCASE ) */
/* EasyCASE ( 1271
   bma255_set_orient_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_set_orient_mode (unsigned char orient_mode);
/* EasyCASE ) */
/* EasyCASE ( 1275
   bma255_get_orient_block */
BMA255_RETURN_FUNCTION_TYPE bma255_get_orient_block (unsigned char *orient_block);
/* EasyCASE ) */
/* EasyCASE ( 1279
   bma255_set_orient_block */
BMA255_RETURN_FUNCTION_TYPE bma255_set_orient_block (unsigned char orient_block);
/* EasyCASE ) */
/* EasyCASE ( 1283
   bma255_get_orient_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_get_orient_hyst (unsigned char *orient_hyst);
/* EasyCASE ) */
/* EasyCASE ( 1287
   bma255_set_orient_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_set_orient_hyst (unsigned char orient_hyst);
/* EasyCASE ) */
/* EasyCASE ( 1291
   bma255_get_theta */
BMA255_RETURN_FUNCTION_TYPE bma255_get_theta(unsigned char channel,unsigned char *theta);
/* EasyCASE ) */
/* EasyCASE ( 1295
   bma255_set_theta */
BMA255_RETURN_FUNCTION_TYPE bma255_set_theta (unsigned char channel,unsigned char theta);
/* EasyCASE ) */
/* EasyCASE ( 1299
   bma255_get_orient_en */
BMA255_RETURN_FUNCTION_TYPE bma255_get_orient_en (unsigned char *orient_en);
/* EasyCASE ) */
/* EasyCASE ( 1303
   bma255_set_orient_en */
BMA255_RETURN_FUNCTION_TYPE bma255_set_orient_en (unsigned char orient_en);
/* EasyCASE ) */
/* EasyCASE ( 1307
   bma255_get_orient_axis */
BMA255_RETURN_FUNCTION_TYPE bma255_get_orient_axis (unsigned char *orient_axis);
/* EasyCASE ) */
/* EasyCASE ( 1311
   bma255_set_orient_axis */
BMA255_RETURN_FUNCTION_TYPE bma255_set_orient_axis (unsigned char orient_axis);
/* EasyCASE ) */
/* EasyCASE ( 1315
   bma255_get_flat_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_get_flat_hyst (unsigned char *flat_hyst);
/* EasyCASE ) */
/* EasyCASE ( 1319
   bma255_set_flat_hyst */
BMA255_RETURN_FUNCTION_TYPE bma255_set_flat_hyst (unsigned char flat_hyst);
/* EasyCASE ) */
/* EasyCASE ( 1323
   bma255_get_flat_hold_time */
BMA255_RETURN_FUNCTION_TYPE bma255_get_flat_hold_time (unsigned char *flat_hold_time);
/* EasyCASE ) */
/* EasyCASE ( 1327
   bma255_set_flat_hold_time */
BMA255_RETURN_FUNCTION_TYPE bma255_set_flat_hold_time (unsigned char flat_hold_time);
/* EasyCASE ) */
/* EasyCASE ( 1331
   bma255_get_fifo_wml_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_wml_trig (unsigned char *fifo_wml_trig);
/* EasyCASE ) */
/* EasyCASE ( 1335
   bma255_set_fifo_wml_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_wml_trig (unsigned char fifo_wml_trig);
/* EasyCASE ) */
/* EasyCASE ( 1339
   bma255_get_self_test_axis */
BMA255_RETURN_FUNCTION_TYPE bma255_get_self_test_axis (unsigned char *self_test_axis);
/* EasyCASE ) */
/* EasyCASE ( 1343
   bma255_set_self_test_axis */
BMA255_RETURN_FUNCTION_TYPE bma255_set_self_test_axis (unsigned char self_test_axis);
/* EasyCASE ) */
/* EasyCASE ( 1347
   bma255_get_self_test_sign */
BMA255_RETURN_FUNCTION_TYPE bma255_get_self_test_sign (unsigned char *self_test_sign);
/* EasyCASE ) */
/* EasyCASE ( 1351
   bma255_set_self_test_sign */
BMA255_RETURN_FUNCTION_TYPE bma255_set_self_test_sign (unsigned char self_test_sign);
/* EasyCASE ) */
/* EasyCASE ( 1355
   bma255_get_self_test_amp */
BMA255_RETURN_FUNCTION_TYPE bma255_get_self_test_amp (unsigned char *self_test_amp);
/* EasyCASE ) */
/* EasyCASE ( 1359
   bma255_set_self_test_amp */
BMA255_RETURN_FUNCTION_TYPE bma255_set_self_test_amp (unsigned char self_test_amp);
/* EasyCASE ) */
/* EasyCASE ( 1371
   bma255_get_nvmprog_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_nvmprog_mode (unsigned char *nvmprog_mode);
/* EasyCASE ) */
/* EasyCASE ( 1367
   bma255_set_nvmprog_mode */
int bma255_set_nvmprog_mode(unsigned char prgmode);
/* EasyCASE ) */
/* EasyCASE ( 954
   bma255_set_nvprog_trig */
int bma255_set_nvprog_trig(unsigned char trig);
/* EasyCASE ) */
/* EasyCASE ( 952
   bma255_get_nvmprog_ready */
int bma255_get_nvmprog_ready(unsigned char *ready);
/* EasyCASE ) */
/* EasyCASE ( 1375
   bma255_set_nvm_load */
BMA255_RETURN_FUNCTION_TYPE bma255_set_nvm_load (unsigned char nvm_load);
/* EasyCASE ) */
/* EasyCASE ( 1379
   bma255_get_nvmprog_remain */
int bma255_get_nvmprog_remain(unsigned char *remain);
/* EasyCASE ) */
/* EasyCASE ( 1383
   bma255_get_spi3 */
BMA255_RETURN_FUNCTION_TYPE bma255_get_spi3 (unsigned char *spi3);
/* EasyCASE ) */
/* EasyCASE ( 1387
   bma255_set_spi3 */
BMA255_RETURN_FUNCTION_TYPE bma255_set_spi3 (unsigned char spi3);
/* EasyCASE ) */
/* EasyCASE ( 1391
   bma255_get_i2c_wdt */
BMA255_RETURN_FUNCTION_TYPE bma255_get_i2c_wdt (unsigned char channel,unsigned char *prog_mode);
/* EasyCASE ) */
/* EasyCASE ( 1395
   bma255_set_i2c_wdt */
BMA255_RETURN_FUNCTION_TYPE bma255_set_i2c_wdt (unsigned char channel,unsigned char prog_mode);
/* EasyCASE ) */
/* EasyCASE ( 1399
   bma255_get_slow_comp */
BMA255_RETURN_FUNCTION_TYPE bma255_get_slow_comp(unsigned char channel,unsigned char *slow_comp);
/* EasyCASE ) */
/* EasyCASE ( 1403
   bma255_set_slow_comp */
BMA255_RETURN_FUNCTION_TYPE bma255_set_slow_comp (unsigned char channel,unsigned char slow_comp);
/* EasyCASE ) */
/* EasyCASE ( 1407
   bma255_get_cal_rdy */
BMA255_RETURN_FUNCTION_TYPE bma255_get_cal_rdy (unsigned char *rdy);
/* EasyCASE ) */
/* EasyCASE ( 1411
   bma255_set_cal_rdy */
BMA255_RETURN_FUNCTION_TYPE bma255_set_cal_rdy (unsigned char rdy);
/* EasyCASE ) */
/* EasyCASE ( 1415
   bma255_get_cal_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_get_cal_trig (unsigned char *cal_trig);
/* EasyCASE ) */
/* EasyCASE ( 1419
   bma255_set_cal_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_set_cal_trig (unsigned char cal_trig);
/* EasyCASE ) */
/* EasyCASE ( 1423
   bma255_set_offset_reset */
BMA255_RETURN_FUNCTION_TYPE bma255_set_offset_reset (unsigned char offset_reset);
/* EasyCASE ) */
/* EasyCASE ( 1427
   bma255_get_offset_target */
BMA255_RETURN_FUNCTION_TYPE bma255_get_offset_target(unsigned char channel,unsigned char *offset);
/* EasyCASE ) */
/* EasyCASE ( 1431
   bma255_set_offset_target */
BMA255_RETURN_FUNCTION_TYPE bma255_set_offset_target (unsigned char channel,unsigned char offset);
/* EasyCASE ) */
/* EasyCASE ( 1561
   bma255_get_offset */
BMA255_RETURN_FUNCTION_TYPE bma255_get_offset(unsigned char channel,unsigned char *offset);
/* EasyCASE ) */
/* EasyCASE ( 1565
   bma255_set_offset */
BMA255_RETURN_FUNCTION_TYPE bma255_set_offset (unsigned char channel,unsigned char offset);
/* EasyCASE ) */
/* EasyCASE ( 1435
   bma255_get_gp */
BMA255_RETURN_FUNCTION_TYPE bma255_get_gp(unsigned char channel,unsigned char *gp);
/* EasyCASE ) */
/* EasyCASE ( 1439
   bma255_set_gp */
BMA255_RETURN_FUNCTION_TYPE bma255_set_gp (unsigned char channel,unsigned char gp);
/* EasyCASE ) */
/* EasyCASE ( 1443
   bma255_get_fifo_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_mode (unsigned char *fifo_mode);
/* EasyCASE ) */
/* EasyCASE ( 1447
   bma255_set_fifo_mode */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_mode (unsigned char fifo_mode);
/* EasyCASE ) */
/* EasyCASE ( 1451
   bma255_get_fifo_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_trig (unsigned char *fifo_trig);
/* EasyCASE ) */
/* EasyCASE ( 1455
   bma255_set_fifo_trig */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_trig (unsigned char fifo_trig);
/* EasyCASE ) */
/* EasyCASE ( 1459
   bma255_get_fifo_trig_src */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_trig_src (unsigned char *trig_src);
/* EasyCASE ) */
/* EasyCASE ( 1463
   bma255_set_fifo_trig_src */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_trig_src (unsigned char trig_src);
/* EasyCASE ) */
/* EasyCASE ( 1467
   bma255_get_fifo_data_sel */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_data_sel (unsigned char *data_sel);
/* EasyCASE ) */
/* EasyCASE ( 1471
   bma255_set_fifo_data_sel */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_data_sel (unsigned char data_sel);
/* EasyCASE ) */
/* EasyCASE ( 1475
   bma255_get_fifo_data_out_reg */
BMA255_RETURN_FUNCTION_TYPE bma255_get_fifo_data_out_reg(unsigned char *out_reg);
/* EasyCASE ) */
/* EasyCASE ( 1479
   bma255_set_fifo_data_out_reg */
BMA255_RETURN_FUNCTION_TYPE bma255_set_fifo_data_out_reg (unsigned char out_reg);
/* EasyCASE ) */
/* EasyCASE ) */
void bma250_soft_reset(void);
unsigned int BMA255_FIFODAQ(void);
extern unsigned char V_BMA255FIFOData_U8R[6];
#define BMA_PWC_pin 20//电源控制引脚


void bma_init(void);
void BMA253_Timer_Handler(void);
#endif

