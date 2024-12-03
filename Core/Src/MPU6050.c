#include "MPU6050.h"
#include <stdio.h>
I2C_HandleTypeDef I2C3_HandleStruct;

/*private Function to initialize i2c , can be overwritten for different platform*/
static void i2c_init()
{
	  __HAL_RCC_I2C3_CLK_ENABLE();
	  I2C3_HandleStruct.Instance = I2C3;
	  I2C3_HandleStruct.Init.ClockSpeed = 400000;
	  I2C3_HandleStruct.Init.DutyCycle = I2C_DUTYCYCLE_2;
	  I2C3_HandleStruct.Init.OwnAddress1 = 0;
	  I2C3_HandleStruct.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  I2C3_HandleStruct.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  I2C3_HandleStruct.Init.OwnAddress2 = 0;
	  I2C3_HandleStruct.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  I2C3_HandleStruct.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  HAL_I2C_Init(&I2C3_HandleStruct);
	  HAL_I2CEx_ConfigAnalogFilter(&I2C3_HandleStruct, I2C_ANALOGFILTER_ENABLE);
	  HAL_I2CEx_ConfigDigitalFilter(&I2C3_HandleStruct,0);
}
/*GPIO init function for I2C3 module
 * PA8 -> SCL | PC9 -> SDA*/
static void gpio_init()
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin =  MPU6050_SCL_PIN ,
        .Mode = GPIO_MODE_AF_OD, /* Alternate Function Open Drain Mode */
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF4_I2C3
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
static HAL_StatusTypeDef mpu6050_write_reg(uint8_t reg_addr , uint8_t *pData , uint16_t size)
{
	if (HAL_I2C_Mem_Write(&I2C3_HandleStruct,(MPU6050_I2C_ADDR << 1),reg_addr , I2C_MEMADD_SIZE_8BIT, pData , size , 100) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_OK;
}
/*Reads a register or multiple registers*/
static MPU6050_Status_e mpu6050_read_reg(uint8_t reg_addr , uint8_t *pData , uint16_t size)
{
	if (HAL_I2C_Mem_Read(&I2C3_HandleStruct,(MPU6050_I2C_ADDR << 1),reg_addr ,I2C_MEMADD_SIZE_8BIT , pData ,size, 100) != HAL_OK){
			return MPU6050_ERROR;
		}
	return MPU6050_OK;
}
static void mpu6050_set_sensitivity(mpu6050_config_t *mpu6050_config){
	switch (mpu6050_config -> acc_fs_sel)
	{
	case(ACC_FS_SEL_2g):
			mpu6050_config -> acc_sensitivity =  16384.f;
	        break;
	case(ACC_FS_SEL_4g):
			mpu6050_config -> acc_sensitivity =  8192.f;
		    break;
	case(ACC_FS_SEL_8g):
			mpu6050_config -> acc_sensitivity =  4096.f;
			break;
	case(ACC_FS_SEL_16g):
			mpu6050_config -> acc_sensitivity =  2048.f;
			break;
	}
	switch (mpu6050_config -> gyro_fs_sel)
	{
	case(GYRO_FS_SEL_250DPS):
			mpu6050_config -> gyro_sensitivity =  131.f;
	        break;
	case(GYRO_FS_SEL_500DPS):
			mpu6050_config -> gyro_sensitivity =  65.5f;
		    break;
	case(GYRO_FS_SEL_1000DPS):
			mpu6050_config -> gyro_sensitivity =  32.8f;
			break;
	case(GYRO_FS_SEL_2000DPS):
			mpu6050_config -> gyro_sensitivity =  16.4f;
			break;
	}
}
static void mpu6050_calibrate(mpu6050_config_t *mpu6050_config)
{
	uint16_t i;
	uint16_t samples = 500;
	int16_t acc_x_offset = 0 , acc_y_offset = 0 , acc_z_offset = 0;
	int16_t gyro_x_offset = 0, gyro_y_offset = 0 , gyro_z_offset = 0;
	mpu6050_raw_data_t mpu6050_raw_data;
	for (i = 0 ; i < samples ; i++)
	{
		mpu6050_get_raw_data(&mpu6050_raw_data, mpu6050_config);
		acc_x_offset += mpu6050_raw_data.acc_x_raw;
		acc_y_offset += mpu6050_raw_data.acc_y_raw;
		acc_z_offset += mpu6050_raw_data.acc_z_raw;
		gyro_x_offset += mpu6050_raw_data.gyro_x_raw;
		gyro_y_offset += mpu6050_raw_data.gyro_y_raw;
		gyro_z_offset += mpu6050_raw_data.gyro_z_raw;
	}
	mpu6050_config -> acc_x_offset  = acc_x_offset  / samples;
	mpu6050_config -> acc_y_offset  = acc_y_offset  / samples;
	mpu6050_config -> acc_z_offset  = acc_z_offset  / samples;

	mpu6050_config -> gyro_x_offset = gyro_x_offset / samples;
	mpu6050_config -> gyro_y_offset = gyro_y_offset / samples;
	mpu6050_config -> gyro_z_offset = gyro_z_offset / samples;
}
/*Reads raw data from the gyroscope and acclerometer*/
 MPU6050_Status_e mpu6050_get_raw_data(mpu6050_raw_data_t *mpu6050_raw_data , mpu6050_config_t *mpu6050_config)
{
	MPU6050_Status_e status;
	uint8_t raw_data[14];
	status = mpu6050_read_reg(ACCEL_XOUT_H, raw_data,14);/*Reading registers from the base address ACCEL_XOUT_H(0x3B) -> GYRO_ZOUT(0x48)*/
    /*Accelerometer raw data*/
	mpu6050_raw_data -> acc_x_raw = (int16_t)((raw_data[0] << 8) | (raw_data[1])) - mpu6050_config -> acc_x_offset;
	mpu6050_raw_data -> acc_y_raw = (int16_t)((raw_data[2] << 8) | (raw_data[3])) - mpu6050_config -> acc_y_offset;
	mpu6050_raw_data -> acc_z_raw = (int16_t)((raw_data[4] << 8) | (raw_data[5])) - mpu6050_config -> acc_z_offset;
	/*Gyroscope raw data */
	mpu6050_raw_data -> gyro_x_raw = (int16_t)((raw_data[8] << 8)  | (raw_data[9]))  - mpu6050_config -> gyro_x_offset;
	mpu6050_raw_data -> gyro_y_raw = (int16_t)((raw_data[10] << 8) | (raw_data[11])) - mpu6050_config -> gyro_y_offset;
	mpu6050_raw_data -> gyro_z_raw = (int16_t)((raw_data[12] << 8) | (raw_data[13])) - mpu6050_config -> gyro_z_offset;
	return status;
}
 MPU6050_Status_e mpu6050_get_raw_data_array(uint8_t *data){
		if( mpu6050_read_reg(ACCEL_XOUT_H, &data[0],6) != MPU6050_OK)/*Reading registers from the base address ACCEL_XOUT_H(0x3B) -> GYRO_ZOUT(0x48)*/
		{
			return MPU6050_ERROR;
		}
		if (mpu6050_read_reg(GYRO_XOUT_H , &data[6],6) != MPU6050_OK)
		{
			return MPU6050_ERROR;
		}
		return MPU6050_OK;
 }
void mpu6050_setSampleRateDiv(uint8_t divisor)
{
	mpu6050_write_reg(SMPRT_DIV ,&divisor , 1);
}
void mpu6050_setFilterBandWidth(band_width_e band_width)
{
	mpu6050_write_reg(CONFIG,(uint8_t *)&band_width,1);
}
void mpu6050_gyroFSConfig(gyro_fs_sel_e gyro_fs_sel)
{
	mpu6050_write_reg(GYRO_CONFIG,(uint8_t *)&gyro_fs_sel,1);
}
void mpu6050_accFSConfig(acc_fs_sel_e acc_fs_sel)
{
	mpu6050_write_reg(ACCEL_CONFIG,(uint8_t *)&acc_fs_sel,1);
}
MPU6050_Status_e mpu6050_init(mpu6050_config_t *mpu6050_config)
{
	/*GPIO configuration*/
    gpio_init();
	/*I2C Initialization*/
	i2c_init();
	/*****************************************************
	 *             MPU6050 configuration                 *
	 *****************************************************/
	/*Setting Sample Rate divider*/
	 mpu6050_setSampleRateDiv(mpu6050_config -> sample_rate_divider);
	/*Setting low pass filter bandwidth*/
	 mpu6050_setFilterBandWidth(mpu6050_config -> band_width);
    /*Setting accelerometer and gyroscope full scale*/
	 mpu6050_gyroFSConfig(mpu6050_config -> gyro_fs_sel);
	 mpu6050_accFSConfig(mpu6050_config ->  acc_fs_sel);
	 /*Setting MPU6050 sensitivity based on the full scale values*/
	 mpu6050_set_sensitivity(mpu6050_config);
    /*Setting MPU6050 Clock Source*/
	 mpu6050_write_reg(PWR_MGMT_1,(uint8_t *)mpu6050_config->mpu6050_clk_sel,1);
	 /*MPU6050 Calibration*/
	 /*Ensuring offsets are zero*/
	 mpu6050_config -> acc_x_offset  = 0;
	 mpu6050_config -> acc_y_offset  = 0;
	 mpu6050_config -> acc_z_offset  = 0;
	 mpu6050_config -> gyro_x_offset = 0;
	 mpu6050_config -> gyro_y_offset = 0;
	 mpu6050_config -> gyro_z_offset = 0;
	 mpu6050_calibrate(mpu6050_config);
	 /*Checking  communication with MPU6050 is correct*/
	 uint8_t dev_id;/*MPU6050 ID*/
	 mpu6050_read_reg(WHO_AM_I,&dev_id,1);
	 if(dev_id != MPU6050_ID)
	 {/*MPU6050 is not recognized*/
		 return MPU6050_ERROR;
	 }
     return MPU6050_OK;
}

//MPU6050_Status_e mpu6050_get_sensitivity(mpu6050_sensitivity_t *mpu6050_sensitivity)
//{
//	uint8_t data[2];
//	if (mpu6050_read_reg(GYRO_CONFIG, data, 2) != MPU6050_OK){
//		return MPU6050_ERROR;
//	}
//	switch(data[0] & MPU6050_FS_MASK)
//	{
//	    case (MPU6050_GYRO_FS_250DPS):
//		    mpu6050_sensitivity -> gyro_senstivity  = 131.f;
//			break;
//		case(MPU6050_GYRO_FS_500DPS):
//		mpu6050_sensitivity -> gyro_senstivity  = 65.5f;
//			break;
//		case(MPU6050_GYRO_FS_1000DPS):
//		    mpu6050_sensitivity -> gyro_senstivity  = 32.8f;
//			break;
//		case(MPU6050_GYRO_FS_2000DPS):
//	      	mpu6050_sensitivity -> gyro_senstivity  = 16.4f;
//	    	break;
//	}
//	switch (data[1] & MPU6050_FS_MASK){
//	    case (MPU6050_ACC_FS_2g):
//		    mpu6050_sensitivity -> acc_sensitivity = 16384.f;
//			break;
//		case(MPU6050_ACC_FS_4g):
//		    mpu6050_sensitivity -> acc_sensitivity = 8192.f;
//			break;
//		case(MPU6050_ACC_FS_8g):
//		    mpu6050_sensitivity -> acc_sensitivity = 4096.f;
//			break;
//		case(MPU6050_ACC_FS_16g):
//		    mpu6050_sensitivity -> acc_sensitivity = 2048.f;
//		    break;
//	}
//	return MPU6050_OK;
//}

MPU6050_Status_e mpu6050_get_scaled_data(mpu6050_data_t *mpu6050_data , mpu6050_config_t *mpu6050_config)
{
	mpu6050_raw_data_t mpu6050_raw_data;
	/*Getting RAW data from the MPU6050*/
	if(mpu6050_get_raw_data(&mpu6050_raw_data  ,mpu6050_config) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}
    /*Accelerometer Raw data conversion*/
	mpu6050_data -> acc_x = mpu6050_raw_data.acc_x_raw / mpu6050_config -> acc_sensitivity;
	mpu6050_data -> acc_y = mpu6050_raw_data.acc_y_raw / mpu6050_config -> acc_sensitivity;
	mpu6050_data -> acc_z = mpu6050_raw_data.acc_z_raw / mpu6050_config -> acc_sensitivity;
	/*Gyroscope Raw data conversion*/
	mpu6050_data -> gyro_x = mpu6050_raw_data.gyro_x_raw / mpu6050_config -> gyro_sensitivity;
	mpu6050_data -> gyro_y = mpu6050_raw_data.gyro_y_raw / mpu6050_config -> gyro_sensitivity;
	mpu6050_data -> gyro_z = mpu6050_raw_data.gyro_z_raw / mpu6050_config -> gyro_sensitivity;

	return MPU6050_OK;
}

