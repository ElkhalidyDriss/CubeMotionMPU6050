#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_
#include "stm32f4xx_hal.h"
#include <string.h>
/******************************************************************
 *                    Platform related macros                     *
 ******************************************************************/
extern I2C_HandleTypeDef I2C3_HandleStruct;



#define MPU6050_SDA_PIN GPIO_PIN_9
#define MPU6050_SCL_PIN GPIO_PIN_8

/******************************************************************
 *                    MPU6050 Register Addresses                   *
 ******************************************************************/
#define SELF_TEST_X       0x0D  // SELF_TEST_X register address
#define SELF_TEST_Y       0x0E  // SELF_TEST_Y register address
#define SELF_TEST_Z       0x0F  // SELF_TEST_Z register address
#define SELF_TEST_A       0x10  // SELF_TEST_A register address
#define SMPRT_DIV         0x19  // Sample Rate Divider register address
#define CONFIG            0x1A  // Configuration register address
#define GYRO_CONFIG       0x1B  // GYRO_CONFIG register address
#define ACCEL_CONFIG      0x1C  // ACCEL_CONFIG register address
#define FIFO_EN           0x23  // FIFO_EN register address
#define I2C_SLV0_ADDR     0x25  // I2C_SLV0_ADDR register address
#define I2C_SLV0_REG      0x26  // I2C_SLV0_REG register address
#define I2C_SLV0_CTRL     0x27  // I2C_SLV0_CTRL register address
#define I2C_SLV1_ADDR     0x28  // I2C_SLV1_ADDR register address
#define I2C_SLV1_REG      0x29  // I2C_SLV1_REG register address
#define I2C_SLV1_CTRL     0x2A  // I2C_SLV1_CTRL register address
#define I2C_SLV2_ADDR     0x2B  // I2C_SLV2_ADDR register address
#define I2C_SLV2_REG      0x2C  // I2C_SLV2_REG register address
#define I2C_SLV2_CTRL     0x2D  // I2C_SLV2_CTRL register address
#define I2C_SLV3_ADDR     0x2E  // I2C_SLV3_ADDR register address
#define I2C_SLV3_REG      0x2F  // I2C_SLV3_REG register address
#define I2C_SLV3_CTRL     0x30  // I2C_SLV3_CTRL register address
#define I2C_SLV4_ADDR     0x31  // I2C_SLV4_ADDR register address
#define I2C_SLV4_REG      0x32  // I2C_SLV4_REG register address
#define I2C_SLV4_DO       0x33  // I2C_SLV4_DO register address
#define I2C_SLV4_CTRL     0x34  // I2C_SLV4_CTRL register address
#define I2C_SLV4_DI       0x35  // I2C_SLV4_DI register address
#define I2C_MST_STATUS    0x36  // I2C_MST_STATUS register address
#define INT_PIN_CFG       0x37  // INT_PIN_CFG register address
#define INT_ENABLE        0x38  // INT_ENABLE register address
#define INT_STATUS        0x3A  // INT_STATUS register address
#define PWR_MGMT_1        0x6B // Power Management 1 register address
#define WHO_AM_I          0x75
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define GYRO_XOUT_H       0x43
#define MPU6050_I2C_ADDR 0b1101000
#define MPU_6050_WRITE_FLAG 0
#define MPU_6050_READ_FLAG  (1 << 0)

#define MPU6050_ID 0x68
#define MPU6050_FS_MASK 0x18
#define MPU6050_ACC_FS_2g   0x00
#define MPU6050_ACC_FS_4g   0x08
#define MPU6050_ACC_FS_8g   0x10
#define MPU6050_ACC_FS_16g  0x18
#define MPU6050_GYRO_FS_250DPS  0x00
#define MPU6050_GYRO_FS_500DPS  0x08
#define MPU6050_GYRO_FS_1000DPS 0x10
#define MPU6050_GYRO_FS_2000DPS 0x18

/*
 * MPU6050 enum types for error handling
 */
typedef enum
{
  MPU6050_OK,
  MPU6050_ERROR,
} MPU6050_Status_e;
/*DLPF(Digital Low Pass filter configuration BW(bandwidth)*/
typedef enum {
	BW_260_HZ,
	BW_184_HZ,
	BW_94_HZ,
	BW_44_HZ,
	BW_21_HZ,
	BW_10_HZ,
	BW_5_HZ
}band_width_e;
typedef enum {/*Gyroscope full scale selection enumeration type*/
	GYRO_FS_SEL_250DPS,/*DPS : Degree Per Second*/
	GYRO_FS_SEL_500DPS,
	GYRO_FS_SEL_1000DPS,
	GYRO_FS_SEL_2000DPS
}gyro_fs_sel_e;
typedef enum {/*Accelerometer Full Scale selection enumeration type*/
	ACC_FS_SEL_2g,/*+ or - 2g*/
    ACC_FS_SEL_4g,
    ACC_FS_SEL_8g,
    ACC_FS_SEL_16g
}acc_fs_sel_e;
typedef enum {
	  MPU6050_INTERNAL_8MHz, /*Internal 8MHz oscillator*/
	  MPU6050_PLL_GYROX,/*PLL with X axis gyroscope reference*/
	  MPU6050_PLL_GYROY, /*PLL with Y axis gyroscope reference*/
	  MPU6050_PLL_GYROZ, /*PLL with Z axis gyroscope reference*/
	  MPU6050_PLL_EXT_32K,/*PLL with external 32.768kHz reference*/
	  MPU6050_PLL_EXT_19MHz, /*PLL with external 19.2MHz reference*/
	  /*bit 6 is reserved*/
	  MPU6050_STOP = 7,/*Stops the clock and keeps the timing generator in reset*/
}mpu6050_clk_sel_e;
typedef struct {
	uint8_t sample_rate_divider;
	band_width_e band_width;
	gyro_fs_sel_e gyro_fs_sel;
	acc_fs_sel_e acc_fs_sel;
	mpu6050_clk_sel_e mpu6050_clk_sel;
	/*Accelerometer sensitivity*/
	float acc_sensitivity ;
	float gyro_sensitivity;
	/*Offsets for calibration purposes */
	int16_t gyro_x_offset;
	int16_t gyro_y_offset;
	int16_t gyro_z_offset;
	int16_t acc_x_offset;
	int16_t acc_y_offset;
	int16_t acc_z_offset;
}mpu6050_config_t;
typedef struct {
	float gyro_senstivity;
	float acc_sensitivity;
}mpu6050_sensitivity_t;
typedef struct
{
    /*Accelerometer*/
    float acc_x;  /*Accelerometer x-axis data (in g)*/
    float acc_y;  /*Accelerometer Y-axis data (in g)*/
    float acc_z;  /*Accelerometer Z-axis data (in g)*/
	/*Gyroscope*/
    float gyro_x; /*Gyroscope X-axis data (in °/s)*/
    float gyro_y; /*Gyroscope Y-axis data (in °/s)*/
    float gyro_z; /*Gyroscope Z-axis data (in °/s)*/
}mpu6050_data_t;
typedef struct {
	/*Accelerometer raw data*/
	int16_t acc_x_raw;
	int16_t acc_y_raw;
	int16_t acc_z_raw;
    /*Gyroscope Raw data*/
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
}mpu6050_raw_data_t;

MPU6050_Status_e mpu6050_init(mpu6050_config_t *mpu6050_config);
MPU6050_Status_e mpu6050_get_raw_data(mpu6050_raw_data_t *mpu6050_raw_data , mpu6050_config_t *mpu6050_config);
MPU6050_Status_e mpu6050_get_scaled_data(mpu6050_data_t *mpu6050_data , mpu6050_config_t *mpu6050_config);
MPU6050_Status_e mpu6050_get_sensitivity(mpu6050_sensitivity_t *mpu6050_sensitivity);
MPU6050_Status_e mpu6050_get_data(uint8_t *data );


#endif /* MPU6050_MPU6050_H_ */
