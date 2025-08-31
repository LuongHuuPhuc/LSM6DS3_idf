/**
 * @author Luong Huu Phuc 
 * @date 2025/05/11
 * @copyright Many sources
 */
#ifndef LSM6D3S_H__
#define LSM6D3S_H__

#ifdef __cplusplus
extern "C" {
#endif 

#include <stdio.h>
#include <stdint.h>
#include "I2C_dev.h"

#pragma once 

/**
 * @note LSM6DS3 co dia chi 7-bit la 110101xb, trong do x duoc xac dinh boi chan SD0/SA0 \note
 *  - SA0 = GND => Dia chi: 0x6A (1101010b) \note
 *  - SA0 = 3.3 => Dia chi: 0x6B (1101011b)
 * @brief Cac tham so quan trong: \brief
 *  - ODR (Output Data Rate): Toc do du lieu dau ra (So lan cap nhat du lieu cua cam bien /s) \brief 
 *  - FS (Full Scale): Dai do - (Accel: g, Gyro: dps) \brief 
 *  - BW (Bandwidth): Bang thong bo loc 
 */

#define LSM6DS3_DEFAULT_ADDR_0     0x6A
#define LSM6DS3_DEFAULT_ADDR_1     0x6B

/**********************DIA CHI DINH DANH CUA LSM6DS3*******************/

#define LSM6DS3_CHIP_ID            0x69
#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10 //Thanh ghi Accelerometer
#define LSM6DS3_CTRL2_G            0X11 //Thanh ghi Gyroscope

#define LSM6DS3_STATUS_REG         0X1E

#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17

#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23
#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25
#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27

#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29
#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B
#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D


typedef struct __attribute__((unused))__lsm6ds3_dev_t{
    uint8_t devAddr;
} lsm6ds3_dev_t;

/*************** INIT *****************/

//Su dung cach cua I2Cdev
esp_err_t lsm6ds3_I2C_dev_config(I2C_dev_init_t *dev, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

//Su dung driver/i2c.h
esp_err_t lsm6ds3_I2C_driver_config(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Doc chip ID cua cam bien 
 * @return - ESP_OK = success 
 * \return - ESP_ERR_NOT_FOUND = fail
 */
esp_err_t lsm6ds3_readPartID(I2C_dev_init_t *dev);

/**
 * @brief Ham khoi tao cam bien, khong goi la khong chay duoc \brief
 *     - Chuc nang kiem tra chip ID nhu lsm6ds3_readPartID() \brief
 *     - Nhung ham nay ghi dia chi vao thanh khi giup khoi tao cam bien
 * 
 * @return (sucess = 1, fail = -1)
 */
int lsm6ds3_begin(I2C_dev_init_t *dev);

void lsm6ds3_end(I2C_dev_init_t *dev);

/*************** Accelerometer *****************/

int lsm6ds3_readAcceleration(I2C_dev_init_t *dev, float *x, float *y, float *z);

float lsm6ds3_accelerationSampleRate(I2C_dev_init_t *dev);

/**
 * @return (sucess = 1, fail = -1)
 */
int lsm6ds3_accelerationAvailable(I2C_dev_init_t *dev);

/*************** Gyroscope *****************/

int lsm6ds3_readGyroscope(I2C_dev_init_t *dev, float *x, float *y, float *z);

float lsm6ds3_gyroscopeSampleRate(I2C_dev_init_t *dev);

/**
 * @return (sucess = 1, fail = -1)
 */
int lsm6ds3_gyroscopeAvailable(I2C_dev_init_t *dev);

/*************** Internal register access (LOW LEVEL API) *****************/

/**
 * @return (true = sucess, false = fail)
 */
bool lsm6ds3_readRegister(I2C_dev_init_t *dev, uint8_t addr, uint8_t *data);

/**
 * @return (true = sucess, false = fail)
 */
bool lsm6ds3_readRegisters(I2C_dev_init_t *dev, uint8_t addr, uint8_t *data, size_t length);

/**
 * @return (true = sucess, false = fail)
 */
bool lsm6ds3_writeRegister(I2C_dev_init_t *dev, uint8_t addr, uint8_t data);

#ifdef __cplusplus
}
#endif 

#endif //LSM6D3S_H__