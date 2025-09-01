/**
 * @file lsm6ds3.c
 * @author Luong Huu Phuc
 * @date 2025/05/11
 * @copyright Many sources
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_log.h>
#include <esp_err.h>
#include "lsm6ds3.h"
#include "I2C_dev.h"
#include "driver/i2c_master.h"

#define CHECK_ARG_LOCAL(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t i2c_dev_handle = NULL;

esp_err_t lsm6ds3_I2C_dev_config(I2C_dev_init_t *dev, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin){
  CHECK_ARG_LOCAL(dev);

  dev->address = LSM6DS3_DEFAULT_ADDR_0;
  dev->port = port;
  dev->cfg.sda_io_num = sda_pin;
  dev->cfg.scl_io_num = scl_pin;

#if defined(CONFIG_IDF_TARGET_ESP32)
  dev->cfg.master.clk_speed = I2CDEV_FREQ_HZ;
#endif
  i2c_dev_create_mutex(dev);

  return ESP_OK;
}

esp_err_t lsm6ds3_I2C_driver_config(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_num){
  i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .master.clk_speed = I2CDEV_FREQ_HZ,
    .scl_io_num = scl_num,
    .sda_io_num = sda_pin,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(i2c_param_config(port, &config));
  ESP_ERROR_CHECK(i2c_driver_install(port, config.mode, 0, 0, 0));
  return ESP_OK;
}

esp_err_t lsm6ds3_readPartID(I2C_dev_init_t *dev){
  uint8_t data = 0;

  I2C_DEV_TAKE_MUTEX(dev);
  I2C_DEV_CHECK(dev, i2c_dev_read_byte(dev, dev->address, LSM6DS3_WHO_AM_I_REG, &data, i2c_dev_read_timeout, NULL));
  I2C_DEV_GIVE_MUTEX(dev);

  if(data != LSM6DS3_CHIP_ID_1 && data != LSM6DS3_CHIP_ID_2 && data != LSM6DS3_CHIP_ID_3){
    ESP_LOGE(pcTaskGetName(NULL), "LSM6DS3_CHIP_ID not found !");
    ESP_LOGI("LSM6DS3_CHIP_ID", "WHO_AM_I = 0x%02X", data);
    return ESP_ERR_NOT_FOUND;
  }else{
    return ESP_OK; 
  }
} 

int lsm6ds3_begin(I2C_dev_init_t *dev){
  uint8_t value, ret_value = 0;

  if(lsm6ds3_readRegister(dev, LSM6DS3_WHO_AM_I_REG, &ret_value) == true){
    value = ret_value;
  }else{
    return -1;
  }

  if(value != LSM6DS3_CHIP_ID_1 && value != LSM6DS3_CHIP_ID_2 && value != LSM6DS3_CHIP_ID_3){
    lsm6ds3_end(dev);
    return -1;
  }
  
  //Set accelerometer: ODR = 104Hz, FS = ±2g,
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL1_XL, 0x40);

  //Set accelerometer: ODR = 104Hz, FS = ±16g (Do nhay kem hon 0x40)
  // lsm6ds3_writeRegister(dev, LSM6DS3_CTRL1_XL, 0x4A);
#if 0
  //Set gyroscope: 0x4C ODR: 104Hz, FS = 2000 dps (rong hon) va trong che do byspass (che do bo qua)
  lsm6ds3_writeRegister(LSM6DS3_CTRL2_G, 0x4C); 
#endif

  //Set gyroscope: ODR = 104Hz, FS = ±250 dps (nhay hon) va trong che do bypass 
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL2_G, 0x40);

  //Set hieu nang gyroscope power mode che do cao va bandwidth len 16MHz
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL7_G, 0x00);

  //Accelerometer filter setup: ODR (Output Data Rate) / 4, HP disabled
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL8_XL, 0x09);

  return 1;
}

void lsm6ds3_end(I2C_dev_init_t *dev){
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL2_G, 0x00);
  lsm6ds3_writeRegister(dev, LSM6DS3_CTRL1_XL, 0x00);
}

int lsm6ds3_readAcceleration(I2C_dev_init_t *dev, float *x, float *y, float *z){
  int16_t data[3];

  if(!lsm6ds3_readRegisters(dev, LSM6DS3_OUTX_L_XL, (uint8_t*)data, sizeof(data))){
    //x = NAN;
    //y = NAN;
    //z = NAN;

    return 0;
  }

#if 0
  *x = data[0] * 4.0 / 32768.0;
  *y = data[1] * 4.0 / 32768.0;
  *z = data[2] * 4.0 / 32768.0;
#endif
  *x = data[0] / 8192.0;
  *y = data[1] / 8192.0;
  *z = data[2] / 8192.0;

  return 1;
}

float lsm6ds3_accelerationSampleRate(I2C_dev_init_t *dev){
  return 104.0F;
}

int lsm6ds3_accelerationAvailable(I2C_dev_init_t *dev){
  uint8_t ret_data = 0;

  //Doc trang thai thanh ghi
  if(!lsm6ds3_readRegister(dev, LSM6DS3_STATUS_REG, &ret_data)){
    return -1;
  }

  if(ret_data & 0x01){
    return 1;
  }else {
    return -1;
  }
}

int lsm6ds3_readGyroscope(I2C_dev_init_t *dev, float *x, float *y, float *z){
  int16_t data[3];

  if(!lsm6ds3_readRegisters(dev, LSM6DS3_OUTX_L_G, (uint8_t*)data, sizeof(data))){
    //x = NAN;
    //y = NAN;
    //z = NAN;

    return 0;
  }
#if 0
  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;
#endif

  *x = data[0] / 131.0;
  *y = data[1] / 131.0;
  *z = data[2] / 131.0;

  return 1;
}

float lsm6ds3_gyroscopeSampleRate(I2C_dev_init_t *dev){
  return 104.0F;
}

int lsm6ds3_gyroscopeAvailable(I2C_dev_init_t *dev){
  uint8_t ret_data = 0;

  //Doc trang thai thanh ghi
  if(!lsm6ds3_readRegister(dev, LSM6DS3_STATUS_REG, &ret_data)){
    return -1;
  }

  if(ret_data & 0x02){
    return 1;
  }else {
    return -1;
  }
}

bool lsm6ds3_readRegister(I2C_dev_init_t *dev, uint8_t addr, uint8_t *data){ 
  if(!lsm6ds3_readRegisters(dev, addr, data, sizeof(data))){
    return false;
  }
  return true;
}

bool lsm6ds3_readRegisters(I2C_dev_init_t *dev, uint8_t addr, uint8_t *data, size_t length){
  if(i2c_dev_read_bytes(dev, dev->address, addr, length, data, I2CDEV_DEFAULT_READ_TIMEOUT, NULL) != ESP_OK){
    return false;
  }
  return true;
}

bool lsm6ds3_writeRegister(I2C_dev_init_t *dev, uint8_t addr, uint8_t data){
  if(!i2c_dev_write_byte(dev, dev->address, addr, data, NULL)){
    return false;
  }
  return true;
}