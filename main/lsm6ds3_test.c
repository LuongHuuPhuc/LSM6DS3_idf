#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <i2c_scan.h>

//components
#include "I2C_dev.h"
#include "lsm6ds3.h"

#define TAG       "LSM6DS3_sensor"
#define SDA_PIN   GPIO_NUM_21
#define SCL_PIN   GPIO_NUM_22

TaskHandle_t lsm6ds3_handle_task = NULL;
I2C_dev_init_t dev;

esp_err_t lsm6ds3_init(void){
  memset(&dev, 0, sizeof(I2C_dev_init_t));
  ESP_ERROR_CHECK(lsm6ds3_I2C_dev_config(&dev, I2C_NUM, SDA_PIN, SCL_PIN));
  ESP_ERROR_CHECK(i2c_dev_install_device(&dev)); //De ngoai nay de tranh bi khoi tao lai driver
  if(lsm6ds3_readPartID(&dev) != ESP_OK){
    ESP_LOGE(TAG, "Not found device !:");
    return ESP_FAIL;
  }else{
    ESP_LOGI(TAG, "Found LSM6DS3 !");
    return ESP_OK;
  }
}

void lsm6ds3_task(void *args){
  float ax, ay, az;
  float gx, gy, gz;

  while(true){
    if(lsm6ds3_accelerationAvailable(&dev)){
      lsm6ds3_readAcceleration(&dev, &ax, &ay, &az);
      printf("%.2f,%.2f,%.2f,", ax, ay, az);
    }

    if(lsm6ds3_gyroscopeAvailable(&dev)){
      lsm6ds3_readGyroscope(&dev, &gx, &gy, &gz);
      printf("%.2f,%.2f,%.2f\n", gx, gy, gz);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void app_main(void){
  ESP_ERROR_CHECK(i2c_dev_initialize());
  if(lsm6ds3_init() != ESP_OK){
    ESP_LOGE(TAG, "Failed to initialize LSM6DS3 !");
    for(;;){ 
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  ESP_LOGI(TAG, "Initialized OK !");

  if(!lsm6ds3_begin(&dev)){
    ESP_LOGE(TAG, "Turn on LSM6DS3 check Failed !");
    vTaskDelete(NULL);
    for(;;){
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  ESP_LOGI(TAG, "Turned on LSM6DS3 check OK !");

  xTaskCreatePinnedToCore(lsm6ds3_task, TAG, 4096, NULL, 5, &lsm6ds3_handle_task, 0);
}