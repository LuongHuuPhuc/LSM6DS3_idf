idf_component_register(SRCS "lsm6ds3_test.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver freertos esp_timer
                      REQUIRES I2C_dev LSM6DS3_lib I2C_SCAN)