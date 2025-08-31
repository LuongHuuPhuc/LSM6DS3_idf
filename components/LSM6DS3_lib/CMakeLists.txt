idf_component_register(SRCS "lsm6ds3.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver esp_timer
                      REQUIRES I2C_dev)