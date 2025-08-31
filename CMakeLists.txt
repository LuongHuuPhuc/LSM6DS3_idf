cmake_minimum_required(VERSION 3.30)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components/
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../I2C_SCAN)

project(lsm6ds3_test)