FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    PRIV_REQUIRES
        esp_common
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_sdspi
        esp_driver_spi
        esp_driver_uart
        esp_system
        fatfs
        freertos
        nvs_flash
        sdmmc
)
