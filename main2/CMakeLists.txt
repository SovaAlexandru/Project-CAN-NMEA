idf_component_register(
    SRCS "esp1_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
        driver 
        esp_driver_uart
        esp_driver_gpio
        esp_timer 
        esp_system
        esp_hw_support
        mbedtls
)