idf_component_register(SRCS "mcu_to_im.c" "mcu_to_gnss.c" "User_Uart.c" "main.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart esp_driver_gpio
                    INCLUDE_DIRS ".")
