idf_component_register(
    SRCS "blaster.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_driver_uart ieee802154 esp_timer esp_driver_rmt esp_driver_gpio
)
