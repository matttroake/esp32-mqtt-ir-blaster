idf_component_register(SRCS "irsender_main.cpp"
                       REQUIRES esp32-rmt-ir mqtt esp_netif esp_wifi nvs_flash mbedtls
                       INCLUDE_DIRS ".")
