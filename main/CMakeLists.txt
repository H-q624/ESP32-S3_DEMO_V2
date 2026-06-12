idf_component_register(SRCS "app_fall_new.cpp" "app_imu_processing.cpp" "app_speaker.cpp" "app_ml307r.c" "app_battery.c" "app_gps.cpp" "app_gpio.cpp" "app_mic.cpp" "main.cpp" "app_file.cpp" "app_mpu6050.cpp" "app_wifi.c" "app_extflash.cpp"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp_driver_i2s esp_driver_spi esp_driver_sdm esp_adc driver nvs_flash esp_http_client json mbedtls esp_wifi esp_timer spiffs)
