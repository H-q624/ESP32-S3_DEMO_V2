set(app_sources
    "app_fall_new.cpp"
    "app_imu_processing.cpp"
    "app_speaker.cpp"
    "app_ml307r.c"
    "app_battery.c"
    "app_gps.cpp"
    "app_gpio.cpp"
    "app_mic.cpp"
    "app_wifi.c"
    "main.cpp"
    "app_file.cpp"
    "app_mpu6050.cpp"
    "app_extflash.cpp")

set(app_requires
    esp_driver_i2s
    esp_driver_spi
    esp_adc
    driver
    mbedtls
    esp_timer
    spiffs
    esp_wifi
    esp_netif
    nvs_flash
    esp_http_client
    esp_event
    json)

if(CONFIG_APP_SR_ENABLE)
    list(APPEND app_sources "app_sr.cpp")
    list(APPEND app_requires esp-sr)
endif()

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "include"
                       REQUIRES ${app_requires})
