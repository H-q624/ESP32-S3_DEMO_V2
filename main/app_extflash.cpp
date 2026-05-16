#include "app_extflash.h"

ExtFlash::ExtFlash(const char* tag) : TAG(tag), spi_handle(nullptr), isReady(false) {
}

ExtFlash::~ExtFlash() {
}

esp_err_t ExtFlash::init() {
    ESP_LOGW(TAG, "External Flash module disabled");
    return ESP_OK;
}

esp_err_t ExtFlash::read_id(uint32_t* id) {
    (void)id;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ExtFlash::read(uint32_t addr, uint8_t* data, size_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ExtFlash::write(uint32_t addr, const uint8_t* data, size_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ExtFlash::erase_sector(uint32_t addr) {
    (void)addr;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ExtFlash::erase_chip() {
    return ESP_ERR_NOT_SUPPORTED;
}

bool ExtFlash::is_ready() {
    return false;
}