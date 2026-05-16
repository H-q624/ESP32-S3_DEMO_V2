#ifndef APP_EXTFLASH_H
#define APP_EXTFLASH_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include <cstdint>

class ExtFlash {
public:
    ExtFlash(const char* tag);
    ~ExtFlash();
    
    esp_err_t init();
    esp_err_t read_id(uint32_t* id);
    esp_err_t read(uint32_t addr, uint8_t* data, size_t len);
    esp_err_t write(uint32_t addr, const uint8_t* data, size_t len);
    esp_err_t erase_sector(uint32_t addr);
    esp_err_t erase_chip();
    bool is_ready();
    
private:
    const char* TAG;
    spi_device_handle_t spi_handle;
    bool isReady;
    
    esp_err_t wait_ready();
    esp_err_t send_cmd(uint8_t cmd, const uint8_t* tx_data, size_t tx_len, 
                       uint8_t* rx_data, size_t rx_len);
};

#endif // APP_EXTFLASH_H
