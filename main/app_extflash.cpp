#include "app_extflash.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ExtFlash";

#define CMD_WRITE_ENABLE  0x06
#define CMD_READ_STATUS   0x05
#define CMD_READ_DATA     0x03
#define CMD_PAGE_PROGRAM  0x02
#define CMD_SECTOR_ERASE  0x20
#define CMD_CHIP_ERASE    0xC7
#define CMD_READ_JEDEC    0x9F

#define W25Q64_PAGE_SIZE   256
#define W25Q64_SECTOR_SIZE 4096
#define W25Q64_STATUS_BUSY 0x01

ExtFlash::ExtFlash(const char *tag) : TAG(tag), spi_handle(nullptr), isReady(false) {}

ExtFlash::~ExtFlash() {
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = nullptr;
    }
    spi_bus_free(SPI2_HOST);
}

esp_err_t ExtFlash::wait_ready() {
    uint8_t status = 0xFF;
    for (int i = 0; i < 500; i++) {
        esp_err_t ret = send_cmd(CMD_READ_STATUS, NULL, 0, &status, 1);
        if (ret != ESP_OK) return ret;
        if ((status & W25Q64_STATUS_BUSY) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t ExtFlash::send_cmd(uint8_t cmd, const uint8_t *tx_data, size_t tx_len,
                             uint8_t *rx_data, size_t rx_len) {
    if (!spi_handle) return ESP_ERR_INVALID_STATE;

    spi_transaction_t t = {};
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};

    tx_buf[0] = cmd;
    if (tx_data && tx_len > 0 && tx_len <= 7) {
        memcpy(&tx_buf[1], tx_data, tx_len);
    }

    t.length = (1 + tx_len) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) return ret;

    if (rx_data && rx_len > 0) {
        if (rx_len <= 7) {
            memcpy(rx_data, &rx_buf[1], rx_len);
        }
    }
    return ESP_OK;
}

esp_err_t ExtFlash::init() {
#if !CONFIG_EXTFLASH_ENABLE
    ESP_LOGW(TAG, "External flash disabled in Kconfig");
    return ESP_OK;
#endif

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = EXTFLASH_MOSI_PIN,
        .miso_io_num = EXTFLASH_MISO_PIN,
        .sclk_io_num = EXTFLASH_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = W25Q64_SECTOR_SIZE,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .spics_io_num = EXTFLASH_CS_PIN,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t jedec = 0;
    ret = read_id(&jedec);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "W25Q64 not detected on CS=IO%d", EXTFLASH_CS_PIN);
        return ret;
    }

    isReady = true;
    ESP_LOGI(TAG, "W25Q64 ready: JEDEC=0x%06lX (CS=IO%d CLK=IO%d MOSI=IO%d MISO=IO%d)",
             (unsigned long)(jedec & 0xFFFFFF), EXTFLASH_CS_PIN, EXTFLASH_CLK_PIN,
             EXTFLASH_MOSI_PIN, EXTFLASH_MISO_PIN);
    return ESP_OK;
}

esp_err_t ExtFlash::read_id(uint32_t *id) {
    if (!spi_handle || !id) return ESP_ERR_INVALID_ARG;

    uint8_t tx[4] = {CMD_READ_JEDEC, 0, 0, 0};
    uint8_t rx[4] = {0};
    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) return ret;

    *id = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    return ESP_OK;
}

esp_err_t ExtFlash::read(uint32_t addr, uint8_t *data, size_t len) {
    if (!isReady || !data || len == 0) return ESP_ERR_INVALID_ARG;

    size_t total = 4 + len;
    uint8_t *tx = (uint8_t *)calloc(total, 1);
    uint8_t *rx = (uint8_t *)calloc(total, 1);
    if (!tx || !rx) {
        free(tx);
        free(rx);
        return ESP_ERR_NO_MEM;
    }

    tx[0] = CMD_READ_DATA;
    tx[1] = (uint8_t)(addr >> 16);
    tx[2] = (uint8_t)(addr >> 8);
    tx[3] = (uint8_t)(addr);

    spi_transaction_t t = {};
    t.length = total * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        memcpy(data, rx + 4, len);
    }
    free(tx);
    free(rx);
    return ret;
}

esp_err_t ExtFlash::write(uint32_t addr, const uint8_t *data, size_t len) {
    if (!isReady || !data || len == 0) return ESP_ERR_INVALID_ARG;

    size_t offset = 0;
    while (offset < len) {
        uint32_t page_offset = addr % W25Q64_PAGE_SIZE;
        size_t chunk = W25Q64_PAGE_SIZE - page_offset;
        if (chunk > len - offset) chunk = len - offset;

        esp_err_t ret = send_cmd(CMD_WRITE_ENABLE, NULL, 0, NULL, 0);
        if (ret != ESP_OK) return ret;

        uint8_t hdr[4] = {
            CMD_PAGE_PROGRAM,
            (uint8_t)(addr >> 16),
            (uint8_t)(addr >> 8),
            (uint8_t)(addr),
        };

        spi_transaction_t cmd_t = {};
        cmd_t.length = 32;
        cmd_t.tx_buffer = hdr;
        ret = spi_device_polling_transmit(spi_handle, &cmd_t);
        if (ret != ESP_OK) return ret;

        spi_transaction_t data_t = {};
        data_t.length = chunk * 8;
        data_t.tx_buffer = data + offset;
        ret = spi_device_polling_transmit(spi_handle, &data_t);
        if (ret != ESP_OK) return ret;

        ret = wait_ready();
        if (ret != ESP_OK) return ret;

        addr += (uint32_t)chunk;
        offset += chunk;
    }
    return ESP_OK;
}

esp_err_t ExtFlash::erase_sector(uint32_t addr) {
    if (!isReady) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = send_cmd(CMD_WRITE_ENABLE, NULL, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    uint8_t hdr[4] = {
        CMD_SECTOR_ERASE,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr),
    };
    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = hdr;
    ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) return ret;
    return wait_ready();
}

esp_err_t ExtFlash::erase_chip() {
    if (!isReady) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = send_cmd(CMD_WRITE_ENABLE, NULL, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    uint8_t cmd = CMD_CHIP_ERASE;
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) return ret;
    return wait_ready();
}

bool ExtFlash::is_ready() { return isReady; }
