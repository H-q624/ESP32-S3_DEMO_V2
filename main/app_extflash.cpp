#include "app_extflash.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define CMD_PSRAM_READ          0x03
#define CMD_PSRAM_WRITE         0x02
#define CMD_PSRAM_READ_ID       0x9F
#define CMD_PSRAM_RESET_ENABLE  0x66
#define CMD_PSRAM_RESET         0x99

#define APS1604M_SIZE_BYTES       (2u * 1024u * 1024u)
#define APS1604M_ERASE_BLOCK_SIZE 4096u
#define APS1604M_MAX_TRANSFER     4096u
#define APS1604M_TEST_ADDR        0x100u

static bool all_same_byte(const uint8_t *data, size_t len, uint8_t value) {
    for (size_t i = 0; i < len; i++) {
        if (data[i] != value) return false;
    }
    return true;
}

static void bb_delay(void) {
    esp_rom_delay_us(2);
}

static void bb_write_byte(uint8_t value) {
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level((gpio_num_t)EXTFLASH_MOSI_PIN, (value >> bit) & 0x01);
        bb_delay();
        gpio_set_level((gpio_num_t)EXTFLASH_CLK_PIN, 1);
        bb_delay();
        gpio_set_level((gpio_num_t)EXTFLASH_CLK_PIN, 0);
        bb_delay();
    }
}

static uint8_t bb_read_byte(void) {
    uint8_t value = 0;
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level((gpio_num_t)EXTFLASH_MOSI_PIN, 0);
        bb_delay();
        gpio_set_level((gpio_num_t)EXTFLASH_CLK_PIN, 1);
        bb_delay();
        if (gpio_get_level((gpio_num_t)EXTFLASH_MISO_PIN)) {
            value |= (uint8_t)(1u << bit);
        }
        gpio_set_level((gpio_num_t)EXTFLASH_CLK_PIN, 0);
        bb_delay();
    }
    return value;
}

static uint32_t bitbang_read_id(void) {
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << EXTFLASH_CS_PIN) |
                        (1ULL << EXTFLASH_CLK_PIN) |
                        (1ULL << EXTFLASH_MOSI_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << EXTFLASH_MISO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    gpio_set_level((gpio_num_t)EXTFLASH_CS_PIN, 1);
    gpio_set_level((gpio_num_t)EXTFLASH_CLK_PIN, 0);
    gpio_set_level((gpio_num_t)EXTFLASH_MOSI_PIN, 0);
    bb_delay();

    gpio_set_level((gpio_num_t)EXTFLASH_CS_PIN, 0);
    bb_delay();
    bb_write_byte(CMD_PSRAM_READ_ID);
    bb_write_byte(0x00);
    bb_write_byte(0x00);
    bb_write_byte(0x00);

    uint8_t id0 = bb_read_byte();
    uint8_t id1 = bb_read_byte();
    uint8_t id2 = bb_read_byte();
    uint8_t id3 = bb_read_byte();

    gpio_set_level((gpio_num_t)EXTFLASH_CS_PIN, 1);
    bb_delay();

    return ((uint32_t)id0 << 24) | ((uint32_t)id1 << 16) |
           ((uint32_t)id2 << 8) | id3;
}

ExtFlash::ExtFlash(const char *tag) : TAG(tag), spi_handle(nullptr), isReady(false) {}

ExtFlash::~ExtFlash() {
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = nullptr;
    }
    spi_bus_free(SPI2_HOST);
}

esp_err_t ExtFlash::send_cmd(uint8_t cmd, const uint8_t *tx_data, size_t tx_len,
                             uint8_t *rx_data, size_t rx_len) {
    if (!spi_handle) return ESP_ERR_INVALID_STATE;

    size_t total = 1 + tx_len + rx_len;
    if (total == 0 || total > APS1604M_MAX_TRANSFER) return ESP_ERR_INVALID_SIZE;

    uint8_t *tx_buf = (uint8_t *)calloc(total, 1);
    uint8_t *rx_buf = (uint8_t *)calloc(total, 1);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = cmd;
    if (tx_data && tx_len > 0) {
        memcpy(&tx_buf[1], tx_data, tx_len);
    }

    spi_transaction_t t = {};
    t.length = total * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret == ESP_OK && rx_data && rx_len > 0) {
        memcpy(rx_data, &rx_buf[1 + tx_len], rx_len);
    }

    free(tx_buf);
    free(rx_buf);
    return ret;
}

esp_err_t ExtFlash::init() {
#if !CONFIG_EXTFLASH_ENABLE
    ESP_LOGW(TAG, "External SPI PSRAM disabled in Kconfig");
    return ESP_OK;
#endif

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = EXTFLASH_MOSI_PIN,
        .miso_io_num = EXTFLASH_MISO_PIN,
        .sclk_io_num = EXTFLASH_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = APS1604M_MAX_TRANSFER,
    };

    uint32_t slow_id = bitbang_read_id();
    ESP_LOGI(TAG, "APS1604M slow GPIO ID: 0x%08lX", (unsigned long)slow_id);

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
    ESP_LOGI(TAG, "APS1604M SPI pins: CS=IO%d CLK=IO%d MOSI=IO%d MISO=IO%d",
             EXTFLASH_CS_PIN, EXTFLASH_CLK_PIN, EXTFLASH_MOSI_PIN, EXTFLASH_MISO_PIN);

    ret = send_cmd(CMD_PSRAM_RESET_ENABLE, NULL, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "APS1604M reset-enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = send_cmd(CMD_PSRAM_RESET, NULL, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "APS1604M reset failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    uint32_t id = 0;
    ret = read_id(&id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "APS1604M ID: 0x%08lX", (unsigned long)id);
        if (id == 0x00000000u || id == 0xFFFFFFFFu) {
            ESP_LOGW(TAG, "APS1604M ID invalid; chip is not driving MISO correctly");
        }
    } else {
        ESP_LOGW(TAG, "APS1604M read ID failed: %s", esp_err_to_name(ret));
    }

    isReady = true;
    ret = self_test();
    if (ret != ESP_OK) {
        isReady = false;
        ESP_LOGE(TAG, "APS1604M memory test failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "APS1604M SPI PSRAM ready: ID=0x%08lX size=%u bytes (CS=IO%d CLK=IO%d MOSI=IO%d MISO=IO%d)",
             (unsigned long)id, (unsigned)APS1604M_SIZE_BYTES, EXTFLASH_CS_PIN, EXTFLASH_CLK_PIN,
             EXTFLASH_MOSI_PIN, EXTFLASH_MISO_PIN);
    return ESP_OK;
}

esp_err_t ExtFlash::read_id(uint32_t *id) {
    if (!spi_handle || !id) return ESP_ERR_INVALID_ARG;

    uint8_t dummy_addr[3] = {0, 0, 0};
    uint8_t rx[4] = {0};
    esp_err_t ret = send_cmd(CMD_PSRAM_READ_ID, dummy_addr, sizeof(dummy_addr), rx, sizeof(rx));
    if (ret != ESP_OK) return ret;

    *id = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
          ((uint32_t)rx[2] << 8) | rx[3];
    return ESP_OK;
}

esp_err_t ExtFlash::read(uint32_t addr, uint8_t *data, size_t len) {
    if (!isReady || !data || len == 0) return ESP_ERR_INVALID_ARG;
    if (addr >= APS1604M_SIZE_BYTES || len > APS1604M_SIZE_BYTES - addr) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > APS1604M_MAX_TRANSFER - 4) chunk = APS1604M_MAX_TRANSFER - 4;

        size_t total = 4 + chunk;
        uint8_t *tx = (uint8_t *)calloc(total, 1);
        uint8_t *rx = (uint8_t *)calloc(total, 1);
        if (!tx || !rx) {
            free(tx);
            free(rx);
            return ESP_ERR_NO_MEM;
        }

        uint32_t cur_addr = addr + (uint32_t)offset;
        tx[0] = CMD_PSRAM_READ;
        tx[1] = (uint8_t)(cur_addr >> 16);
        tx[2] = (uint8_t)(cur_addr >> 8);
        tx[3] = (uint8_t)cur_addr;

        spi_transaction_t t = {};
        t.length = total * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        if (ret == ESP_OK) {
            memcpy(data + offset, rx + 4, chunk);
        }
        free(tx);
        free(rx);
        if (ret != ESP_OK) return ret;

        offset += chunk;
    }
    return ESP_OK;
}

esp_err_t ExtFlash::write(uint32_t addr, const uint8_t *data, size_t len) {
    if (!isReady || !data || len == 0) return ESP_ERR_INVALID_ARG;
    if (addr >= APS1604M_SIZE_BYTES || len > APS1604M_SIZE_BYTES - addr) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > APS1604M_MAX_TRANSFER - 4) chunk = APS1604M_MAX_TRANSFER - 4;

        size_t total = 4 + chunk;
        uint8_t *tx = (uint8_t *)malloc(total);
        if (!tx) return ESP_ERR_NO_MEM;

        uint32_t cur_addr = addr + (uint32_t)offset;
        tx[0] = CMD_PSRAM_WRITE;
        tx[1] = (uint8_t)(cur_addr >> 16);
        tx[2] = (uint8_t)(cur_addr >> 8);
        tx[3] = (uint8_t)cur_addr;
        memcpy(tx + 4, data + offset, chunk);

        spi_transaction_t t = {};
        t.length = total * 8;
        t.tx_buffer = tx;
        esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
        free(tx);
        if (ret != ESP_OK) return ret;

        offset += chunk;
    }
    return ESP_OK;
}

esp_err_t ExtFlash::erase_sector(uint32_t addr) {
    if (!isReady) return ESP_ERR_INVALID_STATE;
    if (addr >= APS1604M_SIZE_BYTES) return ESP_ERR_INVALID_SIZE;

    uint8_t erased[256];
    memset(erased, 0xFF, sizeof(erased));
    uint32_t sector_addr = addr & ~(APS1604M_ERASE_BLOCK_SIZE - 1u);
    size_t remaining = APS1604M_ERASE_BLOCK_SIZE;
    if (remaining > APS1604M_SIZE_BYTES - sector_addr) {
        remaining = APS1604M_SIZE_BYTES - sector_addr;
    }

    while (remaining > 0) {
        size_t chunk = remaining > sizeof(erased) ? sizeof(erased) : remaining;
        esp_err_t ret = write(sector_addr, erased, chunk);
        if (ret != ESP_OK) return ret;
        sector_addr += (uint32_t)chunk;
        remaining -= chunk;
    }
    return ESP_OK;
}

esp_err_t ExtFlash::erase_chip() {
    if (!isReady) return ESP_ERR_INVALID_STATE;

    uint8_t erased[256];
    memset(erased, 0xFF, sizeof(erased));
    uint32_t addr = 0;
    size_t remaining = APS1604M_SIZE_BYTES;
    while (remaining > 0) {
        size_t chunk = remaining > sizeof(erased) ? sizeof(erased) : remaining;
        esp_err_t ret = write(addr, erased, chunk);
        if (ret != ESP_OK) return ret;
        addr += (uint32_t)chunk;
        remaining -= chunk;
    }
    return ESP_OK;
}

esp_err_t ExtFlash::self_test() {
    uint8_t saved[16] = {0};
    uint8_t pattern[16] = {
        0xA5, 0x5A, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78,
        0x87, 0x65, 0x43, 0x21, 0xC3, 0x3C, 0x96, 0x69,
    };
    uint8_t verify[16] = {0};

    esp_err_t ret = read(APS1604M_TEST_ADDR, saved, sizeof(saved));
    if (ret != ESP_OK) return ret;

    ret = write(APS1604M_TEST_ADDR, pattern, sizeof(pattern));
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(1));

    ret = read(APS1604M_TEST_ADDR, verify, sizeof(verify));
    if (ret != ESP_OK) return ret;

    esp_err_t restore_ret = write(APS1604M_TEST_ADDR, saved, sizeof(saved));
    if (restore_ret != ESP_OK) return restore_ret;

    if (memcmp(pattern, verify, sizeof(pattern)) != 0) {
        ESP_LOGE(TAG, "APS1604M self-test mismatch at 0x%06X", APS1604M_TEST_ADDR);
        ESP_LOGE(TAG, "write: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 pattern[0], pattern[1], pattern[2], pattern[3],
                 pattern[4], pattern[5], pattern[6], pattern[7],
                 pattern[8], pattern[9], pattern[10], pattern[11],
                 pattern[12], pattern[13], pattern[14], pattern[15]);
        ESP_LOGE(TAG, "read : %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 verify[0], verify[1], verify[2], verify[3],
                 verify[4], verify[5], verify[6], verify[7],
                 verify[8], verify[9], verify[10], verify[11],
                 verify[12], verify[13], verify[14], verify[15]);
        if (all_same_byte(verify, sizeof(verify), 0x00) ||
            all_same_byte(verify, sizeof(verify), 0xFF)) {
            ESP_LOGE(TAG,
                     "APS1604M no valid MISO data. Check IO%d/SO, IO%d/CS, "
                     "VCC/GND, and SIO2/SIO3 pull-ups",
                     EXTFLASH_MISO_PIN, EXTFLASH_CS_PIN);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

bool ExtFlash::is_ready() { return isReady; }
