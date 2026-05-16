#include "app_extflash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// W25Q64JVSIQ 命令定义
#define W25Q64_CMD_READ_ID          0x9F
#define W25Q64_CMD_READ             0x03
#define W25Q64_CMD_FAST_READ        0x0B
#define W25Q64_CMD_WRITE_ENABLE     0x06
#define W25Q64_CMD_WRITE_DISABLE    0x04
#define W25Q64_CMD_PAGE_PROGRAM     0x02
#define W25Q64_CMD_SECTOR_ERASE     0x20
#define W25Q64_CMD_BLOCK_ERASE_32K  0x52
#define W25Q64_CMD_BLOCK_ERASE_64K  0xD8
#define W25Q64_CMD_CHIP_ERASE       0xC7
#define W25Q64_CMD_READ_STATUS      0x05
#define W25Q64_CMD_WRITE_STATUS     0x01

// W25Q64JVSIQ 容量
#define W25Q64_SIZE                 (8 * 1024 * 1024)  // 8MB
#define W25Q64_PAGE_SIZE            256
#define W25Q64_SECTOR_SIZE          (4 * 1024)        // 4KB

// 硬件连接定义 (从电路图)
#define EXTFLASH_CS_PIN             GPIO_NUM_12
#define EXTFLASH_SCK_PIN            GPIO_NUM_11
#define EXTFLASH_MOSI_PIN           GPIO_NUM_10
#define EXTFLASH_MISO_PIN           GPIO_NUM_13
#define EXTFLASH_WP_PIN             GPIO_NUM_14

ExtFlash::ExtFlash(const char* tag) : TAG(tag), spi_handle(nullptr), isReady(false) {
}

ExtFlash::~ExtFlash() {
    if (spi_handle != nullptr) {
        spi_bus_remove_device(spi_handle);
    }
}

esp_err_t ExtFlash::send_cmd(uint8_t cmd, const uint8_t* tx_data, size_t tx_len,
                            uint8_t* rx_data, size_t rx_len) {
    if (spi_handle == nullptr) {
        ESP_LOGE(TAG, "SPI handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .flags = 0,
        .length = (1 + tx_len + rx_len) * 8,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
        .override_freq_hz = 0,
    };

    size_t total_len = 1 + tx_len + rx_len;
    uint8_t* tx_buf = (uint8_t*)malloc(total_len);
    uint8_t* rx_buf = (uint8_t*)malloc(total_len);

    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = cmd;
    if (tx_data && tx_len > 0) {
        memcpy(tx_buf + 1, tx_data, tx_len);
    }

    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret == ESP_OK && rx_data && rx_len > 0) {
        memcpy(rx_data, rx_buf + 1 + tx_len, rx_len);
    }

    free(tx_buf);
    free(rx_buf);
    return ret;
}

esp_err_t ExtFlash::wait_ready() {
    uint8_t status;
    int timeout = 1000; // 1 second timeout

    do {
        esp_err_t ret = send_cmd(W25Q64_CMD_READ_STATUS, nullptr, 0, &status, 1);
        if (ret != ESP_OK) return ret;
        if (!(status & 0x01)) return ESP_OK; // Busy bit is low
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (--timeout > 0);

    ESP_LOGE(TAG, "Wait ready timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t ExtFlash::init() {
    // 初始化 WP 引脚为高电平 (禁用写保护)
    gpio_config_t wp_config = {
        .pin_bit_mask = (1ULL << EXTFLASH_WP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&wp_config));
    gpio_set_level(EXTFLASH_WP_PIN, 1); // 禁用写保护

    // SPI 总线配置
    spi_bus_config_t buscfg = {
        .mosi_io_num = EXTFLASH_MOSI_PIN,
        .miso_io_num = EXTFLASH_MISO_PIN,
        .sclk_io_num = EXTFLASH_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .data_io_num = -1,
        .flags = 0,
        .intr_flags = 0,
        .data_io_default_level = 0,
    };

    // 初始化 SPI 总线
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // SPI 设备配置
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 24,
        .dummy_bits = 0,
        .mode = 0,                           // SPI mode 0
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 40 * 1000 * 1000,  // 40MHz
        .input_delay_ns = 0,
        .spics_io_num = EXTFLASH_CS_PIN,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr,
        .clock_source = SPI_CLK_SRC_DEFAULT,
    };

    // 添加设备到 SPI 总线
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取设备 ID 验证连接
    uint32_t id;
    ret = read_id(&id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read flash ID: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "External Flash initialized successfully");
    ESP_LOGI(TAG, "Device ID: 0x%06X", id);
    ESP_LOGI(TAG, "Flash Size: 8MB (W25Q64JVSIQ)");
    ESP_LOGI(TAG, "Connection: CS=IO%d, SCK=IO%d, MOSI=IO%d, MISO=IO%d, WP=IO%d",
             EXTFLASH_CS_PIN, EXTFLASH_SCK_PIN, EXTFLASH_MOSI_PIN, 
             EXTFLASH_MISO_PIN, EXTFLASH_WP_PIN);

    isReady = true;
    return ESP_OK;
}

esp_err_t ExtFlash::read_id(uint32_t* id) {
    if (!id) return ESP_ERR_INVALID_ARG;

    uint8_t rx_data[3];
    esp_err_t ret = send_cmd(W25Q64_CMD_READ_ID, nullptr, 0, rx_data, 3);
    if (ret == ESP_OK) {
        *id = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];
    }
    return ret;
}

esp_err_t ExtFlash::read(uint32_t addr, uint8_t* data, size_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    uint8_t addr_buf[3] = {
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)
    };

    return send_cmd(W25Q64_CMD_READ, addr_buf, 3, data, len);
}

esp_err_t ExtFlash::write(uint32_t addr, const uint8_t* data, size_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;
    size_t written = 0;

    while (written < len) {
        // 计算当前页的偏移和可写入的字节数
        size_t page_offset = addr % W25Q64_PAGE_SIZE;
        size_t chunk_len = (len - written < W25Q64_PAGE_SIZE - page_offset) ? 
                          (len - written) : (W25Q64_PAGE_SIZE - page_offset);

        // 发送写使能命令
        ret = send_cmd(W25Q64_CMD_WRITE_ENABLE, nullptr, 0, nullptr, 0);
        if (ret != ESP_OK) return ret;

        // 准备地址
        uint8_t addr_buf[3] = {
            (uint8_t)((addr >> 16) & 0xFF),
            (uint8_t)((addr >> 8) & 0xFF),
            (uint8_t)(addr & 0xFF)
        };

        // 发送页编程命令和数据
        spi_transaction_t t = {
            .flags = 0,
            .length = (1 + 3 + chunk_len) * 8,
            .tx_buffer = nullptr,
            .rx_buffer = nullptr,
            .override_freq_hz = 0,
        };

        size_t total_len = 1 + 3 + chunk_len;
        uint8_t* tx_buf = (uint8_t*)malloc(total_len);
        if (!tx_buf) return ESP_ERR_NO_MEM;

        tx_buf[0] = W25Q64_CMD_PAGE_PROGRAM;
        tx_buf[1] = addr_buf[0];
        tx_buf[2] = addr_buf[1];
        tx_buf[3] = addr_buf[2];
        memcpy(tx_buf + 4, data + written, chunk_len);

        t.tx_buffer = tx_buf;
        ret = spi_device_polling_transmit(spi_handle, &t);
        free(tx_buf);

        if (ret != ESP_OK) return ret;

        // 等待写入完成
        ret = wait_ready();
        if (ret != ESP_OK) return ret;

        written += chunk_len;
        addr += chunk_len;
    }

    return ESP_OK;
}

esp_err_t ExtFlash::erase_sector(uint32_t addr) {
    esp_err_t ret;

    // 发送写使能命令
    ret = send_cmd(W25Q64_CMD_WRITE_ENABLE, nullptr, 0, nullptr, 0);
    if (ret != ESP_OK) return ret;

    // 准备地址
    uint8_t addr_buf[3] = {
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)
    };

    // 发送扇区擦除命令
    ret = send_cmd(W25Q64_CMD_SECTOR_ERASE, addr_buf, 3, nullptr, 0);
    if (ret != ESP_OK) return ret;

    // 等待擦除完成
    return wait_ready();
}

esp_err_t ExtFlash::erase_chip() {
    esp_err_t ret;

    // 发送写使能命令
    ret = send_cmd(W25Q64_CMD_WRITE_ENABLE, nullptr, 0, nullptr, 0);
    if (ret != ESP_OK) return ret;

    // 发送芯片擦除命令
    ret = send_cmd(W25Q64_CMD_CHIP_ERASE, nullptr, 0, nullptr, 0);
    if (ret != ESP_OK) return ret;

    // 等待擦除完成 (芯片擦除可能需要较长时间)
    return wait_ready();
}

bool ExtFlash::is_ready() {
    return isReady;
}
