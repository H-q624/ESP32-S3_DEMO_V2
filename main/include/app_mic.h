#ifndef APP_MIC_H
#define APP_MIC_H

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include <cstddef>
#include <cstdint>

// 麦克风配置 - IO15连接到ADC2_CHANNEL_2
#define MIC_SAMPLE_RATE 16000      // 采样率 16kHz
#define MIC_ADC_UNIT    ADC_UNIT_2
#define MIC_ADC_CHANNEL ADC_CHANNEL_2  // IO15 = ADC2_CHANNEL_2
#define MIC_ADC_ATTEN   ADC_ATTEN_DB_11 // 11dB 衰减，最大量程约 3.3V
#define MIC_BUFFER_SIZE 1024        // 缓冲区大小

// WAV文件相关定义
#define WAV_HEADER_SIZE 44
#define WAV_QUEUE_SIZE 20

// WAV数据包结构
typedef struct {
    uint8_t* data;
    size_t total_size;
    size_t pcm_data_size;
    uint32_t timestamp;
    bool is_wav_header_included;
} mic_wav_packet_t;

// 外部队列声明
extern QueueHandle_t wav_queue;

class MEMS_MIC {
public:
    MEMS_MIC(const char* tag);
    ~MEMS_MIC();
    
    esp_err_t app_mic_init();
    void app_mic_start();
    void app_mic_stop();
    bool app_mic_check_module();
    void app_mic_create_task();
    
    // 测试函数
    static void adc_read_test(MEMS_MIC* instance);
    
private:
    const char* TAG;
    bool isReady;
    TaskHandle_t mic_handler;
    
    // Legacy ADC API成员
    adc_channel_t adc_channel_num;
    adc_unit_t adc_unit_num;
    
    // 采样缓冲区
    int16_t sample_buffer[MIC_BUFFER_SIZE];
    size_t buffer_pos;
    
    void mic_collection_loop();
    static void app_mic_task_entry(void* arg);
};

#endif // APP_MIC_H
