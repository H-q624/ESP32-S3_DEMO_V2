#ifndef _APP_FILE_H_
#define _APP_FILE_H_

#include "app_mpu6050.h"
#include "app_mic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>

#define MAX_FILES             3

// 双缓冲配置
#define BUFFER_COUNT          2           // 双缓冲数量
#define SAMPLES_PER_BUFFER    100         // 每个缓冲区存放100个样本（1秒数据）
#define BUFFER_TIMEOUT_MS     500         // 最大缓冲时间500ms

#include "esp_err.h"

typedef struct{
    FILE* file[MAX_FILES];
    int file_numbers[MAX_FILES];
    size_t file_size[MAX_FILES];  // 对应文件大小存储，避免多次读取        
    int index;
}files_t;

// IMU数据缓冲区结构
typedef struct {
    mpu6050_processed_data_t samples[SAMPLES_PER_BUFFER];
    size_t count;               // 实际样本数
    size_t buffer_id;           // 缓冲区ID（用于调试）
} imu_buffer_t;


class FileStorage {
    protected:
        static bool s_initialized;  // 静态标志位，标记是否已初始化
        static SemaphoreHandle_t s_mutex; // 静态互斥锁
        
        const char* mount_point ;   // 文件路径字符串
        const char* TAG;            // 日志提示模组名
        bool isReady;               // 初始化状态
        uint16_t file_number;       // 文件序号
        files_t files;              // 文件句柄
    public:
        FileStorage(const char* tag); // 构造函数
        virtual ~FileStorage();      // 虚析构函数

        // 基础文件操作
        static void app_file_init_once();  // 修改为静态方法，只初始化一次
        virtual void app_file_init();
        virtual void app_file_check_file_number(const char* prefix); // 确认文件编号
        virtual bool app_file_check_file(const char* filename, size_t* file_size = nullptr); // 检查文件是否已存在        
        virtual esp_err_t app_file_open_file(const char* prefix); // 新建文件并打开
        virtual esp_err_t app_file_write_file(int index,void* data);   // 根据索引写对应文件
        virtual esp_err_t app_file_save_file(int index);  // 根据索引保存文件并释放指针
        virtual bool app_file_check_module(); // 检查组件状态
        void app_file_start();  // 启动组件
        void app_file_stop();   // 关闭组件
        // 必须重构的纯虚函数
        virtual void app_file_create_task()= 0 ; // 创建freertos任务
};

class IMU_Data :public FileStorage{
    private:
        // 双缓冲机制（替代原有单缓冲）
        imu_buffer_t buffers[BUFFER_COUNT];     // 两个缓冲区
        imu_buffer_t* current_buffer;            // 当前写入缓冲区
        size_t current_buffer_id;                // 当前缓冲区索引 (0 或 1)
        uint32_t last_switch_time;               // 上次缓冲区切换时间
        
        // 任务和同步（静态成员，确保只有一个实例）
        static TaskHandle_t s_collection_task;   // 数据采集任务句柄
        static TaskHandle_t s_write_task;        // 文件写入任务句柄
        static QueueHandle_t s_write_queue;      // 写入队列（传递缓冲区指针）
        
        uint32_t last_write_time;       // 上次写入时间
    public:
        const char* imu_filename_prefix;            // 文件名前缀
        IMU_Data(const char* tag);                  // 构造函数
        ~IMU_Data();

        // 基类函数重写
        esp_err_t app_file_open_file(const char* prefix) override;
        void app_file_create_task() override;

        // IMU数据操作（新方法）
        void imu_data_init(size_t size);            // 初始化数据缓冲区
        void collection_task_loop();                // 数据采集任务主循环（高优先级）
        void write_task_loop();                     // 文件写入任务主循环（低优先级）
        void write_buffer_to_storage(imu_buffer_t* buffer); // 批量写入缓冲区
        imu_buffer_t* switch_buffer();              // 切换缓冲区
        
        // 静态任务入口
        static void collection_task_entry(void* arg);
        static void write_task_entry(void* arg);
};

class MIC_Data:public FileStorage{
    private:
        uint32_t sample_rate;           // 采样率（Hz），用于生成WAV头，如16000
        uint8_t bit_depth;              // 位深度（bits），用于生成WAV头，如16

        uint32_t last_write_time;       // 上次写入时间（ms），用于定期刷新
        uint32_t last_flush_time;       // 上次刷新时间（ms）
        
        // WAV文件流式写入相关变量
        uint32_t total_pcm_size;        // 当前文件累计写入的PCM数据总大小（字节）
        bool wav_header_written;        // 标记WAV头是否已写入文件
        uint8_t wav_header[WAV_HEADER_SIZE];  // WAV头缓冲区（44字节）
        long wav_header_position;       // WAV头在文件中的偏移位置

    public:
        int get_current_file_index();   // 获取当前使用的文件索引（public，供模式切换时使用）
        const char* mic_filename_prefix;// 麦克风文件名前缀，如"mic"
        MIC_Data(const char *tag);
        ~MIC_Data();

        // 基类函数重写
        void app_file_check_file_number(const char* prefix) override; // 确认文件编号
        esp_err_t app_file_open_file(const char* prefix) override;
        esp_err_t app_file_save_file(int index) override;  // 重写：关闭前更新WAV头
        void app_file_create_task() override;              // 创建麦克风写入任务

        // 麦克风数据操作
        void mic_data_init(size_t size);    // 初始化音频缓冲区
        
        // WAV文件流式写入方法
        void mic_write_task_loop();         // 麦克风写入任务主循环：从队列接收数据包并写入文件
        void update_wav_header(int index);  // 更新WAV头：根据total_pcm_size修正RIFF和data chunk大小
        void write_wav_packet(mic_wav_packet_t* packet); // 写入WAV数据包到当前打开的文件
        static void mic_write_task_entry(void *arg); // 静态任务入口函数
};

#endif