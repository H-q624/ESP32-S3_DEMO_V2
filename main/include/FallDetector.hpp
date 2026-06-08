/**
 * FallDetector.hpp
 * ESP32-S3 跌倒检测器 — 单头文件, 即拿即用.
 *
 * 算法: 固定全局归一化 + 119pt 滑动峰度 + 300pt Pitch + 三段式判决
 * 硬件成本: ~8KB RAM, <0.1% CPU @ 238Hz (ESP32-S3 @ 240MHz)
 *
 * 使用:
 *   FallDetector detector;
 *   void loop() {
 *       detector.update(ax, ay, az, gx, gy, gz);
 *       if (detector.isFallDetected()) {
 *           // 报警, 然后 detector.clearFall()
 *       }
 *   }
 *
 * 归一化常数来自 SisFall 数据集 Neck_data 1090万样本预计算.
 * 更换传感器或佩戴位置后需重新标定.
 */
#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ========== 环形缓冲区 (单轴) ==========
class RingBuffer {
public:
    RingBuffer(int capacity)
        : m_capacity(capacity), m_count(0), m_head(0) {
        m_data = new float[capacity];
    }
    ~RingBuffer() { delete[] m_data; }

    void push(float val) {
        m_data[m_head] = val;
        m_head = (m_head + 1) % m_capacity;
        if (m_count < m_capacity) m_count++;
    }

    float read(int offset) const {
        int idx = (m_head - 1 - offset + m_capacity) % m_capacity;
        return m_data[idx];
    }

    bool isFull()  const { return m_count >= m_capacity; }
    int  count()   const { return m_count; }
    int  capacity() const { return m_capacity; }

    float mean() const {
        int n = m_count;
        if (n == 0) return 0.0f;
        float s = 0.0f;
        for (int i = 0; i < n; i++) s += read(i);
        return s / n;
    }

    float variance() const {
        int n = m_count;
        if (n == 0) return 0.0f;
        float mu = mean();
        float s = 0.0f;
        for (int i = 0; i < n; i++) {
            float d = read(i) - mu;
            s += d * d;
        }
        return s / n;
    }

    float stddev() const { return std::sqrt(variance()); }

private:
    float* m_data;
    int m_capacity;
    int m_count;
    int m_head;
};


// ========== 三轴环形缓冲区 ==========
class RingBuffer3Axis {
public:
    RingBuffer3Axis(int capacity)
        : buf_x(capacity), buf_y(capacity), buf_z(capacity) {}

    void push(float x, float y, float z) {
        buf_x.push(x); buf_y.push(y); buf_z.push(z);
    }

    RingBuffer& getX() { return buf_x; }
    RingBuffer& getY() { return buf_y; }
    RingBuffer& getZ() { return buf_z; }

    bool isFull() const {
        return buf_x.isFull() && buf_y.isFull() && buf_z.isFull();
    }

private:
    RingBuffer buf_x, buf_y, buf_z;
};


// ========== 跌倒检测器 ==========
class FallDetector {
public:
    // ---- 归一化常数 (SisFall Neck_data 1090万样本) ----
    static constexpr float ACC_MEAN_X = 2886.161510f;
    static constexpr float ACC_MEAN_Y = -17.869233f;
    static constexpr float ACC_MEAN_Z = -619.992727f;
    static constexpr float ACC_STD_X  = 2011.111378f;
    static constexpr float ACC_STD_Y  = 1369.313413f;
    static constexpr float ACC_STD_Z  = 2047.951279f;
    static constexpr float GYR_MEAN_X = -57.350381f;
    static constexpr float GYR_MEAN_Y = 84.430060f;
    static constexpr float GYR_MEAN_Z = 15.593921f;
    static constexpr float GYR_STD_X  = 773.394860f;
    static constexpr float GYR_STD_Y  = 585.159202f;
    static constexpr float GYR_STD_Z  = 513.334267f;

    // ---- 阈值 ----
    static constexpr float K1_A = 4.5f;    // Acc 冲击
    static constexpr float K1_G = 6.0f;    // Gyr 冲击
    static constexpr float K2_A = 2.0f;    // Acc 静止
    static constexpr float K2_G = 3.0f;    // Gyr 静止
    static constexpr float PITCH_TH = -45.0f;  // 姿态

    // ---- 时序参数 ----
    static constexpr int FS          = 238;
    static constexpr int WINDOW_KURT = static_cast<int>(0.5f * FS);  // 119
    static constexpr int WINDOW_PITCH = 300;
    static constexpr int T1_OFFSET   = static_cast<int>(1.0f * FS);  // 冲击后 1s
    static constexpr int T3_OFFSET   = static_cast<int>(3.0f * FS);  // 冲击后 3s
    static constexpr int K2_MIN_LEN  = static_cast<int>(0.5f * FS);  // 连续 0.5s

    // ---- 检测结果 ----
    enum Result : uint8_t { NO_FALL = 0, FALL_DETECTED = 1 };

    // ---- 内部状态 (调试用) ----
    enum State : uint8_t {
        STATE_IDLE       = 0,
        STATE_IMPACTED   = 1,   // 冲击命中, 等待 1s
        STATE_MONITORING = 2,   // 监控 k2 + pitch
    };

    FallDetector()
        : m_kurtAcc(WINDOW_KURT), m_kurtGyr(WINDOW_KURT),
          m_pitchAcc(WINDOW_PITCH),
          m_sampleCount(0), m_impactSample(-1),
          m_state(STATE_IDLE), m_fallResult(false),
          m_k2Pass(false), m_pitchEver(false),
          m_k2Count(0), m_currentPitch(0.0f) {}

    /** 禁止拷贝 (内含 raw pointer) */
    FallDetector(const FallDetector&) = delete;
    FallDetector& operator=(const FallDetector&) = delete;

    /**
     * 每样本调用一次.
     * @return true 代表检测到跌倒 (锁存, 需 clearFall() 复位)
     */
    bool update(float ax, float ay, float az,
                float gx, float gy, float gz) {
        // ---- 固定归一化 (每轴 1 次乘加) ----
        float nax = (ax - ACC_MEAN_X) * (1.0f / ACC_STD_X);
        float nay = (ay - ACC_MEAN_Y) * (1.0f / ACC_STD_Y);
        float naz = (az - ACC_MEAN_Z) * (1.0f / ACC_STD_Z);
        float ngx = (gx - GYR_MEAN_X) * (1.0f / GYR_STD_X);
        float ngy = (gy - GYR_MEAN_Y) * (1.0f / GYR_STD_Y);
        float ngz = (gz - GYR_MEAN_Z) * (1.0f / GYR_STD_Z);

        // ---- 推入缓冲区 ----
        m_kurtAcc.push(nax, nay, naz);
        m_kurtGyr.push(ngx, ngy, ngz);
        m_pitchAcc.push(ax, ay, az);
        m_sampleCount++;

        // 缓冲区未填满, 跳过
        if (!m_kurtAcc.isFull() || !m_pitchAcc.isFull()) {
            return m_fallResult;
        }

        // ---- 计算特征 ----
        float kA = computeKurtosis3Axis(m_kurtAcc);
        float kG = computeKurtosis3Axis(m_kurtGyr);
        m_currentPitch = computePitch();

        // ---- 状态机 ----
        switch (m_state) {
            case STATE_IDLE:
                if (kA > K1_A && kG > K1_G) {
                    m_impactSample = m_sampleCount;
                    m_k2Pass = false;
                    m_pitchEver = false;
                    m_k2Count = 0;
                    m_state = STATE_IMPACTED;
                }
                break;

            case STATE_IMPACTED:
                if (m_sampleCount - m_impactSample >= T1_OFFSET) {
                    m_k2Count = 0;
                    m_state = STATE_MONITORING;
                }
                break;

            case STATE_MONITORING: {
                int elapsed = m_sampleCount - m_impactSample;

                // k2 静止: 两个峰度同时低于阈值
                if (kA < K2_A && kG < K2_G) {
                    m_k2Count++;
                } else {
                    m_k2Count = 0;
                }
                if (m_k2Count >= K2_MIN_LEN) m_k2Pass = true;

                // Pitch 倒地
                if (m_currentPitch > PITCH_TH) m_pitchEver = true;

                // 窗口结束 → 判决
                if (elapsed >= T3_OFFSET) {
                    if (m_k2Pass && m_pitchEver) {
                        m_fallResult = true;
                    }
                    resetState();
                }
                break;
            }
        }

        return m_fallResult;
    }

    /** 复位跌倒标志 */
    void clearFall() { m_fallResult = false; }

    /** 查询跌倒标志 */
    bool isFallDetected() const { return m_fallResult; }

    // ===== 调试接口 =====
    State getState()          const { return m_state; }
    float getPitch()          const { return m_currentPitch; }
    int   getSampleCount()    const { return m_sampleCount; }
    int   getImpactSample()   const { return m_impactSample; }
    bool  getK2Pass()         const { return m_k2Pass; }
    bool  getPitchEver()      const { return m_pitchEver; }

private:
    RingBuffer3Axis m_kurtAcc;   // 归一化 Acc, 119 深
    RingBuffer3Axis m_kurtGyr;   // 归一化 Gyr, 119 深
    RingBuffer3Axis m_pitchAcc;  // 原始 Acc, 300 深

    int   m_sampleCount;
    int   m_impactSample;
    State m_state;
    bool  m_fallResult;

    // 监控中间态
    bool  m_k2Pass;
    bool  m_pitchEver;
    int   m_k2Count;
    float m_currentPitch;

    /** 单轴峰度: mean((x-μ)^4) / (σ^4 + 1) */
    static float kurtosis(const RingBuffer& buf) {
        int n = buf.count();
        if (n == 0) return 0.0f;
        float mu = buf.mean();
        float var = 0.0f, m4 = 0.0f;
        for (int i = 0; i < n; i++) {
            float d = buf.read(i) - mu;
            float d2 = d * d;
            var += d2;
            m4 += d2 * d2;
        }
        var /= n;
        m4  /= n;
        if (var < 1e-12f) return 0.0f;
        return m4 / (var * var + 1.0f);
    }

    static float computeKurtosis3Axis(RingBuffer3Axis& buf) {
        return kurtosis(buf.getX())
             + kurtosis(buf.getY())
             + kurtosis(buf.getZ());
    }

    float computePitch() {
        float mx = m_pitchAcc.getX().mean();
        float my = m_pitchAcc.getY().mean();
        float mz = m_pitchAcc.getZ().mean();
        return std::atan2(-mx, std::sqrt(my * my + mz * mz))
               * 180.0f / M_PI;
    }

    void resetState() {
        m_state = STATE_IDLE;
        m_impactSample = -1;
        m_k2Count = 0;
        m_k2Pass = false;
        m_pitchEver = false;
    }
};
