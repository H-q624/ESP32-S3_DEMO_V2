#ifndef PAPER_FALL_DETECTOR_H
#define PAPER_FALL_DETECTOR_H

#include <cmath>

// 静态环形缓冲区（模板类必须在头文件实现）
template <typename T, int Size>
class CircularBuffer {
private:
    T data[Size];
    int head = 0;
    int count = 0;
public:
    void push(T val) {
        data[head] = val;
        head = (head + 1) % Size;
        if (count < Size) count++;
    }
    void clear() { head = 0; count = 0; }
    int size() const { return count; }
    T get(int i) const { 
        return data[(head - count + Size + i) % Size];
    }
};

// 辅助结构体
struct BiquadState { float v1 = 0, v2 = 0; };
struct Vec3 { float x, y, z; };

class NewFallDetector {
private:
    const float FS = 25.0f;
    const int WINDOW_SIZE_DETECTOR = 25;
    const int PERIODIC_WAIT = 75;
    float threshold;

    BiquadState sos_x[2], sos_y[2], sos_z[2];
    
    // 预计算的 SOS 系数
    const float b0_1 = 0.1839f, b1_1 = 0.3678f, b2_1 = 0.1839f, a1_1 = -0.3289f, a2_1 = 0.0646f;
    const float b0_2 = 0.2533f, b1_2 = 0.5066f, b2_2 = 0.2533f, a1_2 = -0.4531f, a2_2 = 0.4663f;

    float prev_accel[3];
    CircularBuffer<float, 25> j1_window;
    CircularBuffer<float, 25> j2_window;
    CircularBuffer<Vec3, 25> x_states_window;
    CircularBuffer<float, 75> x4_history;

    float k_x[4];
    float k_P[4];
    const float k_Q = 0.000001f; 
    const float k_R[4] = {0.0025f, 0.0025f, 0.0025f, 0.0001f}; 

    bool suspect_fall;
    int wait_counter;
    int decimation_counter;

    // 内部函数声明
    float process_sos(float input, BiquadState* state);

public:
    NewFallDetector(float thresh = 40000.0f);
    void reset_state();
    bool process_100hz_data(float ax_g, float ay_g, float az_g);
};

#endif // NEW_FALL_DETECTOR_H