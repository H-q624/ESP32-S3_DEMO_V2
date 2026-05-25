#include "app_fall_new.h"

NewFallDetector::NewFallDetector(float thresh) : threshold(thresh) {
    reset_state();
}

float NewFallDetector::process_sos(float input, BiquadState* state) {
    // 第一级
    float v_1 = input - a1_1 * state[0].v1 - a2_1 * state[0].v2;
    float y_1 = b0_1 * v_1 + b1_1 * state[0].v1 + b2_1 * state[0].v2;
    state[0].v2 = state[0].v1; state[0].v1 = v_1;
    // 第二级
    float v_2 = y_1 - a1_2 * state[1].v1 - a2_2 * state[1].v2;
    float y_2 = b0_2 * v_2 + b1_2 * state[1].v1 + b2_2 * state[1].v2;
    state[1].v2 = state[1].v1; state[1].v1 = v_2;
    return y_2;
}

void NewFallDetector::reset_state() {
    prev_accel[0] = 0.0f; prev_accel[1] = -256.0f; prev_accel[2] = 0.0f;
    j1_window.clear();
    j2_window.clear();
    x_states_window.clear();
    x4_history.clear();

    for(int i = 0; i < 4; i++) k_P[i] = k_Q;
    k_x[0] = 0.0f; k_x[1] = -256.0f; k_x[2] = 0.0f; k_x[3] = 0.0f;

    for(int i = 0; i < 2; i++) {
        sos_x[i] = {0, 0}; sos_y[i] = {0, 0}; sos_z[i] = {0, 0};
    }
    
    // 初始化滤波器平稳状态
    for(int i = 0; i < 50; i++) {
        process_sos(0.0f, sos_x);
        process_sos(-256.0f, sos_y);
        process_sos(0.0f, sos_z);
    }

    suspect_fall = false;
    wait_counter = 0;
    decimation_counter = 0;
}

bool NewFallDetector::process_100hz_data(float ax_g, float ay_g, float az_g) {
    decimation_counter++;
    if (decimation_counter < 4) return false;
    decimation_counter = 0;

    // 转换为数据集尺度，且反转 Y 轴
    float d_x = ax_g * 256.0f;
    float d_y = -ay_g * 256.0f; 
    float d_z = az_g * 256.0f;

    float fx = process_sos(d_x, sos_x);
    float fy = process_sos(d_y, sos_y);
    float fz = process_sos(d_z, sos_z);

    float j1 = sqrtf(powf(fx - prev_accel[0], 2) + powf(fy - prev_accel[1], 2) + powf(fz - prev_accel[2], 2));
    prev_accel[0] = fx; prev_accel[1] = fy; prev_accel[2] = fz;
    
    j1_window.push(j1);
    float max_j1 = 0.0f;
    for(int i = 0; i < j1_window.size(); i++) {
        if(j1_window.get(i) > max_j1) max_j1 = j1_window.get(i);
    }

    float bay = -256.0f;
    if (x_states_window.size() > 0) {
        float sum_y = 0;
        for(int i = 0; i < x_states_window.size(); i++) sum_y += x_states_window.get(i).y;
        bay = sum_y / x_states_window.size();
    }

    float y_obs[4] = {fx, fy, fz, fy - bay};

    for (int i = 0; i < 4; i++) {
        float x_minus = k_x[i];
        float P_minus = k_P[i] + k_Q;
        float S = P_minus + k_R[i];
        float K = P_minus / S;
        k_x[i] = x_minus + K * (y_obs[i] - x_minus);
        k_P[i] = (1.0f - K) * P_minus;
    }

    x_states_window.push({k_x[0], k_x[1], k_x[2]});

    float max_j2 = 0.0f;
    if (x_states_window.size() == WINDOW_SIZE_DETECTOR) {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for(int i = 0; i < WINDOW_SIZE_DETECTOR; i++) {
            Vec3 v = x_states_window.get(i);
            sum_x += v.x; sum_y += v.y; sum_z += v.z;
        }
        float mean_x = sum_x/WINDOW_SIZE_DETECTOR, mean_y = sum_y/WINDOW_SIZE_DETECTOR, mean_z = sum_z/WINDOW_SIZE_DETECTOR;
        
        float var_x = 0, var_y = 0, var_z = 0;
        for(int i = 0; i < WINDOW_SIZE_DETECTOR; i++) {
            Vec3 v = x_states_window.get(i);
            var_x += powf(v.x - mean_x, 2);
            var_y += powf(v.y - mean_y, 2);
            var_z += powf(v.z - mean_z, 2);
        }
        
        float std_x = sqrtf(var_x/WINDOW_SIZE_DETECTOR);
        float std_y = sqrtf(var_y/WINDOW_SIZE_DETECTOR);
        float std_z = sqrtf(var_z/WINDOW_SIZE_DETECTOR);
        
        float current_j2_norm = sqrtf((powf(std_x, 2) + powf(std_y, 2) + powf(std_z, 2)) / 3.0f);
        j2_window.push(current_j2_norm);

        for(int i = 0; i < j2_window.size(); i++) {
            if(j2_window.get(i) > max_j2) max_j2 = j2_window.get(i);
        }
    }

    float x4 = k_x[3];
    float j3 = max_j1 * (max_j2 * max_j2);

    if (!suspect_fall) {
        if (j3 > threshold) {
            suspect_fall = true;
            wait_counter = 0;
            x4_history.clear();
        }
    } else {
        x4_history.push(x4);
        wait_counter++;
        if (wait_counter >= PERIODIC_WAIT) {
            int zero_crossings = 0;
            for(int i = 1; i < x4_history.size(); i++) {
                if (x4_history.get(i-1) * x4_history.get(i) < 0) zero_crossings++;
            }
            suspect_fall = false; 
            wait_counter = 0;
            
            if (zero_crossings < 4) {
                return true; 
            }
        }
    }
    return false;
}