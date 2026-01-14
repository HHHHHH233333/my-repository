#ifndef __IMU_UTILS
#define __IMU_UTILS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

// ==================== ROS标准单位配置 (关键修改) ====================
#define IMU_SAMPLE_TIMES 200        // 校准采样次数

// ⚠️ 修改1: 将死区改为 rad/s 和 m/s²
// 0.5 度/s ≈ 0.008 rad/s
#define GYRO_DEADZONE 0.008f       // 陀螺仪死区 (rad/s)
// 0.05g ≈ 0.5 m/s²
#define ACCEL_DEADZONE 0.2f        // 加速度计死区 (m/s²), 0.2比较合适

#define MAX_GYRO_VALUE 34.0f       // 最大角速度 (rad/s) ≈ 2000°/s
#define MAX_ACCEL_VALUE 156.0f     // 最大加速度 (m/s²) ≈ 16g

// 滤波器参数 (保持不变，这些是比例系数)
#define ALPHA_GYRO 0.5f            // 降低一点，0.8对噪声抑制可能不够
#define ALPHA_ACCEL 0.3f           // 加速度计噪声大，系数设低一点(强滤波)
#define MEDIAN_WINDOW_SIZE 5       
#define KALMAN_Q 0.001f            
#define KALMAN_R 0.01f             

// MPU6050灵敏度 (用于内部读取)
#define GYRO_SENSITIVITY 16.4f     
#define ACCEL_SENSITIVITY 2048.0f   
#define GRAVITY 9.80665f           

// ==================== 数据结构 ====================
typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    bool is_calibrated;
} imu_offset_t;

typedef struct {
    float prev_gyro_x, prev_gyro_y, prev_gyro_z;
    float prev_accel_x, prev_accel_y, prev_accel_z;
    std::vector<float> gyro_z_buffer;
    float kalman_gyro_z;
    float kalman_p_gyro_z;
    bool initialized;
} filter_state_t;

static imu_offset_t g_imu_offset = {0, 0, 0, 0, 0, 0, false};
static filter_state_t g_filter_state = {0, 0, 0, 0, 0, 0, 
                                        std::vector<float>(), 
                                        0, 1.0, false};

// ==================== 内部工具函数 (保持不变) ====================
static inline float apply_deadzone(float value, float deadzone) {
    return (fabs(value) < deadzone) ? 0.0f : value;
}

static inline bool is_valid_gyro(float gyro) {
    return (fabs(gyro) < MAX_GYRO_VALUE);
}

static inline bool is_valid_accel(float accel) {
    return (fabs(accel) < MAX_ACCEL_VALUE);
}

static inline float lowpass_filter(float new_value, float prev_value, float alpha) {
    return alpha * new_value + (1.0f - alpha) * prev_value;
}

// 卡尔曼滤波
static float kalman_filter_1d(float measurement, float& state, float& p, 
                              float q = KALMAN_Q, float r = KALMAN_R) {
    float p_predict = p + q;
    float K = p_predict / (p_predict + r);
    state = state + K * (measurement - state);
    p = (1.0f - K) * p_predict;
    return state;
}

// ==================== 公开API函数 ====================

/**
 * @brief 获取陀螺仪Z轴偏移量
 * 注意：此函数返回的是原始单位(°/s)，为了配合 main.cpp 的计算逻辑
 */
float get_gyro_z_offset(int imu_fd) {
    signed int imu_buffer[7];
    float gyro_z_sum = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < IMU_SAMPLE_TIMES; i++) {
        int ret = read(imu_fd, imu_buffer, sizeof(imu_buffer));
        if (ret == sizeof(imu_buffer)) {
            // 这里保持原样，算出的是 deg/s
            float gyro_z = (float)(imu_buffer[2]) / GYRO_SENSITIVITY;
            if (fabs(gyro_z) < 200.0f) { // 简单阈值
                gyro_z_sum += gyro_z;
                valid_samples++;
            }
        }
        usleep(5000);
    }
    return (valid_samples > 0) ? (gyro_z_sum / valid_samples) : 0.0f;
}

/**
 * @brief IMU数据滤波与去噪
 * ⚠️ 输入输出单位必须是 ROS 标准单位：rad/s 和 m/s²
 */
void filter_imu_data(float& gyro_x_o, float& gyro_y_o, float& gyro_z_o,
                     float& accel_x_o, float& accel_y_o, float& accel_z_o) {
    
    // 1. 初始化
    if (!g_filter_state.initialized) {
        g_filter_state.prev_gyro_x = gyro_x_o;
        g_filter_state.prev_gyro_y = gyro_y_o;
        g_filter_state.prev_gyro_z = gyro_z_o;
        g_filter_state.prev_accel_x = accel_x_o;
        g_filter_state.prev_accel_y = accel_y_o;
        g_filter_state.prev_accel_z = accel_z_o;
        g_filter_state.kalman_gyro_z = gyro_z_o;
        g_filter_state.kalman_p_gyro_z = 1.0f;
        g_filter_state.initialized = true;
        return;
    }
    
    // 2. 异常值保护 (使用 rad/s 和 m/s² 阈值)
    if (!is_valid_gyro(gyro_x_o)) gyro_x_o = g_filter_state.prev_gyro_x;
    if (!is_valid_gyro(gyro_y_o)) gyro_y_o = g_filter_state.prev_gyro_y;
    if (!is_valid_gyro(gyro_z_o)) gyro_z_o = g_filter_state.prev_gyro_z;
    
    if (!is_valid_accel(accel_x_o)) accel_x_o = g_filter_state.prev_accel_x;
    if (!is_valid_accel(accel_y_o)) accel_y_o = g_filter_state.prev_accel_y;
    if (!is_valid_accel(accel_z_o)) accel_z_o = g_filter_state.prev_accel_z;

    // 3. 死区处理 (使用修正后的宏，单位 rad/s)
    gyro_x_o = apply_deadzone(gyro_x_o, GYRO_DEADZONE);
    gyro_y_o = apply_deadzone(gyro_y_o, GYRO_DEADZONE);
    gyro_z_o = apply_deadzone(gyro_z_o, GYRO_DEADZONE);
    
    // 4. 低通滤波
    gyro_x_o = lowpass_filter(gyro_x_o, g_filter_state.prev_gyro_x, ALPHA_GYRO);
    gyro_y_o = lowpass_filter(gyro_y_o, g_filter_state.prev_gyro_y, ALPHA_GYRO);
    gyro_z_o = lowpass_filter(gyro_z_o, g_filter_state.prev_gyro_z, ALPHA_GYRO);
    
    // 5. 卡尔曼滤波 (只针对 Z 轴，对导航最重要)
    gyro_z_o = kalman_filter_1d(gyro_z_o, 
                                g_filter_state.kalman_gyro_z, 
                                g_filter_state.kalman_p_gyro_z,
                                KALMAN_Q, KALMAN_R);
    
    accel_x_o = lowpass_filter(accel_x_o, g_filter_state.prev_accel_x, ALPHA_ACCEL);
    accel_y_o = lowpass_filter(accel_y_o, g_filter_state.prev_accel_y, ALPHA_ACCEL);
    accel_z_o = lowpass_filter(accel_z_o, g_filter_state.prev_accel_z, ALPHA_ACCEL);
    
    // 6. 更新历史
    g_filter_state.prev_gyro_x = gyro_x_o;
    g_filter_state.prev_gyro_y = gyro_y_o;
    g_filter_state.prev_gyro_z = gyro_z_o;
    g_filter_state.prev_accel_x = accel_x_o;
    g_filter_state.prev_accel_y = accel_y_o;
    g_filter_state.prev_accel_z = accel_z_o;
}

#endif
