#ifndef __IMU_UTILS
#define __IMU_UTILS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

// ==================== 配置参数 ====================
#define IMU_SAMPLE_TIMES 200        // 校准采样次数
#define GYRO_DEADZONE 0.5          // 陀螺仪死区 (°/s)
#define ACCEL_DEADZONE 0.05        // 加速度计死区 (g)
#define MAX_GYRO_VALUE 2000.0      // 最大角速度 (°/s)
#define MAX_ACCEL_VALUE 16.0       // 最大加速度 (g)

// 滤波器参数
#define ALPHA_GYRO 0.8             // 陀螺仪低通滤波系数
#define ALPHA_ACCEL 0.9            // 加速度计低通滤波系数
#define MEDIAN_WINDOW_SIZE 5       // 中值滤波窗口
#define KALMAN_Q 0.001             // 卡尔曼过程噪声
#define KALMAN_R 0.01              // 卡尔曼测量噪声

// MPU6050灵敏度
#define GYRO_SENSITIVITY 16.4      // LSB/(°/s)
#define ACCEL_SENSITIVITY 2048.0   // LSB/g
#define GRAVITY 9.80665            // m/s²

// ==================== 数据结构 ====================
typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    bool is_calibrated;
} imu_offset_t;

typedef struct {
    float prev_gyro_x, prev_gyro_y, prev_gyro_z;
    float prev_accel_x, prev_accel_y, prev_accel_z;
    
    // 中值滤波缓冲区
    std::vector<float> gyro_z_buffer;
    
    // 卡尔曼滤波状态
    float kalman_gyro_z;
    float kalman_p_gyro_z;
    
    bool initialized;
} filter_state_t;

// ==================== 全局静态变量 ====================
static imu_offset_t g_imu_offset = {0, 0, 0, 0, 0, 0, false};
static filter_state_t g_filter_state = {0, 0, 0, 0, 0, 0, 
                                        std::vector<float>(), 
                                        0, 1.0, false};

// ==================== 内部工具函数 ====================

/**
 * @brief 限幅函数
 */
static inline float clamp_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief 死区处理
 */
static inline float apply_deadzone(float value, float deadzone) {
    return (fabs(value) < deadzone) ? 0.0f : value;
}

/**
 * @brief 异常值检测
 */
static inline bool is_valid_gyro(float gyro) {
    return (fabs(gyro) < MAX_GYRO_VALUE);
}

static inline bool is_valid_accel(float accel) {
    return (fabs(accel) < MAX_ACCEL_VALUE);
}

/**
 * @brief 一阶低通滤波器
 */
static inline float lowpass_filter(float new_value, float prev_value, float alpha) {
    return alpha * new_value + (1.0f - alpha) * prev_value;
}

/**
 * @brief 中值滤波器
 */
static float median_filter(std::vector<float>& buffer, float new_value, int window_size) {
    buffer.push_back(new_value);
    if (buffer.size() > (size_t)window_size) {
        buffer.erase(buffer.begin());
    }
    
    if (buffer.size() < 3) {
        return new_value;
    }
    
    std::vector<float> sorted = buffer;
    std::sort(sorted.begin(), sorted.end());
    
    return sorted[sorted.size() / 2];
}

/**
 * @brief 卡尔曼滤波器 (1维)
 */
static float kalman_filter_1d(float measurement, float& state, float& p, 
                              float q = KALMAN_Q, float r = KALMAN_R) {
    // 预测
    float p_predict = p + q;
    
    // 更新
    float K = p_predict / (p_predict + r);
    state = state + K * (measurement - state);
    p = (1.0f - K) * p_predict;
    
    return state;
}

// ==================== 公开API函数 ====================

/**
 * @brief 获取陀螺仪Z轴偏移量（增强版，兼容原接口）
 * @param imu_fd IMU设备文件描述符
 * @return 偏移量 (°/s)
 */
float get_gyro_z_offset(int imu_fd) {
    signed int imu_buffer[7];
    float gyro_z_sum = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < IMU_SAMPLE_TIMES; i++) {
        int ret = read(imu_fd, imu_buffer, sizeof(imu_buffer));
        
        if (ret == sizeof(imu_buffer)) {
            float gyro_z = (float)(imu_buffer[2]) / GYRO_SENSITIVITY;
            
            if (is_valid_gyro(gyro_z) && fabs(gyro_z) < 50.0f) {
                gyro_z_sum += gyro_z;
                valid_samples++;
            }
        }
        usleep(5000);
    }
    
    float offset = (valid_samples > 0) ? (gyro_z_sum / valid_samples) : 0.0f;
    
    g_imu_offset.gyro_z = offset;
    g_imu_offset.is_calibrated = true;
    
    return offset;
}

/**
 * @brief 完整6轴IMU校准（可选高级功能）
 * @param imu_fd IMU设备文件描述符
 * @return 0成功, -1失败
 */
int calibrate_imu_full(int imu_fd) {
    signed int imu_buffer[7];
    float gyro_sum[3] = {0, 0, 0};
    float accel_sum[3] = {0, 0, 0};
    int valid_samples = 0;
    
    for (int i = 0; i < IMU_SAMPLE_TIMES; i++) {
        int ret = read(imu_fd, imu_buffer, sizeof(imu_buffer));
        
        if (ret == sizeof(imu_buffer)) {
            float gyro[3] = {
                (float)(imu_buffer[0]) / GYRO_SENSITIVITY,
                (float)(imu_buffer[1]) / GYRO_SENSITIVITY,
                (float)(imu_buffer[2]) / GYRO_SENSITIVITY
            };
            
            float accel[3] = {
                (float)(imu_buffer[3]) / ACCEL_SENSITIVITY,
                (float)(imu_buffer[4]) / ACCEL_SENSITIVITY,
                (float)(imu_buffer[5]) / ACCEL_SENSITIVITY
            };
            
            bool valid = true;
            for (int j = 0; j < 3; j++) {
                if (!is_valid_gyro(gyro[j]) || !is_valid_accel(accel[j])) {
                    valid = false;
                    break;
                }
            }
            
            if (valid) {
                for (int j = 0; j < 3; j++) {
                    gyro_sum[j] += gyro[j];
                    accel_sum[j] += accel[j];
                }
                valid_samples++;
            }
        }
        
        usleep(5000);
    }
    
    if (valid_samples < IMU_SAMPLE_TIMES * 0.8) {
        return -1;
    }
    
    g_imu_offset.gyro_x = gyro_sum[0] / valid_samples;
    g_imu_offset.gyro_y = gyro_sum[1] / valid_samples;
    g_imu_offset.gyro_z = gyro_sum[2] / valid_samples;
    
    g_imu_offset.accel_x = accel_sum[0] / valid_samples;
    g_imu_offset.accel_y = accel_sum[1] / valid_samples;
    g_imu_offset.accel_z = (accel_sum[2] / valid_samples) - 1.0f;
    
    g_imu_offset.is_calibrated = true;
    
    return 0;
}

/**
 * @brief IMU数据滤波与去噪（完整实现，自动应用）
 * @param gyro_x_o  陀螺仪X轴 (输入/输出, °/s)
 * @param gyro_y_o  陀螺仪Y轴
 * @param gyro_z_o  陀螺仪Z轴
 * @param accel_x_o 加速度计X轴 (输入/输出, g)
 * @param accel_y_o 加速度计Y轴
 * @param accel_z_o 加速度计Z轴
 */
void filter_imu_data(float& gyro_x_o, float& gyro_y_o, float& gyro_z_o,
                     float& accel_x_o, float& accel_y_o, float& accel_z_o) {
    
    // ===== 1. 初始化滤波器状态 =====
    if (!g_filter_state.initialized) {
        g_filter_state.prev_gyro_x = gyro_x_o;
        g_filter_state.prev_gyro_y = gyro_y_o;
        g_filter_state.prev_gyro_z = gyro_z_o;
        g_filter_state.prev_accel_x = accel_x_o;
        g_filter_state.prev_accel_y = accel_y_o;
        g_filter_state.prev_accel_z = accel_z_o;
        
        g_filter_state.kalman_gyro_z = gyro_z_o;
        g_filter_state.kalman_p_gyro_z = 1.0f;
        
        g_filter_state.gyro_z_buffer.reserve(MEDIAN_WINDOW_SIZE);
        
        g_filter_state.initialized = true;
        
        return;
    }
    
    // ===== 2. 异常值检测与保护 =====
    if (!is_valid_gyro(gyro_x_o)) {
        gyro_x_o = g_filter_state.prev_gyro_x;
    }
    if (!is_valid_gyro(gyro_y_o)) {
        gyro_y_o = g_filter_state.prev_gyro_y;
    }
    if (!is_valid_gyro(gyro_z_o)) {
        gyro_z_o = g_filter_state.prev_gyro_z;
    }
    
    if (!is_valid_accel(accel_x_o)) {
        accel_x_o = g_filter_state.prev_accel_x;
    }
    if (!is_valid_accel(accel_y_o)) {
        accel_y_o = g_filter_state.prev_accel_y;
    }
    if (!is_valid_accel(accel_z_o)) {
        accel_z_o = g_filter_state.prev_accel_z;
    }
    
    // ===== 3. 零偏补偿 =====
    if (g_imu_offset.is_calibrated) {
        gyro_x_o -= g_imu_offset.gyro_x;
        gyro_y_o -= g_imu_offset.gyro_y;
        gyro_z_o -= g_imu_offset.gyro_z;
        
        accel_x_o -= g_imu_offset.accel_x;
        accel_y_o -= g_imu_offset.accel_y;
        accel_z_o -= g_imu_offset.accel_z;
    }
    
    // ===== 4. 死区处理 =====
    gyro_x_o = apply_deadzone(gyro_x_o, GYRO_DEADZONE);
    gyro_y_o = apply_deadzone(gyro_y_o, GYRO_DEADZONE);
    gyro_z_o = apply_deadzone(gyro_z_o, GYRO_DEADZONE);
    
    accel_x_o = apply_deadzone(accel_x_o, ACCEL_DEADZONE);
    accel_y_o = apply_deadzone(accel_y_o, ACCEL_DEADZONE);
    
    // ===== 5. 低通滤波 (陀螺仪) =====
    gyro_x_o = lowpass_filter(gyro_x_o, g_filter_state.prev_gyro_x, ALPHA_GYRO);
    gyro_y_o = lowpass_filter(gyro_y_o, g_filter_state.prev_gyro_y, ALPHA_GYRO);
    gyro_z_o = lowpass_filter(gyro_z_o, g_filter_state.prev_gyro_z, ALPHA_GYRO);
    
    // ===== 6. 卡尔曼滤波 (陀螺仪Z轴) =====
    gyro_z_o = kalman_filter_1d(gyro_z_o, 
                                g_filter_state.kalman_gyro_z, 
                                g_filter_state.kalman_p_gyro_z,
                                KALMAN_Q, KALMAN_R);
    
    // ===== 7. 低通滤波 (加速度计) =====
    accel_x_o = lowpass_filter(accel_x_o, g_filter_state.prev_accel_x, ALPHA_ACCEL);
    accel_y_o = lowpass_filter(accel_y_o, g_filter_state.prev_accel_y, ALPHA_ACCEL);
    accel_z_o = lowpass_filter(accel_z_o, g_filter_state.prev_accel_z, ALPHA_ACCEL);
    
    // ===== 8. 更新历史值 =====
    g_filter_state.prev_gyro_x = gyro_x_o;
    g_filter_state.prev_gyro_y = gyro_y_o;
    g_filter_state.prev_gyro_z = gyro_z_o;
    
    g_filter_state.prev_accel_x = accel_x_o;
    g_filter_state.prev_accel_y = accel_y_o;
    g_filter_state.prev_accel_z = accel_z_o;
}

// ==================== 辅助诊断函数 ====================

/**
 * @brief 检查IMU健康状态
 * @return true正常, false异常
 */
bool check_imu_health(float gyro_x, float gyro_y, float gyro_z,
                      float accel_x, float accel_y, float accel_z) {
    if (!is_valid_gyro(gyro_x) || !is_valid_gyro(gyro_y) || !is_valid_gyro(gyro_z)) {
        return false;
    }
    
    float accel_magnitude = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    if (fabs(accel_magnitude - 1.0f) > 0.5f) {
        return false;
    }
    
    return true;
}

/**
 * @brief 打印IMU原始数据（调试用）
 */
void print_imu_data(float gyro_x, float gyro_y, float gyro_z,
                   float accel_x, float accel_y, float accel_z) {
    printf("[IMU] Gyro:  [%7.2f, %7.2f, %7.2f] °/s\n", gyro_x, gyro_y, gyro_z);
    printf("[IMU] Accel: [%7.3f, %7.3f, %7.3f] g\n", accel_x, accel_y, accel_z);
}

/**
 * @brief 重置滤波器状态（切换模式时使用）
 */
void reset_imu_filter() {
    g_filter_state.initialized = false;
    g_filter_state.gyro_z_buffer.clear();
}

/**
 * @brief 获取当前校准状态
 */
bool is_imu_calibrated() {
    return g_imu_offset.is_calibrated;
}

/**
 * @brief 手动设置校准参数（从配置文件加载）
 */
void set_imu_offset(float gx, float gy, float gz, 
                    float ax, float ay, float az) {
    g_imu_offset.gyro_x = gx;
    g_imu_offset.gyro_y = gy;
    g_imu_offset.gyro_z = gz;
    g_imu_offset.accel_x = ax;
    g_imu_offset.accel_y = ay;
    g_imu_offset.accel_z = az;
    g_imu_offset.is_calibrated = true;
}

#endif // __IMU_UTILS
