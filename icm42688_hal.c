/**
 * @file      icm42688_hal.c
 * @brief     ICM42688传感器硬件抽象层实现
 * @author    AzureHalcyon
 * @version   1.0.0
 * @date      2025-03-28
 * @note      ICM42688传感器硬件抽象层实现
 */

#include "icm42688_hal.h"

// 全局变量
icm42688_axis_data_t icm42688_gyro;     /**< 三轴陀螺仪数据 */
icm42688_axis_data_t icm42688_acc;      /**< 三轴加速度计数据 */
float icm42688_transition_factor[2];    /**< 转换系数 */

// 传感器配置
static icm42688_config_t *p_config = NULL;

// 硬件接口
static icm42688_comm_t *p_comm = NULL;
static icm42688_system_t *p_system = NULL;

/**
 * @brief      初始化ICM42688 HAL层
 * @param      comm    通信接口
 * @param      system  系统接口
 * @param      config  ICM42688配置
 */
void icm42688_hal_init(icm42688_comm_t *comm, icm42688_system_t *system, icm42688_config_t *config) {
    p_comm = comm;
    p_system = system;
    p_config = config;
}

/**
 * @brief      根据加速度量程设置转换系数
 * @param      range  加速度量程
 */
static void set_acc_transition_factor(icm42688_acc_sample_t range) {
    switch(range) {
        case ICM42688_ACC_SAMPLE_SGN_2G:
            icm42688_transition_factor[0] = 2.0f / 32768.0f;
            break;
        case ICM42688_ACC_SAMPLE_SGN_4G:
            icm42688_transition_factor[0] = 4.0f / 32768.0f;
            break;
        case ICM42688_ACC_SAMPLE_SGN_8G:
            icm42688_transition_factor[0] = 8.0f / 32768.0f;
            break;
        case ICM42688_ACC_SAMPLE_SGN_16G:
            icm42688_transition_factor[0] = 16.0f / 32768.0f;
            break;
        default:
            icm42688_transition_factor[0] = 8.0f / 32768.0f;
            break;
    }
}

/**
 * @brief      根据陀螺仪量程设置转换系数
 * @param      range  陀螺仪量程
 */
static void set_gyro_transition_factor(icm42688_gyro_sample_t range) {
    switch(range) {
        case ICM42688_GYRO_SAMPLE_SGN_15_125DPS:
            icm42688_transition_factor[1] = 15.125f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_31_25DPS:
            icm42688_transition_factor[1] = 31.25f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_62_5DPS:
            icm42688_transition_factor[1] = 62.5f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_125DPS:
            icm42688_transition_factor[1] = 125.0f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_250DPS:
            icm42688_transition_factor[1] = 250.0f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_500DPS:
            icm42688_transition_factor[1] = 500.0f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_1000DPS:
            icm42688_transition_factor[1] = 1000.0f / 32768.0f;
            break;
        case ICM42688_GYRO_SAMPLE_SGN_2000DPS:
            icm42688_transition_factor[1] = 2000.0f / 32768.0f;
            break;
        default:
            icm42688_transition_factor[1] = 2000.0f / 32768.0f;
            break;
    }
}

/**
 * @brief      配置采样速率
 * @param      rate  采样速率
 */
static void set_sample_rate(icm42688_sample_rate_t rate) {
    uint8_t odr_conf = 0x00;
    
    switch(rate) {
        case ICM42688_SAMPLE_RATE_8000:
            odr_conf = 0x03;
            break;
        case ICM42688_SAMPLE_RATE_4000:
            odr_conf = 0x04;
            break;
        case ICM42688_SAMPLE_RATE_2000:
            odr_conf = 0x05;
            break;
        case ICM42688_SAMPLE_RATE_1000:
            odr_conf = 0x06;
            break;
        case ICM42688_SAMPLE_RATE_500:
            odr_conf = 0x0F;
            break;
        case ICM42688_SAMPLE_RATE_200:
            odr_conf = 0x07;
            break;
        case ICM42688_SAMPLE_RATE_100:
            odr_conf = 0x08;
            break;
        case ICM42688_SAMPLE_RATE_50:
            odr_conf = 0x09;
            break;
        case ICM42688_SAMPLE_RATE_25:
            odr_conf = 0x0A;
            break;
        case ICM42688_SAMPLE_RATE_12_5:
            odr_conf = 0x0B;
            break;
        default:
            odr_conf = 0x06; // 默认1000Hz
            break;
    }
    
    // 设置采样率
    p_comm->write_reg(ICM42688_ACCEL_CONFIG0, odr_conf | (p_config->acc_sample << 5));
    p_comm->write_reg(ICM42688_GYRO_CONFIG0, odr_conf | (p_config->gyro_sample << 5));
}

/**
 * @brief      初始化ICM42688
 * @return     初始化结果，0表示成功，非0表示失败
 */
uint8_t icm42688_init(void) {
    uint8_t id = 0;
    
    // 检查接口是否初始化
    if(p_comm == NULL || p_system == NULL || p_config == NULL) {
        return 1;
    }
    
    // 初始化通信接口
    p_comm->init();
    
    // 延时等待传感器上电稳定
    p_system->delay_ms(10);
    
    // 读取设备ID
    id = p_comm->read_reg(ICM42688_WHO_AM_I);
    if(id != ICM42688_ID) {
        return 2;  // ID不匹配
    }
    
    // 复位并等待
    p_comm->write_reg(ICM42688_DEVICE_CONFIG, 0x01);  // 软复位
    p_system->delay_ms(50);
    
    // 唤醒传感器
    p_comm->write_reg(ICM42688_PWR_MGMT0, 0x0F);  // 使能加速度计和陀螺仪
    p_system->delay_ms(1);
    
    // 设置量程和转换系数
    set_acc_transition_factor(p_config->acc_sample);
    set_gyro_transition_factor(p_config->gyro_sample);
    
    // 设置采样速率
    set_sample_rate(p_config->sample_rate);
    
    // 设置低通滤波器
    p_comm->write_reg(ICM42688_GYRO_CONFIG1, 0x01);   // 低通滤波器带宽配置
    p_comm->write_reg(ICM42688_ACCEL_CONFIG1, 0x01);
    
    // 等待传感器稳定
    p_system->delay_ms(5);
    
    return 0;  // 初始化成功
}

/**
 * @brief      获取加速度计数据
 */
void icm42688_get_acc(void) {
    uint8_t buffer[6];
    
    // 检查接口是否初始化
    if(p_comm == NULL) {
        return;
    }
    
    // 读取加速度数据
    p_comm->read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);
    
    // 合成16位有符号数
    icm42688_acc.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    icm42688_acc.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    icm42688_acc.z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

/**
 * @brief      获取陀螺仪数据
 */
void icm42688_get_gyro(void) {
    uint8_t buffer[6];
    
    // 检查接口是否初始化
    if(p_comm == NULL) {
        return;
    }
    
    // 读取陀螺仪数据
    p_comm->read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);
    
    // 合成16位有符号数
    icm42688_gyro.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    icm42688_gyro.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    icm42688_gyro.z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

/**
 * @brief      将加速度计数据转换为实际物理值
 * @param      acc_value  加速度计原始值
 * @return     加速度物理值，单位g
 */
float icm42688_acc_transition(int16_t acc_value) {
    return (float)acc_value * icm42688_transition_factor[0];
}

/**
 * @brief      将陀螺仪数据转换为实际物理值
 * @param      gyro_value  陀螺仪原始值
 * @return     角速度物理值，单位dps
 */
float icm42688_gyro_transition(int16_t gyro_value) {
    return (float)gyro_value * icm42688_transition_factor[1];
} 