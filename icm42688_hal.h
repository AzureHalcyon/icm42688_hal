/**
 * @file      icm42688_hal.h
 * @brief     ICM42688传感器硬件抽象层
 * @author    AzureHalcyon
 * @version   1.0.0
 * @date      2025-03-28
 * @note      ICM42688传感器头文件,提供ICM42688传感器的硬件抽象接口
 */

#ifndef _ICM42688_HAL_H_
#define _ICM42688_HAL_H_

#include <stdint.h>
#include <stddef.h>

// ICM42688内部寄存器地址
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

#define ICM42688_ID                        0x47  /**< ICM42688 ID */

/**
 * @brief  通信接口类型枚举
 */
typedef enum {
    ICM42688_INTERFACE_SPI,    /**< SPI通信接口 */
    ICM42688_INTERFACE_IIC     /**< IIC通信接口 */
} icm42688_interface_type_t;

/**
 * @brief  加速度计量程枚举
 */
typedef enum {
    ICM42688_ACC_SAMPLE_SGN_2G,   /**< 加速度计量程 ±2G */
    ICM42688_ACC_SAMPLE_SGN_4G,   /**< 加速度计量程 ±4G */
    ICM42688_ACC_SAMPLE_SGN_8G,   /**< 加速度计量程 ±8G */
    ICM42688_ACC_SAMPLE_SGN_16G   /**< 加速度计量程 ±16G */
} icm42688_acc_sample_t;

/**
 * @brief  陀螺仪量程枚举，量程越大，精度越低
 */
typedef enum {
    ICM42688_GYRO_SAMPLE_SGN_15_125DPS,  /**< 陀螺仪量程 ±15.125dps */
    ICM42688_GYRO_SAMPLE_SGN_31_25DPS,   /**< 陀螺仪量程 ±31.25dps */
    ICM42688_GYRO_SAMPLE_SGN_62_5DPS,    /**< 陀螺仪量程 ±62.5dps */
    ICM42688_GYRO_SAMPLE_SGN_125DPS,     /**< 陀螺仪量程 ±125dps */
    ICM42688_GYRO_SAMPLE_SGN_250DPS,     /**< 陀螺仪量程 ±250dps */
    ICM42688_GYRO_SAMPLE_SGN_500DPS,     /**< 陀螺仪量程 ±500dps */
    ICM42688_GYRO_SAMPLE_SGN_1000DPS,    /**< 陀螺仪量程 ±1000dps */
    ICM42688_GYRO_SAMPLE_SGN_2000DPS     /**< 陀螺仪量程 ±2000dps */
} icm42688_gyro_sample_t;

/**
 * @brief  采样速率枚举
 */
typedef enum {
    ICM42688_SAMPLE_RATE_8000,   /**< 采样速率 8000Hz */
    ICM42688_SAMPLE_RATE_4000,   /**< 采样速率 4000Hz */
    ICM42688_SAMPLE_RATE_2000,   /**< 采样速率 2000Hz */
    ICM42688_SAMPLE_RATE_1000,   /**< 采样速率 1000Hz */
    ICM42688_SAMPLE_RATE_200,    /**< 采样速率 200Hz */
    ICM42688_SAMPLE_RATE_100,    /**< 采样速率 100Hz */
    ICM42688_SAMPLE_RATE_50,     /**< 采样速率 50Hz */
    ICM42688_SAMPLE_RATE_25,     /**< 采样速率 25Hz */
    ICM42688_SAMPLE_RATE_12_5,   /**< 采样速率 12.5Hz */
    ICM42688_SAMPLE_RATE_500,    /**< 采样速率 500Hz */
} icm42688_sample_rate_t;

/**
 * @brief  三轴数据结构体
 */
typedef struct {
    int16_t x;    /**< X轴数据 */
    int16_t y;    /**< Y轴数据 */
    int16_t z;    /**< Z轴数据 */
} icm42688_axis_data_t;

/**
 * @brief  ICM42688配置结构体
 */
typedef struct {
    icm42688_interface_type_t interface_type;  /**< 通信接口类型 */
    icm42688_acc_sample_t acc_sample;          /**< 加速度计量程 */
    icm42688_gyro_sample_t gyro_sample;        /**< 陀螺仪量程 */
    icm42688_sample_rate_t sample_rate;        /**< 采样速率 */
} icm42688_config_t;

/**
 * @brief  ICM42688通信接口抽象结构体
 */
typedef struct {
    /**
     * @brief      初始化通信接口
     */
    void (*init)(void);
    
    /**
     * @brief      读取寄存器
     * @param      reg  寄存器地址
     * @return     寄存器值
     */
    uint8_t (*read_reg)(uint8_t reg);
    
    /**
     * @brief      写入寄存器
     * @param      reg    寄存器地址
     * @param      value  写入的值
     */
    void (*write_reg)(uint8_t reg, uint8_t value);
    
    /**
     * @brief      连续读取多个寄存器
     * @param      reg    起始寄存器地址
     * @param      data   数据缓冲区指针
     * @param      len    读取长度
     */
    void (*read_regs)(uint8_t reg, uint8_t *data, uint8_t len);
} icm42688_comm_t;

/**
 * @brief  ICM42688系统接口抽象结构体
 */
typedef struct {
    /**
     * @brief      延时函数(毫秒)
     * @param      ms  延时毫秒数
     */
    void (*delay_ms)(uint16_t ms);
} icm42688_system_t;

// 外部变量
extern icm42688_axis_data_t icm42688_gyro;     /**< 三轴陀螺仪数据 */
extern icm42688_axis_data_t icm42688_acc;      /**< 三轴加速度计数据 */
extern float icm42688_transition_factor[2];    /**< 转换系数 */

void icm42688_hal_init(icm42688_comm_t *comm, icm42688_system_t *system, icm42688_config_t *config);
uint8_t icm42688_init(void);
void icm42688_get_acc(void);
void icm42688_get_gyro(void);
float icm42688_acc_transition(int16_t acc_value);
float icm42688_gyro_transition(int16_t gyro_value);

#endif // _ICM42688_HAL_H_ 