/**
 * @file      ahrs_hal.h
 * @brief     姿态解算算法硬件抽象层
 * @author    AzureHalcyon
 * @version   1.0.0
 * @date      2025-03-28
 * @note      ICM42688姿态解算头文件,提供硬件抽象接口
 */

#ifndef _AHRS_HAL_H_
#define _AHRS_HAL_H_

#include <stdint.h>
#include <math.h>
#include <stddef.h>

#define AHRS_PI (float)3.1415926535

/**
 * @brief  姿态数据结构体
 */
typedef struct
{
    float x; /**< x轴数据 */
    float y; /**< y轴数据 */
    float z; /**< z轴数据 */
} ahrs_vector_float_t;

/**
 * @brief  欧拉角数据结构体
 */
typedef struct
{
    float yaw;   /**< 偏航角 (z轴旋转) */
    float pitch; /**< 俯仰角 (y轴旋转) */
    float roll;  /**< 横滚角 (x轴旋转) */
} ahrs_euler_angle_t;

extern ahrs_vector_float_t north, west; /**< 罗盘方向参考向量 */
extern ahrs_euler_angle_t attitude;     /**< 姿态角度 */

/**
 * @brief  定时器接口结构体
 */
typedef struct
{
    /**
     * @brief      获取当前时间(微秒)
     * @return     当前时间值
     */
    uint32_t (*get_time_us)(void);

    /**
     * @brief      初始化定时器
     */
    void (*init)(void);

    /**
     * @brief      启动定时器
     */
    void (*start)(void);
} ahrs_timer_t;

/**
 * @brief  系统接口结构体
 */
typedef struct
{
    /**
     * @brief      延时函数(毫秒)
     * @param      ms  延时毫秒数
     */
    void (*delay_ms)(uint16_t ms);
} ahrs_system_t;

void ahrs_hal_init(ahrs_timer_t *timer, ahrs_system_t *system);
void ahrs_init(void);
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void ahrs_get_attitude(ahrs_euler_angle_t *euler_angle);
void ahrs_init_attitude_offset(void);
void ahrs_reset_parameters(void);

#endif // _AHRS_HAL_H_