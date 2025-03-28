/**
 * @file      ahrs_hal.c
 * @brief     姿态解算算法硬件抽象层实现
 * @author    AzureHalcyon
 * @version   1.0.0
 * @date      2025-03-28
 * @note      ICM42688姿态解算，部分代码参考了lisn3188的代码,以及官方提供的示例。
 */

#include "ahrs_hal.h"

// 为不同轴设置不同的PI参数
#define KP_ROLL     3.2f    /**< Roll轴比例增益 */
#define KP_PITCH    2.95f   /**< Pitch轴比例增益 */
#define KP_YAW      1.0f    /**< Yaw轴比例增益（通常要小一些）*/

#define KI_ROLL     0.095f  /**< Roll轴积分增益 */
#define KI_PITCH    0.085f  /**< Pitch轴积分增益 */
#define KI_YAW      0.065f  /**< Yaw轴积分增益（通常要小一些）*/

// 死区参数
#define GYRO_DEADBAND     0.01f   /**< 陀螺仪死区（弧度/秒）*/
#define ERROR_DEADBAND    0.0050f /**< 误差死区 */

// 积分限幅，防止积分饱和
#define INT_LIMIT         0.1f    /**< 积分项限幅 */

// 四元数和积分误差
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

// 姿态角数据
ahrs_vector_float_t north, west;  /**< 罗盘方向参考向量 */
ahrs_euler_angle_t attitude;      /**< 姿态角度 */

// 姿态角度补偿
static float pitch_offset = 0.0f;
static float roll_offset = 0.0f;

// 定时器和系统接口
static ahrs_timer_t *p_timer = NULL; //NULL的作用是防止未初始化时使用
static ahrs_system_t *p_system = NULL;

// 时间相关变量
static uint32_t lastTime = 0;

/**
 * @brief      快速平方根倒数
 * @param      x    输入值
 * @return     平方根倒数
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief      限制误差积分项
 * @param      value  待限制的积分值
 * @param      limit  限制值
 * @return     限制后的积分值
 */
static float limitIntegral(float value, float limit) {
    if(value > limit) return limit;
    if(value < -limit) return -limit;
    return value;
}

/**
 * @brief      应用死区
 * @param      value     输入值
 * @param      deadband  死区范围
 * @return     应用死区后的值
 */
static float applyDeadband(float value, float deadband) {
    if(fabs(value) < deadband) {
        return 0.0f;
    }
    return value;
}

/**
 * @brief      初始化姿态解算HAL层
 * @param      timer   定时器接口
 * @param      system  系统接口
 */
void ahrs_hal_init(ahrs_timer_t *timer, ahrs_system_t *system) {
    p_timer = timer;
    p_system = system;
}

/**
 * @brief      姿态解算算法（改进版）
 * @param      gx        x轴角速度(弧度制)
 * @param      gy        y轴角速度(弧度制)
 * @param      gz        z轴角速度(弧度制)
 * @param      ax        x轴加速度
 * @param      ay        y轴加速度
 * @param      az        z轴加速度
 * @param      mx        x轴磁力计数据(如果不使用磁力计则传入0)
 * @param      my        y轴磁力计数据(如果不使用磁力计则传入0)
 * @param      mz        z轴磁力计数据(如果不使用磁力计则传入0)
 */
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;
    uint32_t currentTime;
    
    // 检查硬件接口是否初始化
    if(p_timer == NULL) return;
    
    // 应用陀螺仪死区
    gx = applyDeadband(gx, GYRO_DEADBAND);
    gy = applyDeadband(gy, GYRO_DEADBAND);
    gz = applyDeadband(gz, GYRO_DEADBAND);
    
    // 计算中间值
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    
    // 判断加速度是否有效
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;
    
    // 获取时间间隔
    currentTime = p_timer->get_time_us();
    if(lastTime == 0) {
        lastTime = currentTime;
        return;
    }
    
    // 计算时间差
    if(currentTime < lastTime) {
        halfT = ((float)(currentTime + (0xFFFFFFFF - lastTime)) / 2000000.0f);
    } else {
        halfT = ((float)(currentTime - lastTime) / 2000000.0f);
    }
    lastTime = currentTime;
    
    // 加速度计归一化
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    // 磁力计归一化(如果使用)
    if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        norm = invSqrt(mx*mx + my*my + mz*mz);
        mx = mx * norm;
        my = my * norm;
        mz = mz * norm;
    }
    
    // 根据当前四元数计算出重力方向
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    // 计算参考方向
    north.x = 1 - 2*(q3q3 + q2q2);
    north.y = 2*(-q0*q3 + q1*q2);
    north.z = 2*(q0*q2 - q1*q3);
    
    west.x = 2*(q0*q3 + q1*q2);
    west.y = 1 - 2*(q3q3 + q1q1);
    west.z = 2*(-q0*q1 + q2*q3);
    
    // 计算误差，估计重力方向和测量重力方向的叉积
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // 应用误差死区
    ex = applyDeadband(ex, ERROR_DEADBAND);
    ey = applyDeadband(ey, ERROR_DEADBAND);
    ez = applyDeadband(ez, ERROR_DEADBAND);
    
    // PI控制器，分别处理三个轴
    if(ex != 0.0f) {
        exInt = limitIntegral(exInt + ex * KI_ROLL * halfT, INT_LIMIT);
        gx = gx + KP_ROLL * ex + exInt;
    }
    
    if(ey != 0.0f) {
        eyInt = limitIntegral(eyInt + ey * KI_PITCH * halfT, INT_LIMIT);
        gy = gy + KP_PITCH * ey + eyInt;
    }
    
    if(ez != 0.0f) {
        ezInt = limitIntegral(ezInt + ez * KI_YAW * halfT, INT_LIMIT);
        gz = gz + KP_YAW * ez + ezInt;
    }
    
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    
    // 四元数归一化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
    
    // 计算欧拉角
    attitude.yaw   = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1) * 180/AHRS_PI;
    attitude.pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 180/AHRS_PI - pitch_offset;
    attitude.roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180/AHRS_PI - roll_offset;
    
    // 调整yaw角范围
    if(attitude.yaw < 0) attitude.yaw += 360.0f;
}

/**
 * @brief      姿态解算算法参数重置
 */
void ahrs_reset_parameters(void) {
    // 重置四元数
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    
    // 清零误差积分
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
}

/**
 * @brief      AHRS初始化
 */
void ahrs_init(void) {
    // 检查硬件接口是否初始化
    if(p_timer == NULL) return;
    
    // 初始化定时器
    p_timer->init();
    p_timer->start();
    
    // 重置参数
    ahrs_reset_parameters();
    
    // 初始化参考向量
    north.x = 0.0f;
    north.y = 0.0f;
    north.z = 1.0f;
    
    west.x = 1.0f;
    west.y = 0.0f;
    west.z = 0.0f;
    
    // 重置时间戳
    lastTime = 0;
}

/**
 * @brief      获取姿态角度
 * @param      euler_angle  欧拉角结构体指针
 */
void ahrs_get_attitude(ahrs_euler_angle_t *euler_angle) {
    if(euler_angle == NULL)
        return;
        
    euler_angle->roll = attitude.roll;
    euler_angle->pitch = attitude.pitch;
    euler_angle->yaw = attitude.yaw;
}

/**
 * @brief      初始化姿态补偿参数
 */
void ahrs_init_attitude_offset(void) {
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    const uint16_t samples = 100;
    
    // 检查硬件接口是否初始化
    if(p_system == NULL) return;
    
    // 先稳定一段时间
    for(uint16_t i = 0; i < 100; i++) {
        p_system->delay_ms(5);
    }
    
    // 重置算法参数，确保校准从干净状态开始
    ahrs_reset_parameters();
    
    // 读取多次数据求平均
    for(uint16_t i = 0; i < samples; i++) {
        p_system->delay_ms(5);
        roll_sum += attitude.roll;
        pitch_sum += attitude.pitch;
    }
    
    // 计算偏移量
    roll_offset = roll_sum / samples;
    pitch_offset = pitch_sum / samples;
    
    // 重新计算当前姿态
    attitude.roll -= roll_offset;
    attitude.pitch -= pitch_offset;
} 