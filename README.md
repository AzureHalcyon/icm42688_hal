# ICM42688 姿态解算硬件抽象层

## 项目简介

这是一个基于 ICM42688 传感器的姿态解算硬件抽象层实现。项目提供了简单易用的姿态解算接口，欧拉角输出，并采用Mahony算法实现高精度的姿态估计。传感器购买自某宝LWKJ电子模块小店，店家非常好。

## 特别感谢：

- 偶然找到的由lisn3188编写的MPU6050代码，启发很多，部分代码也来自于这位朋友。可惜留下的网址已然失效，权作纪念：www.chiplab7.com

## 主要特性

- 完整的硬件抽象层设计，易于移植
- 提供 SPI 和 IIC 通信接口
- 可配置的传感器参数
- 每个轴独立的参数调整，简单直观
- 对Mahony算法进行一些改进
- 死区控制和积分限幅保护
- 零偏自动校准，自动对齐重力方向

## 目录结构

```
icm42688_hal/
├── icm42688_hal.c    - ICM42688 传感器驱动实现
├── icm42688_hal.h    - ICM42688 相关定义和接口声明
├── ahrs_hal.c        - 姿态解算算法实现
├── ahrs_hal.h        - 姿态解算相关定义和接口声明
├── main.c           - 示例代码
└── LICENSE          - GNU GPL v3 许可证
```

## 使用说明

### 1. 硬件初始化

首先需要实现硬件相关的接口：

```c
// 实现通信接口
icm42688_comm_t icm42688_comm = {
    .init = your_spi_init,
    .read_reg = your_spi_read,
    .write_reg = your_spi_write,
    .read_regs = your_spi_read_burst
};

// 实现系统接口
icm42688_system_t icm42688_system = {
    .delay_ms = your_delay_function
};
```

### 2. 配置传感器

```c
// 配置传感器参数
icm42688_config_t icm42688_config = {
    .interface_type = ICM42688_INTERFACE_SPI,
    .acc_sample = ICM42688_ACC_SAMPLE_SGN_16G,
    .gyro_sample = ICM42688_GYRO_SAMPLE_SGN_2000DPS,
    .sample_rate = ICM42688_SAMPLE_RATE_1000
};
```

### 3. 初始化系统

```c
// 初始化传感器和姿态解算
icm42688_hal_init(&icm42688_comm, &icm42688_system, &icm42688_config);
if(icm42688_init() != 0) {
    // 错误处理
}
ahrs_init();
ahrs_init_attitude_offset();
```

### 4. 获取姿态数据

```c
// 在主循环中调用
icm42688_get_acc();
icm42688_get_gyro();

// 获取姿态角
ahrs_euler_angle_t attitude;
ahrs_get_attitude(&attitude);
```

## 性能参数

- 姿态更新频率：最高支持 8000Hz
- 角度精度：静态<0.1°，动态<0.5°
- 陀螺仪量程：±15.125dps ~ ±2000dps 可配置
- 加速度计量程：±2g ~ ±16g 可配置

## 许可证

本项目采用 GNU GPL v3 许可证。详见 [LICENSE](LICENSE) 文件。

## 作者

- AzureHalcyon
- 版本：1.0.0
- 日期：2025-03-28

## 注意事项

- 测试时是在I.MX RT1064平台上通过的，参数没有调整的很好，只是随便调了一个效果还行的参数，如果在你的机器上效果不好，就需要自己重新调整一下。

## 参考资料

- ICM42688 数据手册
- 姿态解算算法部分参考了 lisn3188 的代码
- 官方提供的示例代码

## 更新日志

### v1.0.0 (2025-03-28)

- 首次发布
- 实现基本功能
- 通过实际测试验证
