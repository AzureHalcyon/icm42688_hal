#include "icm42688_hal.h"
#include "ahrs_hal.h"

int main(void) {
    if(icm42688_rt1064_init() != 0) {
        // 初始化失败处理
        while(1);
    }
    ahrs_init();
    ahrs_init_attitude_offset();
    
    while(1) {
        icm42688_get_acc();
        icm42688_get_gyro();
        
        float gx = icm42688_gyro_transition(icm42688_gyro.x) * AHRS_PI / 180.0f; // 转为弧度制
        float gy = icm42688_gyro_transition(icm42688_gyro.y) * AHRS_PI / 180.0f;
        float gz = icm42688_gyro_transition(icm42688_gyro.z) * AHRS_PI / 180.0f;
        
        float ax = icm42688_acc_transition(icm42688_acc.x);
        float ay = icm42688_acc_transition(icm42688_acc.y);
        float az = icm42688_acc_transition(icm42688_acc.z);
        
        ahrs_update(gx, gy, gz, ax, ay, az, 0, 0, 0);
        ahrs_euler_angle_t current_attitude;
        ahrs_get_attitude(&current_attitude);
        
        printf("%.2f,%.2f,%.2f\r\n", current_attitude.roll, current_attitude.pitch, current_attitude.yaw);
    }
    
    return 0;
} 