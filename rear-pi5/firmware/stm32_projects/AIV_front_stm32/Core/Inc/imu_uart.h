#ifndef IMU_UART_H
#define IMU_UART_H

#include "stm32f4xx_hal.h"

typedef struct {
    float accel_x, accel_y, accel_z;  // g 단위
    float gyro_x, gyro_y, gyro_z;     // deg/s 단위
    float roll, pitch, yaw;           // deg 단위
    float temp_c;
    uint8_t accel_updated, gyro_updated, angle_updated; // 새 값 들어왔는지 플래그
} IMU_Data;

// huartX = 모듈 TX가 연결된 USART 핸들 (예: &huart6)
void IMU_UART_Init(UART_HandleTypeDef *huart);

// HAL_UART_RxCpltCallback 안에서 해당 huart일 때 호출
void IMU_UART_ByteReceived(UART_HandleTypeDef *huart);

extern IMU_Data g_imu;
extern volatile uint32_t g_byte_count;   // 디버그용: 지금까지 받은 총 바이트 수

#endif
