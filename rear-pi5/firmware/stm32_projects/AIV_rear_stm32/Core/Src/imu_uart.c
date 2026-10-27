#include "imu_uart.h"

IMU_Data g_imu;
volatile uint32_t g_byte_count = 0;   // 디버그용: 지금까지 받은 총 바이트 수

static UART_HandleTypeDef *s_huart;
static uint8_t s_rxByte;
static uint8_t s_frame[11];
static uint8_t s_frameIdx = 0;

// 인터럽트 기반 1바이트씩 수신 시작/재시작
void IMU_UART_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_frameIdx = 0;
    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1);
}

static void parse_frame(uint8_t *f)
{
    // 체크섬 검증: 앞 10바이트 합의 하위 1바이트 == 마지막 바이트
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum += f[i];
    if (sum != f[10]) return; // 체크섬 안 맞으면 버림

    int16_t d0 = (int16_t)((f[3] << 8) | f[2]);
    int16_t d1 = (int16_t)((f[5] << 8) | f[4]);
    int16_t d2 = (int16_t)((f[7] << 8) | f[6]);
    int16_t t  = (int16_t)((f[9] << 8) | f[8]);

    switch (f[1]) {
        case 0x51: // 가속도
            g_imu.accel_x = d0 / 32768.0f * 16.0f;
            g_imu.accel_y = d1 / 32768.0f * 16.0f;
            g_imu.accel_z = d2 / 32768.0f * 16.0f;
            g_imu.temp_c  = t / 340.0f + 36.53f;
            g_imu.accel_updated = 1;
            break;
        case 0x52: // 각속도
            g_imu.gyro_x = d0 / 32768.0f * 2000.0f;
            g_imu.gyro_y = d1 / 32768.0f * 2000.0f;
            g_imu.gyro_z = d2 / 32768.0f * 2000.0f;
            g_imu.gyro_updated = 1;
            break;
        case 0x53: // 자세각 (Roll/Pitch/Yaw)
            g_imu.roll  = d0 / 32768.0f * 180.0f;
            g_imu.pitch = d1 / 32768.0f * 180.0f;
            g_imu.yaw   = d2 / 32768.0f * 180.0f;
            g_imu.angle_updated = 1;
            break;
        default:
            break; // 다른 타입 패킷은 무시
    }
}

// main.c 의 HAL_UART_RxCpltCallback 안에서:
// if (huart->Instance == USART6) IMU_UART_ByteReceived(huart);
// 형태로 호출해주면 됨
void IMU_UART_ByteReceived(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) return;

    g_byte_count++;   // 디버그용: 바이트 하나 들어올 때마다 증가

    if (s_frameIdx == 0) {
        // 프레임 시작 바이트(0x55)를 찾을 때까지 대기
        if (s_rxByte == 0x55) {
            s_frame[0] = s_rxByte;
            s_frameIdx = 1;
        }
    } else {
        s_frame[s_frameIdx++] = s_rxByte;
        if (s_frameIdx >= 11) {
            parse_frame(s_frame);
            s_frameIdx = 0;
        }
    }

    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1); // 다음 1바이트 계속 수신
}
