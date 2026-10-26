/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imu_uart.h"   /* front 보드에서 가져온 IMU(자세각) 파서 */
#include "ssd1306.h"    /* OLED (I2C2, 주소 0x3C) */
#include "ssd1306_fonts.h"

#define MCP_CS_PORT   GPIOA
#define MCP_CS_PIN    GPIO_PIN_8

#define MCP_RESET     0xC0
#define MCP_READ      0x03
#define MCP_WRITE     0x02
#define MCP_BITMOD    0x05
#define MCP_READ_RX0  0x90

#define REG_CANCTRL   0x0F
#define REG_CNF1      0x2A
#define REG_CNF2      0x29
#define REG_CNF3      0x28
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_RXB0CTRL  0x60
#define REG_RXB1CTRL  0x70

/* CAN ID 필터/마스크 레지스터 (MCP2515 데이터시트 기준)
   RXF0/1 -> RXB0용 필터, RXF2 -> RXB1용 필터, RXM0/1 -> 각 버퍼의 마스크 */
#define REG_RXF0SIDH  0x00
#define REG_RXF0SIDL  0x01
#define REG_RXF1SIDH  0x04
#define REG_RXF1SIDL  0x05
#define REG_RXF2SIDH  0x08
#define REG_RXF2SIDL  0x09
#define REG_RXM0SIDH  0x20
#define REG_RXM0SIDL  0x21
#define REG_RXM1SIDH  0x24
#define REG_RXM1SIDL  0x25

/* 우리가 실제로 쓰는 물체 데이터 프레임의 CAN ID (MR76) */
#define MR76_OBJECT_ID  0x60B
/* [임시 테스트] 1이면 차량이 아닌 물체(사람/벽/기둥)도 출력·전송. 실차 테스트 전에 반드시 0으로! */
#define TEST_ALL_CLASSES 1

/* [v16] MCP2515 읽기 방식: 1 = 한 번에 묶어서 읽는 빠른 방식, 0 = 예전(v15) 방식 (비교 측정용) */
#ifndef FAST_MCP_READ
#define FAST_MCP_READ 1
#endif

/* [v17] 레이더 프레임을 한꺼번에 먼저 읽어 모은 뒤(버퍼가 넘치지 않게) 처리하는 방식
   RAD_BATCH: 한 번에 모으는 최대 프레임 수, RAD_SPIN_US: 마지막 프레임 뒤 이만큼(us) 더 기다려 이어지는 프레임을 받음
   OBJ_PRINT_MAX: 디버그 'OBJ ...' 줄을 1초에 최대 몇 줄 찍을지 (Pi로 보내는 데이터에는 영향 없음) */
#define RAD_BATCH      24
#define RAD_SPIN_US    400
#define OBJ_PRINT_MAX  40
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;   /* 새로 추가: ST-Link VCP(PA2/PA3)로 나가는 디버그 출력용 */
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* 레이더(MCP2515) 인터럽트 수신 관련 */
volatile uint8_t  radar_int_flag = 0;   /* PC8(INT) 폴링 대신 이 플래그가 서면 그때만 읽음 */
uint32_t          last_radar_rx_ms = 0; /* 마지막으로 정상 수신한 시각 -> 타임아웃/Safe Mode 판단용 */
uint8_t           radar_safe_mode = 0;  /* 100ms 넘게 응답 없으면 1로 전환 */

/* [v15 진단] RADSTAT 로그용 카운터 (동작 판단에는 쓰지 않고 로그만 찍는다) */
uint32_t rad_na = 0;        /* 1초 동안 받은 0x60A(상태) 프레임 수 */
uint32_t rad_nb = 0;        /* 1초 동안 받은 0x60B(물체) 프레임 수 */
uint32_t rad_nveh = 0;      /* 그중 objClass==1(차량)인 수 */
uint32_t rad_npub = 0;      /* RCS 필터까지 통과해서 Pi로 보낸 수 */
uint32_t rad_nother = 0;    /* 0x60A/0x60B가 아닌 ID 수 */
uint32_t rad_gap_max = 0;   /* 1초 동안 프레임 사이 최대 간격(ms) */
uint32_t rad_safe_cnt = 0;  /* Safe Mode 진입 누적 횟수 */
uint32_t rad_ovf = 0;       /* MCP2515 수신 버퍼 넘침 누적 횟수 */
uint8_t  rad_seen = 0;      /* 첫 프레임을 받은 적이 있는가 */
uint32_t rad_exp = 0;       /* [v16] 0x60A가 알려준 "이번 사이클 물체 수"의 1초 합 (받아야 할 0x60B 수) */
uint32_t rd_cyc_sum = 0;    /* [v16] mcp2515_check_and_read가 프레임 하나 읽는 데 쓴 CPU 클럭 합 */
uint32_t rd_cyc_max = 0;
uint32_t rd_n = 0;
typedef struct { uint32_t id; uint8_t dlc; uint8_t data[8]; } rad_frame_t;
rad_frame_t rad_fb[RAD_BATCH];   /* [v17] 모아 둔 프레임 */
uint32_t rad_pr_cnt = 0;         /* [v17] 이번 1초에 찍은 OBJ 줄 수 */
extern volatile uint32_t dbg_drop_count;   /* 디버그 출력이 꽉 차서 버려진 횟수 (아래에서 정의) */

/* IMU 값 출력 주기(200ms)용 */
uint32_t last_imu_print_ms = 0;

/* 정지물체 필터용: OBD가 마지막으로 읽은 자차 속도 */
uint8_t  last_ego_speed_kmh = 0;
uint32_t last_ego_speed_ms  = 0;   /* 이 값을 읽은 시각 -> 오래되면(폴링 지연) 필터에 안 씀 */
#define EGO_SPEED_STALE_MS 500     /* OBD 폴링 300ms 주기라 500ms 넘으면 신뢰 안 함 */
uint32_t last_obd_poll = 0;
uint8_t  obd_fail_count = 0;   /* OBD 연속 응답 실패 횟수 (3회 이상이면 느리게 폴링) */
uint8_t  mcp_dbg[6] = {0};    /* 진단용: [0]=리셋직후 CANSTAT, [1]=CNF2 읽은값, [2]=CNF3 읽은값, [3]=Normal전환후 CANSTAT */
uint8_t  mcp_ok = 0;
uint8_t  reset_cause = 2;              /* 0=전원 켬, 1=워치독 리셋, 2=그 외 (HEARTBEAT용) */
/* OLED 표시용 */
uint8_t  oled_ok = 0;                 /* I2C 스캔에서 0x3C가 잡히면 1 */
/* 워치독(IWDG): 메인 루프가 40ms 넘게 멈추면 STM32를 강제 리셋 (팀 규칙 6: 40ms 기준 임의 변경 금지) */
#define ENABLE_IWDG       1
#define WDG_TEST_HANG_MS  0     /* 0=끔. 예: 15000 이면 부팅 15초 뒤 일부러 멈춰서 워치독 리셋이 걸리는지 시험 */
uint32_t last_oled_ms = 0;
static uint8_t oled_page = 0xFF;      /* 화면 전송 진행 상황: 0~(페이지수-1)=전송 중, 그 이상=대기 */
#define OLED_PAGES (SSD1306_HEIGHT / 8)
int32_t  disp_dist10 = 0, disp_vrel10 = 0;   /* 마지막 레이더 물체(거리 m*10, 상대속도 m/s*10) */
uint32_t disp_obj_ms = 0;                    /* 마지막 물체 수신 시각 (0이면 아직 없음) */           /* 1이면 MCP2515 칩과 SPI 통신 확인됨 */

/* 정지물체 필터용: IMU로 "자차가 지금 완전히 정지해 있는가" 판단
   (가속도 적분으로 속도를 구하는 건 오차가 금방 커져서 안 함 -> "정지냐 아니냐"만 판단) */
#define IMU_STATIONARY_HOLD_MS    150   /* 이 시간 동안 계속 조용해야 "정지"로 인정 */
#define IMU_GYRO_THRESHOLD_DPS    3.0f  /* 이보다 회전이 크면 "움직임"으로 봄 */
#define IMU_ACCEL_DEV_THRESHOLD_G 0.05f /* 부팅 시 기준값(baseline)에서 이만큼 벗어나면 "움직임" */
uint8_t  imu_baseline_set = 0;
float    imu_baseline_ax, imu_baseline_ay, imu_baseline_az;  /* 부팅 직후(정지 가정) 가속도 기준값 */
uint32_t imu_last_motion_ms = 0;   /* 마지막으로 "움직임 감지"였던 시각 */

/* ---- Pi5 -> STM32 수신(USART1 RX): 0x22 EGO_SPEED (GPS 속도) ---- */
uint8_t           rpi_rx_byte = 0;          /* 인터럽트로 1바이트씩 받는 버퍼 */
volatile uint16_t gps_speed_x100 = 0;       /* 0.01 km/h */
volatile uint8_t  gps_src = 0xFF;           /* 0=OBD 1=GPS 0xFF=없음 */
volatile uint16_t gps_age_ms = 0xFFFF;      /* Pi5가 알려준 "측정 후 경과시간" */
volatile uint32_t gps_rx_ms = 0;            /* STM32가 마지막으로 0x22를 받은 시각 (0이면 아직 없음) */
volatile uint32_t rpi_rx_bytes = 0, rpi_rx_ok = 0, rpi_rx_bad = 0;   /* 진단용 카운터 */
#define GPS_STALE_MS 1500                   /* 이보다 오래된 GPS 속도는 안 씀 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t mcp2515_init(void);
uint8_t mcp2515_check_and_read(uint8_t *data, uint8_t *dlc, uint32_t *id);
uint8_t radar_dbg_read(uint8_t addr);   /* [v15 진단] MCP2515 레지스터 읽기 */
void radar_dbg_clear_ovf(void);         /* [v17] MCP2515 수신 넘침 플래그 지우기 */
void obd_init(void);
void obd_poll_task(void);
void obd_rx_start(void);
void obd_rx_byte_isr(uint32_t now);
void oled_update(void);
void proto_task(void);
void dbg_tx(const uint8_t *p, uint16_t len, uint32_t unused);
void oled_push_step(void);
void ssd1306_UpdatePage(uint8_t page);   /* ssd1306.c에 추가한 함수 */
void proto_send_obd(void);
void radar_publish(uint8_t id, float dist_long_m, float dist_lat_m, float vrel_ms, float rcs_dbsm, uint8_t dyn);
void rpi_rx_byte_isr(void);
void gps_dbg_task(void);
uint8_t get_ego_speed(uint16_t *kmh10, uint8_t *src);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* [LED 진단] LD2(PA5) 초기화 후 켬 -> LED가 켜지면 여기(USER CODE 2)까지 도달한 것 */
  {
    GPIO_InitTypeDef led_gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    led_gpio.Pin = GPIO_PIN_5;
    led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    led_gpio.Pull = GPIO_NOPULL;
    led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &led_gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }

  /* 리셋 원인 기록(HEARTBEAT에 실어 보냄) 후 플래그 클리어 */
  if (RCC->CSR & RCC_CSR_IWDGRSTF)                         reset_cause = 1;
  else if (RCC->CSR & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) reset_cause = 0;
  else                                                     reset_cause = 2;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* 디버그 출력(USART2, ST-Link 가상 COM)도 인터럽트 큐 방식으로: 출력 때문에 메인 루프가 멈추지 않게 함 */
  HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* RPi 링크(USART1) 송신 인터럽트 활성화: 논블로킹 송신용 */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &rpi_rx_byte, 1);   /* Pi5가 보내는 0x22(GPS 속도) 수신 시작 */

  /* 리셋 원인 출력 (워치독 리셋이면 IWDG로 표시됨) */
  {
    char rc[40];
    int rl = sprintf(rc, "RESET CAUSE: %s\r\n", reset_cause == 1 ? "IWDG (watchdog)" : (reset_cause == 0 ? "power-on" : "other/pin"));
    dbg_tx((uint8_t*)rc, rl, 0);
  }

  /* [디버그] 부팅 확인용: 이 글자가 나오면 main()까지는 정상 도달 */
  { char boot1[] = "BOOT1\r\n"; dbg_tx((uint8_t*)boot1, strlen(boot1), 100); }

  /* 전원 안정화 대기 (디스플레이 부팅 시간 확보) */
  HAL_Delay(2000);

  /* [디버그] HAL_Delay(2000)까지 통과했다는 표시 */
  { char boot2[] = "BOOT2\r\n"; dbg_tx((uint8_t*)boot2, strlen(boot2), 100); }

  /* I2C2 버스 주소 스캔 - 디스플레이가 실제로 응답하는지, 주소가 뭔지 확인
     (huart1은 ST-Link VCP와 연결 안 된 핀(PA9/PA10)이라, 디버그 출력은 huart2로 바꿈) */
  {
    char scan_msg[64];
    int scan_len;
    uint8_t found_any = 0;

    char start_msg[] = "\r\n--- I2C2 Scan Start ---\r\n";
    dbg_tx((uint8_t*)start_msg, strlen(start_msg), 100);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 1, 10) == HAL_OK)
      {
        scan_len = sprintf(scan_msg, "Found I2C device at 0x%02X\r\n", addr);
        dbg_tx((uint8_t*)scan_msg, scan_len, 100);
        found_any = 1;
        if (addr == 0x3C) oled_ok = 1;
      }
    }

    if (!found_any)
    {
      char none_msg[] = "No I2C device found on I2C2. Check wiring (SCL=PB10, SDA=PC12).\r\n";
      dbg_tx((uint8_t*)none_msg, strlen(none_msg), 100);
    }

    char end_msg[] = "--- I2C2 Scan End ---\r\n\r\n";
    dbg_tx((uint8_t*)end_msg, strlen(end_msg), 100);
  }

  /* OLED 초기화 + 부팅 화면 (0x3C가 스캔에서 잡혔을 때만) */
  if (oled_ok)
  {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("AIV REAR", Font_11x18, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("booting...", Font_7x10, White);
    ssd1306_UpdateScreen();
  }

  /* MCP2515(레이더용 SPI2CAN) 초기화 */
  mcp_ok = mcp2515_init();
  last_radar_rx_ms = HAL_GetTick();  /* 부팅 직후 바로 타임아웃 뜨는 것 방지 */

  {
    char canMsg[80];
    if (mcp_ok)
      strcpy(canMsg, "MCP2515 OK: chip responded, Normal mode (interrupt mode)\r\n");
    else
      strcpy(canMsg, "MCP2515 FAIL: no chip response -> check SPI wiring/5V/CS\r\n");
    dbg_tx((uint8_t*)canMsg, strlen(canMsg), 100);
    /* 진단값 출력: 정상이면 stat0=80 cnf2=90 cnf3=02 stat3=00 */
    char dbg[120];
    int dl = sprintf(dbg, "MCP dbg: afterReset=%02X cfgMode=%02X wr/rd(A5)=%02X cnf2=%02X cnf3=%02X normal=%02X (OK: 80 80 A5 90 02 00)\r\n",
                     mcp_dbg[0], mcp_dbg[4], mcp_dbg[5], mcp_dbg[1], mcp_dbg[2], mcp_dbg[3]);
    dbg_tx((uint8_t*)dbg, dl, 100);
  }

  /* IMU(자세각 모듈) 초기화 - USART3(huart3, PC10/PC11), 인터럽트 기반 1바이트씩 수신
     front 보드에서 검증된 imu_uart.c/h 그대로 가져옴. 모듈 TX -> STM32 PC11(USART3_RX)로 배선 필요 */
  IMU_UART_Init(&huart3);
  {
    char imuReady[] = "IMU UART init done\r\n";
    dbg_tx((uint8_t*)imuReady, strlen(imuReady), 100);
  }

  /* OBD(Seeed Serial CAN-BUS 모듈) 초기화 - USART6(huart6, PC6/PC7) */
  obd_init();
  /* OBD 수신 인터럽트 시작 (obd_init의 블로킹 수신이 끝난 뒤에 켠다) */
  HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  obd_rx_start();
  {
    char obdReady[] = "OBD-II init done\r\n";
    dbg_tx((uint8_t*)obdReady, strlen(obdReady), 100);
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if ENABLE_IWDG
  /* 독립 워치독 시작: LSI 32kHz / 4 = 8kHz, 리로드 320 -> 약 40ms.
     (LSI 실제 주파수 편차로 약 27~75ms 범위. 루프 최대 소요가 9ms 수준이라 여유 있음)
     초기화(부팅 대기, 스캔 등)가 다 끝난 뒤, 메인 루프 바로 앞에서 시작한다. */
  IWDG->KR  = 0xCCCC;   /* 시작 */
  IWDG->KR  = 0x5555;   /* 설정 레지스터 쓰기 허용 */
  IWDG->PR  = 0;        /* 분주 /4 */
  IWDG->RLR = 319;      /* 320 카운트 */
  {
    uint32_t wt = HAL_GetTick();
    while (IWDG->SR != 0 && (HAL_GetTick() - wt) < 50) { }
  }
  IWDG->KR  = 0xAAAA;   /* 첫 kick */
  /* 디버거로 멈췄을 때(브레이크포인트 등) 워치독이 리셋을 걸지 않도록 정지 */
#ifdef DBGMCU_APB1_FZ_DBG_IWDG_STOP
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#else
  DBGMCU->APB1FZ |= (1u << 12);
#endif
#endif
  /* [v16] CPU 클럭 카운터(DWT) 켜기: 프레임 읽는 시간 측정용 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  while (1)
  {
    /* [LED 진단] 메인 루프에 도달하면 LD2가 0.5초마다 깜빡임 */
    {
      static uint32_t last_led_ms = 0;
      if (HAL_GetTick() - last_led_ms >= 500)
      {
        last_led_ms = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
    }

    /* 루프 1바퀴 최대 소요시간 측정 (5초마다 출력): 워치독 40ms 기준을 맞출 수 있는지 판단용 */
    {
      static uint32_t lp_last = 0, lp_max = 0, lp_rep = 0;
      uint32_t ln = HAL_GetTick();
      if (lp_last != 0 && (ln - lp_last) > lp_max) lp_max = ln - lp_last;
      lp_last = ln;
      if (ln - lp_rep >= 5000)
      {
        lp_rep = ln;
        char lm[40];
        int ll = sprintf(lm, "LOOP max=%lums\r\n", (unsigned long)lp_max);
        dbg_tx((uint8_t*)lm, ll, 100);
        lp_max = 0;
      }
    }

    /* RPi로 보내는 이진 프레임 (논블로킹: 큐에 넣기만 하고 실제 송신은 인터럽트가 처리) */
    proto_task();

    /* IMU(자세각) 출력: 인터럽트로 계속 갱신되는 g_imu 값을 200ms마다 한 번씩 내보냄.
       실시간 수신 자체는 IMU_UART_ByteReceived가 인터럽트에서 알아서 처리하므로
       여기서는 그냥 "지금 값이 뭔지 가끔 찍어보기"만 함. */
    if (HAL_GetTick() - last_imu_print_ms >= 200)
    {
      last_imu_print_ms = HAL_GetTick();
      if (g_imu.angle_updated)
      {
        int roll10  = (int)(g_imu.roll  * 10.0f);
        int pitch10 = (int)(g_imu.pitch * 10.0f);
        int yaw10   = (int)(g_imu.yaw   * 10.0f);
        char msg[100];
        int len = sprintf(msg, "IMU roll=%s%d.%01d pitch=%s%d.%01d yaw=%s%d.%01d bytes=%lu\r\n",
                           (roll10  < 0 ? "-" : ""), (roll10  < 0 ? -roll10  : roll10)  / 10, (roll10  < 0 ? -roll10  : roll10)  % 10,
                           (pitch10 < 0 ? "-" : ""), (pitch10 < 0 ? -pitch10 : pitch10) / 10, (pitch10 < 0 ? -pitch10 : pitch10) % 10,
                           (yaw10   < 0 ? "-" : ""), (yaw10   < 0 ? -yaw10   : yaw10)   / 10, (yaw10   < 0 ? -yaw10   : yaw10)   % 10,
                           g_byte_count);
        dbg_tx((uint8_t*)msg, len, 100);
      }
      else
      {
        char msg[] = "IMU waiting for data...\r\n";
        dbg_tx((uint8_t*)msg, strlen(msg), 100);
      }
    }

    /* OLED 화면 갱신 (500ms마다) */
    /* 그림은 500ms마다 새로 그리고, 화면 전송은 한 바퀴에 한 줄(페이지)씩 나눠서 한다 (약 3ms씩) */
    if (oled_ok)
    {
      if (oled_page >= OLED_PAGES)
      {
        if (HAL_GetTick() - last_oled_ms >= 500)
        {
          last_oled_ms = HAL_GetTick();
          oled_update();   /* 버퍼에 그리기만 함 (전송은 아래 oled_push_step) */
        }
      }
      else
      {
        oled_push_step();
      }
    }

    /* 레이더(MCP2515) 인터럽트 수신 처리.
       매 루프마다 SPI로 "새 데이터 왔어?"를 계속 묻던 폴링 방식을 버리고,
       MCP2515의 INT 핀(PC8)이 "데이터 도착!" 신호를 줄 때만 확인한다.
       수신 버퍼가 2칸뿐이라, 신호가 한 번 와도 밀려있는 프레임을 while로 다 비운다
       (버퍼가 안 비었으면 mcp2515_check_and_read가 계속 1을 반환함). */
    /* INT 핀이 Low로 계속 붙어있으면(=아직 안 읽은 프레임이 남음) 에지 신호를 못 받으므로
       핀 레벨도 같이 본다 (RXB1에 쌓인 프레임 때문에 INT가 계속 Low로 멈추는 문제 방지) */
    if (radar_int_flag || HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      radar_int_flag = 0;

      uint8_t data[8] = {0};
      uint8_t dlc = 0;
      uint32_t canId = 0;

      /* [v17] 1단계: 도착하는 프레임을 최대한 빨리 읽어서 모은다 (여기서는 계산/출력을 하지 않음).
         마지막 프레임 뒤 RAD_SPIN_US 동안 새 프레임이 없으면 끝낸다. */
      uint8_t nfb = 0;
      {
        uint32_t spin_cyc = (SystemCoreClock / 1000000U) * RAD_SPIN_US;
        uint32_t last_cyc = DWT->CYCCNT;
        while (nfb < RAD_BATCH)
        {
          if (mcp2515_check_and_read(rad_fb[nfb].data, &rad_fb[nfb].dlc, &rad_fb[nfb].id))
          {
            nfb++;
            last_cyc = DWT->CYCCNT;
          }
          else if ((DWT->CYCCNT - last_cyc) > spin_cyc)
          {
            break;
          }
        }
      }

      /* 2단계: 모은 프레임을 하나씩 처리 (예전과 같은 처리) */
      for (uint8_t bi = 0; bi < nfb; bi++)
      {
        memcpy(data, rad_fb[bi].data, 8);
        dlc = rad_fb[bi].dlc;
        canId = rad_fb[bi].id;
        uint32_t now_ms = HAL_GetTick();
        if (rad_seen)
        {
          uint32_t gap_ms = now_ms - last_radar_rx_ms;
          if (gap_ms > rad_gap_max) rad_gap_max = gap_ms;
        }
        rad_seen = 1;
        last_radar_rx_ms = now_ms;
        if (radar_safe_mode)
        {
          char rcv[48];
          int rcl = sprintf(rcv, "RADAR RECOVERED t=%lu\r\n", (unsigned long)now_ms);
          dbg_tx((uint8_t*)rcv, rcl, 100);
        }
        radar_safe_mode = 0;
        if (canId == 0x60A)    { rad_na++; rad_exp += data[0]; }
        else if (canId == 0x60B) rad_nb++;
        else                     rad_nother++;

        if (canId == 0x60B)
        {
          /* MR76(NanoRadar) 물체 프레임 디코딩 */
          uint8_t objId    = data[0];
          float   distLong = (data[1] * 32 + (data[2] >> 3)) * 0.2f - 500.0f;   /* 전후 거리(m) */
          float   distLat  = ((data[2] & 0x07) * 256 + data[3]) * 0.2f - 204.6f; /* 좌우 거리(m) */
          float   vrelLong = (data[4] * 4 + (data[5] >> 6)) * 0.25f - 128.0f;    /* 상대속도(m/s) */
          float   rcs      = data[7] * 0.5f - 64.0f;                             /* 반사강도(dBsm) */

          /* data[6]는 세 필드가 비트로 나뉘어 들어있음 (하위 3비트/다음 2비트/상위 3비트):
             dynProp(0~2비트): 0=이동 1=정지 2=마주옴 3=좌측횡단 4=우측횡단 5=알수없음 6=멈춤
             objClass(3~4비트): 0=점형체(사람·기둥·펜스 등) 1=차량 2,3=예약
             (5~7비트는 vrelLat 상위비트라 지금은 안 씀) */
          uint8_t dynProp  = data[6] & 0x07;
          uint8_t objClass = (data[6] >> 3) & 0x03;
          if (objClass == 1) rad_nveh++;

          /* 차량 검증: 점형체(사람/기둥 등)는 레이더 단계에서 걸러내서
             라즈베리파이 카메라 쪽으로 아예 안 넘어가게 한다 (계획서 3단계 "차량 여부 검증") */
          if (!TEST_ALL_CLASSES && objClass != 1)
          {
            /* 차량이 아니면 이번 물체는 건너뛰고 while 루프에서 다음 물체로 */
          }
          else
          {

          /* nano.specs 라이브러리가 %.1f를 지원 안 해서 정수(*10)로 변환 후 출력.
             부호(sign)는 따로 찍는다: 정수 나눗셈은 -0.5를 0으로 잘라서 부호가 사라지기 때문 */
          int distLong10 = (int)(distLong * 10.0f);
          int distLat10  = (int)(distLat  * 10.0f);
          int vrelLong10 = (int)(vrelLong * 10.0f);
          int rcs10      = (int)(rcs      * 10.0f);

          /* 노이즈 필터: 반사강도가 너무 낮은(-5dBsm 미만) 물체는 잡음으로 보고 무시 */
          if (rcs >= -5.0f)
          {
            rad_npub++;
            disp_dist10 = distLong10; disp_vrel10 = vrelLong10; disp_obj_ms = HAL_GetTick();
            char msg[140];
            if (rad_pr_cnt < OBJ_PRINT_MAX)
            {
            rad_pr_cnt++;
            int len = sprintf(msg, "OBJ id=%d dist=%s%d.%01dm lat=%s%d.%01dm vrel=%s%d.%01dm/s rcs=%s%d.%01d dyn=%d cls=%d\r\n",
                               objId,
                               (distLong10 < 0 ? "-" : ""), (distLong10 < 0 ? -distLong10 : distLong10) / 10, (distLong10 < 0 ? -distLong10 : distLong10) % 10,
                               (distLat10  < 0 ? "-" : ""), (distLat10  < 0 ? -distLat10  : distLat10)  / 10, (distLat10  < 0 ? -distLat10  : distLat10)  % 10,
                               (vrelLong10 < 0 ? "-" : ""), (vrelLong10 < 0 ? -vrelLong10 : vrelLong10) / 10, (vrelLong10 < 0 ? -vrelLong10 : vrelLong10) % 10,
                               (rcs10      < 0 ? "-" : ""), (rcs10      < 0 ? -rcs10      : rcs10)      / 10, (rcs10      < 0 ? -rcs10      : rcs10)      % 10,
                               dynProp, objClass);
            dbg_tx((uint8_t*)msg, len, 100);
            }

            /* RPi로 0x12 RADAR_OBJECT 프레임 전송 (필터 통과한 차량 물체만) */
            radar_publish(objId, distLong, distLat, vrelLong, rcs, dynProp);
          }
          }   /* else(objClass==VEHICLE) 끝 */
        }
        /* else if (canId == 0x60A)
        {
          char msg[40];
          int len = sprintf(msg, "objCount=%d\r\n", data[0]);
          dbg_tx((uint8_t*)msg, len, 100);
        } */
      }   /* for: 모아 둔 프레임 처리 끝 */
    }

    /* 타임아웃/Safe Mode: 100ms 넘게 레이더 응답이 없으면 센서 이상으로 보고 전환.
       (안전 판단은 STM32가 독립적으로 계속 하되, 레이더가 죽었다는 걸 알린다) */
    if (!radar_safe_mode && (HAL_GetTick() - last_radar_rx_ms > 100))
    {
      radar_safe_mode = 1;
      rad_safe_cnt++;
      char msg[64];
      int ml = sprintf(msg, "RADAR TIMEOUT -> SAFE MODE t=%lu\r\n", (unsigned long)HAL_GetTick());
      dbg_tx((uint8_t*)msg, ml, 100);
    }

    /* [v15 진단] 1초마다 레이더 상태 한 줄 (논블로킹 출력, 동작에는 영향 없음)
       a=0x60A 수, b=0x60B 수, veh=그중 차량 클래스, pub=Pi로 보낸 수, oth=다른 ID,
       gap=프레임 사이 최대 간격(ms), safe=Safe Mode 진입 누적, ovf=버퍼 넘침 누적,
       tec/rec=MCP2515 CAN 오류 카운터(오르면 CAN 선/속도 문제), drop=디버그 출력 유실 */
    {
      static uint32_t last_rs_ms = 0;
      if (HAL_GetTick() - last_rs_ms >= 1000)
      {
        last_rs_ms = HAL_GetTick();
        char rs[256];
        uint32_t mhz = SystemCoreClock / 1000000U;
        uint32_t rd_avg_us = rd_n ? (rd_cyc_sum / rd_n) / mhz : 0;
        uint32_t rd_max_us = rd_cyc_max / mhz;
        int rsl = sprintf(rs, "RADSTAT t=%lu a=%lu b=%lu exp=%lu veh=%lu pub=%lu oth=%lu gap=%lu safe=%lu ovf=%lu tec=%u rec=%u drop=%lu rd=%lu/%lu\r\n",
                          (unsigned long)last_rs_ms,
                          (unsigned long)rad_na, (unsigned long)rad_nb, (unsigned long)rad_exp, (unsigned long)rad_nveh,
                          (unsigned long)rad_npub, (unsigned long)rad_nother, (unsigned long)rad_gap_max,
                          (unsigned long)rad_safe_cnt, (unsigned long)rad_ovf,
                          (unsigned)radar_dbg_read(0x1C), (unsigned)radar_dbg_read(0x1D),
                          (unsigned long)dbg_drop_count,
                          (unsigned long)rd_avg_us, (unsigned long)rd_max_us);
        dbg_tx((uint8_t*)rs, rsl, 100);
        rad_na = rad_nb = rad_nveh = rad_npub = rad_nother = rad_gap_max = 0;
        rad_exp = 0; rd_cyc_sum = 0; rd_cyc_max = 0; rd_n = 0;
        rad_pr_cnt = 0;
        if (radar_dbg_read(0x2D) & 0xC0) { rad_ovf++; radar_dbg_clear_ovf(); }   /* ovf = 넘침이 있었던 "초" 수 */
      }
    }

    /* OBD-II 속도 폴링 (논블로킹: 수신은 인터럽트, 여기서는 잠깐 훑고 바로 리턴) */
    obd_poll_task();
    gps_dbg_task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if ENABLE_IWDG
    IWDG->KR = 0xAAAA;   /* 워치독 kick: 루프 한 바퀴가 정상적으로 끝났을 때만 */
#endif

#if WDG_TEST_HANG_MS
    if (HAL_GetTick() > WDG_TEST_HANG_MS)
    {
      char hm[] = "WDG TEST: hanging on purpose\r\n";
      dbg_tx((uint8_t*)hm, strlen(hm), 0);
      while (1) { }   /* kick 안 함 -> 약 40ms 뒤 리셋되어야 함 */
    }
#endif
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;   /* OLED 화면 갱신을 빠르게 (100kHz면 갱신에 약 90ms 걸려 메인 루프가 느려짐) */
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  /* MCP2515 최대 10MHz -> 약 2.6MHz로 낮춤 (원래 /2 = 21MHz라 너무 빠름) */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function (새로 추가: ST-Link VCP 디버그 출력용)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin (PA2/PA3, USART2 AF) */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 -> MCP2515 INT (레이더 수신 인터럽트용)
    MCP2515의 INT 핀은 Open-drain, Active-Low라서 데이터가 오면 0V로 떨어뜨립니다.
    그래서 IT_RISING이 아니라 IT_FALLING으로, 풀업을 켜서 평소엔 항상 High를 유지하게 합니다. */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PC8 = EXTI Line8 -> NVIC의 EXTI9_5 라인을 켜야 실제로 인터럽트가 걸립니다. */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /*Configure GPIO pin : PA8 (MCP2515 CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* MCP2515의 INT 핀(PC8)이 Low로 떨어지면 HAL이 자동으로 호출해주는 콜백.
   여기서는 무거운 작업(SPI 통신 등)을 하지 않고 플래그만 세운다.
   실제 데이터를 읽는 건 메인 루프에서 -> 인터럽트 서비스 시간을 최소화하기 위함. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8)
  {
    radar_int_flag = 1;
  }
}

/* IMU 모듈이 1바이트씩 보낼 때마다 HAL이 자동으로 호출해주는 콜백.
   USART3(huart3, PC10/PC11)로 들어온 바이트만 imu_uart.c의 파서로 넘긴다. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    IMU_UART_ByteReceived(huart);
  }
  else if (huart->Instance == USART6)
  {
    obd_rx_byte_isr(HAL_GetTick());
  }
  else if (huart->Instance == USART1)
  {
    rpi_rx_byte_isr();
  }
}

/* IMU로 "자차가 지금 완전히 정지해 있는가"를 판단.
   방법: 부팅 직후(차가 서 있다고 가정) 가속도 값을 기준값(baseline)으로 저장해두고,
   지금 가속도가 그 기준값에서 얼마나 벗어났는지 + 각속도(gyro) 크기를 같이 본다.
   가속도 적분으로 "속도(km/h)"까지 구하려는 게 아니라, 딱 "정지냐 아니냐"만 판단하는 용도.
   주의: 부팅할 때 차가 진짜 정지해 있다는 가정이 깔려 있다 (시동 걸자마자 켜는 상황 전제). */
uint8_t imu_is_stationary(void)
{
  if (!g_imu.accel_updated) return 0;   /* 아직 가속도 값이 한 번도 안 들어왔으면 판단 불가 */

  if (!imu_baseline_set)
  {
    /* 부팅 후 첫 가속도 값을 그대로 기준값으로 삼음 */
    imu_baseline_ax = g_imu.accel_x;
    imu_baseline_ay = g_imu.accel_y;
    imu_baseline_az = g_imu.accel_z;
    imu_baseline_set = 1;
    imu_last_motion_ms = HAL_GetTick();
    return 0;  /* 기준값 막 잡은 시점이라 아직 판단 보류 */
  }

  float dax = g_imu.accel_x - imu_baseline_ax;
  float day = g_imu.accel_y - imu_baseline_ay;
  float daz = g_imu.accel_z - imu_baseline_az;
  float accelDev = sqrtf(dax * dax + day * day + daz * daz);

  float gyroMag = fabsf(g_imu.gyro_x) + fabsf(g_imu.gyro_y) + fabsf(g_imu.gyro_z);

  uint8_t movingNow = (accelDev > IMU_ACCEL_DEV_THRESHOLD_G) || (gyroMag > IMU_GYRO_THRESHOLD_DPS);

  if (movingNow)
  {
    imu_last_motion_ms = HAL_GetTick();
    return 0;
  }

  /* 마지막 움직임 감지 이후 IMU_STATIONARY_HOLD_MS 이상 조용했으면 "정지"로 인정
     (순간적으로 조용한 걸 정지로 착각하지 않기 위한 일종의 디바운스) */
  return (HAL_GetTick() - imu_last_motion_ms) >= IMU_STATIONARY_HOLD_MS;
}

/* 11비트 표준 CAN ID(0~0x7FF)를 필터/마스크 레지스터에 쓸 형태로 쪼갠다.
   SIDH = ID[10:3], SIDL의 상위 3비트(bit7:5) = ID[2:0] (MCP2515 표준 ID 배치) */
/* ===== OLED 표시 =====
   128x64 화면, 6x8 폰트 -> 한 줄 21글자, 7줄 표시.
   값 x10 정수를 "-12.3" 형태 문자열로 만드는 도우미 (nano.specs가 %f 미지원이라 정수로 처리) */
static void fmt10(char *out, int v10)
{
    int a = (v10 < 0) ? -v10 : v10;
    sprintf(out, "%s%d.%d", (v10 < 0 ? "-" : ""), a / 10, a % 10);
}

/* 화면 버퍼의 한 페이지(128바이트)를 전송: 호출될 때마다 다음 페이지로 넘어간다 */
void oled_push_step(void)
{
    if (oled_page < OLED_PAGES)
    {
        ssd1306_UpdatePage(oled_page);
        oled_page++;
    }
}

void oled_update(void)
{
    char line[32];
    char a[16], b[16], c[16];
    uint32_t now = HAL_GetTick();

    ssd1306_Fill(Black);

    /* 1줄: 제목 + MCP2515 상태 */
    sprintf(line, "AIV REAR  MCP:%s", mcp_ok ? "OK" : "NG");
    ssd1306_SetCursor(0, 0);  ssd1306_WriteString(line, Font_6x8, White);

    /* 2줄: 레이더 상태 */
    sprintf(line, "RADAR: %s", radar_safe_mode ? "SAFE (no data)" : "OK");
    ssd1306_SetCursor(0, 9);  ssd1306_WriteString(line, Font_6x8, White);

    /* 3~4줄: 마지막 레이더 물체 (1초 넘게 새 물체 없으면 --- 표시) */
    if (disp_obj_ms != 0 && (now - disp_obj_ms) < 1000)
    {
        fmt10(a, disp_dist10);
        fmt10(b, disp_vrel10);
        sprintf(line, "DIST: %sm", a);
        ssd1306_SetCursor(0, 18); ssd1306_WriteString(line, Font_6x8, White);
        sprintf(line, "VREL: %sm/s", b);
        ssd1306_SetCursor(0, 27); ssd1306_WriteString(line, Font_6x8, White);
    }
    else
    {
        ssd1306_SetCursor(0, 18); ssd1306_WriteString("DIST: ---", Font_6x8, White);
        ssd1306_SetCursor(0, 27); ssd1306_WriteString("VREL: ---", Font_6x8, White);
    }

    /* 5줄: 자차 속도 (OBD 우선, 없으면 Pi5가 보내준 GPS 속도) */
    {
        uint16_t sp10; uint8_t src;
        if (get_ego_speed(&sp10, &src))
            sprintf(line, "SPD:%d.%d %s", sp10 / 10, sp10 % 10, src == 0 ? "OBD" : "GPS");
        else
            sprintf(line, "SPD: --- (no src)");
    }
    ssd1306_SetCursor(0, 36); ssd1306_WriteString(line, Font_6x8, White);

    /* 6~7줄: IMU 자세각 */
    if (g_imu.angle_updated)
    {
        fmt10(a, (int)(g_imu.roll  * 10.0f));
        fmt10(b, (int)(g_imu.pitch * 10.0f));
        fmt10(c, (int)(g_imu.yaw   * 10.0f));
        sprintf(line, "R:%s P:%s", a, b);
        ssd1306_SetCursor(0, 45); ssd1306_WriteString(line, Font_6x8, White);
        sprintf(line, "YAW:%s", c);
        ssd1306_SetCursor(0, 54); ssd1306_WriteString(line, Font_6x8, White);
    }
    else
    {
        ssd1306_SetCursor(0, 45); ssd1306_WriteString("IMU: waiting...", Font_6x8, White);
    }

    oled_page = 0;   /* 이제부터 페이지를 한 줄씩 전송 */
}

/* =====================================================================
   STM32 -> RPi 이진 프레임 송신  (규격: AIV_STM32-RPi_UART_메시지규격_v0.2.md)
   프레임: STX(02) TYPE LEN PAYLOAD XOR(TYPE^LEN^PAYLOAD) ETX(03), 리틀엔디안
   - 송신은 링 버퍼에 넣기만 하고 UART 인터럽트가 뒤에서 내보냄 (메인 루프 블로킹 없음)
   - 버퍼가 가득 차면 그 프레임은 버림(재시도 없음): tx_drop_count 증가
   ===================================================================== */
#define PROTO_STX          0x02
#define PROTO_ETX          0x03
#define PROTO_MAX_PAYLOAD  27
#define TX_BUF_SIZE        512

#define MSG_SENSOR_SUMMARY 0x10
#define MSG_HEARTBEAT      0x11
#define MSG_RADAR_OBJECT   0x12
#define MSG_IMU_DATA       0x13
#define MSG_OBD_DATA       0x14

/* SENSOR_STATUS 비트 */
#define ST_RADAR_OK    (1u << 0)
#define ST_IMU_OK      (1u << 1)
#define ST_OBD_OK      (1u << 2)
#define ST_CAN_CTRL_OK (1u << 3)
#define ST_SAFE_MODE   (1u << 4)
#define ST_BRAKE       (1u << 5)

/* IMU 차량 기준축 변환 (+X 전방, +Y 좌측, +Z 위). 센서 장착 방향이 확정되면 부호를 맞출 것 */
#define IMU_SIGN_X   (+1)
#define IMU_SIGN_Y   (+1)
#define IMU_SIGN_Z   (+1)

/* 브레이크 추정: 전방 가속도 <= -100mg 가 3샘플(20Hz 기준 150ms) 이어지면 ON, -80mg보다 커지면 OFF (제안값) */
#define BRAKE_ON_MG    (-100)
#define BRAKE_OFF_MG   (-80)
#define BRAKE_ON_COUNT 3

#define SENSOR_TIMEOUT_MS 100   /* 센서 타임아웃 기준 (규칙 5: 임의 변경 금지) */
#define OBD_FRESH_MS      500   /* OBD는 폴링 주기가 ~300ms라 500ms 기준 (제안) */

static uint8_t           tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0, tx_chunk = 0;
static volatile uint8_t  tx_busy = 0;
volatile uint32_t        tx_drop_count = 0;   /* 버퍼 가득 참으로 버린 프레임 수(진단용) */

static uint32_t imu_last_bytes_seen = 0;
static uint32_t imu_last_rx_ms = 0;           /* IMU 바이트가 마지막으로 들어온 시각 */
static uint8_t  brake_active = 0;
static uint8_t  brake_cnt = 0;

typedef struct { uint32_t ts; int16_t dist_long_cm; int16_t vrel_cms; } radar_ent_t;
#define RADAR_TBL_N 64
static radar_ent_t radar_tbl[RADAR_TBL_N];    /* 물체 ID별 최신값. ts==0이면 비어있음 */

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static int16_t clamp16(int32_t v) { return (v > 32767) ? 32767 : ((v < -32768) ? -32768 : (int16_t)v); }
static int8_t  clamp8(int32_t v)  { return (v > 127) ? 127 : ((v < -128) ? -128 : (int8_t)v); }

/* 링 버퍼에서 한 덩어리를 꺼내 인터럽트 송신 시작 (인터럽트 금지 상태 또는 ISR에서 호출) */
static void uart_tx_kick(void)
{
    if (tx_busy || tx_head == tx_tail) return;
    uint16_t len = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail) : (uint16_t)(TX_BUF_SIZE - tx_tail);
    if (HAL_UART_Transmit_IT(&huart1, &tx_buf[tx_tail], len) == HAL_OK)
    {
        tx_chunk = len;
        tx_busy = 1;
    }
}

/* 프레임을 만들어 송신 큐에 넣는다. 성공 1, 버퍼 부족/길이 오류로 버림 0. 절대 기다리지 않는다. */
static uint8_t proto_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) return 0;

    uint8_t frame[5 + PROTO_MAX_PAYLOAD];
    uint8_t chk = (uint8_t)(type ^ len);
    frame[0] = PROTO_STX;
    frame[1] = type;
    frame[2] = len;
    for (uint8_t i = 0; i < len; i++) { frame[3 + i] = payload[i]; chk ^= payload[i]; }
    frame[3 + len] = chk;
    frame[4 + len] = PROTO_ETX;
    uint16_t total = (uint16_t)(len + 5);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t used = (uint16_t)((tx_head - tx_tail + TX_BUF_SIZE) % TX_BUF_SIZE);
    uint16_t freesp = (uint16_t)(TX_BUF_SIZE - 1 - used);
    if (freesp < total)
    {
        tx_drop_count++;
        __set_PRIMASK(primask);
        return 0;
    }
    for (uint16_t i = 0; i < total; i++)
    {
        tx_buf[tx_head] = frame[i];
        tx_head = (uint16_t)((tx_head + 1) % TX_BUF_SIZE);
    }
    uart_tx_kick();
    __set_PRIMASK(primask);
    return 1;
}

/* ---- 디버그 텍스트(USART2) 논블로킹 송신 ----
   HAL_UART_Transmit은 전송이 끝날 때까지 기다려서 메인 루프를 수 ms씩 멈췄다.
   지금은 링 버퍼에 복사만 하고 실제 송신은 인터럽트가 한다. 가득 차면 그 메시지는 버림(디버그용이라 무방). */
#define DBG_BUF_SIZE 1024
static uint8_t           dbg_buf[DBG_BUF_SIZE];
static volatile uint16_t dbg_head = 0, dbg_tail = 0, dbg_chunk = 0;
static volatile uint8_t  dbg_busy = 0;
volatile uint32_t        dbg_drop_count = 0;

static void dbg_kick(void)
{
    if (dbg_busy || dbg_head == dbg_tail) return;
    uint16_t len = (dbg_head > dbg_tail) ? (uint16_t)(dbg_head - dbg_tail) : (uint16_t)(DBG_BUF_SIZE - dbg_tail);
    if (HAL_UART_Transmit_IT(&huart2, &dbg_buf[dbg_tail], len) == HAL_OK)
    {
        dbg_chunk = len;
        dbg_busy = 1;
    }
}

void dbg_tx(const uint8_t *p, uint16_t len, uint32_t unused)
{
    (void)unused;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t used = (uint16_t)((dbg_head - dbg_tail + DBG_BUF_SIZE) % DBG_BUF_SIZE);
    uint16_t freesp = (uint16_t)(DBG_BUF_SIZE - 1 - used);
    if (freesp < len)
    {
        dbg_drop_count++;
        __set_PRIMASK(primask);
        return;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        dbg_buf[dbg_head] = p[i];
        dbg_head = (uint16_t)((dbg_head + 1) % DBG_BUF_SIZE);
    }
    dbg_kick();
    __set_PRIMASK(primask);
}

/* 송신 한 덩어리 완료 -> 꼬리 이동 후 남은 게 있으면 이어서 송신 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_tail = (uint16_t)((tx_tail + tx_chunk) % TX_BUF_SIZE);
        tx_busy = 0;
        uart_tx_kick();
    }
    else if (huart->Instance == USART2)
    {
        dbg_tail = (uint16_t)((dbg_tail + dbg_chunk) % DBG_BUF_SIZE);
        dbg_busy = 0;
        dbg_kick();
    }
}

/* UART 오류: RPi 링크는 현재 덩어리를 포기하고 계속, IMU는 수신을 다시 시작(오버런 시 수신이 멈추기 때문) */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 송신 쪽: 송신이 실제로 중단된 경우에만 현재 덩어리를 포기 (수신 오류는 송신과 무관) */
        if (tx_busy && huart->gState != HAL_UART_STATE_BUSY_TX)
        {
            tx_tail = (uint16_t)((tx_tail + tx_chunk) % TX_BUF_SIZE);
            tx_busy = 0;
            tx_chunk = 0;
            uart_tx_kick();
        }
        /* 수신 쪽: 오버런 등으로 수신이 멈췄으면 다시 시작 */
        if (huart->RxState == HAL_UART_STATE_READY)
            HAL_UART_Receive_IT(&huart1, &rpi_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        IMU_UART_Init(&huart3);
    }
    else if (huart->Instance == USART6)
    {
        obd_rx_start();   /* OBD 수신 오류(오버런 등) 후 수신 재시작 */
    }
    else if (huart->Instance == USART2)
    {
        dbg_tail = (uint16_t)((dbg_tail + dbg_chunk) % DBG_BUF_SIZE);
        dbg_busy = 0;
        dbg_chunk = 0;
        dbg_kick();
    }
}

static uint8_t imu_is_ok(uint32_t now)  { return (imu_last_rx_ms != 0) && ((now - imu_last_rx_ms) <= SENSOR_TIMEOUT_MS); }
static uint8_t obd_is_ok(uint32_t now)  { return (last_ego_speed_ms != 0) && ((now - last_ego_speed_ms) <= OBD_FRESH_MS); }

static uint8_t build_sensor_status(uint32_t now)
{
    uint8_t st = 0;
    uint8_t radar_ok = radar_safe_mode ? 0 : 1;
    uint8_t imu_ok = imu_is_ok(now);
    if (radar_ok)  st |= ST_RADAR_OK;
    if (imu_ok)    st |= ST_IMU_OK;
    if (obd_is_ok(now)) st |= ST_OBD_OK;
    if (mcp_ok)    st |= ST_CAN_CTRL_OK;
    if (!radar_ok || !imu_ok) st |= ST_SAFE_MODE;   /* 핵심 센서(레이더/IMU) 불량 -> 보수적 동작 */
    if (brake_active) st |= ST_BRAKE;
    return st;
}

/* 0x12 RADAR_OBJECT: 필터를 통과한 차량 물체 1개를 보내고, 최근 물체 표도 갱신 */
void radar_publish(uint8_t id, float dist_long_m, float dist_lat_m, float vrel_ms, float rcs_dbsm, uint8_t dyn)
{
    uint32_t now = HAL_GetTick();
    int16_t dl = clamp16((int32_t)(dist_long_m * 100.0f));
    int16_t dt = clamp16((int32_t)(dist_lat_m  * 100.0f));
    int16_t vr = clamp16((int32_t)(vrel_ms     * 100.0f));
    int8_t  rc = clamp8((int32_t)(rcs_dbsm * 2.0f));

    radar_tbl[id % RADAR_TBL_N].ts = (now == 0) ? 1 : now;
    radar_tbl[id % RADAR_TBL_N].dist_long_cm = dl;
    radar_tbl[id % RADAR_TBL_N].vrel_cms = vr;

    uint8_t p[14];
    put_u32(&p[0], now);
    p[4] = id;
    put_u16(&p[5],  (uint16_t)dl);
    put_u16(&p[7],  (uint16_t)dt);
    put_u16(&p[9],  (uint16_t)vr);
    p[11] = (uint8_t)rc;
    p[12] = dyn;
    p[13] = 0xFF;   /* CYCLE_INFO: 사이클 정보 미제공(0x60A 미사용) */
    proto_send(MSG_RADAR_OBJECT, p, 14);
}

/* 0x14 OBD_DATA */
void proto_send_obd(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t p[5];
    if (last_ego_speed_ms != 0)
    {
        uint32_t age = now - last_ego_speed_ms;
        p[0] = last_ego_speed_kmh;
        put_u16(&p[3], (uint16_t)((age > 0xFFFE) ? 0xFFFE : age));
    }
    else
    {
        p[0] = 0xFF;
        put_u16(&p[3], 0xFFFF);
    }
    put_u16(&p[1], 0xFFFF);   /* RPM: 아직 폴링 안 함(PID 0x0C 미구현) */
    proto_send(MSG_OBD_DATA, p, 5);
}

/* 주기 송신: 50Hz SENSOR_SUMMARY, 10Hz HEARTBEAT, 20Hz IMU_DATA. 메인 루프에서 매번 호출(즉시 리턴). */
void proto_task(void)
{
    static uint32_t t_sum = 0, t_hb = 0, t_imu = 0;
    uint32_t now = HAL_GetTick();

    /* IMU 수신 신선도: 바이트 카운터가 움직이면 데이터가 들어오는 중 */
    if (g_byte_count != imu_last_bytes_seen)
    {
        imu_last_bytes_seen = g_byte_count;
        imu_last_rx_ms = (now == 0) ? 1 : now;
    }

    /* 0x10 SENSOR_SUMMARY (50Hz = 20ms) */
    if (now - t_sum >= 20)
    {
        t_sum = now;
        uint16_t tgt = 0xFFFF;
        int16_t  rel = 0x7FFF;
        if (!radar_safe_mode)
        {
            /* 임시: 최근 100ms 안에 들어온 물체 중 앞쪽으로 가장 가까운 것 (진짜 앞차 특정은 RPi가 함) */
            for (uint8_t i = 0; i < RADAR_TBL_N; i++)
            {
                if (radar_tbl[i].ts != 0 && (now - radar_tbl[i].ts) <= SENSOR_TIMEOUT_MS && radar_tbl[i].dist_long_cm > 0)
                {
                    uint16_t d = (uint16_t)radar_tbl[i].dist_long_cm;
                    if (tgt == 0xFFFF || d < tgt) { tgt = d; rel = radar_tbl[i].vrel_cms; }
                }
            }
        }
        uint8_t p[11];
        put_u32(&p[0], now);
        p[4] = 0xFF;                       /* K_VALUE: 계산 로직 미구현 -> 0xFF(계산 안 됨) */
        put_u16(&p[5], tgt);
        put_u16(&p[7], (uint16_t)rel);
        p[9] = 0xFF;                       /* DANGER_FLAG: 판단 로직 미구현 -> 0xFF(판단 불가) */
        p[10] = build_sensor_status(now);
        proto_send(MSG_SENSOR_SUMMARY, p, 11);
    }

    /* 0x11 HEARTBEAT (10Hz = 100ms) */
    if (now - t_hb >= 100)
    {
        t_hb = now;
        uint8_t p[6];
        put_u32(&p[0], now);
        p[4] = build_sensor_status(now);
        p[5] = reset_cause;
        proto_send(MSG_HEARTBEAT, p, 6);
    }

    /* 0x13 IMU_DATA (20Hz = 50ms) + 브레이크 추정 */
    if (now - t_imu >= 50)
    {
        t_imu = now;
        uint8_t imu_ok = imu_is_ok(now);

        int16_t ax = clamp16((int32_t)(IMU_SIGN_X * g_imu.accel_x * 1000.0f));
        int16_t ay = clamp16((int32_t)(IMU_SIGN_Y * g_imu.accel_y * 1000.0f));
        int16_t az = clamp16((int32_t)(IMU_SIGN_Z * g_imu.accel_z * 1000.0f));
        int16_t gx = clamp16((int32_t)(IMU_SIGN_X * g_imu.gyro_x * 10.0f));
        int16_t gy = clamp16((int32_t)(IMU_SIGN_Y * g_imu.gyro_y * 10.0f));
        int16_t gz = clamp16((int32_t)(IMU_SIGN_Z * g_imu.gyro_z * 10.0f));
        int16_t ro = clamp16((int32_t)(g_imu.roll  * 10.0f));
        int16_t pi = clamp16((int32_t)(g_imu.pitch * 10.0f));
        int16_t ya = clamp16((int32_t)(g_imu.yaw   * 10.0f));

        if (imu_ok)
        {
            if (ax <= BRAKE_ON_MG)
            {
                if (brake_cnt < 255) brake_cnt++;
                if (brake_cnt >= BRAKE_ON_COUNT) brake_active = 1;
            }
            else
            {
                brake_cnt = 0;
                if (ax > BRAKE_OFF_MG) brake_active = 0;
            }
        }
        else
        {
            brake_cnt = 0;
            brake_active = 0;   /* IMU 불량이면 추정 안 함 (IMU_OK 비트로 구분) */
        }

        uint8_t p[23];
        put_u32(&p[0], now);
        put_u16(&p[4],  (uint16_t)ax);
        put_u16(&p[6],  (uint16_t)ay);
        put_u16(&p[8],  (uint16_t)az);
        put_u16(&p[10], (uint16_t)gx);
        put_u16(&p[12], (uint16_t)gy);
        put_u16(&p[14], (uint16_t)gz);
        put_u16(&p[16], (uint16_t)ro);
        put_u16(&p[18], (uint16_t)pi);
        put_u16(&p[20], (uint16_t)ya);
        p[22] = (uint8_t)((imu_ok ? 0x01 : 0x00) | (brake_active ? 0x02 : 0x00));
        proto_send(MSG_IMU_DATA, p, 23);
    }
}

static void mcp2515_split_std_id(uint16_t id, uint8_t *sidh, uint8_t *sidl) {
    *sidh = (uint8_t)(id >> 3);
    *sidl = (uint8_t)((id & 0x07) << 5);
}

static void mcp_cs(uint8_t low) {
    HAL_GPIO_WritePin(MCP_CS_PORT, MCP_CS_PIN, low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t mcp_xfer(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 100);
    return rx;
}

static void mcp_write(uint8_t addr, uint8_t val) {
    mcp_cs(1);
    mcp_xfer(MCP_WRITE);
    mcp_xfer(addr);
    mcp_xfer(val);
    mcp_cs(0);
}

static uint8_t mcp_read(uint8_t addr) {
    uint8_t val;
    mcp_cs(1);
    mcp_xfer(MCP_READ);
    mcp_xfer(addr);
    val = mcp_xfer(0x00);
    mcp_cs(0);
    return val;
}

static void mcp_bitmod(uint8_t addr, uint8_t mask, uint8_t val) {
    mcp_cs(1);
    mcp_xfer(MCP_BITMOD);
    mcp_xfer(addr);
    mcp_xfer(mask);
    mcp_xfer(val);
    mcp_cs(0);
}

static void mcp_reset(void) {
    mcp_cs(1);
    mcp_xfer(MCP_RESET);
    mcp_cs(0);
    HAL_Delay(10);
}

/* CAN ID 필터/마스크 설정: id 하나만 통과시키고 나머지는 MCP2515가 하드웨어에서 버림.
   RXB0(필터0,1)과 RXB1(필터2) 둘 다 같은 ID로 맞추고, RXB0가 꽉 차면 RXB1로
   자동 이월(BUKT)되게 해서 버퍼 2칸을 최대한 활용한다.
   설정모드에서만 이 레지스터들을 쓸 수 있어서, mcp2515_init()이 reset 직후(설정모드) 호출한다. */
static void mcp2515_set_filter(uint16_t id) {
    uint8_t sidh, sidl, msidh, msidl;

    mcp2515_split_std_id(id, &sidh, &sidl);
    mcp2515_split_std_id(0x7FF, &msidh, &msidl);  /* 마스크 전부 1 = 정확히 일치하는 ID만 통과 */

    mcp_write(REG_RXF0SIDH, sidh); mcp_write(REG_RXF0SIDL, sidl);  /* RXB0용 필터0 */
    mcp_write(REG_RXF1SIDH, sidh); mcp_write(REG_RXF1SIDL, sidl);  /* RXB0용 필터1(둘 다 채워야 함) */
    mcp_write(REG_RXF2SIDH, sidh); mcp_write(REG_RXF2SIDL, sidl);  /* RXB1용 필터2 */

    /* 0x60A(레이더 상태/물체 개수)도 통과시킨다: 물체가 없어도 주기적으로 오므로 "레이더 살아있음" 신호로 쓴다.
       RXF0/RXF1은 RXB0용, RXF2~5는 RXB1용. id=0x60B, 0x60A를 번갈아 채운다. */
    { uint8_t ah, al; mcp2515_split_std_id(0x60A, &ah, &al);
      mcp_write(REG_RXF1SIDH, ah); mcp_write(REG_RXF1SIDL, al);
      mcp_write(0x10, ah); mcp_write(0x11, al);   /* RXF3 */
      mcp_write(0x1C, ah); mcp_write(0x1D, al);   /* RXF5 */
      mcp_write(0x18, sidh); mcp_write(0x19, sidl); /* RXF4 = 0x60B */
    }
    mcp_write(REG_RXM0SIDH, msidh); mcp_write(REG_RXM0SIDL, msidl); /* RXB0 마스크 */
    mcp_write(REG_RXM1SIDH, msidh); mcp_write(REG_RXM1SIDL, msidl); /* RXB1 마스크 */
}

/* MCP2515 칩이 SPI로 실제 응답하는지 확인하면서 초기화. 성공 1, 실패 0.
   (칩이 없거나 배선이 잘못되면 읽은 값이 0x00/0xFF로 나와서 실패로 판정됨) */
#define REG_CANSTAT_ADDR 0x0E
uint8_t mcp2515_init(void) {
    uint8_t ok = 1;
    mcp_reset();

    /* 리셋 직후엔 설정(Configuration) 모드여야 함: CANSTAT 상위 3비트 = 100 (0x80) */
    mcp_dbg[0] = mcp_read(REG_CANSTAT_ADDR);   /* 참고용(리셋 명령이 먹었다면 0x80) */

    /* 리셋 명령이 안 먹은 경우(이전 실행에서 Normal 모드로 남아있는 등)를 대비해
       설정(Configuration) 모드를 직접 요청: CANCTRL의 REQOP=100 */
    mcp_write(REG_CANCTRL, 0x80);
    HAL_Delay(10);
    mcp_dbg[4] = mcp_read(REG_CANSTAT_ADDR);
    if ((mcp_dbg[4] & 0xE0) != 0x80) ok = 0;

    /* SPI 쓰기->읽기 왕복 테스트: 기본값이 아닌 값(0xA5)을 써서 그대로 읽히는지 확인 */
    mcp_write(REG_CNF1, 0xA5);
    mcp_dbg[5] = mcp_read(REG_CNF1);
    if (mcp_dbg[5] != 0xA5) ok = 0;

    /* 500kbps @ 8MHz 크리스탈 기준 */
    mcp_write(REG_CNF1, 0x00);
    mcp_write(REG_CNF2, 0x90);
    mcp_write(REG_CNF3, 0x02);

    /* 쓴 값이 그대로 읽히는지 확인 (SPI 통신 왕복 검증) */
    mcp_dbg[1] = mcp_read(REG_CNF2);
    mcp_dbg[2] = mcp_read(REG_CNF3);
    if (mcp_dbg[1] != 0x90) ok = 0;
    if (mcp_dbg[2] != 0x02) ok = 0;

    /* CAN ID 필터: 0x60B(물체 데이터)만 통과, 나머지(0x201, 0x60A 등)는 MCP2515가 버림 */
    mcp2515_set_filter(MR76_OBJECT_ID);

    /* RXM=00(필터/마스크 사용), RXB0CTRL의 BUKT=1로 RXB0 꽉 차면 RXB1로 이월 */
    mcp_write(REG_RXB0CTRL, 0x04);
    mcp_write(REG_RXB1CTRL, 0x00);

    /* RX 인터럽트 플래그만 사용 (폴링) */
    mcp_write(REG_CANINTE, 0x03);

    /* Normal 모드로 전환 */
    mcp_write(REG_CANCTRL, 0x00);
    HAL_Delay(10);

    /* Normal 모드로 넘어갔는지 확인: CANSTAT 상위 3비트 = 000 */
    mcp_dbg[3] = mcp_read(REG_CANSTAT_ADDR);
    if ((mcp_dbg[3] & 0xE0) != 0x00) ok = 0;

    return ok;
}

static uint8_t mcp2515_read_orig(uint8_t *data, uint8_t *dlc, uint32_t *id) {
    uint8_t intf = mcp_read(REG_CANINTF);
    uint8_t useBuf1;
    if (intf & 0x01)      useBuf1 = 0;   /* RXB0에 프레임 있음 */
    else if (intf & 0x02) useBuf1 = 1;   /* RXB1로 넘어온 프레임 (이걸 안 읽으면 INT가 계속 Low로 멈춤) */
    else return 0;

    mcp_cs(1);
    mcp_xfer(useBuf1 ? 0x94 : MCP_READ_RX0);   /* 0x94 = RXB1 읽기 */
    uint8_t sidh = mcp_xfer(0x00);
    uint8_t sidl = mcp_xfer(0x00);
    mcp_xfer(0x00); /* EID8 */
    mcp_xfer(0x00); /* EID0 */
    uint8_t dlcReg = mcp_xfer(0x00);
    uint8_t len = dlcReg & 0x0F;
    for (uint8_t i = 0; i < len && i < 8; i++) {
        data[i] = mcp_xfer(0x00);
    }
    mcp_cs(0);

    *id = ((uint32_t)sidh << 3) | (sidl >> 5);
    *dlc = len;

    mcp_bitmod(REG_CANINTF, useBuf1 ? 0x02 : 0x01, 0x00); /* 읽은 버퍼의 인터럽트 플래그 클리어 */
    if (mcp_read(0x2D) & 0xC0) rad_ovf++;                 /* [v15 진단] 지우기 전에 넘침이 있었는지 센다 */
    mcp_bitmod(0x2D, 0xC0, 0x00);                         /* 오버플로 플래그(EFLG RX0OVR/RX1OVR) 클리어 */
    return 1;
}

/* [v16] 빠른 읽기: SPI 호출 수를 줄여서(프레임 1개 약 28번 -> 5번 이하) 읽는 시간을 줄인다.
   - 레지스터/버퍼를 바이트마다 따로 읽던 것을 한 번의 SPI 전송으로 묶음 (CS는 전송 동안 Low 유지)
   - 판단 로직(어느 버퍼를 읽는지, 플래그 지우기, 오버플로 세기)은 예전 방식과 같다 */
static void mcp_txrx(const uint8_t *tx, uint8_t *rx, uint16_t n) {
    mcp_cs(1);
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)tx, rx, n, 100);
    mcp_cs(0);
}

static uint8_t mcp2515_read_fast(uint8_t *data, uint8_t *dlc, uint32_t *id) {
    uint8_t tx[14], rx[14];

    tx[0] = 0xB0; tx[1] = 0x00;                  /* RX STATUS 명령: 어느 수신 버퍼에 프레임이 있는지 한 번에 알려줌 */
    mcp_txrx(tx, rx, 2);
    uint8_t st = rx[1];
    uint8_t useBuf1;
    if (st & 0x40)      useBuf1 = 0;             /* bit6 = RXB0에 프레임 있음 */
    else if (st & 0x80) useBuf1 = 1;             /* bit7 = RXB1에 프레임 있음 */
    else return 0;

    memset(tx, 0, sizeof(tx));
    tx[0] = useBuf1 ? 0x94 : MCP_READ_RX0;       /* 0x94 = RXB1, 0x90 = RXB0 읽기 */
    mcp_txrx(tx, rx, 14);                        /* rx[1..5] = SIDH SIDL EID8 EID0 DLC, rx[6..13] = 데이터 */
    uint8_t sidh = rx[1], sidl = rx[2];
    uint8_t len = rx[5] & 0x0F;
    for (uint8_t i = 0; i < len && i < 8; i++) data[i] = rx[6 + i];
    *id = ((uint32_t)sidh << 3) | (sidl >> 5);
    *dlc = len;

    uint8_t bm[4] = { MCP_BITMOD, REG_CANINTF, (uint8_t)(useBuf1 ? 0x02 : 0x01), 0x00 };
    mcp_txrx(bm, rx, 4);                          /* 읽은 버퍼의 인터럽트 플래그 지우기 */
    return 1;
}

/* 메인 루프가 부르는 읽기 함수: 선택한 방식으로 읽고, 프레임을 읽었을 때만 걸린 시간을 잰다 */
uint8_t mcp2515_check_and_read(uint8_t *data, uint8_t *dlc, uint32_t *id) {
    uint32_t c0 = DWT->CYCCNT;
#if FAST_MCP_READ
    uint8_t r = mcp2515_read_fast(data, dlc, id);
#else
    uint8_t r = mcp2515_read_orig(data, dlc, id);
#endif
    if (r) {
        uint32_t d = DWT->CYCCNT - c0;
        rd_cyc_sum += d;
        if (d > rd_cyc_max) rd_cyc_max = d;
        rd_n++;
    }
    return r;
}

/* [v15 진단] 메인 루프(위쪽)에서 MCP2515 레지스터를 읽기 위한 통로 (TEC=0x1C, REC=0x1D) */
uint8_t radar_dbg_read(uint8_t addr)
{
    return mcp_read(addr);
}

/* [v17] 수신 넘침 플래그(EFLG의 RX0OVR/RX1OVR) 지우기 */
void radar_dbg_clear_ovf(void)
{
    mcp_bitmod(0x2D, 0xC0, 0x00);
}

/* ===== OBD(Seeed Serial CAN-BUS 모듈) 관련 =====
   ELM327 AT 명령어가 아니라, 14바이트 전송/12바이트 수신의 바이너리 CAN 프레임 방식.
   USART6(huart6, PC6/PC7), 기본 baud 9600. */

/* ===== OBD 초기화/폴링 (v2: 차 CAN 폭주 대응) =====
   중요: 이 모듈은 "데이터 모드"에서 차 버스에 흐르는 모든 CAN 프레임을 12바이트씩 UART로 계속 쏟아낸다(9600bps라 거의 포화).
   - 예전 코드의 "쌓인 글자 비우기" 루프는 글자가 계속 오면 영원히 끝나지 않아, 차에 꽂으면 부팅이 거기서 멈췄다.
   - 폭주 속에서는 12바이트 프레임 경계(간격 기준)를 못 찾으므로, 응답 패턴을 "미끄러지는 창"으로 찾는다.
   - "+++"는 앞뒤 1초 이상 UART 송신을 멈추고 개행 없이 보낸다. 응답에 SETTING MODE가 확인될 때만 AT+Q를 보낸다
     (확인 안 된 상태에서 AT+Q 같은 글자를 보내면 모듈이 14바이트 프레임 경계를 잃어 차 버스에 쓰레기 프레임을 쏠 수 있다). */
static uint8_t obd_wait_text(const char *key, uint32_t total_ms)
{
    uint8_t ch, m = 0;
    const uint8_t klen = (uint8_t)strlen(key);
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < total_ms)
    {
        if (HAL_UART_Receive(&huart6, &ch, 1, 20) != HAL_OK) continue;
        if (ch == (uint8_t)key[m]) { if (++m == klen) return 1; }
        else m = (ch == (uint8_t)key[0]) ? 1 : 0;
    }
    return 0;
}

static void obd_drain_bounded(uint32_t max_ms)
{
    uint8_t ch;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < max_ms && HAL_UART_Receive(&huart6, &ch, 1, 2) == HAL_OK) {}
}

/* 모듈이 차 버스의 모든 프레임을 쏟아내지 않도록, 진단 응답 ID(0x7E8~0x7EF)만 받게 마스크/필터를 설정한다.
   설정 모드에서만 가능. 마스크 0x7F8 + 필터 0x7E8 -> 0x7E8~0x7EF만 통과.
   대괄호 포함 형식만 모듈이 받아들인다(전방차 실차 테스트로 확인).
   저장(전원 껐다 켜도 유지) 여부는 문서에 없다 -> 부팅마다 다시 설정한다(여러 번 해도 무해). */
static uint8_t obd_cfg_cmd(const char *cmd)
{
    char m[96]; int l;
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, (uint16_t)strlen(cmd), 200);
    uint8_t ok = obd_wait_text("OK", 500);
    l = sprintf(m, "OBDCFG %.*s -> %s\r\n", (int)strlen(cmd) - 1, cmd, ok ? "OK" : "NO/ERROR");
    dbg_tx((uint8_t*)m, l, 100);
    return ok;
}

static void obd_set_filter(void)
{
    char cmd[40];
    /* 실차에서 확인된 형식: 대괄호 포함 AT+M=[N][EXT][값] / AT+F=[N][EXT][값] */
    sprintf(cmd, "AT+M=[%d][%d][%08X]\n", 0, 0, 0x7F8);
    if (!obd_cfg_cmd(cmd)) return;           /* 실패 시 필터 없이 계속 */
    sprintf(cmd, "AT+M=[%d][%d][%08X]\n", 1, 0, 0x7F8);
    obd_cfg_cmd(cmd);
    for (int n = 0; n <= 5; n++)
    {
        sprintf(cmd, "AT+F=[%d][%d][%08X]\n", n, 0, 0x7E8);
        obd_cfg_cmd(cmd);
    }
}

void obd_init(void)
{
    char m[64]; int l;

    obd_drain_bounded(50);
    HAL_Delay(1200);                                              /* +++ 앞 정지(guard time) */
    HAL_UART_Transmit(&huart6, (uint8_t*)"+++", 3, 200);          /* 개행 없음 */
    uint8_t ok1 = obd_wait_text("SETTING MODE", 3000);
    l = sprintf(m, "OBDLINK +++ %s\r\n", ok1 ? "OK (setting mode)" : "NO REPLY");
    dbg_tx((uint8_t*)m, l, 100);

    if (ok1)
    {
        obd_set_filter();
        HAL_UART_Transmit(&huart6, (uint8_t*)"AT+Q\n", 5, 200);
        uint8_t ok2 = obd_wait_text("DATA MODE", 1500);
        l = sprintf(m, "OBDLINK AT+Q %s\r\n", ok2 ? "OK (data mode)" : "NO REPLY");
        dbg_tx((uint8_t*)m, l, 100);
    }
    obd_drain_bounded(50);
}

/* ===== OBD 논블로킹 폴링 =====
   수신은 인터럽트로 링 버퍼에 쌓고, 메인 루프(obd_poll_task)는 "잠깐 훑고 바로 리턴"한다.
   PID 0x0D(차속): 0x7DF로 요청 -> 응답 ID 0x7E8~0x7EF, 데이터 [03 41 0D 속도 ...]
   수신 바이트 열에서 [00 00 07 E8~EF 03 41 0D 속도] 8바이트 패턴을 찾는다. */
#define OBD_RING_N        256         /* 2의 거듭제곱 */
#define OBD_RESP_TIMEOUT  300         /* 요청 후 응답 대기 한도(ms) */

static volatile uint8_t  obd_ring[OBD_RING_N];
static volatile uint8_t  obd_ring_head = 0, obd_ring_tail = 0;
static uint8_t  obd_rx_byte;
static uint8_t  obd_tx_buf[14];
static uint8_t  obd_win[8];           /* 최근 수신 8바이트 (미끄러지는 창) */
static uint8_t  obd_waiting = 0;      /* 1: 요청을 보내고 응답을 기다리는 중 */
static uint32_t obd_req_ms = 0;
static volatile uint32_t obd_raw_cnt = 0;   /* 진단: 모듈에서 받은 전체 바이트 수 */
static uint32_t obd_ok_cnt = 0;             /* 진단: 속도 응답 성공 횟수 */

void obd_rx_byte_isr(uint32_t now)
{
    (void)now;
    obd_raw_cnt++;
    uint8_t next = (uint8_t)((obd_ring_head + 1) & (OBD_RING_N - 1));
    if (next != obd_ring_tail)
    {
        obd_ring[obd_ring_head] = obd_rx_byte;
        obd_ring_head = next;
    }
    HAL_UART_Receive_IT(&huart6, &obd_rx_byte, 1);   /* 다음 1바이트 계속 수신 */
}

void obd_rx_start(void)
{
    HAL_UART_Receive_IT(&huart6, &obd_rx_byte, 1);
}

/* 요청 프레임(14바이트: ID4 + EXT + RTR + DATA8)을 인터럽트 방식으로 송신 시작. 성공 1 */
static uint8_t obd_start_request(uint32_t now)
{
    memset(obd_tx_buf, 0, sizeof(obd_tx_buf));
    obd_tx_buf[0] = 0x00; obd_tx_buf[1] = 0x00; obd_tx_buf[2] = 0x07; obd_tx_buf[3] = 0xDF;  /* ID 0x7DF */
    obd_tx_buf[4] = 0;    /* 표준 프레임 */
    obd_tx_buf[5] = 0;    /* 데이터 프레임 */
    obd_tx_buf[6]  = 0x02; obd_tx_buf[7] = 0x01; obd_tx_buf[8] = 0x0D;   /* 길이2, 서비스01, PID 0x0D */

    obd_ring_tail = obd_ring_head;   /* 옛 수신 데이터 버림 */
    memset(obd_win, 0xFF, sizeof(obd_win));
    if (HAL_UART_Transmit_IT(&huart6, obd_tx_buf, 14) != HAL_OK) return 0;
    obd_waiting = 1;
    obd_req_ms = now;
    return 1;
}

/* 메인 루프에서 매번 호출: 수신 바이트 검사 -> 성공/타임아웃 처리 -> 다음 요청 시점이면 요청 */
void obd_poll_task(void)
{
    uint32_t now = HAL_GetTick();

    /* [진단] 1초마다: 모듈에서 받은 바이트 수 / 속도 응답 성공 수 */
    {
        static uint32_t last_stat_ms = 0;
        if (now - last_stat_ms >= 1000)
        {
            last_stat_ms = now;
            char m[80];
            int l = sprintf(m, "OBDSTAT rx=%lu ok=%lu fail=%d\r\n",
                            (unsigned long)obd_raw_cnt, (unsigned long)obd_ok_cnt, obd_fail_count);
            dbg_tx((uint8_t*)m, l, 100);
        }
    }

    /* 1) 링 버퍼의 바이트를 창에 밀어 넣으며 응답 패턴 검사 */
    while (obd_ring_tail != obd_ring_head)
    {
        uint8_t b = obd_ring[obd_ring_tail];
        obd_ring_tail = (uint8_t)((obd_ring_tail + 1) & (OBD_RING_N - 1));

        if (!obd_waiting) continue;                 /* 요청 안 했을 때 온 데이터는 버림 */

        for (uint8_t i = 0; i < 7; i++) obd_win[i] = obd_win[i + 1];
        obd_win[7] = b;

        if (obd_win[0] == 0x00 && obd_win[1] == 0x00 && obd_win[2] == 0x07 && (obd_win[3] & 0xF8) == 0xE8 &&
            obd_win[4] == 0x03 && obd_win[5] == 0x41 && obd_win[6] == 0x0D)
        {
            if (obd_fail_count >= 3)
            {
                char m[] = "OBD responding again -> normal polling\r\n";
                dbg_tx((uint8_t*)m, strlen(m), 100);
            }
            obd_fail_count = 0;
            obd_ok_cnt++;
            last_ego_speed_kmh = obd_win[7];
            last_ego_speed_ms  = (now == 0) ? 1 : now;
            {
                char m[40];
                int l = sprintf(m, "OBD speed=%dkm/h\r\n", obd_win[7]);
                dbg_tx((uint8_t*)m, l, 100);
            }
            obd_waiting = 0;
            last_obd_poll = now;
            proto_send_obd();
            break;   /* 남은 바이트는 다음 요청 때 버려짐 */
        }
    }

    /* 2) 응답 대기 시간 초과 처리 (기다리지 않고 타이머로만 판단) */
    if (obd_waiting && (now - obd_req_ms) > OBD_RESP_TIMEOUT)
    {
        obd_waiting = 0;
        last_obd_poll = now;
        if (obd_fail_count < 255) obd_fail_count++;
        if (obd_fail_count == 3)
        {
            char m[] = "OBD no response x3 -> slow polling (3s)\r\n";
            dbg_tx((uint8_t*)m, strlen(m), 100);
        }
        proto_send_obd();
    }

    /* 3) 다음 요청 시점: 정상 300ms 주기, 응답이 3번 연속 없으면 3초 주기 */
    if (!obd_waiting)
    {
        uint32_t interval = (obd_fail_count >= 3) ? 3000 : 300;
        if ((now - last_obd_poll) >= interval)
        {
            if (!obd_start_request(now)) last_obd_poll = now;   /* 송신 시작 실패 시 다음 주기에 재시도 */
        }
    }
}

/* ===== Pi5 -> STM32 프레임 수신 (USART1 RX, 인터럽트 1바이트씩) =====
   프레임: STX(02) TYPE LEN PAYLOAD XOR(TYPE^LEN^PAYLOAD) ETX(03)
   지금은 0x22 EGO_SPEED(LEN=5)만 해석하고, 다른 타입은 검증만 하고 버린다. 기다리지 않는다. */
/* 상태기계 한 단계. 반환: 0=정상 진행, 1=프레임 오류(호출자가 STX 다음 바이트부터 다시 훑는다: 규격 §4 규칙 2) */
static uint8_t rpi_rx_st = 0, rpi_rx_type = 0, rpi_rx_len = 0, rpi_rx_idx = 0, rpi_rx_chk = 0;
static uint8_t rpi_rx_raw[PROTO_MAX_PAYLOAD + 5];
static uint8_t rpi_rx_rawn = 0;

static uint8_t rpi_rx_step(uint8_t b)
{
    if (rpi_rx_st == 0)
    {
        if (b == PROTO_STX) { rpi_rx_raw[0] = b; rpi_rx_rawn = 1; rpi_rx_st = 1; }
        return 0;
    }
    rpi_rx_raw[rpi_rx_rawn++] = b;
    switch (rpi_rx_st)
    {
    case 1: rpi_rx_type = b; rpi_rx_chk = b; rpi_rx_st = 2; return 0;
    case 2:
        rpi_rx_len = b; rpi_rx_chk ^= b; rpi_rx_idx = 0;
        if (b > PROTO_MAX_PAYLOAD) { rpi_rx_st = 0; return 1; }
        rpi_rx_st = (b == 0) ? 4 : 3;
        return 0;
    case 3:
        rpi_rx_chk ^= b;
        if (++rpi_rx_idx >= rpi_rx_len) rpi_rx_st = 4;
        return 0;
    case 4:
        if (b != rpi_rx_chk) { rpi_rx_st = 0; return 1; }
        rpi_rx_st = 5;
        return 0;
    default: /* 5: ETX */
        rpi_rx_st = 0;
        if (b != PROTO_ETX) return 1;
        if (rpi_rx_type == 0x22 && rpi_rx_len == 5)
        {
            const uint8_t *pl = &rpi_rx_raw[3];
            gps_speed_x100 = (uint16_t)(pl[0] | (pl[1] << 8));
            gps_src        = pl[2];
            gps_age_ms     = (uint16_t)(pl[3] | (pl[4] << 8));
            gps_rx_ms      = (HAL_GetTick() == 0) ? 1 : HAL_GetTick();
        }
        rpi_rx_ok++;   /* 형식이 맞는 프레임 수 (타입 무관) */
        return 0;
    }
}

void rpi_rx_byte_isr(void)
{
    uint8_t buf[PROTO_MAX_PAYLOAD + 8];
    uint8_t n = 1, i = 0;
    buf[0] = rpi_rx_byte;
    rpi_rx_bytes++;
    while (i < n)
    {
        if (rpi_rx_step(buf[i++]))
        {
            /* 오류: 실패한 프레임의 STX 다음 바이트들 + 아직 처리 안 한 바이트를 처음부터 다시 훑는다 */
            rpi_rx_bad++;
            uint8_t tmp[PROTO_MAX_PAYLOAD + 8];
            uint8_t m = 0;
            for (uint8_t k = 1; k < rpi_rx_rawn && m < sizeof(tmp); k++) tmp[m++] = rpi_rx_raw[k];
            for (uint8_t k = i;     k < n         && m < sizeof(tmp); k++) tmp[m++] = buf[k];
            for (uint8_t k = 0; k < m; k++) buf[k] = tmp[k];
            n = m; i = 0;
        }
    }
    HAL_UART_Receive_IT(&huart1, &rpi_rx_byte, 1);   /* 다음 1바이트 */
}

/* 자차 속도 선택(표시용): OBD가 신선하면 OBD, 아니면 Pi5가 준 GPS 속도. 없으면 0 반환.
   kmh10 = km/h x10 (예: 123 = 12.3km/h) */
uint8_t get_ego_speed(uint16_t *kmh10, uint8_t *src)
{
    uint32_t now = HAL_GetTick();
    if (last_ego_speed_ms != 0 && (now - last_ego_speed_ms) < 1000)
    {
        *kmh10 = (uint16_t)last_ego_speed_kmh * 10;
        *src = 0;
        return 1;
    }
    uint16_t sp, age; uint8_t s; uint32_t rx;
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    sp = gps_speed_x100; s = gps_src; age = gps_age_ms; rx = gps_rx_ms;
    __set_PRIMASK(pm);
    if (rx != 0 && s != 0xFF && s != 0 && age != 0xFFFF && ((now - rx) + age) < GPS_STALE_MS)
    {
        *kmh10 = (uint16_t)(sp / 10);
        *src = 1;
        return 1;
    }
    return 0;
}

/* 1초마다 Pi5 수신 상황을 디버그 터미널에 출력 (배선/전송이 되는지 눈으로 확인용) */
void gps_dbg_task(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 1000) return;
    last = now;
    uint16_t sp10; uint8_t src;
    char m[100];
    int l;
    if (get_ego_speed(&sp10, &src))
        l = sprintf(m, "RPIRX bytes=%lu ok=%lu bad=%lu | SPD=%d.%d src=%s\r\n",
                    (unsigned long)rpi_rx_bytes, (unsigned long)rpi_rx_ok, (unsigned long)rpi_rx_bad,
                    sp10 / 10, sp10 % 10, src == 0 ? "OBD" : "GPS");
    else
        l = sprintf(m, "RPIRX bytes=%lu ok=%lu bad=%lu | SPD=none\r\n",
                    (unsigned long)rpi_rx_bytes, (unsigned long)rpi_rx_ok, (unsigned long)rpi_rx_bad);
    dbg_tx((uint8_t*)m, l, 100);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  /* [LED 진단] 초기화 실패로 여기 오면 LD2가 빠르게 깜빡임 */
  {
    GPIO_InitTypeDef led_gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    led_gpio.Pin = GPIO_PIN_5;
    led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    led_gpio.Pull = GPIO_NOPULL;
    led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &led_gpio);
  }

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    for (volatile uint32_t i = 0; i < 1500000; i++) { }
  }

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
