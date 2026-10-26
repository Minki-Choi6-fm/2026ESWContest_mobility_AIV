/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 전방차 STM32 (NUCLEO-F446RE) - IMU/OBD -> RPi4 이진 프레임 송신
  *
  *  후방차 v10 펌웨어에서 레이더(MCP2515/SPI)만 뺀 버전.
  *  - IMU  : USART3 (PC10/PC11) 115200, imu_uart.c 파서
  *  - OBD  : USART6 (PC6/PC7) 9600 (혼다는 응답 없음 -> 3초 주기 저속 폴링, 속도는 GPS 폴백 예정)
  *  - RPi4 : USART1 (PA9/PA10) 115200, 프레임 규격 v0.2 (0x10/0x11/0x13/0x14)
  *  - OLED : I2C1 (PB8 SCL / PB9 SDA), 주소 0x3C
  *  - 디버그: USART2 (ST-Link 가상 COM) 115200
  *  - 워치독 IWDG 약 40ms, 메인 루프 논블로킹
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
#include "imu_uart.h"
#include "ssd1306.h"    /* OLED (I2C1, 주소 0x3C) */
#include "ssd1306_fonts.h"
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* IMU 값 출력 주기(200ms)용 */
uint32_t last_imu_print_ms = 0;

/* OBD 자차 속도 (혼다는 OBD 미동작 -> 대부분 "값 없음") */
uint8_t  last_ego_speed_kmh = 0;
uint32_t last_ego_speed_ms  = 0;
uint32_t last_obd_poll = 0;
uint8_t  obd_fail_count = 0;
uint8_t  reset_cause = 2;              /* 0=전원 켬, 1=워치독 리셋, 2=그 외 (HEARTBEAT용) */
uint8_t  oled_ok = 0;                  /* I2C 스캔에서 0x3C가 잡히면 1 */

/* 워치독(IWDG): 메인 루프가 40ms 넘게 멈추면 강제 리셋 (팀 규칙 6: 40ms 기준 임의 변경 금지) */
#define ENABLE_IWDG       1
#define WDG_TEST_HANG_MS  0     /* 0=끔. 예: 15000 이면 부팅 15초 뒤 일부러 멈춰 워치독 시험 */
uint32_t last_oled_ms = 0;
uint32_t oled_retry_ms = 0;           /* [front v4b] OLED 가 꺼져 있을 때 다음 복구 시도 시각 */
uint32_t oled_fail_total = 0;         /* [front v4b] OLED 통신 실패로 OLED 를 끈 누적 횟수 */
uint32_t oled_retry_total = 0;        /* [front v4b] 복구 시도 누적 횟수 */
uint32_t oled_retry_ok_total = 0;     /* [front v4b] 복구 성공 누적 횟수 */
uint32_t oled_push_total = 0;         /* [front v4b] 화면 한 장(전 페이지) 전송 완료 누적 횟수 */
uint32_t oled_stat_ms = 0;            /* [front v4b] OLEDSTAT 출력 시각 */
static uint8_t oled_page = 0xFF;      /* 화면 전송 진행: 0~(페이지수-1)=전송 중, 그 이상=대기 */
#define OLED_PAGES (SSD1306_HEIGHT / 8)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void obd_init(void);
void obd_poll_task(void);
void obd_rx_start(void);
void obd_rx_byte_isr(uint32_t now);
void oled_update(void);
void proto_task(void);
void dbg_tx(const uint8_t *p, uint16_t len, uint32_t unused);
void oled_push_step(void);
void ssd1306_UpdatePage(uint8_t page);   /* ssd1306.c에 추가한 함수 */
/* [front v4b] OLED I2C 보호/복구 (ssd1306.c 에 추가한 것들) */
void ssd1306_InitFast(void);
void ssd1306_ClearI2cError(void);
extern volatile uint8_t  ssd1306_i2c_err;
extern volatile uint32_t ssd1306_i2c_err_cnt;
extern volatile uint32_t ssd1306_i2c_hal_err;
extern volatile uint8_t  ssd1306_i2c_hal_st;
static void oled_i2c_bus_recover(void);
static void oled_try_recover(void);
void proto_send_obd(void);
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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* [LED 진단] LD2(PA5) 켬 -> 여기까지 도달했다는 표시 */
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

  /* 디버그 출력(USART2)/RPi 링크(USART1) 송신은 인터럽트 큐 방식: 출력 때문에 메인 루프가 멈추지 않게 함 */
  HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* 리셋 원인 출력 (워치독 리셋이면 IWDG로 표시됨) */
  {
    char rc[40];
    int rl = sprintf(rc, "RESET CAUSE: %s\r\n", reset_cause == 1 ? "IWDG (watchdog)" : (reset_cause == 0 ? "power-on" : "other/pin"));
    dbg_tx((uint8_t*)rc, rl, 0);
  }
  { char fw[] = "FW front v4b (OLED I2C guard + diag)\r\n"; dbg_tx((uint8_t*)fw, strlen(fw), 0); }

  { char boot1[] = "BOOT1 (FRONT)\r\n"; dbg_tx((uint8_t*)boot1, strlen(boot1), 100); }

  /* 전원 안정화 대기 (디스플레이 부팅 시간 확보) */
  HAL_Delay(2000);

  /* I2C1 버스 주소 스캔 - OLED(0x3C)가 응답하는지 확인 */
  {
    char scan_msg[64];
    int scan_len;
    uint8_t found_any = 0;

    /* [front v4b] 스캔 전에 I2C 버스를 풀어 준다: OLED 가 이전 재시작 때 통신 중간에 멈춰 SDA 를 붙잡고 있어도 풀림 */
    oled_i2c_bus_recover();

    char start_msg[] = "\r\n--- I2C1 Scan Start ---\r\n";
    dbg_tx((uint8_t*)start_msg, strlen(start_msg), 100);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 10) == HAL_OK)
      {
        scan_len = sprintf(scan_msg, "Found I2C device at 0x%02X\r\n", addr);
        dbg_tx((uint8_t*)scan_msg, scan_len, 100);
        found_any = 1;
        if (addr == 0x3C) oled_ok = 1;
      }
    }
    if (!found_any)
    {
      char none_msg[] = "No I2C device found on I2C1. Check wiring (SCL=PB8, SDA=PB9).\r\n";
      dbg_tx((uint8_t*)none_msg, strlen(none_msg), 100);
    }
    char end_msg[] = "--- I2C1 Scan End ---\r\n\r\n";
    dbg_tx((uint8_t*)end_msg, strlen(end_msg), 100);
  }

  /* OLED 초기화 + 부팅 화면 (0x3C가 잡혔을 때만) */
  if (oled_ok)
  {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("AIV FRONT", Font_11x18, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("booting...", Font_7x10, White);
    ssd1306_UpdateScreen();
    if (ssd1306_i2c_err)   /* [front v4b] 초기화 중 통신 실패 -> OLED 만 끄고 계속 진행 (주기적으로 복구 시도) */
    {
      oled_ok = 0;
      oled_fail_total++;
      char f[48]; int fl = sprintf(f, "OLED I2C FAIL at boot err=%u\r\n", (unsigned)ssd1306_i2c_err);
      dbg_tx((uint8_t*)f, fl, 0);
    }
  }
  oled_retry_ms = HAL_GetTick() + 3000;   /* [front v4b] OLED 가 꺼져 있으면 3초 뒤부터 복구 시도 */

  /* IMU 초기화 - USART3(huart3, PC10/PC11), 인터럽트 기반 1바이트씩 수신 (모듈 TX -> PC11) */
  IMU_UART_Init(&huart3);
  { char m[] = "IMU UART init done\r\n"; dbg_tx((uint8_t*)m, strlen(m), 100); }

  /* OBD(Seeed Serial CAN-BUS 모듈) 초기화 - USART6(PC6/PC7) */
  obd_init();
  HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  obd_rx_start();
  { char m[] = "OBD-II init done\r\n"; dbg_tx((uint8_t*)m, strlen(m), 100); }

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

    /* [front v4b] 5초마다 OLED 상태 한 줄 (조용히 실패하는 일이 없도록) */
    if (HAL_GetTick() - oled_stat_ms >= 5000)
    {
        oled_stat_ms = HAL_GetTick();
        char st[160];
        int sl = sprintf(st, "OLEDSTAT ok=%u pushes=%lu fails=%lu retries=%lu retry_ok=%lu err=%u hal_err=0x%lX t=%lu\r\n",
                         (unsigned)oled_ok, (unsigned long)oled_push_total, (unsigned long)oled_fail_total,
                         (unsigned long)oled_retry_total, (unsigned long)oled_retry_ok_total,
                         (unsigned)ssd1306_i2c_err, (unsigned long)ssd1306_i2c_hal_err, (unsigned long)HAL_GetTick());
        dbg_tx((uint8_t*)st, sl, 0);
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

      /* [front v4b] OLED 통신 실패 -> OLED 만 끄고 STM32 는 계속 동작 */
      if (ssd1306_i2c_err)
      {
        oled_ok = 0;
        oled_page = 0xFF;
        oled_fail_total++;
        oled_retry_ms = HAL_GetTick() + 1000;
        char f[96];
        int fl = sprintf(f, "OLED I2C FAIL err=%u hal=%u i2cerr=0x%lX cnt=%lu t=%lu\r\n",
                         (unsigned)ssd1306_i2c_err, (unsigned)ssd1306_i2c_hal_st,
                         (unsigned long)ssd1306_i2c_hal_err, (unsigned long)ssd1306_i2c_err_cnt,
                         (unsigned long)HAL_GetTick());
        dbg_tx((uint8_t*)f, fl, 0);
      }
    }
    else if (HAL_GetTick() >= oled_retry_ms)
    {
      oled_try_recover();   /* [front v4b] 꺼진 OLED 를 3초마다 한 번씩 되살려 본다 (1~수 ms) */
    }

    /* OBD-II 속도 폴링 (논블로킹) */
    obd_poll_task();


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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART2 Initialization Function
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
}

/* ===== OLED 표시 =====
   128x64 화면, 6x8 폰트 -> 한 줄 21글자, 7줄 표시.
   값 x10 정수를 "-12.3" 형태 문자열로 만드는 도우미 (nano.specs가 %f 미지원이라 정수로 처리) */
static void fmt10(char *out, int v10)
{
    int a = (v10 < 0) ? -v10 : v10;
    sprintf(out, "%s%d.%d", (v10 < 0 ? "-" : ""), a / 10, a % 10);
}

/* ===== [front v4b] OLED I2C 버스 복구 / 재시도 (후방차 v18c 와 같은 방식, I2C1: SCL=PB8, SDA=PB9) =====
   I2C 통신 도중 STM32 가 재시작되거나 노이즈가 끼면 OLED 가 SDA 선을 Low 로 붙잡은 채 멈출 수 있다.
   SCL 을 최대 9번 흔들어 OLED 가 하던 바이트를 끝내게 하고 STOP 조건을 보내면 풀린다. 소요 약 0.2 ms 이하. */
static void oled_us_delay(uint32_t us)
{
    uint32_t n = (SystemCoreClock / 1000000U) * us / 8U + 1U;
    for (volatile uint32_t i = 0; i < n; i++) { }
}

static void oled_i2c_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOB, &g);    /* SCL */
    g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOB, &g);    /* SDA */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    oled_us_delay(10);
    for (uint8_t i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); oled_us_delay(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   oled_us_delay(5);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); oled_us_delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   oled_us_delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   oled_us_delay(5);
    HAL_I2C_Init(&hi2c1);
}

static void oled_try_recover(void)
{
    char r[96];
    int rl;
    oled_retry_ms = HAL_GetTick() + 3000;
    oled_retry_total++;
    oled_i2c_bus_recover();
    ssd1306_ClearI2cError();
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY))
    {
        rl = sprintf(r, "OLED RETRY #%lu: bus BUSY (SDA/SCL stuck) t=%lu\r\n", (unsigned long)oled_retry_total, (unsigned long)HAL_GetTick());
        dbg_tx((uint8_t*)r, rl, 0);
        return;
    }
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x3C << 1), 1, 3) != HAL_OK)
    {
        rl = sprintf(r, "OLED RETRY #%lu: no ACK from 0x3C t=%lu\r\n", (unsigned long)oled_retry_total, (unsigned long)HAL_GetTick());
        dbg_tx((uint8_t*)r, rl, 0);
        return;
    }
    ssd1306_InitFast();
    if (ssd1306_i2c_err)
    {
        rl = sprintf(r, "OLED RETRY #%lu: init write failed err=%u t=%lu\r\n", (unsigned long)oled_retry_total, (unsigned)ssd1306_i2c_err, (unsigned long)HAL_GetTick());
        dbg_tx((uint8_t*)r, rl, 0);
        ssd1306_ClearI2cError();
        return;
    }
    oled_ok = 1;
    oled_page = 0xFF;
    last_oled_ms = 0;
    oled_retry_ok_total++;
    rl = sprintf(r, "OLED RECOVERED t=%lu fails=%lu retries=%lu\r\n", (unsigned long)HAL_GetTick(), (unsigned long)oled_fail_total, (unsigned long)oled_retry_total);
    dbg_tx((uint8_t*)r, rl, 0);
}

/* 화면 버퍼의 한 페이지(128바이트)를 전송: 호출될 때마다 다음 페이지로 넘어간다 */
void oled_push_step(void)
{
    if (oled_page < OLED_PAGES)
    {
        ssd1306_UpdatePage(oled_page);
        oled_page++;
        if (oled_page >= OLED_PAGES && !ssd1306_i2c_err) oled_push_total++;   /* [front v4b] 한 장 전송 완료 */
    }
}

extern volatile uint32_t tx_drop_count;
static uint8_t imu_is_ok(uint32_t now);

void oled_update(void)
{
    char line[48];
    char a[16], b[16], c[16];
    uint32_t now = HAL_GetTick();

    ssd1306_Fill(Black);

    sprintf(line, "AIV FRONT  %s", reset_cause == 1 ? "WDG-RST" : "");
    ssd1306_SetCursor(0, 0);  ssd1306_WriteString(line, Font_6x8, White);

    sprintf(line, "IMU: %s", imu_is_ok(now) ? "OK" : "NO DATA");
    ssd1306_SetCursor(0, 9);  ssd1306_WriteString(line, Font_6x8, White);

    if (last_ego_speed_ms != 0 && (now - last_ego_speed_ms) < 1000)
        sprintf(line, "OBD SPD: %dkm/h", last_ego_speed_kmh);
    else
        sprintf(line, "OBD SPD: --- (none)");
    ssd1306_SetCursor(0, 18); ssd1306_WriteString(line, Font_6x8, White);

    sprintf(line, "RPi TX drop: %lu", (unsigned long)tx_drop_count);
    ssd1306_SetCursor(0, 27); ssd1306_WriteString(line, Font_6x8, White);

    if (g_imu.angle_updated)
    {
        fmt10(a, (int)(g_imu.roll  * 10.0f));
        fmt10(b, (int)(g_imu.pitch * 10.0f));
        fmt10(c, (int)(g_imu.yaw   * 10.0f));
        sprintf(line, "R:%s P:%s", a, b);
        ssd1306_SetCursor(0, 36); ssd1306_WriteString(line, Font_6x8, White);
        sprintf(line, "YAW:%s", c);
        ssd1306_SetCursor(0, 45); ssd1306_WriteString(line, Font_6x8, White);
    }
    else
    {
        ssd1306_SetCursor(0, 36); ssd1306_WriteString("IMU: waiting...", Font_6x8, White);
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
#define MSG_RADAR_OBJECT   0x12   /* 전방차는 레이더 없음: 사용 안 함 */
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
        tx_tail = (uint16_t)((tx_tail + tx_chunk) % TX_BUF_SIZE);
        tx_busy = 0;
        tx_chunk = 0;
        uart_tx_kick();
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

/* 전방차: 레이더/CAN 컨트롤러 없음 -> RADAR_OK, CAN_CTRL_OK 비트는 항상 0.
   IMU가 끊기면 SAFE_MODE. */
static uint8_t build_sensor_status(uint32_t now)
{
    uint8_t st = 0;
    uint8_t imu_ok = imu_is_ok(now);
    if (imu_ok)         st |= ST_IMU_OK;
    if (obd_is_ok(now)) st |= ST_OBD_OK;
    if (!imu_ok)        st |= ST_SAFE_MODE;
    if (brake_active)   st |= ST_BRAKE;
    return st;
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
        uint16_t tgt = 0xFFFF;             /* 전방차: 레이더 없음 -> 목표거리 없음 */
        int16_t  rel = 0x7FFF;
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

/* ===== OBD(Seeed Serial CAN-BUS 모듈) 관련 =====
   ELM327 AT 명령어가 아니라, 14바이트 전송/12바이트 수신의 바이너리 CAN 프레임 방식.
   USART6(huart6, PC6/PC7), 기본 baud 9600. */

/* 차량 없이도 모듈 자체가 살아있는지 확인하는 핑 테스트 (+++ -> OK 응답 기대).
   실차 연결 전이면 응답이 없을 수 있는데, 그건 정상이라 초기화 자체를 막지는 않는다. */
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
   제조사 문서의 표기가 모호해서(대괄호가 글자인지 불명), 대괄호 없는 형식을 먼저 시도하고 ERROR면 대괄호 형식을 시도한다.
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
    uint8_t bracket = 0;

    sprintf(cmd, "AT+M=%d%d%08X\n", 0, 0, 0x7F8);
    if (!obd_cfg_cmd(cmd))
    {
        sprintf(cmd, "AT+M=[%d][%d][%08X]\n", 0, 0, 0x7F8);
        if (!obd_cfg_cmd(cmd)) return;       /* 두 형식 모두 실패: 필터 없이 계속 */
        bracket = 1;
    }
    for (int n = 1; n <= 1; n++)
    {
        sprintf(cmd, bracket ? "AT+M=[%d][%d][%08X]\n" : "AT+M=%d%d%08X\n", n, 0, 0x7F8);
        obd_cfg_cmd(cmd);
    }
    for (int n = 0; n <= 5; n++)
    {
        sprintf(cmd, bracket ? "AT+F=[%d][%d][%08X]\n" : "AT+F=%d%d%08X\n", n, 0, 0x7E8);
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
  while (1)
  {
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
