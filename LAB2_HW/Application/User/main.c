/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team (edited)
  * @brief   Main program functions (ACC basic / QA = SMD→EXTI→TCP)
  ******************************************************************************
  */

#define basic  0
#define QA     1   /* option problem_A: Wake-Up(SMD) → EXTI → TCP */
#define Question QA   /* ← 換成 basic / QA 來切燒錄的 code */

#include "main.h"
#include <string.h>

#define TERMINAL_USE
#if defined (TERMINAL_USE)
#include <stdio.h>
#endif

/* ================== Wi-Fi ================== */
#define SSID       "Eddy"
#define PASSWORD   "11111111"
static uint8_t RemoteIP[] = {172,20,10,12};
#define RemotePORT  8002

#define WIFI_WRITE_TIMEOUT   10000
#define WIFI_READ_TIMEOUT    10000
#define CONNECTION_TRIAL_MAX 10

/* ================== LSM6DSL ================== */
#define LSM6DSL_I2C_ADDR_7BIT   (0x6A)
#define LSM6DSL_I2C_ADDR_8BIT   (LSM6DSL_I2C_ADDR_7BIT << 1)

#define LSM6DSL_WHO_AM_I        0x0F
#define LSM6DSL_WAKE_UP_SRC     0x1B
#define LSM6DSL_ALL_INT_SRC     0x1A

#define LSM6DSL_CTRL1_XL        0x10
#define LSM6DSL_TAP_CFG         0x58
#define LSM6DSL_WAKE_UP_THS     0x5B
#define LSM6DSL_WAKE_UP_DUR     0x5C
#define LSM6DSL_MD1_CFG         0x5E

/* Gyro (optional) */
#define LSM6DSL_CTRL2_G         0x11
#define LSM6DSL_OUTX_L_G        0x22
#define LSM6DSL_OUTX_H_G        0x23
#define LSM6DSL_OUTY_L_G        0x24
#define LSM6DSL_OUTY_H_G        0x25
#define LSM6DSL_OUTZ_L_G        0x26
#define LSM6DSL_OUTZ_H_G        0x27

/* ===== INT1 pin（IoT01A = PD11）===== */
#ifndef LSM6DSL_INT1_GPIO_PORT
#define LSM6DSL_INT1_GPIO_PORT          GPIOD
#define LSM6DSL_INT1_GPIO_PIN           GPIO_PIN_11
#define LSM6DSL_INT1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#endif

/* BSP handles */
extern SPI_HandleTypeDef hspi;
extern I2C_HandleTypeDef hI2cHandler;
#define SENSOR_I2C_HANDLE hI2cHandler

#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* ================== Private ================== */
static uint8_t RxData[500];
static volatile uint8_t btn_pressed = 0;
#if (Question==QA)
static volatile uint8_t smd_event   = 0;
#endif

/* Prototypes */
#if defined (TERMINAL_USE)
#ifdef __GNUC__
int __io_putchar(int ch);
#else
int fputc(int ch, FILE *f);
#endif
#endif

static void SystemClock_Config(void);
static int32_t ConnectToServer(void);

/* I2C helpers */
static int32_t LSM6DSL_ReadReg(uint8_t reg, uint8_t *pdata);
static int32_t LSM6DSL_WriteReg(uint8_t reg, uint8_t value);

/* Gyro (optional) */
static int32_t LSM6DSL_Gyro_Init(void);
static int32_t LSM6DSL_Gyro_GetXYZ(float *gx, float *gy, float *gz);

#if (Question==QA)
static int32_t LSM6DSL_Wakeup_Init(void);
#endif

/* ================== TCP connect wrapper ================== */
static int32_t ConnectToServer(void)
{
  int16_t trials = CONNECTION_TRIAL_MAX;

  TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\r\n",
          RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePORT);

  while (trials--)
  {
    if (WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT",
                                  RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
    {
      TERMOUT("> TCP Connection opened successfully.\r\n");
      return 0;   /* socket id = 0 */
    }
    HAL_Delay(500);
  }

  TERMOUT("> ERROR : Cannot open Connection\r\n");
  return -1;
}

/* ================== Main ================== */
int main(void)
{
  uint8_t  MAC_Addr[6] = {0};
  uint8_t  IP_Addr[4]  = {0};
  uint8_t  TxData[]    = "STM32 : Panda!";
  int32_t  Socket      = -1;
  int32_t  ret;

  HAL_Init();
  SystemClock_Config();

  BSP_LED_Init(LED2);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

#if defined (TERMINAL_USE)
  hDiscoUart.Instance                    = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate               = 115200;
  hDiscoUart.Init.WordLength             = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits               = UART_STOPBITS_1;
  hDiscoUart.Init.Parity                 = UART_PARITY_NONE;
  hDiscoUart.Init.Mode                   = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling           = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  BSP_COM_Init(COM1, &hDiscoUart);
#endif

  TERMOUT("\r\n****** WIFI TCP Client + ACC Demo ******\r\n");

  /* ===== Sensors ===== */
#if (Question==basic)
  if (BSP_ACCELERO_Init() != 0) {
    TERMOUT("> ERROR : ACCELERO init failed\r\n");
  } else {
    TERMOUT("> ACCELERO init OK (basic mode)\r\n");
  }
  if (LSM6DSL_Gyro_Init() != 0) {
    TERMOUT("> WARN : Gyro init failed\r\n");
  } else {
    TERMOUT("> Gyro init OK (26Hz, 245 dps)\r\n");
  }
#elif (Question==QA)
  TERMOUT("> Init LSM6DSL for Significant Motion (Wake-Up -> INT1)...\r\n");
  if (BSP_ACCELERO_Init() != 0) {
    TERMOUT("> ERROR : ACCELERO basic init failed\r\n");
  } else if (LSM6DSL_Wakeup_Init() != 0) {
    TERMOUT("> ERROR : LSM6DSL wake-up init failed\r\n");
  } else {
    TERMOUT("> LSM6DSL Wake-Up enabled (INT1 -> EXTI)\r\n");

    /* ★ 這裡插：先讀 WHO_AM_I 再做 EXTI 設定 */
    uint8_t v = 0;
    if (LSM6DSL_ReadReg(LSM6DSL_WHO_AM_I, &v) == 0) {
      TERMOUT("WHO_AM_I=0x%02X (expect 0x6A)\r\n", v);
    } else {
      TERMOUT("> ERROR : WHO_AM_I read failed\r\n");
    }

    /* --- EXTI for LSM6DSL INT1 (PD11) --- */
    LSM6DSL_INT1_GPIO_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = LSM6DSL_INT1_GPIO_PIN;   /* PD11 */
    gi.Mode  = GPIO_MODE_IT_RISING;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LSM6DSL_INT1_GPIO_PORT, &gi);

    __HAL_GPIO_EXTI_CLEAR_IT(LSM6DSL_INT1_GPIO_PIN);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
#endif /* Question==QA */

  /* ===== WiFi ===== */
  if (WIFI_Init() == WIFI_STATUS_OK)
  {
    TERMOUT("> WIFI Module Initialized.\r\n");

    if (WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK) {
      TERMOUT("> es-wifi MAC : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
              MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
              MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
    }

    if (WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi connected\r\n");

      if (WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK) {
        TERMOUT("> es-wifi IP : %d.%d.%d.%d\r\n",
                IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
        Socket = ConnectToServer();
      } else {
        TERMOUT("> ERROR : cannot get IP address\r\n");
        BSP_LED_On(LED2);
      }
    }
    else
    {
      TERMOUT("> ERROR : es-wifi NOT connected\r\n");
      BSP_LED_On(LED2);
    }
  }
  else
  {
    TERMOUT("> ERROR : WIFI Module cannot be initialized.\r\n");
  }

  /* ===== Loop ===== */
  while (1)
  {
    if (Socket == -1) {
      HAL_Delay(1000);
      Socket = ConnectToServer();
      continue;
    }

#if (Question==basic)
    if (btn_pressed) {
      btn_pressed = 0;

      uint16_t slen = 0;
      if (WIFI_SendData(Socket, TxData, strlen((char*)TxData),
                        &slen, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {
        TERMOUT("> WARN : Send failed, will reconnect\r\n");
        WIFI_CloseClientConnection(0);
        Socket = -1;
        continue;
      } else {
        TERMOUT("TX (%u): %s\r\n", slen, TxData);
      }

      int16_t pDataXYZ[3];
      BSP_ACCELERO_AccGetXYZ(pDataXYZ);

      float gxf=0, gyf=0, gzf=0;
      int gyro_ok = (LSM6DSL_Gyro_GetXYZ(&gxf, &gyf, &gzf) == 0);

      char line2[160];
      if (gyro_ok) {
        snprintf(line2, sizeof(line2),
                 "ACC XYZ = [%d, %d, %d]  GYRO dps = [%.2f, %.2f, %.2f]\n",
                 pDataXYZ[0], pDataXYZ[1], pDataXYZ[2], gxf, gyf, gzf);
      } else {
        snprintf(line2, sizeof(line2),
                 "ACC XYZ = [%d, %d, %d]  GYRO dps = [NA, NA, NA]\n",
                 pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
      }

      uint16_t slen2 = 0;
      if (WIFI_SendData(Socket, (uint8_t*)line2, (uint16_t)strlen(line2),
                        &slen2, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {
        TERMOUT("> WARN : ACC+GYRO send failed, will reconnect\r\n");
        WIFI_CloseClientConnection(0);
        Socket = -1;
        continue;
      } else {
        TERMOUT("%s", line2);
      }
    }

#elif (Question==QA)
    if (smd_event) {
      smd_event = 0;
      const char *evt = "EVENT: SMD\n";
      uint16_t sl;
      if (WIFI_SendData(Socket, (uint8_t*)evt, strlen(evt),
                        &sl, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {
        TERMOUT("> WARN : SMD send failed, will reconnect\r\n");
        WIFI_CloseClientConnection(0);
        Socket = -1;
      } else {
        TERMOUT("SMD event sent.\r\n");
      }
    }
#endif

    uint16_t rlen = 0;
    ret = WIFI_ReceiveData(Socket, RxData, sizeof(RxData) - 1, &rlen, 200);

    if (ret == WIFI_STATUS_OK && rlen > 0) {
      RxData[rlen] = 0;
      TERMOUT("Received: %s\r\n", RxData);

      const char *reply = "STM32 : Hello!\n";
      uint16_t slen = 0;
      (void)WIFI_SendData(Socket, (uint8_t*)reply, strlen(reply),
                          &slen, WIFI_WRITE_TIMEOUT);
    }
    else if (ret != WIFI_STATUS_OK && ret != WIFI_STATUS_TIMEOUT) {
      TERMOUT("> WARN : Receive err (%ld), reconnect\r\n", (long)ret);
      WIFI_CloseClientConnection(0);
      Socket = -1;
    }

    HAL_Delay(10);
  }
}

/* ================== Clock ================== */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 1;
  RCC_OscInitStruct.PLL.PLLN            = 40;  /* 4 MHz * 40 / 2 = 80 MHz */
  RCC_OscInitStruct.PLL.PLLR            = 2;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1);
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    while(1);
  }
}

#if defined (TERMINAL_USE)
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif

/* ================== IRQ / Callback ================== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_1:
      SPI_WIFI_ISR();      /* es-wifi INT */
      break;

    case GPIO_PIN_13:
      btn_pressed = 1;     /* USER button */
      break;

#if (Question==QA)
    case LSM6DSL_INT1_GPIO_PIN:  /* PD11 */
      smd_event = 1;
      BSP_LED_Toggle(LED2);
      break;
#endif

    default:
      break;
  }
}

/* ================== LSM6DSL I2C Helpers ================== */
static int32_t LSM6DSL_ReadReg(uint8_t reg, uint8_t *pdata)
{
  if (HAL_I2C_Mem_Read(&SENSOR_I2C_HANDLE,
                       LSM6DSL_I2C_ADDR_8BIT,
                       reg, I2C_MEMADD_SIZE_8BIT,
                       pdata, 1, 100) != HAL_OK) {
    return -1;
  }
  return 0;
}

static int32_t LSM6DSL_WriteReg(uint8_t reg, uint8_t value)
{
  if (HAL_I2C_Mem_Write(&SENSOR_I2C_HANDLE,
                        LSM6DSL_I2C_ADDR_8BIT,
                        reg, I2C_MEMADD_SIZE_8BIT,
                        &value, 1, 100) != HAL_OK) {
    return -1;
  }
  return 0;
}

/* Gyro init: ODR=26Hz, FS=245 dps */
static int32_t LSM6DSL_Gyro_Init(void)
{
  return LSM6DSL_WriteReg(LSM6DSL_CTRL2_G, 0x20);
}

static int32_t LSM6DSL_Gyro_GetXYZ(float *gx, float *gy, float *gz)
{
  uint8_t lo, hi;
  int16_t rx, ry, rz;

  if (LSM6DSL_ReadReg(LSM6DSL_OUTX_L_G, &lo) != 0) return -1;
  if (LSM6DSL_ReadReg(LSM6DSL_OUTX_H_G, &hi) != 0) return -1;
  rx = (int16_t)((hi << 8) | lo);

  if (LSM6DSL_ReadReg(LSM6DSL_OUTY_L_G, &lo) != 0) return -1;
  if (LSM6DSL_ReadReg(LSM6DSL_OUTY_H_G, &hi) != 0) return -1;
  ry = (int16_t)((hi << 8) | lo);

  if (LSM6DSL_ReadReg(LSM6DSL_OUTZ_L_G, &lo) != 0) return -1;
  if (LSM6DSL_ReadReg(LSM6DSL_OUTZ_H_G, &hi) != 0) return -1;
  rz = (int16_t)((hi << 8) | lo);

  const float SENS = 0.00875f; /* 245 dps */
  *gx = rx * SENS; *gy = ry * SENS; *gz = rz * SENS;
  return 0;
}

#if (Question==QA)
/* Wake-Up(SMD) init：ODR=26Hz, FS=±2g, 開中斷路徑、門檻、路由到 INT1 */
static int32_t LSM6DSL_Wakeup_Init(void)
{
  if (LSM6DSL_WriteReg(LSM6DSL_CTRL1_XL, 0x20) != 0) return -1; /* 26Hz, ±2g */
  if (LSM6DSL_WriteReg(LSM6DSL_TAP_CFG,  0x20) != 0) return -2; /* INT path */
  if (LSM6DSL_WriteReg(LSM6DSL_WAKE_UP_THS, 0x02) != 0) return -3; /* 門檻(可調) */
  if (LSM6DSL_WriteReg(LSM6DSL_WAKE_UP_DUR, 0x00) != 0) return -4;
  if (LSM6DSL_WriteReg(LSM6DSL_MD1_CFG,  0x20) != 0) return -5;  /* INT1_WU */
  return 0;
}
#endif
