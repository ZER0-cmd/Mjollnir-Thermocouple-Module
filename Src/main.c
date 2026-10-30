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
#include "MAX31856.h"
#include "aesir_protocol.h"
#include <stdbool.h>
#include <string.h>
#include <assert.h>
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

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
#define MODULE_I2C               hi2c1
#define MODULE_ID                0x66U /* Matched to the 0x45 address tested in your terminal */
#define MODULE_RECORD_BYTES      10U   /* 4 byte timestamp + 3 byte temp + 2 byte cold junction + 1 byte fault */
#define MODULE_BATCH_COUNT       10U   /* Readings per publication */
#define MODULE_PAYLOAD_BYTES     (MODULE_RECORD_BYTES * MODULE_BATCH_COUNT)
#define MODULE_FRAME_BYTES       (AESIR_HEADER_BYTES + MODULE_PAYLOAD_BYTES + AESIR_CRC_BYTES)

_Static_assert(MODULE_RECORD_BYTES > 0U && MODULE_BATCH_COUNT > 0U,
               "A batch must contain readings");
_Static_assert(MODULE_PAYLOAD_BYTES <= AESIR_MAX_PAYLOAD_BYTES,
               "Reduce batch size: payload must fit one frame");

static uint8_t module_payload[MODULE_PAYLOAD_BYTES];
static size_t module_count;
static uint16_t module_message_id;
static volatile uint8_t published[AESIR_MAX_FRAME_BYTES];
static volatile uint16_t published_length;
static uint8_t snapshot[AESIR_MAX_FRAME_BYTES];
static uint16_t snapshot_length;
static uint8_t selected_offset, rx_byte;
static bool expect_offset;
static uint8_t padding = 0xFFU;
static volatile uint32_t module_i2c_error;

/* Helper to store 16-bit values in Big-Endian format */
static void StoreU16BE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8U);
    p[1] = (uint8_t)(value & 0xFFU);
}

/* Helper to store 24-bit values in Big-Endian format */
static void StoreU24BE(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 16U);
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value & 0xFFU);
}

/* Helper to store 32-bit values in Big-Endian format */
static void StoreU32BE(uint8_t *p, uint32_t value)
{
    for (uint32_t i = 0; i < 4U; ++i) {
        p[i] = (uint8_t)(value >> (8U * (3U - i)));
    }
}

static int Module_PublishFrame(const uint8_t *frame, size_t length, void *ctx)
{
    (void)ctx;
    if (frame == NULL || length != MODULE_FRAME_BYTES) { return -1; }
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    for (size_t i = 0; i < length; ++i) { published[i] = frame[i]; }
    published_length = (uint16_t)length;
    __set_PRIMASK(saved_mask);
    return 0;
}

static aesir_status_t Module_AppendRecord(const uint8_t *record)
{
    if (record == NULL) { return AESIR_ERR_NULL; }
    memcpy(&module_payload[module_count * MODULE_RECORD_BYTES],
           record, MODULE_RECORD_BYTES);
    if (++module_count < MODULE_BATCH_COUNT) { return AESIR_STATUS_WAITING; }

    const aesir_message_t message = {
        .address = MODULE_ID, .type = AESIR_TYPE_DATA,
        .priority = AESIR_PRIORITY_SENSOR, .message_id = module_message_id,
        .payload = module_payload, .payload_length = MODULE_PAYLOAD_BYTES
    };
    const aesir_transport_t transport = {
        .write_frame = Module_PublishFrame, .context = NULL
    };
    aesir_status_t result = aesir_send(&transport, &message);
    module_count = 0U;
    if (result == AESIR_STATUS_MESSAGE_READY) { ++module_message_id; }
    return result;
}

static void Module_CheckI2C(HAL_StatusTypeDef result)
{
    if (result != HAL_OK) {
        module_i2c_error = HAL_I2C_GetError(&MODULE_I2C);
        if (module_i2c_error == HAL_I2C_ERROR_NONE) {
            module_i2c_error = HAL_I2C_ERROR_INVALID_PARAM;
        }
    }
}

static void Module_ListenIfReady(I2C_HandleTypeDef *h)
{
    if (HAL_I2C_GetState(h) == HAL_I2C_STATE_READY) {
        Module_CheckI2C(HAL_I2C_EnableListen_IT(h));
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  TC_HandleTypeDef sensor;
  TC_Init(&sensor, &hspi1, SENS_CS_GPIO_Port, SENS_CS_Pin);

  Module_CheckI2C(HAL_I2C_EnableListen_IT(&MODULE_I2C));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int count = 0;
    while (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin)) {
        HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
        if (count++ > 200) {
            count = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);

    uint8_t record[MODULE_RECORD_BYTES];

    // Bytes 0-3: Timestamp (milliseconds since startup)
    StoreU32BE(&record[0], HAL_GetTick());

    // Bytes 4-6: Temperature raw registers (3 bytes: LTCBH, LTCBM, LTCBL)
    uint8_t tc_bytes[3];
    TC_ReadRegisters(&sensor, MAX31856_REG_LTCBH, tc_bytes, 3U);
    record[4] = tc_bytes[0];
    record[5] = tc_bytes[1];
    record[6] = tc_bytes[2];

    // Bytes 7-8: Cold Junction temperature registers (2 bytes: CJTH, CJTL)
    uint8_t cj_bytes[2];
    TC_ReadRegisters(&sensor, MAX31856_REG_CJTH, cj_bytes, 2U);
    record[7] = cj_bytes[0];
    record[8] = cj_bytes[1];

    // Byte 9: Fault status register (1 byte)
    uint8_t fault_status = 0U;
    TC_Fault(&sensor, &fault_status);
    record[9] = fault_status;

    aesir_status_t result = Module_AppendRecord(record);

    if (result < AESIR_STATUS_WAITING) {
        Error_Handler();
    }

    HAL_Delay(10);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = (MODULE_ID << 1); /* Shifted left to fit STM32 HAL addressing alignment */
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|SENS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin SENS_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SENS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *h, uint8_t direction, uint16_t address)
{
    (void)address;
    if (h != &MODULE_I2C) { return; }
    if (direction == I2C_DIRECTION_TRANSMIT) { /* Motherboard writes */
        expect_offset = true;
        Module_CheckI2C(HAL_I2C_Slave_Seq_Receive_IT(
            h, &rx_byte, 1U, I2C_FIRST_FRAME));
    } else { /* Motherboard reads */
        if (selected_offset == 0U) {
            snapshot_length = published_length;
            for (uint16_t i = 0; i < snapshot_length; ++i) {
                snapshot[i] = published[i];
            }
        }
        if (selected_offset < snapshot_length) {
            Module_CheckI2C(HAL_I2C_Slave_Seq_Transmit_IT(
                h, &snapshot[selected_offset],
                (uint16_t)(snapshot_length - selected_offset), I2C_FIRST_FRAME));
        } else {
            Module_CheckI2C(HAL_I2C_Slave_Seq_Transmit_IT(
                h, &padding, 1U, I2C_FIRST_FRAME));
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h != &MODULE_I2C) { return; }
    if (expect_offset) {
        selected_offset = rx_byte;
        expect_offset = false;
    }
    Module_CheckI2C(HAL_I2C_Slave_Seq_Receive_IT(h, &rx_byte, 1U, I2C_NEXT_FRAME));
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h == &MODULE_I2C) {
        Module_CheckI2C(HAL_I2C_Slave_Seq_Transmit_IT(h, &padding, 1U, I2C_NEXT_FRAME));
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *h)
{
    if (h == &MODULE_I2C) { Module_ListenIfReady(h); }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    if (h != &MODULE_I2C) { return; }
    uint32_t error = HAL_I2C_GetError(h) & ~HAL_I2C_ERROR_AF;
    if (error != HAL_I2C_ERROR_NONE) { module_i2c_error = error; }
    Module_ListenIfReady(h);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the target source file name and line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
