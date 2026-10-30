/**
 * @file MAX31856.h
 * @brief STM32 HAL driver interface for the MAX31856 thermocouple converter.
 *
 * Tested by inspection against the MAX31856 register map. Hardware validation
 * is still required on the target board.
 */
#ifndef MAX31856_H
#define MAX31856_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Read addresses: bit 7 = 0. Write addresses: bit 7 = 1. */
#define MAX31856_REG_CR0       0x00U
#define MAX31856_REG_CR1       0x01U
#define MAX31856_REG_MASK      0x02U
#define MAX31856_REG_CJHF      0x03U
#define MAX31856_REG_CJLF      0x04U
#define MAX31856_REG_LTHFTH    0x05U
#define MAX31856_REG_LTHFTL    0x06U
#define MAX31856_REG_LTLFTH    0x07U
#define MAX31856_REG_LTLFTL    0x08U
#define MAX31856_REG_CJTO      0x09U
#define MAX31856_REG_CJTH      0x0AU
#define MAX31856_REG_CJTL      0x0BU
#define MAX31856_REG_LTCBH     0x0CU
#define MAX31856_REG_LTCBM     0x0DU
#define MAX31856_REG_LTCBL     0x0EU
#define MAX31856_REG_SR        0x0FU

#define MAX31856_CR0_AUTOCONVERT  0x80U
#define MAX31856_CR0_1SHOT        0x40U
#define MAX31856_CR0_FILTER_50HZ  0x01U

typedef struct
{
    SPI_HandleTypeDef *spihandle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} TC_HandleTypeDef;

typedef enum
{
    TC_TYPE_B = 0x00,
    TC_TYPE_E = 0x01,
    TC_TYPE_J = 0x02,
    TC_TYPE_K = 0x03,
    TC_TYPE_N = 0x04,
    TC_TYPE_R = 0x05,
    TC_TYPE_S = 0x06,
    TC_TYPE_T = 0x07
} TC_ThermocoupleType_t;

typedef enum
{
    TC_CONVERSION_ONESHOT,       /* Trigger a conversion on request */
    TC_CONVERSION_ONESHOT_NOWAIT,/* Configure one-shot mode; do not wait */
    TC_CONVERSION_CONTINUOUS
} TC_ConversionMode_t;

typedef enum
{
    TC_FILTER_50HZ,
    TC_FILTER_60HZ
} TC_NoiseFilter_t;

/**
 * Initialize the handle and configure the chip for K-type, continuous
 * conversion, 60 Hz filtering, and unmasked fault reporting.
 * Ensure CS is configured as a GPIO output and is high before calling.
 */
HAL_StatusTypeDef TC_Init(TC_HandleTypeDef *sensor,
                          SPI_HandleTypeDef *spihandle,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin);

HAL_StatusTypeDef TC_ReadRegisters(TC_HandleTypeDef *sensor, uint8_t start_addr,uint8_t *data,uint16_t length);

HAL_StatusTypeDef TC_ReadTemperature(TC_HandleTypeDef *sensor,
                                     float *temperature_c);
HAL_StatusTypeDef TC_ReadColdJunctionTemperature(TC_HandleTypeDef *sensor,
                                                 float *temperature_c);

HAL_StatusTypeDef TC_SetThermocoupleType(TC_HandleTypeDef *sensor,
                                         TC_ThermocoupleType_t type);
HAL_StatusTypeDef TC_GetThermocoupleType(TC_HandleTypeDef *sensor,
                                         TC_ThermocoupleType_t *type);

HAL_StatusTypeDef TC_SetConversionMode(TC_HandleTypeDef *sensor,
                                       TC_ConversionMode_t mode);
HAL_StatusTypeDef TC_GetConversionMode(TC_HandleTypeDef *sensor,
                                       TC_ConversionMode_t *mode);

HAL_StatusTypeDef TC_SetNoiseFilter(TC_HandleTypeDef *sensor,
                                    TC_NoiseFilter_t filter);

/* Returns the raw Fault Status Register (0 means no currently reported fault). */
HAL_StatusTypeDef TC_Fault(TC_HandleTypeDef *sensor, uint8_t *fault_status);

/* Read/write a single register. addr is the 7-bit register address (0x00-0x0F). */
HAL_StatusTypeDef ReadReg(TC_HandleTypeDef *sensor, uint8_t addr,
                          uint8_t *data);
HAL_StatusTypeDef WriteReg(TC_HandleTypeDef *sensor, uint8_t addr,
                           uint8_t data);

/* Convert a raw MAX31856 fault-status register value to a bitmask; see datasheet. */
HAL_StatusTypeDef TC_ClearFaults(TC_HandleTypeDef *sensor);

#ifdef __cplusplus
}
#endif

#endif /* MAX31856_H */
