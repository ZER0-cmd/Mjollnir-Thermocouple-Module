/**
 * @file MAX31856.c
 * @brief STM32 HAL driver for the MAX31856 thermocouple converter.
 */
#include "MAX31856.h"

static HAL_StatusTypeDef TC_ValidateHandle(const TC_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->spihandle == NULL) ||
        (sensor->cs_port == NULL))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef ReadReg(TC_HandleTypeDef *sensor, uint8_t addr,
                          uint8_t *data)
{
    if ((TC_ValidateHandle(sensor) != HAL_OK) || (data == NULL) ||
        (addr > MAX31856_REG_SR))
    {
        return HAL_ERROR;
    }

    uint8_t tx[2] = { (uint8_t)(addr & 0x7FU), 0x00U };
    uint8_t rx[2] = { 0U, 0U };

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        sensor->spihandle, tx, rx, (uint16_t)sizeof(tx), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return status;
    }

    *data = rx[1];
    return HAL_OK;
}

HAL_StatusTypeDef WriteReg(TC_HandleTypeDef *sensor, uint8_t addr,
                           uint8_t data)
{
    if ((TC_ValidateHandle(sensor) != HAL_OK) ||
        (addr > MAX31856_REG_CJTL))
    {
        return HAL_ERROR; /* Registers above 0x0B are read-only. */
    }

    uint8_t tx[2] = { (uint8_t)(addr | 0x80U), data };

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        sensor->spihandle, tx, (uint16_t)sizeof(tx), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef TC_ReadRegisters(TC_HandleTypeDef *sensor,
                                          uint8_t start_addr,
                                          uint8_t *data,
                                          uint16_t length)
{
    if ((TC_ValidateHandle(sensor) != HAL_OK) || (data == NULL) ||
        (length == 0U) || ((uint16_t)start_addr + length > 16U))
    {
        return HAL_ERROR;
    }

    /* The first received byte corresponds to the address phase and is discarded. */
    uint8_t tx[4] = { (uint8_t)(start_addr & 0x7FU), 0U, 0U, 0U };
    uint8_t rx[4] = { 0U, 0U, 0U, 0U };
    if (length > 3U)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        sensor->spihandle, tx, rx, (uint16_t)(length + 1U), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return status;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        data[i] = rx[i + 1U];
    }
    return HAL_OK;
}

HAL_StatusTypeDef TC_Init(TC_HandleTypeDef *sensor,
                          SPI_HandleTypeDef *spihandle,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin)
{
    if ((sensor == NULL) || (spihandle == NULL) || (cs_port == NULL))
    {
        return HAL_ERROR;
    }

    sensor->spihandle = spihandle;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* Unmask all faults, select K type, continuous conversion, 60 Hz filter. */
    HAL_StatusTypeDef status = WriteReg(sensor, MAX31856_REG_MASK, 0x00U);
    if (status != HAL_OK) return status;

    status = WriteReg(sensor, MAX31856_REG_CR1, (uint8_t)TC_TYPE_K);
    if (status != HAL_OK) return status;

    status = WriteReg(sensor, MAX31856_REG_CR0, MAX31856_CR0_AUTOCONVERT);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef TC_ReadTemperature(TC_HandleTypeDef *sensor,
                                     float *temperature_c)
{
    if ((TC_ValidateHandle(sensor) != HAL_OK) || (temperature_c == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t bytes[3];
    HAL_StatusTypeDef status = TC_ReadRegisters(
        sensor, MAX31856_REG_LTCBH, bytes, 3U);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 19-bit signed two's-complement value is left-aligned in 24 bits.
       The lowest five bits are not part of the temperature value. */
    int32_t raw24 = ((int32_t)bytes[0] << 16) |
                    ((int32_t)bytes[1] << 8) |
                    (int32_t)bytes[2];

    if ((raw24 & 0x00800000L) != 0)
    {
        raw24 |= (int32_t)0xFF000000UL;
    }

    int32_t raw19 = raw24 >> 5;
    *temperature_c = (float)raw19 * 0.0078125f;
    return HAL_OK;
}

HAL_StatusTypeDef TC_ReadColdJunctionTemperature(TC_HandleTypeDef *sensor,
                                                 float *temperature_c)
{
    if ((TC_ValidateHandle(sensor) != HAL_OK) || (temperature_c == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t bytes[2];
    HAL_StatusTypeDef status = TC_ReadRegisters(
        sensor, MAX31856_REG_CJTH, bytes, 2U);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Cold-junction result is a signed 14-bit value in bits [15:2],
       with a resolution of 1/64 degree Celsius. */
    int16_t raw16 = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    int16_t raw14 = (int16_t)(raw16 >> 2);
    *temperature_c = (float)raw14 / 64.0f;
    return HAL_OK;
}

HAL_StatusTypeDef TC_SetThermocoupleType(TC_HandleTypeDef *sensor,
                                         TC_ThermocoupleType_t type)
{
    if ((type < TC_TYPE_B) || (type > TC_TYPE_T))
    {
        return HAL_ERROR;
    }

    uint8_t cr1;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR1, &cr1);
    if (status != HAL_OK) return status;

    cr1 = (uint8_t)((cr1 & 0xF0U) | ((uint8_t)type & 0x0FU));
    return WriteReg(sensor, MAX31856_REG_CR1, cr1);
}

HAL_StatusTypeDef TC_GetThermocoupleType(TC_HandleTypeDef *sensor,
                                         TC_ThermocoupleType_t *type)
{
    if (type == NULL) return HAL_ERROR;

    uint8_t cr1;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR1, &cr1);
    if (status != HAL_OK) return status;

    uint8_t value = (uint8_t)(cr1 & 0x0FU);
    if (value > (uint8_t)TC_TYPE_T) return HAL_ERROR;
    *type = (TC_ThermocoupleType_t)value;
    return HAL_OK;
}

HAL_StatusTypeDef TC_SetConversionMode(TC_HandleTypeDef *sensor,
                                       TC_ConversionMode_t mode)
{
    uint8_t cr0;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR0, &cr0);
    if (status != HAL_OK) return status;

    cr0 &= (uint8_t)~(MAX31856_CR0_AUTOCONVERT | MAX31856_CR0_1SHOT);

    switch (mode)
    {
        case TC_CONVERSION_CONTINUOUS:
            cr0 |= MAX31856_CR0_AUTOCONVERT;
            break;
        case TC_CONVERSION_ONESHOT:
        case TC_CONVERSION_ONESHOT_NOWAIT:
            /* The device starts one conversion when 1SHOT is written as 1.
               This driver does not poll for conversion completion. */
            cr0 |= MAX31856_CR0_1SHOT;
            break;
        default:
            return HAL_ERROR;
    }

    return WriteReg(sensor, MAX31856_REG_CR0, cr0);
}

HAL_StatusTypeDef TC_GetConversionMode(TC_HandleTypeDef *sensor,
                                       TC_ConversionMode_t *mode)
{
    if (mode == NULL) return HAL_ERROR;

    uint8_t cr0;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR0, &cr0);
    if (status != HAL_OK) return status;

    if ((cr0 & MAX31856_CR0_AUTOCONVERT) != 0U)
    {
        *mode = TC_CONVERSION_CONTINUOUS;
    }
    else
    {
        /* The 1SHOT bit is a trigger, not a persistent mode indicator.
           A cleared bit means the device is idle in one-shot configuration. */
        *mode = TC_CONVERSION_ONESHOT_NOWAIT;
    }
    return HAL_OK;
}

HAL_StatusTypeDef TC_SetNoiseFilter(TC_HandleTypeDef *sensor,
                                    TC_NoiseFilter_t filter)
{
    if ((filter != TC_FILTER_50HZ) && (filter != TC_FILTER_60HZ))
    {
        return HAL_ERROR;
    }

    uint8_t cr0;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR0, &cr0);
    if (status != HAL_OK) return status;

    if (filter == TC_FILTER_50HZ)
        cr0 |= MAX31856_CR0_FILTER_50HZ;
    else
        cr0 &= (uint8_t)~MAX31856_CR0_FILTER_50HZ;

    return WriteReg(sensor, MAX31856_REG_CR0, cr0);
}

HAL_StatusTypeDef TC_Fault(TC_HandleTypeDef *sensor, uint8_t *fault_status)
{
    if (fault_status == NULL) return HAL_ERROR;
    return ReadReg(sensor, MAX31856_REG_SR, fault_status);
}

HAL_StatusTypeDef TC_ClearFaults(TC_HandleTypeDef *sensor)
{
    uint8_t cr0;
    HAL_StatusTypeDef status = ReadReg(sensor, MAX31856_REG_CR0, &cr0);
    if (status != HAL_OK) return status;

    /* FAULTCLR is bit 1. Set it to clear the fault status, then restore CR0. */
    status = WriteReg(sensor, MAX31856_REG_CR0, (uint8_t)(cr0 | 0x02U));
    if (status != HAL_OK) return status;

    return WriteReg(sensor, MAX31856_REG_CR0, (uint8_t)(cr0 & (uint8_t)~0x02U));
}
