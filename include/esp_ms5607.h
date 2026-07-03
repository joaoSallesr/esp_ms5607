#pragma once

#include <stdint.h>

#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#define LOW  0
#define HIGH 1

#define SPI_CLOCK_HZ 1920000

/*
 * MS5607 timing constants in µs - based on datasheet timings
 */
#define MS5607_RESET_US   2800 /* 2.80 ms */
#define MS5607_OSR256_US  600  /* 0.60 ms */
#define MS5607_OSR512_US  1170 /* 1.17 ms */
#define MS5607_OSR1024_US 2280 /* 2.28 ms */
#define MS5607_OSR2048_US 4540 /* 4.54 ms */
#define MS5607_OSR4096_US 9040 /* 9.04 ms */

/*
 * MS5607 commands
 */
#define MS5607_CMD_READ    0x00 /* returns 24 bit result */
#define MS5607_CMD_RESET   0x1E
#define MS5607_CMD_D1_256  0x40 /* initiate uncompensated pressure conversion with selected OSR*/
#define MS5607_CMD_D1_512  0x42
#define MS5607_CMD_D1_1024 0x44
#define MS5607_CMD_D1_2048 0x46
#define MS5607_CMD_D1_4096 0x48
#define MS5607_CMD_D2_256  0x50 /* initiate uncompensated temperature conversion with selected OSR*/
#define MS5607_CMD_D2_512  0x52
#define MS5607_CMD_D2_1024 0x54
#define MS5607_CMD_D2_2048 0x56
#define MS5607_CMD_D2_4096 0x58

/* After the conversion, using ADC read command the result is clocked out with the MSB first. If the conversion is not
executed before the ADC read command, or the ADC read command is repeated, it will give 0 as the output result.
If the ADC read command is sent during conversion the result will be 0, the conversion will not stop and the final
result will be wrong. Conversion sequence sent during the already started conversion process will yield incorrect
result as well.*/

/*
 * MS5607 PROM calibration commands - returns 16 bit result
 */
#define MS5607_PRM_ADD0 0xA0 /* Factory data and setup */
#define MS5607_PRM_ADD1 0xA2 /* Pressure sensitivity */
#define MS5607_PRM_ADD2 0xA4 /* Pressure offset */
#define MS5607_PRM_ADD3 0xA6 /* Temperature coefficient of pressure sensitivity */
#define MS5607_PRM_ADD4 0xA8 /* Temperature coefficient of pressure offset */
#define MS5607_PRM_ADD5 0xAA /* Reference temperature */
#define MS5607_PRM_ADD6 0xAC /* Temperature coefficient of the temperature */
#define MS5607_PRM_ADD7 0xAE /* Serial code and CRC */

/*
 * MS5607 Pressure Oversampling Rate
 */
typedef enum {
    MS5607_OSR_256  = 0x01,
    MS5607_OSR_512  = 0x03,
    MS5607_OSR_1024 = 0x05,
    MS5607_OSR_2048 = 0x07,
    MS5607_OSR_4096 = 0x09,
} ms5607_osr_t;

/*
 * MS5607 calibration coefficients (PROM 1-6 addresses)
 */
typedef struct {
    uint16_t pressure_sensitivty;     /* SENS T1 */
    uint16_t pressure_offset;         /* OFF T1 */
    uint16_t temperature_sensitivity; /* TCS */
    uint16_t temperature_offset;      /* TCO */
    uint16_t temperature_reference;   /* TREF */
    uint16_t temperature_coefficient; /* TEMPSENS */
} ms5607_calibration_t;

/*
 * MS5607 configuration structure
 */
typedef struct {
    spi_host_device_t    spi_host;        /*!< SPI bus selected */
    gpio_num_t           cs;              /*!< MS5607 CS pin */
    gpio_num_t           ps;              /*!< Protocol Select pin */
    ms5607_osr_t         pressure_osr;    /*!< MS5607 Pressure OSR Options */
    ms5607_osr_t         temperature_osr; /*!< MS5607 Temperature OSR Options */
    SemaphoreHandle_t    spi_mutex;       /*!< MS5607 SPI bus mutex */
    ms5607_calibration_t calibration;
} ms5607_config_t;

/*
 * MS5607 context structure
 */
struct ms5607_context_t {
    ms5607_config_t     dev_config;
    spi_device_handle_t spi_handle;
};

/*
 * MS5607 context definition
 */
typedef struct ms5607_context_t  ms5607_context_t;
typedef struct ms5607_context_t *ms5607_handle_t;

/**
 * @brief MS5607 SPI transaction helper.
 *
 * @param [in] handle MS5607 device handle.
 * @param [in] cmd MS5607 command byte.
 * @param [out] rx Transaction result.
 * @param [in] len Length of transaction.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ms5607_send_cmd(ms5607_handle_t handle, const uint8_t cmd, uint8_t *rx, size_t len);

/**
 * @brief Issues system reset command for the MS5607.
 *
 * @param [in] handle MS5607 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ms5607_reset(ms5607_handle_t handle);

/**
 * @brief Reads calibration value from PROM.
 *
 * @param [in] handle MS5607 device handle.
 * @param [in] address PROM address.
 * @param [out] out_val Calibration value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ms5607_read_prom(ms5607_handle_t handle, const uint8_t address, uint16_t *out_val);

/**
 * @brief Initializes an MS5607 device using device configuration.
 *
 * @param [in] ms5607_config MS5607 device configuration.
 * @param [out] out_handle MS5607 device handle.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ms5607_init(const ms5607_config_t *ms5607_config, ms5607_handle_t *out_handle);

esp_err_t ms5607_start_conversion(ms5607_handle_t handle, const uint8_t cmd);
esp_err_t ms5607_read_adc(ms5607_handle_t handle);

esp_err_t ms5607_temperature_calculation(ms5607_handle_t handle);
esp_err_t ms5607_pressure_compensation(ms5607_handle_t handle);