#include "esp_ms5607.h"

static const char *TAG = "MS5607";

static const ms5607_osr_config_t osr_config[] = {
    {0x00, MS5607_OSR256_US},  {0x02, MS5607_OSR512_US},  {0x04, MS5607_OSR1024_US},
    {0x06, MS5607_OSR2048_US}, {0x08, MS5607_OSR4096_US},
};

static inline void cs_low(ms5607_handle_t handle) { gpio_set_level(handle->dev_config.cs, LOW); }

static inline void cs_high(ms5607_handle_t handle) { gpio_set_level(handle->dev_config.cs, HIGH); }

esp_err_t ms5607_send_cmd(ms5607_handle_t handle, const uint8_t cmd, uint8_t *rx, size_t len) {
    uint8_t tx[4] = {cmd, 0, 0, 0};

    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    return spi_device_polling_transmit(handle->spi_handle, &t);
}

esp_err_t ms5607_reset(ms5607_handle_t handle) {
    esp_err_t err;

    xSemaphoreTake(handle->dev_config.spi_mutex, portMAX_DELAY);
    spi_device_acquire_bus(handle->spi_handle, portMAX_DELAY);
    cs_low(handle);

    err = ms5607_send_cmd(handle, MS5607_CMD_RESET, NULL, 1);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to send RESET command");

    ets_delay_us(MS5607_RESET_US);
    cs_high(handle);
    spi_device_release_bus(handle->spi_handle);
    xSemaphoreGive(handle->dev_config.spi_mutex);

    return err;
}

esp_err_t ms5607_read_prom(ms5607_handle_t handle, const uint8_t address, uint16_t *out_val) {
    esp_err_t err;

    xSemaphoreTake(handle->dev_config.spi_mutex, portMAX_DELAY);
    spi_device_acquire_bus(handle->spi_handle, portMAX_DELAY);
    cs_low(handle);

    uint8_t rx[3];
    err = ms5607_send_cmd(handle, address, rx, 3);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to read prom: 0x%02x", address);

    cs_high(handle);
    spi_device_release_bus(handle->spi_handle);
    xSemaphoreGive(handle->dev_config.spi_mutex);

    *out_val = ((uint16_t)rx[1] << 8) | rx[2];

    return err;
}

esp_err_t ms5607_init(const ms5607_config_t *ms5607_config, ms5607_handle_t *out_handle) {
    esp_err_t err;

    /* validate memory allocation */
    ms5607_handle_t handle = (ms5607_handle_t)calloc(1, sizeof(*handle));
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, TAG, "No memory for MS5607 device");

    /* copy device config to handle */
    handle->dev_config = *ms5607_config;

    /* chip select config */
    gpio_config_t cs_conf = {
        .pin_bit_mask = 1ULL << ms5607_config->cs,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&cs_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin");
        goto err_handle;
    }
    cs_high(handle);

    /* SPI device configuration */
    const spi_device_interface_config_t ms_dev_config = {
        .command_bits     = 0,   // raw data
        .address_bits     = 0,   // raw data
        .dummy_bits       = 0,   // no extra delay
        .mode             = 0,   // CPOL=0 (idle low), CPHA=0 (sample on rising edge)
        .duty_cycle_pos   = 128, // 50% duty cycle
        .cs_ena_pretrans  = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz   = SPI_CLOCK_HZ,
        .input_delay_ns   = 0,
        .spics_io_num     = -1, // CS managed manually with delays
        .queue_size       = 1,  // queue not used
    };

    /* add MS5607 to SPI bus */
    err = spi_bus_add_device(ms5607_config->spi_host, &ms_dev_config, &handle->spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MS5607 device to SPI bus");
        goto err_handle;
    }

    /* MS5607 system reset */
    err = ms5607_reset(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send RESET cmd");
        goto err_handle;
    }

    /* MS5607 read PROM values */
    ms5607_calibration_t calibration_values;
    ms5607_read_prom(handle, MS5607_PRM_ADD1, &calibration_values.pressure_sensitivty);
    ms5607_read_prom(handle, MS5607_PRM_ADD2, &calibration_values.pressure_offset);
    ms5607_read_prom(handle, MS5607_PRM_ADD3, &calibration_values.temperature_sensitivity);
    ms5607_read_prom(handle, MS5607_PRM_ADD4, &calibration_values.temperature_offset);
    ms5607_read_prom(handle, MS5607_PRM_ADD5, &calibration_values.temperature_reference);
    ms5607_read_prom(handle, MS5607_PRM_ADD6, &calibration_values.temperature_coefficient);

    /* Save calibration values */
    handle->dev_config.calibration = calibration_values;

    *out_handle = handle;
    return ESP_OK;

err_handle:
    if (handle->spi_handle)
        spi_bus_remove_device(handle->spi_handle);
    free(handle);
    return err;
}

esp_err_t ms5607_start_conversion(ms5607_handle_t handle, ms5607_type_t type) {
    esp_err_t err;

    uint8_t  osr;
    uint8_t  cmd;
    uint32_t delay_us;

    if (type == MS5607_PRESSURE) {
        osr = handle->dev_config.pressure_osr;
        cmd = MS5607_CMD_D1 | osr_config[osr].osr;
    } else {
        osr = handle->dev_config.temperature_osr;
        cmd = MS5607_CMD_D2 | osr_config[osr].osr;
    }

    delay_us = osr_config[osr].delay_us;

    xSemaphoreTake(handle->dev_config.spi_mutex, portMAX_DELAY);
    spi_device_acquire_bus(handle->spi_handle, portMAX_DELAY);
    cs_low(handle);

    err = ms5607_send_cmd(handle, cmd, NULL, 1);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to start %s conversion", type == MS5607_PRESSURE ? "pressure" : "temperature");

    ets_delay_us(delay_us);
    cs_high(handle);
    spi_device_release_bus(handle->spi_handle);
    xSemaphoreGive(handle->dev_config.spi_mutex);

    return err;
}