/**
 * @file app_main.c
 * @brief Main application for reading BME280 sensor and outputting data via UART
 * @note This module is hardware-independent and uses abstracted drivers
 */

#include "app_main.h"
#include "bme280.h"
#include "console.h"
#include "error_codes.h"
#include "board.h"
#include <stdbool.h>
#include <string.h>

/* Configuration */
#define BME280_MEASUREMENT_INTERVAL_MS   2000  /* 2 seconds between measurements */
#define BME280_SPI_SPEED_HZ              1000000  /* 1 MHz SPI speed */

/* BME280 Device Context */
static struct bme280_dev g_bme280_dev;
static struct bme280_settings g_bme280_settings;
static bool g_bme280_initialized = false;

/* Function prototypes for BME280 interface functions */
static int8_t bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void bme280_delay_us(uint32_t period, void *intf_ptr);

/**
 * @brief Initialize BME280 sensor
 * @return ERROR_OK on success, error code on failure
 */
static uint32_t app_bme280_init(void)
{
    int8_t rslt;

    /* Configure BME280 device structure */
    g_bme280_dev.intf = BME280_SPI_INTF;  /* Use SPI interface */
    g_bme280_dev.intf_ptr = NULL;         /* Can be used to pass SPI handle */
    g_bme280_dev.read = bme280_spi_read;
    g_bme280_dev.write = bme280_spi_write;
    g_bme280_dev.delay_us = bme280_delay_us;

    /* Initialize the sensor */
    rslt = bme280_init(&g_bme280_dev);
    if (rslt != BME280_OK)
    {
        console_printf("BME280 initialization failed: %d\r\n", rslt);
        return ERROR_FAIL;
    }

    /* Always read the current settings before writing */
    rslt = bme280_get_sensor_settings(&g_bme280_settings, &g_bme280_dev);
    if (rslt != BME280_OK)
    {
        console_printf("Failed to get BME280 settings: %d\r\n", rslt);
        return ERROR_FAIL;
    }

    /* Configure the sensor settings */
    g_bme280_settings.osr_p = BME280_OVERSAMPLING_2X;
    g_bme280_settings.osr_t = BME280_OVERSAMPLING_2X;
    g_bme280_settings.osr_h = BME280_OVERSAMPLING_2X;
    g_bme280_settings.filter = BME280_FILTER_COEFF_4;
    g_bme280_settings.standby_time = BME280_STANDBY_TIME_250_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS,
                                      &g_bme280_settings,
                                      &g_bme280_dev);
    if (rslt != BME280_OK)
    {
        console_printf("Failed to set BME280 settings: %d\r\n", rslt);
        return ERROR_FAIL;
    }

    /* Set the sensor to normal mode */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &g_bme280_dev);
    if (rslt != BME280_OK)
    {
        console_printf("Failed to set BME280 mode: %d\r\n", rslt);
        return ERROR_FAIL;
    }

    g_bme280_initialized = true;
    console_printf("BME280 initialized successfully\r\n");

    return ERROR_OK;
}

/**
 * @brief Read and display sensor data
 * @return ERROR_OK on success, error code on failure
 */
static uint32_t app_bme280_read_and_display(void)
{
    int8_t rslt;
    struct bme280_data comp_data;

    if (!g_bme280_initialized)
    {
        return ERROR_NOT_INITIALIZED;
    }

    /* Get sensor data */
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &g_bme280_dev);
    if (rslt != BME280_OK)
    {
        console_printf("Failed to read sensor data: %d\r\n", rslt);
        return ERROR_FAIL;
    }

    /* Display sensor data with proper formatting */
    console_printf("\r\n=== BME280 Sensor Data ===\r\n");

#ifdef BME280_DOUBLE_ENABLE
    /* Double precision output */
    console_printf("Temperature: %.2f C\r\n", comp_data.temperature);
    console_printf("Pressure:    %.2f hPa\r\n", comp_data.pressure / 100.0); /* Convert Pa to hPa */
    console_printf("Humidity:    %.2f %%\r\n", comp_data.humidity);
#else
    /* Integer output with scaling */
    /* Temperature: Output in °C with 0.01°C resolution */
    console_printf("Temperature: %.2f C\r\n", comp_data.temperature / 100.0);

    /* Pressure: Output in hPa with 0.01 hPa resolution */
    console_printf("Pressure:    %.2f hPa\r\n", comp_data.pressure / 100.0);

    /* Humidity: Output in % with 0.001% resolution */
    console_printf("Humidity:    %.2f %%\r\n", comp_data.humidity / 1024.0);
#endif

    console_printf("=======================\r\n");

    return ERROR_OK;
}

/**
 * @brief SPI read function for BME280
 * @note This is a wrapper for your SPI driver
 */
static int8_t bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* TODO: Implement using your SPI driver
     * Example implementation:
     * 1. Pull CS pin low
     * 2. Send register address with read bit set
     * 3. Read data
     * 4. Pull CS pin high
     */

    /* For now, return error to indicate not implemented */
    (void)reg_addr;
    (void)reg_data;
    (void)len;
    (void)intf_ptr;

    return BME280_E_COMM_FAIL;
}

/**
 * @brief SPI write function for BME280
 * @note This is a wrapper for your SPI driver
 */
static int8_t bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* TODO: Implement using your SPI driver
     * Example implementation:
     * 1. Pull CS pin low
     * 2. Send register address with write bit cleared
     * 3. Send data
     * 4. Pull CS pin high
     */

    /* For now, return error to indicate not implemented */
    (void)reg_addr;
    (void)reg_data;
    (void)len;
    (void)intf_ptr;

    return BME280_E_COMM_FAIL;
}

/**
 * @brief Delay function for BME280
 * @note This should be implemented using your system's delay function
 */
static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    /* TODO: Implement microsecond delay using your system timer
     * Example: HAL_Delay(period / 1000) for milliseconds
     */

    /* For now, use a simple loop (not accurate) */
    volatile uint32_t i;
    for (i = 0; i < (period * 10); i++)
    {
        __asm__("nop");
    }

    (void)intf_ptr;
}

/**
 * @brief Application main function
 * @note This is the main application entry point called from main.c
 */
void app_main(void)
{
    uint32_t ret;

    /* Initialize console output */
    console_printf("\r\n");
    console_printf("==================================\r\n");
    console_printf("BME280 Sensor Application v%d.%d.%d\r\n",
                   VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    console_printf("==================================\r\n\r\n");

    /* Initialize BME280 sensor */
    ret = app_bme280_init();
    if (ret != ERROR_OK)
    {
        console_printf("Sensor initialization failed. Stopping application.\r\n");
        return;
    }

    /* Main application loop */
    while (1)
    {
        /* Read and display sensor data */
        ret = app_bme280_read_and_display();
        if (ret != ERROR_OK)
        {
            console_printf("Error reading sensor data: %lu\r\n", ret);
        }

        /* TODO: Implement delay using your system's delay function
         * Example: HAL_Delay(BME280_MEASUREMENT_INTERVAL_MS);
         */

        /* Simple delay loop (not accurate) */
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}
