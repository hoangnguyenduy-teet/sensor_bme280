#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <stddef.h>
#include "do.h"
#include <spi_do.h>


// Register Address
#define BME280_REG_CHIP_ID 					UINT8_C(0xD0)
#define BME280_REG_RESET					UINT8_C(0xE0)
#define BME280_REG_TEMP_PRESS_CALIB_DATA	UINT8_C(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA		UINT8_C(0xE1)
#define BME280_REG_CTRL_HUM_				UINT8_C(0xF2)
#define BME280_REG_STATUS					UINT8_C(0xF3)
#define BME280_REG_PWR_CTRL					UINT8_C(0xF4)
#define BME280_REG_CTRL_MEAS				UINT8_C(0xF4)
#define BME280_REG_CONFIG					UINT8_C(0xF5)
#define BME280_REG_DATA						UINT8_C(0xF7)

// C standard macros
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

// Success code
#define BME280_OK							UINT8_C(0x00)

// Warning code
#define BME280_W_INVALID_OSR_MACRO			INT8_C(1)

// Error codes
#define BME280_E_NULL_PTR                         INT8_C(-1)
#define BME280_E_COMM_FAIL                        INT8_C(-2)
#define BME280_E_INVALID_LEN                      INT8_C(-3)
#define BME280_E_DEV_NOT_FOUND                    INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL                  INT8_C(-5)
#define BME280_E_NVM_COPY_FAILED                  INT8_C(-6)

// BME280 chip identifier
#define BME280_CHIP_ID						UINT8_C(0x60)

// I2C addresses
#define BME280_I2C_ADDR_PRIM				UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC					UINT8_C(0x77)

// Macros related to size
#define BME280_LEN_TEMP_PRESS_CALIB_DATA	UINT8_C(0x1A)
#define BME280_LEN_HUMIDITY_CALIB_DATA		UINT8_C(0x07)
#define BME280_LEN_P_T_H_DATA				UINT8_C(0x08)

// Sensor power mode
#define BME280_POWERMODE_SLEEP				UINT8_C(0x00)
#define BME280_POWERMODE_FORCED				UINT8_C(0x01)
#define BME280_POWERMODE_NORMAL				UINT8_C(0x03)

#define BME280_SENSOR_MODE_MSK				UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS				UINT8_C(0x00)

/*! @name Soft reset command */
#define BME280_SOFT_RESET_COMMAND                 UINT8_C(0xB6)

#define BME280_STATUS_IM_UPDATE                   UINT8_C(0x01)
#define BME280_STATUS_MEAS_DONE                   UINT8_C(0x08)

// Sensor component selection macros
#define BME280_PRESS                              UINT8_C(1)
#define BME280_TEMP                               UINT8_C(1 << 1)
#define BME280_HUM                                UINT8_C(1 << 2)
#define BME280_ALL                                UINT8_C(0x07)

// Settings selection macros
#define BME280_SEL_OSR_PRESS                      UINT8_C(1)
#define BME280_SEL_OSR_TEMP                       UINT8_C(1 << 1)
#define BME280_SEL_OSR_HUM                        UINT8_C(1 << 2)
#define BME280_SEL_FILTER                         UINT8_C(1 << 3)
#define BME280_SEL_STANDBY                        UINT8_C(1 << 4)
#define BME280_SEL_ALL_SETTINGS                   UINT8_C(0x1F)

// Oversampling macros
#define BME280_NO_OVERSAMPLING                    UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X                    UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X                    UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X                    UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X                    UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X                   UINT8_C(0x05)
#define BME280_OVERSAMPLING_MAX                   UINT8_C(16)

#define BME280_CTRL_HUM_MSK                       UINT8_C(0x07)
#define BME280_CTRL_HUM_POS                       UINT8_C(0x00)
#define BME280_CTRL_PRESS_MSK                     UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS                     UINT8_C(0x02)
#define BME280_CTRL_TEMP_MSK                      UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS                      UINT8_C(0x05)

// Measurement delay calculation macros
#define BME280_MEAS_OFFSET                        UINT16_C(1250)
#define BME280_MEAS_DUR                           UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET               UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR                UINT16_C(1000)
#define BME280_STARTUP_DELAY                      UINT16_C(2000)

// Length macros
#define BME280_MAX_LEN                            UINT8_C(10)

// Standby duration selection macroS
#define BME280_STANDBY_TIME_0_5_MS                (0x00)
#define BME280_STANDBY_TIME_62_5_MS               (0x01)
#define BME280_STANDBY_TIME_125_MS                (0x02)
#define BME280_STANDBY_TIME_250_MS                (0x03)
#define BME280_STANDBY_TIME_500_MS                (0x04)
#define BME280_STANDBY_TIME_1000_MS               (0x05)
#define BME280_STANDBY_TIME_10_MS                 (0x06)
#define BME280_STANDBY_TIME_20_MS                 (0x07)

#define BME280_STANDBY_MSK                        UINT8_C(0xE0)
#define BME280_STANDBY_POS                        UINT8_C(0x05)

// Bit shift macros
#define BME280_12_BIT_SHIFT                       UINT8_C(12)
#define BME280_8_BIT_SHIFT                        UINT8_C(8)
#define BME280_4_BIT_SHIFT                        UINT8_C(4)

// Filter coefficient selection macros
#define BME280_FILTER_COEFF_OFF                   (0x00)
#define BME280_FILTER_COEFF_2                     (0x01)
#define BME280_FILTER_COEFF_4                     (0x02)
#define BME280_FILTER_COEFF_8                     (0x03)
#define BME280_FILTER_COEFF_16                    (0x04)

#define BME280_FILTER_MSK                         UINT8_C(0x1C)
#define BME280_FILTER_POS                         UINT8_C(0x02)

// Macro to combine two 8 bit data's to form a 16 bit data
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/*! @name Macro to SET and GET BITS of a register */
#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))


// Interface selections enum
enum bme280_intf {
	// SPI interface
	BME280_SPI_INTF,
	// I2C interface
	BME280_I2C_INTF
};

typedef BME280_INTF_RET_TYPE (*bme280_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

typedef BME280_INTF_RET_TYPE (*bme280_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

typedef void (*bme280_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

struct bme280_calib_data {
	// Calibration coefficient for the temperature sensor
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	// Calibration coefficient for the pressure sensor
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	// Calibration coefficient for the humidity sensor
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t  dig_H6;

	int32_t t_fine; // Variable to store the intermediate temperature coefficient
};

#ifdef BME280_DOUBLE_ENABLE
struct bme280_data
{
    /*! Compensated pressure */
    double pressure;

    /*! Compensated temperature */
    double temperature;

    /*! Compensated humidity */
    double humidity;
};
#else
struct bme280_data
{
    /*! Compensated pressure */
    uint32_t pressure;

    /*! Compensated temperature */
    int32_t temperature;

    /*! Compensated humidity */
    uint32_t humidity;
};
#endif /*! BME280_USE_FLOATING_POINT */

struct bme280_uncomp_data {
	uint32_t pressure; // un-compensated pressure
	uint32_t humidity; // un-compensated humidity
	uint32_t temperature; // un-compensated temperature
};

struct bme280_settings {
	uint8_t osr_p; // pressure oversampling
	uint8_t osr_t; // temperature oversampling
	uint8_t osr_h; // humidity oversampling
	uint8_t filter; // filter coefficient
	uint8_t standby_time;
};

struct bme280_dev {
	uint8_t chip_id; // id chip

	enum bme280_intf intf;

	void *intf_ptr;

	BME280_INTF_RET_TYPE intf_rslt; // variable to store result of read/write function

	bme280_read_fptr_t read; // read function pointer

	bme280_write_fptr_t write; // write function poiter

	bme280_delay_us_fptr_t delay_us; // delay function pointer

	struct bme280_calib_data calib_data;
};


int8_t bme280_init(struct bme280_dev *dev);

int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_settings *settings, struct bme280_dev *dev);

int8_t bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev);

int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev);

int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev);

int8_t bme280_soft_reset(struct bme280_dev *dev);

int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);

int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data, struct bme280_data *comp_data, struct bme280_calib_data *calib_data);

int8_t bme280_cal_meas_delay(uint32_t *max_delay, const struct bme280_settings *settings);
#endif

