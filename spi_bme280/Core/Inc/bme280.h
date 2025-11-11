#ifndef BME280_H_
#define BME280_H_
  #include "stdint.h"
  #include "stdbool.h"
  #include "stm32f1xx.h"
  #include <string.h>

// Định nghĩa các thanh ghi BME280
	#define  BME280_REG_ID         0xD0
	#define  BME280_REG_RESET      0xE0
	#define  BME280_REG_CTRL_HUM   0xF2
	#define  BME280_REG_STATUS     0xF3
	#define  BME280_REG_CTRL_MEAS  0xF4
	#define  BME280_REG_CONFIG      0xF5
	#define  BME280_REG_PRESS_MSB   0xF8
	#define  BME280_REG_TEMP_LSB    0xFB
	#define  BME280_REG_TEMP_XLSB   0xFC
	#define  BME280_REG_TEMP_MSB    0xFD


	#define BME280_REG_CALIB_00     0x88
	#define BME280_REG_CALIB_H1     0xA1
	#define BME280_REG_CALIB_H2_LSB 0xE1

	#define BME280_SOFTRESET_VAL   0xB6
	#define BME280_CHIP_ID         0x60

//Modes & oversampling
	#define BME280_MODE_SLEEP 0x00
	#define BME280_MODE_FORCED 0x01
	#define BME280_MODE_NORMAL 0x03
	#define BME280_OSRS_SKIP 0x00
	#define BME280_OSRS_1 0x01
	#define BME280_OSRS_2 0x02
	#define BME280_OSRS_4 0x03
	#define BME280_OSRS_8 0x04
	#define BME280_OSRS_16 0x05

//config fields
	#define BME280_TSB_125_MS (0x02 << 5)
	#define BME280_FILTER_OFF (0x00 << 2)

// I2C Addresses
	#define BME280_I2C_ADDR_PRIM  0x76
	#define BME280_I2C_ADDR_ALT   0x77

// SPI masks: datasheet uses MSB = 1 for read, 0 for write
	#define BME280_SPI_READ  0x80
	#define BME280_SPI_WRITE 0x7F

// Return codes
	#define BME280_OK              0
	#define BME280_E_NULL_PTR      -1
	#define BME280_E_COMM_FAIL     -2
	#define BME280_E_DEV_NOT_FOUND -3

// Define buffer
	#define SPI_BUFFER_SIZE 32

// Interface type
typedef enum {
	BME280_INTERFACE_I2C = 0,
	BME280_INTERFACE_SPI
}bme280_interface_t;

// State machine cho bme280
typedef enum {
	BME280_STATE_IDLE = 0,
	BME280_STATE_TRIGGER_MEASURE,
	BME280_STATE_WAIT_MEASURE,
	BME280_STATE_READ_RAW_DATA,
	BME280_STATE_PROCESS_DATA
}bme280_state_t;

typedef int (*bme280_read_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t length);
typedef int (*bme280_write_fptr_t)(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t length);
typedef void (*bme280_delay_ms_fptr_t)(uint32_t ms);
typedef void (*bme280_cs_control_fptr_t)(uint8_t assert); // active=1, release=0

typedef struct {
	uint32_t start_time;
	uint32_t delay_ms;
	uint8_t is_running;
}bme280_delay_t;

// Calibration data structure
typedef struct bme280_calib_t {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t  dig_H6;
}bme280_calib_t;

// Cấu trúc mở rộng cho SPI interrupt
typedef struct {
  volatile uint8_t tx_complete;
  volatile uint8_t rx_complete;
  uint8_t tx_buffer[SPI_BUFFER_SIZE];
  uint8_t rx_buffer[SPI_BUFFER_SIZE];
} bme280_spi_buffer_t;

// Device handle
typedef struct BME280_dev_t {
	bme280_interface_t interface;
	uint8_t dev_id;

	bme280_read_fptr_t read;
	bme280_write_fptr_t write;
	bme280_delay_ms_fptr_t delay_ms;
	bme280_cs_control_fptr_t cs_control; // Only SPI

	SPI_HandleTypeDef *spi_handle;

	struct bme280_calib_t calib;
	int32_t t_fine;

	uint32_t measurement_start_time;
	int32_t raw_temp;
	int32_t raw_press;
	int32_t raw_hum;

	volatile bme280_state_t state;
	bme280_spi_buffer_t spi_buf;

} BME280_dev_t;

// Public API
	int bme280_init(BME280_dev_t *dev);
	int bme280_soft_reset(BME280_dev_t *dev);
	int bme280_read_id(BME280_dev_t *dev, uint8_t *id);
	int bme280_trigger_forced_measurement(BME280_dev_t *dev);
	int bme280_read_raw_temp_press_hum(BME280_dev_t *dev, int32_t *raw_temp, int32_t *raw_p, int32_t *raw_h );
	int bme280_compensate_all(BME280_dev_t *dev, int32_t raw_temp, int32_t raw_p, int32_t raw_h, int32_t *temp_x100, int32_t *press_x100, int32_t *hum_x1024);
	void user_cs_control(uint8_t assert);
	void bme280_process(BME280_dev_t *dev);
	void bme280_set_state(BME280_dev_t *dev, bme280_state_t state);
	bme280_state_t bme280_get_state(BME280_dev_t *dev);

	// SPI callback functions - gọi từ main.c
	void bme280_spi_tx_callback(BME280_dev_t *dev);
	void bme280_spi_txrx_callback(BME280_dev_t *dev);
	void bme280_spi_error_callback(BME280_dev_t *dev);

	// Hàm delay
	void bme280_delay_start(bme280_delay_t *delay, uint32_t ms);
	uint8_t bme280_delay_check(bme280_delay_t *delay);
#endif // BME280_H_
