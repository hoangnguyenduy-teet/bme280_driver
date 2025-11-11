 #include "bme280.h"

// Forward declarations
static int bme280_spi_read(BME280_dev_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);
static int bme280_spi_write(BME280_dev_t *dev, uint8_t reg_addr, const uint8_t *data, uint16_t len);


static int bme280_read_reg(BME280_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
  if (!dev || !dev->read) return BME280_E_NULL_PTR;

  if (dev->interface == BME280_INTERFACE_SPI) {
    // Sử dụng SPI interrupt
    return bme280_spi_read(dev, reg, buf, len);
  } else {
    // I2C implementation (giữ nguyên)
    if (dev->cs_control) dev->cs_control(1);
    int ret = dev->read(dev->dev_id, reg, buf, len);
    if (dev->cs_control) dev->cs_control(0);
    return (ret == 0) ? BME280_OK : BME280_E_COMM_FAIL;
  }
}

static int bme280_write_reg(BME280_dev_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len) {
  if (!dev || !dev->write) return BME280_E_NULL_PTR;

  if (dev->interface == BME280_INTERFACE_SPI) {
    // Sử dụng SPI interrupt
    return bme280_spi_write(dev, reg, buf, len);
  } else {
    // I2C implementation (giữ nguyên)
    if (dev->cs_control) dev->cs_control(1);
    int ret = dev->write(dev->dev_id, reg, buf, len);
    if (dev->cs_control) dev->cs_control(0);
    return (ret == 0) ? BME280_OK : BME280_E_COMM_FAIL;
  }
}


  int bme280_read_id(BME280_dev_t *dev, uint8_t *id) {
	  if (!dev || !id) return BME280_E_NULL_PTR;
	  uint8_t tmp = 0;
	  if (bme280_read_reg(dev, BME280_REG_ID, &tmp, 1) != BME280_OK)
		  return BME280_E_COMM_FAIL;
	*id = tmp;
	return BME280_OK;
  }

  int bme280_soft_reset(BME280_dev_t *dev) {
	  if (!dev) return BME280_E_COMM_FAIL;
	  uint8_t v = BME280_SOFTRESET_VAL;
	  int8_t r = bme280_write_reg(dev, BME280_REG_RESET, &v, 1);
	  if (r != BME280_OK) return r;
	  dev->delay_ms(2);
	  return BME280_OK;
  }

  static int bme280_read_calib(BME280_dev_t *dev) {
	  uint8_t buf[26];
	  if (bme280_read_reg(dev, BME280_REG_CALIB_00, buf, 24) != BME280_OK)
		  return BME280_E_COMM_FAIL;
	// Byte-concentration
	dev->calib.dig_T1 = (uint16_t)(buf[0] | (buf[1] << 8));
	dev->calib.dig_T2 = (int16_t)(buf[2] | (buf[3] << 8));
	dev->calib.dig_T3 = (int16_t)(buf[4] | (buf[5] << 8));

	dev->calib.dig_P1 = (uint16_t)(buf[6] | (buf[7] << 8));
	dev->calib.dig_P2 = (int16_t)(buf[8] | (buf[9] << 8));
	dev->calib.dig_P3 = (int16_t)(buf[10] | (buf[11] << 8));
	dev->calib.dig_P4 = (int16_t)(buf[12] | (buf[13] << 8));
	dev->calib.dig_P5 = (int16_t)(buf[14] | (buf[15] << 8));
	dev->calib.dig_P6 = (int16_t)(buf[16] | (buf[17] << 8));
	dev->calib.dig_P7 = (int16_t)(buf[18] | (buf[19] << 8));
	dev->calib.dig_P8 = (int16_t)(buf[20] | (buf[21] << 8));
	dev->calib.dig_P9 = (int16_t)(buf[22] | (buf[23] << 8));

	// Read H1
	if (bme280_read_reg(dev,  BME280_REG_CALIB_H1, &buf[24], 1) != BME280_OK)
		return BME280_E_COMM_FAIL;
	dev->calib.dig_H1 = buf[24];
	// Read H2-H6
	uint8_t buf_h[7];
    if (bme280_read_reg(dev, BME280_REG_CALIB_H2_LSB, buf_h, 7) != BME280_OK) return BME280_E_COMM_FAIL;
    dev->calib.dig_H2 = (int16_t)(buf_h[0] | (buf_h[1] << 8));
    dev->calib.dig_H3 = buf_h[2];
    dev->calib.dig_H4 = (int16_t)((buf_h[3] << 4) | (buf_h[4] & 0x0F));
    dev->calib.dig_H5 = (int16_t)((buf_h[5] << 4) | (buf_h[4] >> 4));
    dev->calib.dig_H6 = (int8_t)buf_h[6];

    return BME280_OK; // Return OK if all read and byte-concentration is success
  }

 int bme280_init(BME280_dev_t *dev) {
	 if(!dev) return BME280_E_NULL_PTR; // Kiểm tra tính hợp lệ của con trỏ dev

	 // Đọc ID chip
	 uint8_t id;
	 if (bme280_read_id(dev, &id) != BME280_OK) return BME280_E_COMM_FAIL;
	 if (id != BME280_CHIP_ID) return BME280_E_DEV_NOT_FOUND;
	 bme280_soft_reset(dev);
	 if (bme280_read_calib(dev) != BME280_OK) return BME280_E_COMM_FAIL;
	 // Thiết lập oversampling độ ẩm
	 uint8_t ctrl_hum = BME280_OSRS_1 & 0x07;
	 if (bme280_read_reg(dev, BME280_REG_CTRL_HUM, &ctrl_hum, 1) != BME280_OK)
		 return BME280_E_COMM_FAIL;
	// Thiết lập ctrl_meas
	uint8_t ctrl_meas = (BME280_OSRS_1 << 5) | (BME280_OSRS_1 << 2) | BME280_MODE_SLEEP;
	if (bme280_read_reg(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1) != BME280_OK)
		return BME280_E_COMM_FAIL;
	// Thiết lập config với cấu hình mặc định
	uint8_t config = BME280_TSB_125_MS | BME280_FILTER_OFF;
	if (bme280_read_reg(dev, BME280_REG_CONFIG, &config, 1) != BME280_OK)
		return BME280_E_COMM_FAIL;

	return BME280_OK;
 }

 int bme280_trigger_forced_measurement(BME280_dev_t *dev) {
	 if (!dev) return BME280_E_NULL_PTR; // kiểm tra con trỏ dev
	 uint8_t ctrl_meas;
	 if (bme280_read_reg(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1) != BME280_OK)
		 return BME280_E_COMM_FAIL;
	ctrl_meas = (ctrl_meas & 0xFC) | BME280_MODE_FORCED;
    if (bme280_write_reg(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1) != BME280_OK)
	    return BME280_E_COMM_FAIL;

	uint8_t status = 0;
	do {
        if (bme280_read_reg(dev, BME280_REG_STATUS, &status, 1) != BME280_OK) return BME280_E_COMM_FAIL;
        dev->delay_ms(2);
    } while (status & 0x08);

	return BME280_OK;
 }

 // Đọc giá trị ADC thô của temp, hum, press
 int bme280_read_raw_temp_press_hum(BME280_dev_t *dev, int32_t *raw_temp, int32_t *raw_p, int32_t *raw_h) {
	 if (!dev || !raw_temp || !raw_p || !raw_h) return BME280_E_NULL_PTR;
    uint8_t buf[8];
    if (bme280_read_reg(dev, BME280_REG_PRESS_MSB, buf, 8) != BME280_OK) return BME280_E_COMM_FAIL;

    int32_t adc_p = (int32_t)((((uint32_t)buf[0]) << 12) | (((uint32_t)buf[1]) << 4) | (buf[2] >> 4));
    int32_t adc_temp = (int32_t)((((uint32_t)buf[3]) << 12) | (((uint32_t)buf[4]) << 4) | (buf[5] >> 4));
    int32_t adc_h = (int32_t)((((uint32_t)buf[6]) << 8) | buf[7]);

    *raw_p = adc_p;
    *raw_temp = adc_temp;
    *raw_h = adc_h;
    return BME280_OK;
 }

 // Xử lý các giá trị ADC thô
 int bme280_compensate_all( BME280_dev_t *dev, int32_t raw_temp, int32_t raw_p, int32_t raw_h, int32_t *temp_x100, int32_t *press_x100, int32_t *hum_x1024)
{
    if (!dev) return BME280_E_NULL_PTR; // Kiểm tra con trỏ dev

    int32_t var1, var2, T;
    uint16_t dig_T1 = dev->calib.dig_T1;
    int16_t dig_T2 = dev->calib.dig_T2;
    int16_t dig_T3 = dev->calib.dig_T3;

    var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - (int32_t)dig_T1) * ((raw_temp >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    if (temp_x100) *temp_x100 = T;

    int64_t var1_p, var2_p, p;
    int64_t adc_P = raw_p;
    int16_t dig_P1 = dev->calib.dig_P1;
    int16_t dig_P2 = dev->calib.dig_P2;
    int16_t dig_P3 = dev->calib.dig_P3;
    int16_t dig_P4 = dev->calib.dig_P4;
    int16_t dig_P5 = dev->calib.dig_P5;
    int16_t dig_P6 = dev->calib.dig_P6;
    int16_t dig_P7 = dev->calib.dig_P7;
    int16_t dig_P8 = dev->calib.dig_P8;
    int16_t dig_P9 = dev->calib.dig_P9;

    var1_p = ((int64_t)dev->t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)dig_P5) << 17);
    var2_p = var2_p + (((int64_t)dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)dig_P3) >> 8) + ((var1_p * (int64_t)dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p) * ((int64_t)dig_P1)) >> 33;

    if (var1_p == 0) {
        if (press_x100) *press_x100 = 0;
    } else {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dig_P7) << 4);
        int64_t p_pa = p >> 8;
        if (press_x100) *press_x100 = (int32_t)(p_pa * 100);
    }

    int32_t v_x1_u32r;
    int16_t dig_H1 = dev->calib.dig_H1;
    int16_t dig_H2 = dev->calib.dig_H2;
    int16_t dig_H3 = dev->calib.dig_H3;
    int16_t dig_H4 = dev->calib.dig_H4;
    int16_t dig_H5 = dev->calib.dig_H5;
    int8_t  dig_H6 = dev->calib.dig_H6;

    v_x1_u32r = dev->t_fine - ((int32_t)76800);
    v_x1_u32r = (((((raw_h << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                    ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    uint32_t hum = (uint32_t)(v_x1_u32r >> 12);
    if (hum_x1024) *hum_x1024 = (int32_t)hum;

    return BME280_OK;
}


// State machine xử lý BME280
void bme280_process(BME280_dev_t *dev)
{
  static int32_t raw_temp, raw_press, raw_hum;

  if (!dev) return;

  switch(dev->state)
  {
    case BME280_STATE_TRIGGER_MEASURE:
      // Kích hoạt đo lường bằng interrupt
      if (bme280_trigger_forced_measurement(dev) == BME280_OK)
      {
        dev->measurement_start_time = HAL_GetTick();
        dev->state = BME280_STATE_WAIT_MEASURE;
      }
      break;

    case BME280_STATE_WAIT_MEASURE:
      // Chờ measurement hoàn thành (khoảng 10ms cho BME280)
      if (HAL_GetTick() - dev->measurement_start_time > 20)
      {
        dev->state = BME280_STATE_READ_RAW_DATA;
      }
      break;

    case BME280_STATE_READ_RAW_DATA:
      // Đọc dữ liệu thô bằng interrupt
      if (bme280_read_raw_temp_press_hum(dev, &raw_temp, &raw_press, &raw_hum) == BME280_OK)
      {
        dev->state = BME280_STATE_PROCESS_DATA;
      }
      break;

    case BME280_STATE_PROCESS_DATA:
      // Xử lý dữ liệu
      int32_t temperature, pressure, humidity;
      if (bme280_compensate_all(dev, raw_temp, raw_press, raw_hum,
                                &temperature, &pressure, &humidity) == BME280_OK)
      {


        // Measurement hoàn thành, chuyển về trạng thái idle
        dev->state = BME280_STATE_IDLE;
      }
      break;

    case BME280_STATE_IDLE:
    default:
      break;
  }
}

static int bme280_spi_read(BME280_dev_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (!dev || len + 1 > SPI_BUFFER_SIZE || !dev->spi_handle) return BME280_E_COMM_FAIL;

    // Reset flags
    dev->spi_buf.tx_complete = 0;
    dev->spi_buf.rx_complete = 0;

    // Chuẩn bị buffer truyền
    uint8_t local_tx_buffer[SPI_BUFFER_SIZE];
    uint8_t local_rx_buffer[SPI_BUFFER_SIZE];

    local_tx_buffer[0] = reg_addr | BME280_SPI_READ;
    for (uint16_t i = 0; i < len; i++) {
        local_tx_buffer[i + 1] = 0xFF;
    }

    // Active CS
    if (dev->cs_control) dev->cs_control(1);

    // SỬA: sử dụng dev->spi_handle thay vì &hspi1
    if (HAL_SPI_TransmitReceive_IT(dev->spi_handle, local_tx_buffer, local_rx_buffer, len + 1) != HAL_OK) {
        if (dev->cs_control) dev->cs_control(0);
        return BME280_E_COMM_FAIL;
    }

    // Chờ hoàn thành
    uint32_t timeout = HAL_GetTick();
    while (!dev->spi_buf.rx_complete) {
        if (HAL_GetTick() - timeout > 100) {
            if (dev->cs_control) dev->cs_control(0);
            return BME280_E_COMM_FAIL;
        }
    }

    // Copy dữ liệu nhận được
    for (uint16_t i = 0; i < len; i++) {
        data[i] = local_rx_buffer[i + 1];
    }

    // De-active CS
    if (dev->cs_control) dev->cs_control(0);

    return BME280_OK;
}

// Hàm ghi SPI với interrupt - SỬA: sử dụng dev->spi_handle
static int bme280_spi_write(BME280_dev_t *dev, uint8_t reg_addr, const uint8_t *data, uint16_t len) {
    if (!dev || len + 1 > SPI_BUFFER_SIZE || !dev->spi_handle) return BME280_E_COMM_FAIL;

    // Reset flag
    dev->spi_buf.tx_complete = 0;

    // Chuẩn bị buffer truyền
    uint8_t local_tx_buffer[SPI_BUFFER_SIZE];

    local_tx_buffer[0] = reg_addr & BME280_SPI_WRITE;
    for (uint16_t i = 0; i < len; i++) {
        local_tx_buffer[i + 1] = data[i];
    }

    // Active CS
    if (dev->cs_control) dev->cs_control(1);

    // SỬA: sử dụng dev->spi_handle thay vì &hspi1
    if (HAL_SPI_Transmit_IT(dev->spi_handle, local_tx_buffer, len + 1) != HAL_OK) {
        if (dev->cs_control) dev->cs_control(0);
        return BME280_E_COMM_FAIL;
    }

    // Chờ hoàn thành
    uint32_t timeout = HAL_GetTick();
    while (!dev->spi_buf.tx_complete) {
        if (HAL_GetTick() - timeout > 100) {
            if (dev->cs_control) dev->cs_control(0);
            return BME280_E_COMM_FAIL;
        }
    }

    // De-active CS
    if (dev->cs_control) dev->cs_control(0);

    return BME280_OK;
}

// SPI callback functions - được gọi từ HAL callbacks trong main.c
void bme280_spi_tx_callback(BME280_dev_t *dev)
{
  if (dev) dev->spi_buf.tx_complete = 1;
}

void bme280_spi_txrx_callback(BME280_dev_t *dev)
{
  if (dev) dev->spi_buf.rx_complete = 1;
}

void bme280_spi_error_callback(BME280_dev_t *dev)
{
  if (dev) {
    dev->spi_buf.tx_complete = 1;
    dev->spi_buf.rx_complete = 1;
  }
}

// Helper functions để quản lý state
void bme280_set_state(BME280_dev_t *dev, bme280_state_t state)
{
  if (dev) dev->state = state;
}

bme280_state_t bme280_get_state(BME280_dev_t *dev)
{
  return dev ? dev->state : BME280_STATE_IDLE;
}

// Hàm non-blocking delay
void bme280_delay_start(bme280_delay_t *delay, uint32_t ms){
	delay->start_time = HAL_GetTick();
	delay->delay_ms = ms;
	delay->is_running = 1;
}

uint8_t bme280_delay_check(bme280_delay_t *delay){
	if (!delay->is_running){
		return 1; // Hàm không chạy
	}
	if ((HAL_GetTick() - delay->start_time) >= delay->delay_ms) {
	        delay->is_running = 0;
	        return 1; // Đã hoàn thành
	    }

	return 0; // Chưa hoàn thành
}

