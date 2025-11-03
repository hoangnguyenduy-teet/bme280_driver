#include "main.h"
#include "bme280.h"

SPI_HandleTypeDef hspi1;

// Biến toàn cục cho BME280
BME280_dev_t bme280;
int32_t temperature, pressure, humidity;

// Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t ms);
void user_cs_control(uint8_t assert);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  // Cấu hình BME280
  bme280.interface = BME280_INTERFACE_SPI;
  bme280.dev_id = 0; // Không sử dụng cho SPI
  bme280.read = user_spi_read;
  bme280.write = user_spi_write;
  bme280.delay_ms = user_delay_ms;
  bme280.cs_control = user_cs_control;

  // Khởi tạo BME280
  if (bme280_init(&bme280) != BME280_OK)
  {
    Error_Handler();
  }

  while (1)
  {
    // Kích hoạt đo lường
    if (bme280_trigger_forced_measurement(&bme280) != BME280_OK)
    {
      Error_Handler();
    }

    // Đọc giá trị thô
    int32_t raw_temp, raw_press, raw_hum;
    if (bme280_read_raw_temp_press_hum(&bme280, &raw_temp, &raw_press, &raw_hum) != BME280_OK)
    {
      Error_Handler();
    }

    // Bù giá trị
    if (bme280_compensate_all(&bme280, raw_temp, raw_press, raw_hum,
                              &temperature, &pressure, &humidity) != BME280_OK)
    {
      Error_Handler();
    }

    // Chuyển đổi đơn vị:
    // temperature: °C * 100 (ví dụ: 25.35°C = 2535)
    // pressure: Pa * 100 (ví dụ: 101325 Pa = 10132500)
    // humidity: % * 1024 (ví dụ: 50% = 51200)

    HAL_Delay(5000); // Đợi 5 giây giữa các lần đo
  }
}

// Hàm đọc SPI
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  uint8_t tx_buffer[32];
  uint8_t rx_buffer[32];

  // Thêm bit read vào địa chỉ register
  tx_buffer[0] = reg_addr | BME280_SPI_READ;

  if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, len + 1, HAL_MAX_DELAY) != HAL_OK)
    return BME280_E_COMM_FAIL;

  // Copy dữ liệu nhận được (bỏ byte đầu tiên)
  memcpy(data, &rx_buffer[1], len);
  return BME280_OK;
}

// Hàm ghi SPI
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
  uint8_t tx_buffer[32];

  // Thêm bit write vào địa chỉ register
  tx_buffer[0] = reg_addr & BME280_SPI_WRITE;

  // Copy dữ liệu cần ghi
  memcpy(&tx_buffer[1], data, len);

  if (HAL_SPI_Transmit(&hspi1, tx_buffer, len + 1, HAL_MAX_DELAY) != HAL_OK)
    return BME280_E_COMM_FAIL;

  return BME280_OK;
}

// Hàm điều khiển chip-select
void user_cs_control(uint8_t assert)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (assert) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// Hàm delay
void user_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

// Cấu hình SPI1
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Cấu hình GPIO
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Cấu hình chân CS (PA4)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Mặc định CS ở trạng thái không active
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Error_Handler(void)
{
  while (1);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

