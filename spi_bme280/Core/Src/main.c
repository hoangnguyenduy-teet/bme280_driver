#include "main.h"
#include "bme280.h"

SPI_HandleTypeDef hspi1;
BME280_dev_t bme280;
bme280_delay_t main_delay;


// Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void user_cs_control(uint8_t assert);


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  // Cấu hình BME280
  bme280.interface = BME280_INTERFACE_SPI;
  bme280.dev_id = 0;
  bme280.read = NULL;
  bme280.write = NULL;
  bme280.cs_control = user_cs_control;
  bme280.spi_handle = &hspi1;
  bme280.state = BME280_STATE_IDLE;

  // Khởi tạo SPI buffer
  bme280.spi_buf.tx_complete = 0;
  bme280.spi_buf.rx_complete = 0;
  memset(bme280.spi_buf.tx_buffer, 0, SPI_BUFFER_SIZE);
  memset(bme280.spi_buf.rx_buffer, 0, SPI_BUFFER_SIZE);

  if (bme280_init(&bme280) != BME280_OK)
  {
    Error_Handler();
  }

  // Bắt đầu quá trình đo
  bme280_set_state(&bme280, BME280_STATE_TRIGGER_MEASURE);

  while (1)
  {
    // Xử lý state machine BME280
    bme280_process(&bme280);

    // Chạy các task khác
    if (bme280_delay_check(&main_delay)){

    	// Đã hết delay, thực hiện công việc và restart delay
    	bme280_delay_start(&main_delay, 1);
    }
  }
}

// gọi các hàm callback của BME280 driver
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    bme280_spi_tx_callback(&bme280);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    bme280_spi_txrx_callback(&bme280);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    bme280_spi_error_callback(&bme280);
  }
}

// Điều khiển chân CS
void user_cs_control(uint8_t assert)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (assert) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Error_Handler(void)
{
  while (1);
}

// SystemClock_Config
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
