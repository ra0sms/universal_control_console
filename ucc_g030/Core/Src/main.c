/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void USART1_Send_String (char* str);
char* ConvertGradusToChar (uint32_t degree);
void Check_Usart_Data();
void USART1_Send (char chr);
uint32_t ConvertCharToGradus(char * buffer);
void Send_Azimuth_to_USART(int az);
uint8_t Button_Pressed(uint8_t btn_id);
void Init_Buttons();
void Buttons_Poll(void);
void Set_N(void);
void Set_NE(void);
void Set_E(void);
void Set_SE(void);
void Set_S(void);
void Set_SW(void);
void Set_W(void);
void Set_NW(void);
void Set_300(void);
void Set_390(void);
void Set_430(void);
void Set_470(void);
void Set_510(void);
void Set_560(void);
void Set_PA(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t relay_outs[1] = {0};
uint8_t load_outs[1] = {0};
uint8_t udn_outs[1] = {0};
uint8_t led_outs[1] = {0};
uint8_t flag_stop = 0;
uint8_t flag_status = 0;
uint8_t flag_move = 0;
uint8_t flag_set_settings = 0;
uint8_t flag_get_settings = 0;
int current_azimuth = 0;
char rx_buffer[10];
uint8_t current_outs_settings[8];

uint8_t button_settings[8] = {1, 2, 4, 8, 16, 32, 64, 128};

#define BUTTON_COUNT 15
#define DEBOUNCE_MS 25
#define POLL_INTERVAL_MS 5

typedef struct {
  GPIO_TypeDef* port;
  uint32_t pin;
  uint8_t state;
  uint8_t prev_state;
  uint32_t last_change;
} Button;

static Button buttons[BUTTON_COUNT];
static uint32_t system_tick = 0;

// Объявляем тип для функций-обработчиков
typedef void (*ButtonHandler)(void);

// Массив обработчиков для каждой кнопки
static const ButtonHandler button_handlers[] = {
  Set_N,   // 0
  Set_NE,  // 1
  Set_E,   // 2
  Set_SE,  // 3
  Set_S,   // 4
  Set_SW,  // 5
  Set_W,   // 6
  Set_NW,  // 7
  Set_300, // 8
  Set_390, // 9
  Set_430, // 10
  Set_470, // 11
  Set_510, // 12
  Set_560, // 13
  Set_PA   // 14
};


#define SETTINGS_FLASH_ADDR  0x0800F800  // Последняя страница 2KB Flash
#define SETTINGS_MAGIC_NUM   0xAA55BB66  // Маркер валидности данных

typedef struct {
  uint32_t magic;      // Маркер валидности
  uint8_t btn_values[8]; // Значения кнопок (0-255)
  uint32_t crc32;      // Контрольная сумма
} ButtonSettings;

uint32_t Calculate_CRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

HAL_StatusTypeDef Save_Button_Settings(const uint8_t* values) {
  ButtonSettings settings;

  // Подготовка структуры
  settings.magic = SETTINGS_MAGIC_NUM;
  memcpy(settings.btn_values, values, 8);
  settings.crc32 = Calculate_CRC32((uint8_t*)&settings, sizeof(ButtonSettings)-4);

  // Стирание страницы
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Page = (SETTINGS_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE,
    .NbPages = 1
};
  uint32_t page_error;
  HAL_FLASHEx_Erase(&erase, &page_error);

  // Запись данных
  uint64_t* data = (uint64_t*)&settings;
  for (int i = 0; i < sizeof(ButtonSettings)/8; i++) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                    SETTINGS_FLASH_ADDR + i*8,
                    data[i]);
  }
  HAL_FLASH_Lock();

  return HAL_OK;
}


HAL_StatusTypeDef Load_Button_Settings(uint8_t* values) {
  ButtonSettings* settings = (ButtonSettings*)SETTINGS_FLASH_ADDR;

  // Проверка маркера и CRC
  if (settings->magic != SETTINGS_MAGIC_NUM ||
      settings->crc32 != Calculate_CRC32((uint8_t*)settings, sizeof(ButtonSettings)-4)) {
    return HAL_ERROR; // Данные невалидны
      }

  memcpy(values, settings->btn_values, 8);
  return HAL_OK;
}

void Beep(uint32_t duration_ms) {
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(duration_ms);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
}

void Buttons_Poll(void) {
  system_tick += POLL_INTERVAL_MS;
  if (system_tick >= UINT32_MAX - POLL_INTERVAL_MS) system_tick = 0;
}


void Init_Buttons(void) {
  // Конфигурация всех кнопок
  const struct {
    GPIO_TypeDef* port;
    uint32_t pin;
  } button_configs[] = {
    [0] = {BTN_N_GPIO_Port, BTN_N_Pin},
    [1] = {BTN_NE_GPIO_Port, BTN_NE_Pin},
    [2] = {BTN_E_GPIO_Port, BTN_E_Pin},
    [3] = {BTN_SE_GPIO_Port, BTN_SE_Pin},
    [4] = {BTN_S_GPIO_Port, BTN_S_Pin},
    [5] = {BTN_SW_GPIO_Port, BTN_SW_Pin},
    [6] = {BTN_W_GPIO_Port, BTN_W_Pin},
    [7] = {BTN_NW_GPIO_Port, BTN_NW_Pin},
    [8] = {BTN_300_GPIO_Port, BTN_300_Pin},
    [9] = {BTN_390_GPIO_Port, BTN_390_Pin},
    [10] = {BTN_430_GPIO_Port, BTN_430_Pin},
    [11] = {BTN_470_GPIO_Port, BTN_470_Pin},
    [12] = {BTN_510_GPIO_Port, BTN_510_Pin},
    [13] = {BTN_560_GPIO_Port, BTN_560_Pin},
    [14] = {BTN_PA_GPIO_Port, BTN_PA_Pin}
  };
  const uint8_t button_count = sizeof(button_configs)/sizeof(button_configs[0]);
  for (uint8_t i = 0; i < button_count; i++) {
    buttons[i] = (Button){
      .port = button_configs[i].port,
      .pin = button_configs[i].pin,
      .state = 0,          // Исходное состояние - не нажата
      .prev_state = 0,     // Предыдущее состояние - не нажата
      .last_change = 0     // Время последнего изменения
    };
  }
}


// Проверка нажатия кнопки (возвращает 1 при нажатии)
uint8_t Button_Pressed(uint8_t btn_id) {
  if (btn_id >= BUTTON_COUNT) return 0;

  Button* btn = &buttons[btn_id];
  uint8_t current = LL_GPIO_IsInputPinSet(btn->port, btn->pin) ? 0 : 1;

  // Если состояние изменилось
  if (current != btn->prev_state) {
    btn->last_change = system_tick;
    btn->prev_state = current;
  }
  // Если состояние стабильно дольше времени дребезга
  else if ((system_tick - btn->last_change) >= DEBOUNCE_MS) {
    if (current && !btn->state) {
      btn->state = current;
      return 1;
    }
    btn->state = current;
  }

  return 0;
}


uint32_t ConvertCharToGradus(char* buffer) {
  uint32_t result = (buffer[1] - '0') * 100 +
                   (buffer[2] - '0') * 10 +
                   (buffer[3] - '0');
  memset(buffer, 0, 6);
  return result;
}

char* ConvertGradusToChar(uint32_t degree)
{
  static char result[10];
  degree = degree % 1000;
  snprintf(result, sizeof(result), "+0%03d\r\n", degree);
  return result;
}

void Send_Azimuth_to_USART(int az) {
  if (((az>=0) && (az<= 22)) ||(az > 337)&&(az<=360))
    current_azimuth = 0;
  if ((az>=23) && (az<= 67))
    current_azimuth = 45;
  if ((az>=68) && (az<= 112))
    current_azimuth = 90;
  if ((az>=113) && (az<= 157))
    current_azimuth = 135;
  if ((az>=158) && (az<= 202))
    current_azimuth = 180;
  if ((az>=203) && (az<= 247))
    current_azimuth = 225;
  if ((az>=248) && (az<= 292))
    current_azimuth = 270;
  if ((az>=293) && (az<= 337))
    current_azimuth = 315;
  USART1_Send_String(ConvertGradusToChar(current_azimuth));
}

void Check_Usart_Data()
{
  if (flag_status)
  {
    flag_status = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
    USART1_Send_String(ConvertGradusToChar(current_azimuth));
  }
  else if (flag_move)
  {
    int azimuth_from_usart = 0;
    flag_move = 0;
    azimuth_from_usart = ConvertCharToGradus(rx_buffer);
    //set outputs
    Send_Azimuth_to_USART(azimuth_from_usart);
  }
  else if (flag_stop)
  {
    flag_stop = 0;
    USART1_Send_String("Command Stop\n");
  }
  else if (flag_set_settings){
    flag_set_settings = 0;

    if (Save_Button_Settings(current_outs_settings) == HAL_OK) {
      USART1_Send_String("OK: Settings saved\n");
    } else {
      USART1_Send_String("ERR: Save failed\n");
    }
    USART1_Send_String("Values:");
    for (int i = 0; i < 8; i++) {
      char buf[10];
      snprintf(buf, sizeof(buf), " %d", current_outs_settings[i]);
      USART1_Send_String(buf);
    }
    USART1_Send_String("\n");
    Beep(50);
  }
  else if (flag_get_settings) {
    flag_get_settings = 0;
    if (Load_Button_Settings(button_settings) != HAL_OK) {
      memset(button_settings, 0, 8);
    }
    USART1_Send_String("Values:");
    for (int i = 0; i < 8; i++) {
      char buf[10];
      snprintf(buf, sizeof(buf), " %d", current_outs_settings[i]);
      USART1_Send_String(buf);
    }
    USART1_Send_String("\n");
  }
}

void Say_Hi(void) {
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(150);
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);
  LL_mDelay(50);
}

void USART1_Send (char chr){
  while (!(USART1->ISR & USART_ISR_TC));
  USART1->TDR = chr;
}

void USART1_Send_String (char* str){
  uint8_t i = 0;
  while(str[i])
    USART1_Send (str[i++]);
}

void SetCS()
{
  LL_GPIO_SetOutputPin(RCLK_GPIO_Port, RCLK_Pin);
}

void ResetCS()
{
  LL_GPIO_ResetOutputPin(RCLK_GPIO_Port, RCLK_Pin);
}

void StrobCS()
{
  SetCS();
  ResetCS();
  LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);
}

void HC595_Send() {
  ResetCS();
  HAL_SPI_Transmit(&hspi1,(uint8_t*)led_outs, 1, 500);
  HAL_SPI_Transmit(&hspi1,(uint8_t*)udn_outs, 1, 500);
  HAL_SPI_Transmit(&hspi1,(uint8_t*)load_outs, 1, 500);
  HAL_SPI_Transmit(&hspi1,(uint8_t*)relay_outs, 1, 500);
  StrobCS();
}


void Check_Buttons() {
  for (uint8_t i = 0; i < sizeof(button_handlers)/sizeof(button_handlers[0]); i++) {
    if (Button_Pressed(i)) {
      button_handlers[i]();
    }
  }
}

void Set_N() {
  led_outs[0] = 0b00000001;
  relay_outs[0] = 0b00001000;
  udn_outs[0] = current_outs_settings[0];
  HC595_Send();
}

void Set_NE() {
  led_outs[0] = 0b00000010;
  relay_outs[0] = 0b00001001;
  udn_outs[0] = current_outs_settings[1];
  HC595_Send();
}

void Set_E() {
  led_outs[0] = 0b00000100;
  relay_outs[0] = 0b00001100;
  udn_outs[0] = current_outs_settings[2];
  HC595_Send();
}

void Set_SE() {
  led_outs[0] = 0b00001000;
  relay_outs[0] = 0b00000010;
  udn_outs[0] = current_outs_settings[3];
  HC595_Send();
}

void Set_S() {
  led_outs[0] = 0b00010000;
  relay_outs[0] = 0b00000000;
  udn_outs[0] = current_outs_settings[4];
  HC595_Send();
}

void Set_SW() {
  led_outs[0] = 0b00100000;
  relay_outs[0] = 0b00000001;
  udn_outs[0] = current_outs_settings[5];
  HC595_Send();
}

void Set_W() {\
  led_outs[0] = 0b01000000;
  relay_outs[0] = 0b00000100;
  udn_outs[0] = current_outs_settings[6];
  HC595_Send();
}

void Set_NW() {
  led_outs[0] = 0b10000000;
  relay_outs[0] = 0b00001010;
  udn_outs[0] = current_outs_settings[7];
  HC595_Send();
}

void Set_300() {
  load_outs[0] = 0b00000001;
  HC595_Send();
}

void Set_390() {
  load_outs[0] = 0b00000010;
  HC595_Send();
}

void Set_430() {
  load_outs[0] = 0b00000100;
  HC595_Send();
}

void Set_470() {
  load_outs[0] = 0b00001000;
  HC595_Send();
}

void Set_510() {
  load_outs[0] = 0b00010000;
  HC595_Send();
}

void Set_560() {
  load_outs[0] = 0b00100000;
  HC595_Send();
}

void Set_PA() {
  LL_GPIO_TogglePin(PA_ON_GPIO_Port, PA_ON_Pin);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  LL_USART_Enable(USART2);
  LL_USART_EnableIT_RXNE(USART1);
  Init_Buttons();
  relay_outs[0] = 0;
  load_outs[0] = 0;
  udn_outs[0] = 0;
  led_outs[0] = 0;
  HC595_Send();
  Say_Hi();
  Load_Button_Settings(current_outs_settings);
  if (current_outs_settings[0] == 0xFF) {
    Save_Button_Settings(button_settings);
    Load_Button_Settings(current_outs_settings);
  }

  /* USER CODE END 2 */

  /* Infinite loopc */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Check_Usart_Data();
    Buttons_Poll();
    Check_Buttons();
    LL_mDelay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  //hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  //hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 31999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9 [PA11]   ------> USART1_TX
  PA10 [PA12]   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RCLK_GPIO_Port, RCLK_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TX_GND_GPIO_Port, TX_GND_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PA_ON_GPIO_Port, PA_ON_Pin);

  /**/
  GPIO_InitStruct.Pin = BTN_470_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_470_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_390_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_390_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_430_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_430_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RCLK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RCLK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_510_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_510_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_560_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_560_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PTT_IN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(PTT_IN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_N_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_N_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_NE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_NE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_E_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_E_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TX_GND_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TX_GND_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_300_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_300_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PA_ON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PA_ON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_SE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_SE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_S_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_S_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_SW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_SW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_W_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_W_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_NW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_NW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_PA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_PA_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
