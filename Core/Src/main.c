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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "u8g2.h"
#include "MAX30100_PulseOximeter.h" // For Heart Rate sensor
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define WAVEFORM_BUFFER_SIZE 512 // Reduced buffer size for efficiency
// MERGE: Brought over the Circular Buffer definition
typedef struct {
    uint16_t buffer[WAVEFORM_BUFFER_SIZE];
    int head; // Index of the next position to write to
} CircularBuffer_t;

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} DS3231_Time;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Page state defines ---
#define PAGE_TEMP 0
#define PAGE_HEART_RATE 1

// --- Temperature Sensor Defines ---
#define MLX90614_I2C_ADDRESS    (0x5A << 1)
#define MLX90614_TOBJ1          0x07
#define DS3231_I2C_ADDRESS      (0x68 << 1) // Added RTC I2C address

// --- Heart Rate Sensor Defines ---
#define X_SCALE 4

// --- NEW: Power Management Variables ---
#define ACTIVE_TIMEOUT_MS 180000          // 2 minutes (3 * 60 * 1000)
#define ADC_BUF_LEN 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// --- Display & State Variables ---
u8g2_t u8g2;
uint8_t currentPage = PAGE_TEMP;
volatile uint8_t spi_dma_finished = 1; // MERGE: DMA flag

// --- Temperature Page Variables ---
float object_temp_c;
char temp_str[20], temp[30];

// --- Heart Rate Page Variables ---
PulseOximeter_t pox;
CircularBuffer_t waveform; // MERGE: Use circular buffer
float bpm = 0;
uint8_t spO2 = 0;
char statusStr[32] = "--";
char bpmStr[32] = "--";
uint16_t waveformMin = 65535;
uint16_t waveformMax = 0;
bool fan_is_running = false;
uint32_t fan_stop_timestamp = 0;
volatile bool onBeatDetected = false;
volatile bool beat_indicator_active = false;
volatile uint32_t beat_timestamp = 0;
volatile bool heartbeat_active = false;
volatile uint32_t heartbeat_timestamp = 0;

// --- NEW: Power Management Variables ---
volatile bool is_active = false;            // Start in the inactive (sleep) state
volatile uint32_t last_motion_time = 0;   // Timestamp of the last detected motion
volatile bool just_woke_up = false;         // NEW: Flag to handle tasks right after waking
volatile bool is_in_calibration = true;   // NEW: Flag for 1-minute startup calibration

//Flame sensor
uint16_t adc_buf[ADC_BUF_LEN];
volatile bool adcDataReadyFlag = false;
uint16_t avg_adc_val=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// Display driver prototypes
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

// Page drawing prototypes
void drawTemperaturePage(void);
void drawHeartRatePage(void);

// Helper function prototypes
void I2C_ClearBus(I2C_HandleTypeDef *hi2c);
void onBeatDetected_callback();
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void CBuffer_init(CircularBuffer_t *cb);
void CBuffer_add(CircularBuffer_t *cb, uint16_t value);
uint16_t CBuffer_get(CircularBuffer_t *cb, int index);

//flame sensor
void flameSensor(void);
void drawFlameAlertPage(int fan_speed_percent);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CBuffer_init(CircularBuffer_t *cb) {
    cb->head = 0;
    memset(cb->buffer, 0, sizeof(cb->buffer));
}

void CBuffer_add(CircularBuffer_t *cb, uint16_t value) {
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % WAVEFORM_BUFFER_SIZE;
}

uint16_t CBuffer_get(CircularBuffer_t *cb, int index) {
    int physical_index = (cb->head + index) % WAVEFORM_BUFFER_SIZE;
    return cb->buffer[physical_index];
}

void I2C_ClearBus(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // First, de-initialize the I2C peripheral to take control of the pins
    HAL_I2C_DeInit(hi2c);

    // Re-configure the SCL pin as a GPIO output (open-drain)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
    // Toggle SCL 9 times to clear the bus
		for(int i = 0; i < 9; i++) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_Delay(1);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
			{
				break; // SDA released
			}
		}

    }
    // After clearing, re-initialize the I2C peripheral
    MX_I2C1_Init();
}

// This will be used to map temperature to the thermometer bar's width.
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drawHeart(u8g2_t *u8g2, int x, int y, int size)
{
    // Two circles for the top lobes
    u8g2_DrawDisc(u8g2, x - size/4, y, size/4, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, x + size/4, y, size/4, U8G2_DRAW_ALL);
    // Triangle for the bottom point
    u8g2_DrawTriangle(u8g2, x - size/2, y, x + size/2, y, x, y + size/2 + 1);
}

HAL_StatusTypeDef MLX90614_ReadReg(uint8_t reg_address, uint16_t *data) {
    HAL_StatusTypeDef status;
    uint8_t rx_buffer[3];
    status = HAL_I2C_Mem_Read(&hi2c1, MLX90614_I2C_ADDRESS, reg_address,
                              I2C_MEMADD_SIZE_8BIT, rx_buffer, 3, 100);
    if (status != HAL_OK) return status;
    *data = (uint16_t)rx_buffer[0] | ((uint16_t)rx_buffer[1] << 8);
    return HAL_OK;
}

float MLX90614_CalcTemp(uint16_t raw_data) {
    return (float)raw_data * 0.02 - 273.15;
}

uint8_t BCD_to_Decimal(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time *time) {
    uint8_t rx_buffer[7];
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, DS3231_I2C_ADDRESS, 0x00,
                              I2C_MEMADD_SIZE_8BIT, rx_buffer, 7, 100);
    if (status == HAL_OK) {
        time->seconds    = BCD_to_Decimal(rx_buffer[0]);
        time->minutes    = BCD_to_Decimal(rx_buffer[1]);
        time->hour       = BCD_to_Decimal(rx_buffer[2]);
        time->dayofmonth = BCD_to_Decimal(rx_buffer[4]);
        time->month      = BCD_to_Decimal(rx_buffer[5]);
        time->year       = BCD_to_Decimal(rx_buffer[6]);
    }
    return status;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // --- Start Timers, ADC, and PWM ---
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //Calibrate the adc on power-up for better accuracy
  HAL_ADCEx_Calibration_Start(&hadc1);
  //Read the sensor once to get the ambient level
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,ADC_BUF_LEN);

  u8g2_Setup_sh1106_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0); // Put display to wake

  I2C_ClearBus(&hi2c1); // Clear bus for both I2C sensors

  // MERGE: Initialize Heart Rate Sensor and Circular Buffer
  CBuffer_init(&waveform);
  PulseOximeter_init(&pox);
  // --- REGISTER THE CALLBACK ---
  BeatDetector_setOnBeatDetectedCallback(&pox.beatDetector, onBeatDetected_callback);

  if (PulseOximeter_begin(&pox, &hi2c1)) {
    PulseOximeter_setIRLedCurrent(&pox, MAX30100_LED_CURR_14_2MA);
  } else {
    // Handle heart rate sensor init error if needed
    while(1);
  }


  // --- CHANGE: Set initial state to ACTIVE and start timers ---
  is_active = true;
  just_woke_up = true; // Set to true to run wake-up routine once
  last_motion_time = HAL_GetTick(); // Start the 2-minute timer immediately
  uint32_t start_time = HAL_GetTick(); // Record the power-on time for calibration
  uint32_t last_sensor_read_time = 0;
  uint32_t last_display_update_time = 0;
  uint16_t raw_object_temp;
  uint8_t last_ir_state = GPIO_PIN_SET; // Start with the sensor uncovered
  uint32_t last_flip_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// MERGE: ALWAYS update the heart rate sensor to keep the algorithm running
		PulseOximeter_update(&pox);

	    // --- FLAME SENSOR (HIGHEST PRIORITY) ---
	    // 1. Check if new ADC data is available
	    if (adcDataReadyFlag == true) {
	        adcDataReadyFlag = false; // Reset the flag immediately
	        flameSensor(); // Calculate the average flame sensor value
	    }
	    // Let's define two thresholds now
	    const uint16_t FLAME_ALERT_THRESHOLD = 1000; // A very strong flame
	    const uint16_t FLAME_DETECT_THRESHOLD = 3500; // Any detectable flame
	    // 2. Check if a high-priority flame event has occurred
	    // Let's define a flame alert threshold, e.g., 3500
	    if (avg_adc_val < FLAME_DETECT_THRESHOLD && avg_adc_val > 10)
	    {
	        // --- FLAME ALERT MODE ---
	        fan_is_running = true; // Mark the fan as active
	        fan_stop_timestamp = 0;  // Reset the stop timer every time flame is seen

	        // 2. Set the Fan Speed based on flame intensity
	        if (avg_adc_val < FLAME_ALERT_THRESHOLD) { // Strong Flame
	            // Set fan to MAX SPEED (100%)
	        	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 48000);
		        // Sound the buzzer (assuming buzzer is on PB13)
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Active low buzzer

		        // Show the alert page
		        drawFlameAlertPage(100);
	        } else { // Weaker Flame (between 1000 and 3499)
	            // Set fan to HALF SPEED (50%)
	        	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 33600);
		        // Sound the buzzer (assuming buzzer is on PB13)
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Active low buzzer

		        // Show the alert page
		        drawFlameAlertPage(70);
	        }


	        // Skip all other logic
	        continue;
	    }
		 else{
		        // Check if the fan was running and needs to be turned off after a delay
		        if (fan_is_running) {
			        // --- NO FLAME DETECTED ---
			        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer OFF
					last_motion_time = HAL_GetTick();

		            if (fan_stop_timestamp == 0) {
		                // This is the first moment we've seen the flame is gone.
		                // Record the timestamp.
		            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 33600);
		                fan_stop_timestamp = HAL_GetTick();
		            }

		            // Check if 10 seconds (10000 ms) have passed since the flame disappeared
		            if (HAL_GetTick() - fan_stop_timestamp > 10000) {
		                // 10 seconds have passed, so turn the fan off and reset state
		                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Fan OFF
		                fan_is_running = false;
		                fan_stop_timestamp = 0;
		            }
		        }
		        // --- NEW: Handle the 1-minute calibration period ---
			if (is_in_calibration) {
				if (HAL_GetTick() - start_time > 60000) { // 60 seconds have passed
					is_in_calibration = false;
				}
			}

			if (is_active)
			{
				// --- CHECK IF WE JUST WOKE UP ---
				if (just_woke_up) {
					just_woke_up = false; // Clear the flag

					// Now that the clock is stable, wake up the display
					u8g2_SetPowerSave(&u8g2, 0);

					// Re-initialize the I2C bus and the HR sensor to give them a fresh start
					I2C_ClearBus(&hi2c1);

					// Wake the sensor up
					MAX30100_resume(&pox.hrm);
				}

				// --- ACTIVE MODE ---
			if (pox.latestIRValue > 0) {
				CBuffer_add(&waveform, pox.latestIRValue);
			}

			if(onBeatDetected == true){
				onBeatDetected = false;
				if(bpm>40 && (currentPage == PAGE_HEART_RATE)){
					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
						last_motion_time = HAL_GetTick();

					 // Trigger heart animation
					 heartbeat_active = true;
					 heartbeat_timestamp = HAL_GetTick();

						// Start the timer for the LED/Buzzer blip
						beat_indicator_active = true;
						beat_timestamp = HAL_GetTick();
				}
			}

			// This block is checked on every loop to see if it's time to turn things off.
			if (beat_indicator_active && (HAL_GetTick() - beat_timestamp > 50)) {
				// Turn OFF the LED and Buzzer
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

				beat_indicator_active = false; // Stop checking until the next beat
			}

			// --- 1. PERIODIC SENSOR READS AND CALCULATIONS (every 500ms) ---
			if(HAL_GetTick() - last_sensor_read_time >= 500){
				last_sensor_read_time = HAL_GetTick();
				//Read the temperature from the MLX90614 sensor
				if (MLX90614_ReadReg(MLX90614_TOBJ1, &raw_object_temp) == HAL_OK) {
					object_temp_c = MLX90614_CalcTemp(raw_object_temp);
				} else {
					object_temp_c = -99.9; // Use a specific value to indicate an error
				}
				// 2. Format the temperature value into a display-ready string
				snprintf(temp_str, sizeof(temp_str), "%.1f C", object_temp_c);

				// Get Heart Rate Sensor Data
				bpm = PulseOximeter_getHeartRate(&pox);
				spO2 = PulseOximeter_getSpO2(&pox);

				if (bpm > 40 && (currentPage == PAGE_HEART_RATE)) {
					sprintf(bpmStr, "%.0f", bpm);
					sprintf(statusStr, "%u%%", spO2);

						// Recalculate waveform scale
						waveformMin = 65535;
						waveformMax = 0;
						for (int i = 0; i < WAVEFORM_BUFFER_SIZE; ++i) {
							uint16_t val = CBuffer_get(&waveform, i);
							if (val > waveformMax) waveformMax = val;
							if (val < waveformMin) waveformMin = val;
						}
				} else {
					sprintf(bpmStr, "--");
					sprintf(statusStr, "--");
				}
			}
			// --- 2. CHECK FOR PAGE SWITCH (IR SENSOR) ---
			uint8_t current_ir_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

			// Check for a "falling edge" (from not-covered to covered)
			// and make sure enough time has passed since the last flip (debounce)
			if (current_ir_state == GPIO_PIN_RESET && last_ir_state == GPIO_PIN_SET && (HAL_GetTick() - last_flip_time > 500))
			{
				// Toggle the page
				currentPage = (currentPage == PAGE_TEMP) ? PAGE_HEART_RATE : PAGE_TEMP;

				// Record the time of this flip to prevent rapid re-triggering
				last_flip_time = HAL_GetTick();
			}
			// Update the last state for the next loop iteration
			last_ir_state = current_ir_state;

			// --- 3. DYNAMIC DISPLAY UPDATE ---
			uint32_t display_interval = (currentPage == PAGE_HEART_RATE) ? 40 : 200; // Fast for HR, slow for Temp
			if (HAL_GetTick() - last_display_update_time > display_interval) {
				last_display_update_time = HAL_GetTick();

				// Call the appropriate drawing function based on the current page
				if (currentPage == PAGE_TEMP) {
					drawTemperaturePage();
				} else {
					drawHeartRatePage();
				}
			}
			HAL_Delay(2);
			// Check if the 5-minute active timer has expired
			if (HAL_GetTick() - last_motion_time > ACTIVE_TIMEOUT_MS)
			{
				is_active = false;        // Go back to the inactive state
				u8g2_SetPowerSave(&u8g2, 1); // Put the display to sleep
			}
		  }
		  else
		  {
			  // --- DEEP SLEEP MODE ---
			  MAX30100_shutdown(&pox.hrm);

			  // Halt the CPU and wait for an interrupt (from the PIR sensor)
			  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  }
		 }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 47999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//set the flag
	adcDataReadyFlag = true;
}

void flameSensor(void){
  //1. Calculate the average of the buffer
  uint32_t sum=0;
  for(int i=0; i<ADC_BUF_LEN;i++){
	  sum+=adc_buf[i];
  }
  avg_adc_val = sum/ADC_BUF_LEN;

}

void drawFlameAlertPage(int fan_speed_percent)
{
    char fan_speed_str[20];
    sprintf(fan_speed_str, "Fan: %d%%", fan_speed_percent);

    // Simple fan animation state
    static int fan_state = 0;
    fan_state = (fan_state + 1) % 4; // Cycle through 4 states
    char fan_char[] = "|/-\\";

    u8g2_FirstPage(&u8g2);
    do {
        // --- Draw Alert Message ---
        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 5, 20, "FLAME ALERT!");

        // --- Draw Fan Speed and Animation ---
        u8g2_SetFont(&u8g2, u8g2_font_7x13B_tr);
        u8g2_DrawStr(&u8g2, 10, 45, fan_speed_str);

        // Draw the spinning character next to the text
        char animation_str[2] = { fan_char[fan_state], '\0' };
        u8g2_DrawStr(&u8g2, 80, 45, animation_str);

    } while (u8g2_NextPage(&u8g2));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // --- NEW: Ignore any triggers during the 1-minute startup calibration ---
    if (is_in_calibration) {
        return;
    }
    // Check if the interrupt was from the PIR Sensor (Motion)
    if (GPIO_Pin == GPIO_PIN_10) {
		// Any valid motion interrupt should reset the inactivity timer.
		last_motion_time = HAL_GetTick();

		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			// If the device was asleep, this interrupt is a wake-up call.
			// Set the flags that the main loop will use to exit sleep mode.
			if (is_active == false) {
				is_active = true;
				just_woke_up = true;
			}
    }
    // Check if the interrupt was from the Flame Sensor's Digital Out
    else if (GPIO_Pin == GPIO_PIN_1) {
        // A flame detection is a high-priority event.
        // Force the device to wake up immediately.
        is_active = true;
        just_woke_up = true;
    }
}

uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            HAL_Delay(1);
            break;
        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;
        case U8X8_MSG_GPIO_CS:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, arg_int);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1);
            break;
    }
    return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            spi_dma_finished = 0;
            HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)arg_ptr, arg_int);
            while (spi_dma_finished == 0); // Wait for current transfer
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            u8x8_gpio_SetDC(u8x8, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            break;
        default:
            return 0;
    }
    return 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    spi_dma_finished = 1;
  }
}

void drawTemperaturePage(void)
{
    u8g2_ClearBuffer(&u8g2);   // Clear old contents

    DS3231_Time currentTime;
    DS3231_GetTime(&currentTime);

    char time_str[10];
    char date_str[16];

    snprintf(time_str, sizeof(time_str), "%02d:%02d", currentTime.hour, currentTime.minutes);
    snprintf(date_str, sizeof(date_str), "%02d/%02d/20%02d", currentTime.dayofmonth, currentTime.month, currentTime.year);

	// 3. Draw the temperature on the OLED display
	u8g2_FirstPage(&u8g2);
	do {
        // --- Draw the top status bar line ---
        // --- Draw Status Bar ---
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        // Draw time on the left
        u8g2_DrawStr(&u8g2, 2, 10, time_str);
        // Draw date on the right
        u8g2_uint_t date_width = u8g2_GetStrWidth(&u8g2, date_str);
        u8g2_DrawStr(&u8g2, 126 - date_width, 10, date_str);
        // Draw separator line
        u8g2_DrawHLine(&u8g2, 0, 12, 128); // A line at y=12

        // --- Draw the SMALLER Thermometer Icon ---
        // Bulb (radius 6) and Stem (22 pixels tall)
        u8g2_DrawCircle(&u8g2, 20, 53, 6, U8G2_DRAW_ALL);
        u8g2_DrawRFrame(&u8g2, 17, 28, 7, 25, 2);

        // --- Animate the SMALLER fill ---
        // Using a 10°C to 40°C range, mapped to the new 18px stem height
        long mercury_height = map((long)(object_temp_c * 10), 100, 500, 0, 18);
        if (mercury_height < 0) mercury_height = 0;
        if (mercury_height > 18) mercury_height = 18;
        // Fill the bulb and stem
        u8g2_DrawDisc(&u8g2, 20, 53, 4, U8G2_DRAW_ALL);
        if (mercury_height > 0) {
            u8g2_DrawBox(&u8g2, 19, 47 - mercury_height, 3, mercury_height);
        }

        // --- Draw the Text Elements ---
        // Draw the "TEMPERATURE" label in a small font
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(&u8g2, 40, 28, "TEMPERATURE");

        // Format the temperature as a FLOAT with one decimal place
        snprintf(temp_str, sizeof(temp_str), "%.1f \xb0" "C", object_temp_c);
        // Draw the temperature value in a medium font
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
        u8g2_DrawStr(&u8g2, 40, 54, temp_str);

    } while (u8g2_NextPage(&u8g2));
}

/**
  * @brief  Draws the complete Heart Rate Monitor page.
  */
void drawHeartRatePage(void)
{
    // MERGE: This is the full drawing logic from our previous project
    u8g2_FirstPage(&u8g2);
    do {
        // --- Draw Title and Top Separator ---
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
        u8g2_DrawStr(&u8g2, 36, 6, "HEART MONITOR");
        u8g2_DrawHLine(&u8g2, 0, 8, 128);

        // --- Draw BPM Vitals (Left Side) ---
        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 8, 25, bpmStr);
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(&u8g2, 12, 35, "BPM");

        // --- Draw SpO2 Vitals (Right Side) ---
        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
        u8g2_DrawStr(&u8g2, 54, 25, statusStr);
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(&u8g2, 52, 35, "SpO2");

        // --- Heart Animation ---
        if (heartbeat_active) {
            uint32_t elapsed = HAL_GetTick() - heartbeat_timestamp;
            if (elapsed < 300) {
                float phase = (elapsed / 300.0f) * 3.14159f;
                float scale = 1.0f + 0.4f * sinf(phase);
                drawHeart(&u8g2, 108, 20, (int)(18 * scale));
            } else {
                heartbeat_active = false;
            }
        }
        if (!heartbeat_active) {
            drawHeart(&u8g2, 108, 20, 18); // Resting heart
        }

        // --- Draw Horizontal Separator ---
        u8g2_DrawHLine(&u8g2, 0, 38, 128);

        // --- Draw Waveform ---
        if (bpm > 40) {
            if (waveformMax == waveformMin) {
                waveformMax = waveformMin + 1; // Avoid division by zero
            }
            for (int i = X_SCALE; i < WAVEFORM_BUFFER_SIZE; i += X_SCALE) {
                int32_t p1 = map(CBuffer_get(&waveform, i-X_SCALE), waveformMin, waveformMax, 63, 45);
                int32_t p2 = map(CBuffer_get(&waveform, i), waveformMin, waveformMax, 63, 45);
                u8g2_DrawLine(&u8g2, (i - X_SCALE) / X_SCALE, p1, i / X_SCALE, p2);
            }
        }
    } while (u8g2_NextPage(&u8g2));
}


void onBeatDetected_callback()
{
	onBeatDetected = true;
}

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
