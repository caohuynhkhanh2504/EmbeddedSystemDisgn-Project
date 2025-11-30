/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * Chức năng: Ứng dụng hiển thị và điều chỉnh nhiệt độ/độ ẩm sử dụng LCD 16x2 I2C 
  * và 4 nút bấm (OK, UP, DOWN, RIGHT).
  * * Sử dụng thư viện I2C LCD bên ngoài (i2c_lcd.h/c).
  * ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>    
#include <stdlib.h>
#include "i2c_lcd.h"  
#include "mk_dht11.h"
#include "ds1302.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SLAVE_ADDR 0x4E 
#define DHT11_READ_INTERVAL 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MOTOR_START_DELAY 20000
#define MOTOR_PULSE_PERIOD 10         
#define MOTOR_STEPS_PER_REV 200         
#define MOTOR_MICROSTEP 4               
#define MOTOR_STEPS_PER_CYCLE (MOTOR_STEPS_PER_REV * MOTOR_MICROSTEP) 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t motor_enabled = 0;        
uint32_t motor_start_time = 0;  
uint32_t last_motor_pulse = 0;    
uint8_t motor_pulse_state = 0;   
uint32_t motor_step_count = 0;   
uint8_t motor_initialized = 0;

I2C_LCD_HandleTypeDef lcd1;
dht11_t dht11_sensor;

// Trạng thái (0: Màn hình 1, 1: Màn hình 2, 2: Màn hình 3 (Set Nhiệt), 3: Màn hình 4 (Set Độ ẩm))
int current_screen = 0; 

float set_temp = 25.0; 
float set_hum = 60.0;
float measured_temp = 30.5;
float measured_hum = 75.2;
uint8_t read_status = 0;
char buffer[17]; 

uint32_t last_dht11_read = 0;
const uint32_t DHT11_INTERVAL = 2000; // 2s

// Thời gian debounce cho nút bấm
uint32_t last_button_time = 0; 
const uint32_t DEBOUNCE_TIME = 200; // 200ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

int is_button_pressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Handle_Buttons(void);
void Update_LCD_Display(void);
void Read_DHT11(void);
void Handle_Motor(void);
void Control_Outputs(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Control_Outputs(void)
{
    
    if (measured_temp <= set_temp - 2)
    {
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);  
    }
    else if (measured_temp >= set_temp + 2)
    {
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET); 
    }
    
    if (measured_hum <= set_hum - 2)
    {
        HAL_GPIO_WritePin(HUMI_GPIO_Port, HUMI_Pin, GPIO_PIN_SET);   
    }
    else if (measured_hum >= set_hum + 2)
    {
        HAL_GPIO_WritePin(HUMI_GPIO_Port, HUMI_Pin, GPIO_PIN_RESET); 
    }
}

void Read_DHT11(void)
{
	if (HAL_GetTick() - last_dht11_read >= DHT11_INTERVAL)
    {
        last_dht11_read = HAL_GetTick();
        read_status = readDHT11(&dht11_sensor);
        
        if(read_status == 1)
        {
            measured_temp = dht11_sensor.temperature;
            measured_hum = dht11_sensor.humidty;
        }
    }
}
	
void Handle_Motor(void)
{
    uint32_t current_time = HAL_GetTick();
    
    if (!motor_initialized)
    {
        motor_start_time = current_time;
        motor_initialized = 1;
    }
    
    // Kiểm tra nếu đã đủ thời gian để bật motor
    if (!motor_enabled && (current_time - motor_start_time >= MOTOR_START_DELAY))
    {
        motor_enabled = 1;
        motor_step_count = 0;
        motor_pulse_state = 0;
        last_motor_pulse = current_time;
        HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    }
    
    // Nếu motor đang hoạt động
    if (motor_enabled)
    {
        
        if (current_time - last_motor_pulse >= (MOTOR_PULSE_PERIOD / 2))
        {
            last_motor_pulse = current_time;
            
            // Đảo trạng thái xung
            if (motor_pulse_state == 0)
            {
                HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
                motor_pulse_state = 1;
                motor_step_count++; 
            }
            else
            {
                HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
                motor_pulse_state = 0;
            }
            
            // Kiểm tra nếu đã đủ số bước
            if (motor_step_count >= MOTOR_STEPS_PER_CYCLE)
            {
                motor_enabled = 0;
                motor_start_time = current_time; // Reset bộ đếm
                motor_step_count = 0;
                motor_pulse_state = 0;
                HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
            }
        }
    }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

lcd1.hi2c = &hi2c1;
  lcd1.address = I2C_SLAVE_ADDR;
  
  // 2. Khởi tạo LCD
  lcd_init(&lcd1);
  lcd_clear(&lcd1);
  lcd_puts(&lcd1, "System Ready!");
  HAL_Delay(1000);
	init_dht11(&dht11_sensor, &htim2, dht11_GPIO_Port, dht11_Pin);
	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Read_DHT11();
		Handle_Motor();
		Control_Outputs();
    Handle_Buttons();    
    Update_LCD_Display(); 
    HAL_Delay(1);       
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HUMI_Pin|LIGHT_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : dht11_Pin */
  GPIO_InitStruct.Pin = dht11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dht11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OK_Pin UP_Pin DOWN_Pin RIGHT_Pin */
  GPIO_InitStruct.Pin = OK_Pin|UP_Pin|DOWN_Pin|RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HUMI_Pin LIGHT_Pin STEP_Pin */
  GPIO_InitStruct.Pin = HUMI_Pin|LIGHT_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Hàm đọc nút bấm có debounce
int is_button_pressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) 
    {
        if (HAL_GetTick() - last_button_time > DEBOUNCE_TIME)
        {
            last_button_time = HAL_GetTick();
            return 1;
        }
    }
    return 0;
}

void Handle_Buttons(void)
{
    // Nút RIGHT (0 -> 1 -> 2 -> 3 -> 0)
    if (is_button_pressed(RIGHT_GPIO_Port, RIGHT_Pin))
    {
        current_screen++;
        if (current_screen > 3) current_screen = 0;
        lcd_clear(&lcd1); 
    }
    
    // Nút UP/DOWN: 
    if (current_screen == 2) 
    {
        if (is_button_pressed(UP_GPIO_Port, UP_Pin)) // UP
        {
            set_temp += 0.5;
            if (set_temp > 50.0) set_temp = 50.0;
        }
        if (is_button_pressed(DOWN_GPIO_Port, DOWN_Pin)) // DOWN 
        {
            set_temp -= 0.5;
            if (set_temp < 0.0) set_temp = 0.0;
        }
    }
    else if (current_screen == 3) 
    {
        if (is_button_pressed(UP_GPIO_Port, UP_Pin)) // UP 
        {
            set_hum += 1;
            if (set_hum > 100.0) set_hum = 100.0;
        }
        if (is_button_pressed(DOWN_GPIO_Port, DOWN_Pin)) // DOWN
        {
            set_hum -= 1;
            if (set_hum < 0.0) set_hum = 0.0;
        }
    }
    
    // Nút OK 
    if (current_screen == 2 || current_screen == 3)
    {
        if (is_button_pressed(OK_GPIO_Port, OK_Pin))
        {
            current_screen = 0;
            lcd_clear(&lcd1); 
        }
    }
}


void Update_LCD_Display(void)
{
   
    switch (current_screen)
    {
        case 0:
            lcd_gotoxy(&lcd1, 0, 0); 
            sprintf(buffer, "Cur: %.1fC/%.1f%%", measured_temp, measured_hum);
            lcd_puts(&lcd1, buffer); 
            
            lcd_gotoxy(&lcd1, 0, 1);
            sprintf(buffer, "Set: %.1fC/%.1f%%", set_temp, set_hum);
            lcd_puts(&lcd1, buffer); 
            break;
            
        case 1:
            lcd_gotoxy(&lcd1, 0, 0);
            lcd_puts(&lcd1, "Date:           ");
            
            lcd_gotoxy(&lcd1, 0, 1);
            lcd_puts(&lcd1, "1/12/2025 Mon   ");
            break;
            
        case 2: 
            lcd_gotoxy(&lcd1, 0, 0);
            lcd_puts(&lcd1, "Set Temperature");
            
            lcd_gotoxy(&lcd1, 0, 1);
            sprintf(buffer, ">> %.1f C <<", set_temp); 
            lcd_puts(&lcd1, buffer);
            break;

        case 3: 
            lcd_gotoxy(&lcd1, 0, 0);
            lcd_puts(&lcd1, "Set Humidity");
            
            lcd_gotoxy(&lcd1, 0, 1);
            sprintf(buffer, ">> %.1f %% <<", set_hum);
            lcd_puts(&lcd1, buffer);
            break;
    }
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
