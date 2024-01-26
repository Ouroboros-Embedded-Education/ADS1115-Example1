/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stddef.h>

#include "driver_ads1115.h"
#include "driver_ads1115_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	int16_t Raw;
	float Value;
	uint8_t Rdy;
	uint16_t SPS;
}Alrt_ADS_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
ads1115_handle_t Ads1115;
uint16_t Alrt_Rdy_Cnt;
Alrt_ADS_t AlrtData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Callbacks
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == ADS1115_ALRT_Pin){
		AlrtData.Rdy = 1;
		Alrt_Rdy_Cnt++;
	}
}

/*
 * Initialize
 */
void _init_ads1115(ads1115_handle_t *Handle, uint8_t isRdy){
	/* Inicializa o Handle do ADS115 e atribui as funcoes */
	DRIVER_ADS1115_LINK_INIT(Handle, ads1115_handle_t);
	// Funcao de inicializacao da I2C
	DRIVER_ADS1115_LINK_IIC_INIT(Handle, ads1115_interface_iic_init);
	// Funcao de deinicializacao do I2C
	DRIVER_ADS1115_LINK_IIC_DEINIT(Handle, ads1115_interface_iic_deinit);
	// Funcao para leitura de registradores do ADS1115
    DRIVER_ADS1115_LINK_IIC_READ(Handle, ads1115_interface_iic_read);
    // Funcao para escrever em registradores do ADS1115
    DRIVER_ADS1115_LINK_IIC_WRITE(Handle, ads1115_interface_iic_write);
    // Funcao de Delay, em millisegundos
    DRIVER_ADS1115_LINK_DELAY_MS(Handle, ads1115_interface_delay_ms);
    // Funcao para geracao de Logs do driver
    DRIVER_ADS1115_LINK_DEBUG_PRINT(Handle, ads1115_interface_debug_print);

    /* Configura o driver do ADS1115 em Modo Alert ou Ready */

    // Alert Mode
    if (isRdy == 0){
    	int16_t HighT, LowT;
        // Atribui o Endereço de acordo com o sinal no terminal de ADDR, neste caso, GND
        ads1115_set_addr_pin(Handle, ADS1115_ADDR_GND);
        // Iniciliza o chip do ADS1115
        ads1115_init(Handle);
        // Define o canal do MUX para AIN0 como V+ e GND como V- (Vin = AIN0-GND)
        ads1115_set_channel(Handle, ADS1115_CHANNEL_AIN0_GND);
        // Define o range do PGA para a tensao de +-4.096V
        ads1115_set_range(Handle, ADS1115_RANGE_4P096V);
        // Define a velocidade de leitura máxima, de 860 Samples per Second
        ads1115_set_rate(Handle, ADS1115_RATE_860SPS);

        ads1115_set_compare_mode(Handle, ADS1115_COMPARE_WINDOW);
        ads1115_set_comparator_queue(Handle, ADS1115_COMPARATOR_QUEUE_4_CONV);

        ads1115_convert_to_register(Handle, 1.5, &LowT);
        ads1115_convert_to_register(Handle, 1.8, &HighT);
        ads1115_set_compare_threshold(Handle, HighT, LowT);
        // Desabilita o comparador
        ads1115_set_compare(Handle, ADS1115_BOOL_TRUE);

        // Inicia a conversao continua do ADS1115
        ads1115_start_continuous_read(Handle);
    }
    // Ready Mode
    else{
        // Atribui o Endereço de acordo com o sinal no terminal de ADDR, neste caso, GND
        ads1115_set_addr_pin(Handle, ADS1115_ADDR_GND);
        // Iniciliza o chip do ADS1115
        ads1115_init(Handle);
        // Define o canal do MUX para AIN0 como V+ e GND como V- (Vin = AIN0-GND)
        ads1115_set_channel(Handle, ADS1115_CHANNEL_AIN0_GND);
        // Define o range do PGA para a tensao de +-4.096V
        ads1115_set_range(Handle, ADS1115_RANGE_4P096V);
        // Define a velocidade de leitura máxima, de 860 Samples per Second
        ads1115_set_rate(Handle, ADS1115_RATE_860SPS);

        ads1115_set_comparator_queue(Handle, ADS1115_COMPARATOR_QUEUE_1_CONV);
        ads1115_set_compare_threshold(Handle, 0x8000, 0x7FFF);
        // Desabilita o comparador
        ads1115_set_compare(Handle, ADS1115_BOOL_TRUE);

        // Inicia a conversao continua do ADS1115
        ads1115_start_continuous_read(Handle);
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
	uint32_t Tick1000;
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
  /* USER CODE BEGIN 2 */
  _init_ads1115(&Ads1115, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Tick1000 = HAL_GetTick();
  AlrtData.Rdy = 0;
  AlrtData.SPS = 0;
  while (1)
  {
	  if (AlrtData.Rdy == 1){
		  AlrtData.Rdy = 0;
		  ads1115_continuous_read(&Ads1115, &AlrtData.Raw, &AlrtData.Value);
	  }
	  if ((HAL_GetTick() - Tick1000) >= 1000){
		  Tick1000 = HAL_GetTick();
		  AlrtData.SPS = Alrt_Rdy_Cnt;
		  Alrt_Rdy_Cnt = 0;
	  }
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
  hi2c1.Init.ClockSpeed = 400000;
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

  /*Configure GPIO pin : ADS1115_ALRT_Pin */
  GPIO_InitStruct.Pin = ADS1115_ALRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADS1115_ALRT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
