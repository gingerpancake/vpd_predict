/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "network.h"
#include <math.h>
#include "ai_platform.h"
#include "network_data.h"
#include "sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	LL_OK,
	LL_BUSY,
	LL_ERROR,
	LL_TIMEOUT
}LL_ERROR_HANDLER;

typedef enum {
	AI_OUT_TEMP,
	AI_OUT_HUMI,
}AI_PREDICT_OUTPUT_DATA;

typedef union {
	struct {
		float ex_temp;
		float ex_humi;
		float in_temp;
		float in_humi;
		float hour_sin;
		float hour_cos;
		float month_sin;
		float month_cos;
	};

	float feature[AI_NETWORK_IN_1_CHANNEL];
}AI_INPUT_DATA;

typedef enum {
	VPD_TOO_LOW,
	VPD_LOW,
	VPD_IDEAL,
	VPD_HIGH,
	VPD_TOO_HIGH
}VPD_STATUS;

ai_error err;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VPD_IDEAL_MIN     0.8f
#define VPD_IDEAL_MAX     1.2f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float ai_output_size[AI_NETWORK_OUT_1_SIZE];

ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

ai_buffer *ai_input;
ai_buffer *ai_output;

static uint8_t sequence_initialized = 0;

ai_handle network;

static AI_INPUT_DATA sequence[AI_NETWORK_IN_1_HEIGHT];

ai_u32	ai_err_code = 0x00;
ai_u32	ai_err_type = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AI_Init(void);
static void AI_Get_InOutputs(void);
static void AI_Run(void);
static void AI_Update_Sequence(AI_INPUT_DATA *new_data);
static void AI_Init_Sequence(AI_INPUT_DATA *first_data);
float Vpd_Calculator(float temperature, float humidity);
VPD_STATUS Get_Vpd_State(float vpd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  AI_Init();
  AI_Get_InOutputs();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
    {
  /* USER CODE BEGIN WHILE */
	  //begin main
  /* USER CODE END WHILE */
    }
}
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float Vpd_Calculator(float temperature, float humidity) {

	float svp;
	float vpd;

	svp = 0.6108f * expf((17.27f * temperature) / (temperature + 237.3f));
	vpd = svp * (1.0f - humidity / 100.0f);

	return vpd;
}

VPD_STATUS Get_Vpd_State(float vpd) {

	if (vpd < 0.6f) return VPD_TOO_LOW;
	if (vpd < 0.8f) return VPD_LOW;
	if (vpd <= 1.2f) return VPD_IDEAL;
	if (vpd <= 1.4f) return VPD_HIGH;

	return VPD_TOO_HIGH;
}

static void AI_Init(void) {
	ai_handle act_addr[] = { AI_HANDLE_PTR(activations) };

	err = ai_network_create_and_init(&network, act_addr, NULL);

	/* error debuging */
	if(err.type != AI_ERROR_NONE)
	{
		/* add error management code begin */

		/* add error management code finish*/
		Error_Handler();
	}
}

static void AI_Get_InOutputs(void) {
	ai_input = ai_network_inputs_get(network, NULL);
	ai_output = ai_network_outputs_get(network, NULL);

	ai_input[0].data = AI_HANDLE_PTR(sequence);
	ai_output[0].data = AI_HANDLE_PTR(ai_output_size);
}

static void AI_Run(void) {
	ai_i32 batch;

	batch = ai_network_run(network, ai_input, ai_output);

	if(batch != 1) {
		err = ai_network_get_error(network);
		/* error management code begin */
		ai_err_code = err.code;
		ai_err_type = err.type;
		/* error management code finish */
		Error_Handler();
	}
}

static void AI_Update_Sequence(AI_INPUT_DATA *new_data)
{
	if(new_data == NULL)
	{
	    return;
	}

	if(sequence_initialized == 0)
		{
			AI_Init_Sequence(new_data);
			return;
		}

	for(int i = 0; i < AI_NETWORK_IN_1_HEIGHT - 1; i++)
	{
		sequence[i] = sequence[i + 1];
	}

	sequence[AI_NETWORK_IN_1_HEIGHT - 1] = *new_data;


}

static void AI_Init_Sequence(AI_INPUT_DATA *first_data)
{
	for(int i = 0; i < AI_NETWORK_IN_1_HEIGHT; i++)
	{
		sequence[i] = *first_data;
	}

	sequence_initialized = 1;
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
#ifdef USE_FULL_ASSERT
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
