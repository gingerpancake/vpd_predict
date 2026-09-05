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
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "network.h"
#include <math.h>
#include "ai_platform.h"
#include "network_data.h"
#include "sensor.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

typedef enum {
	APP_OK = 0,
	APP_ERR
}APP_STATUS;

APP_STATUS app_status;
ai_error err;
volatile SYSTEM_STATUS system_status = RAIN_STOP;
AI_INPUT_DATA ai_input_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VPD_IDEAL_MIN     0.8f
#define VPD_IDEAL_MAX     1.2f

#define IN_SENSOR_TIMEOUT_MS	500U
#define EX_SENSOR_TIMEOUT_MS	500U
#define MAX_CYCLE_FAILS        	3U

//x_scaler value
#define X_EX_TEMP			0.02136752f
#define	X_EX_HUMI			0.01136364f
#define	X_IN_TEMP			0.03030303f
#define	X_IN_HUMI			0.01333333f
#define	X_HOUR_SIN			0.5f
#define	X_HOUR_COS			0.5f
#define	X_MONTH_SIN			0.5f
#define	X_MONTH_COS			0.66666667f

//y_scaler value
#define	Y_IN_TEMP			0.03030303f
#define	Y_IN_HUMI			0.01333333f

//x_scaler min value
#define X_EX_TEMP_MIN			0.24786325f
#define	X_EX_HUMI_MIN			-0.125f
#define	X_IN_TEMP_MIN			-0.12121212f
#define	X_IN_HUMI_MIN			-0.33333333f
#define	X_HOUR_SIN_MIN			0.5f
#define	X_HOUR_COS_MIN			0.5f
#define	X_MONTH_SIN_MIN			0.5f
#define	X_MONTH_COS_MIN			0.66666667f

//y_scaler min value
#define	Y_IN_TEMP_MIN			-0.12121212f
#define	Y_IN_HUMI_MIN			-0.33333333f

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

float Temp = 0;
float Humi = 0;

float hour_sin;
float hour_cos;

float month_sin;
float month_cos;

static uint32_t cycle_fail_count = 0U;

static const float inital_input[AI_NETWORK_IN_1_SIZE] = {
    27.0f, 87.0f, 24.0f, 94.0f, 0.0f, 1.0f, -1.0f, -0.0f,
    27.1f, 86.0f, 24.0f, 94.0f, 0.258819045f, 0.965925826f, -1.0f, -0.0f,
    26.7f, 87.0f, 24.0f, 95.0f, 0.5f, 0.866025404f, -1.0f, -0.0f,
    25.9f, 93.0f, 24.0f, 95.0f, 0.707106781f, 0.707106781f, -1.0f, -0.0f,
    26.0f, 90.0f, 24.0f, 96.0f, 0.866025404f, 0.5f, -1.0f, -0.0f,
    25.8f, 90.0f, 23.0f, 96.0f, 0.965925826f, 0.258819045f, -1.0f, -0.0f,
    25.3f, 93.0f, 23.0f, 95.0f, 1.0f, 0.0f, -1.0f, -0.0f,
    25.7f, 92.0f, 26.0f, 88.0f, 0.965925826f, -0.258819045f, -1.0f, -0.0f,

    27.1f, 84.0f, 29.0f, 76.0f, 0.866025404f, -0.5f, -1.0f, -0.0f,
    28.9f, 77.0f, 33.0f, 60.0f, 0.707106781f, -0.707106781f, -1.0f, -0.0f,
    30.8f, 68.0f, 34.0f, 54.0f, 0.5f, -0.866025404f, -1.0f, -0.0f,
    32.2f, 61.0f, 34.0f, 55.0f, 0.258819045f, -0.965925826f, -1.0f, -0.0f,
    33.7f, 56.0f, 35.0f, 50.0f, 0.0f, -1.0f, -1.0f, -0.0f,
    34.1f, 55.0f, 36.0f, 48.0f, -0.258819045f, -0.965925826f, -1.0f, -0.0f,
    34.3f, 58.0f, 36.0f, 49.0f, -0.5f, -0.866025404f, -1.0f, -0.0f,
    34.0f, 58.0f, 36.0f, 50.0f, -0.707106781f, -0.707106781f, -1.0f, -0.0f,

    33.4f, 60.0f, 35.0f, 51.0f, -0.866025404f, -0.5f, -1.0f, -0.0f,
    34.3f, 56.0f, 32.0f, 63.0f, -0.965925826f, -0.258819045f, -1.0f, -0.0f,
    28.5f, 81.0f, 30.0f, 75.0f, -1.0f, -0.0f, -1.0f, -0.0f,
    29.9f, 79.0f, 29.0f, 80.0f, -0.965925826f, 0.258819045f, -1.0f, -0.0f,
    30.4f, 73.0f, 28.0f, 81.0f, -0.866025404f, 0.5f, -1.0f, -0.0f,
    30.6f, 69.0f, 28.0f, 77.0f, -0.707106781f, 0.707106781f, -1.0f, -0.0f,
    30.3f, 68.0f, 27.0f, 79.0f, -0.5f, 0.866025404f, -1.0f, -0.0f,
    29.3f, 71.0f, 26.0f, 84.0f, -0.258819045f, 0.965925826f, -1.0f, -0.0f,

    29.3f, 68.0f, 26.0f, 87.0f, 0.0f, 1.0f, -1.0f, -0.0f,
    28.4f, 72.0f, 25.0f, 87.0f, 0.258819045f, 0.965925826f, -1.0f, -0.0f,
    27.9f, 74.0f, 25.0f, 87.0f, 0.5f, 0.866025404f, -1.0f, -0.0f,
    27.9f, 72.0f, 25.0f, 87.0f, 0.707106781f, 0.707106781f, -1.0f, -0.0f,
    27.7f, 73.0f, 24.0f, 89.0f, 0.866025404f, 0.5f, -1.0f, -0.0f,
    27.6f, 72.0f, 24.0f, 89.0f, 0.965925826f, 0.258819045f, -1.0f, -0.0f,
    26.7f, 76.0f, 24.0f, 89.0f, 1.0f, 0.0f, -1.0f, -0.0f,
    27.3f, 74.0f, 25.0f, 88.0f, 0.965925826f, -0.258819045f, -1.0f, -0.0f,

    27.9f, 72.0f, 27.0f, 83.0f, 0.866025404f, -0.5f, -1.0f, -0.0f,
    29.3f, 71.0f, 29.0f, 72.0f, 0.707106781f, -0.707106781f, -1.0f, -0.0f,
    30.3f, 66.0f, 31.0f, 63.0f, 0.5f, -0.866025404f, -1.0f, -0.0f,
    32.1f, 64.0f, 31.0f, 65.0f, 0.258819045f, -0.965925826f, -1.0f, -0.0f,
    32.0f, 66.0f, 32.0f, 67.0f, 0.0f, -1.0f, -1.0f, -0.0f,
    32.0f, 63.0f, 32.0f, 63.0f, -0.258819045f, -0.965925826f, -1.0f, -0.0f,
    31.3f, 68.0f, 31.0f, 67.0f, -0.5f, -0.866025404f, -1.0f, -0.0f,
    31.1f, 69.0f, 31.0f, 69.0f, -0.707106781f, -0.707106781f, -1.0f, -0.0f,

    31.2f, 69.0f, 31.0f, 68.0f, -0.866025404f, -0.5f, -1.0f, -0.0f,
    31.0f, 70.0f, 30.0f, 74.0f, -0.965925826f, -0.258819045f, -1.0f, -0.0f,
    30.2f, 73.0f, 29.0f, 83.0f, -1.0f, -0.0f, -1.0f, -0.0f,
    29.9f, 75.0f, 28.0f, 86.0f, -0.965925826f, 0.258819045f, -1.0f, -0.0f,
    29.9f, 76.0f, 27.0f, 88.0f, -0.866025404f, 0.5f, -1.0f, -0.0f,
    29.4f, 79.0f, 27.0f, 89.0f, -0.707106781f, 0.707106781f, -1.0f, -0.0f,
    29.3f, 80.0f, 26.0f, 90.0f, -0.5f, 0.866025404f, -1.0f, -0.0f,
    28.9f, 80.0f, 26.0f, 90.0f, -0.258819045f, 0.965925826f, -1.0f, -0.0f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AI_Init(void);
static void AI_Get_InOutputs(void);
static void AI_Run(void);
static void AI_Update_Sequence(AI_INPUT_DATA *new_data);
static void AI_Init_Sequence(void);
float Vpd_Calculator(uint8_t temperature, uint8_t humidity);
VPD_STATUS Get_Vpd_State(float vpd);
static float Y_Inverse_Scale(float scaled_value, float scale, float min);
static void X_Scale(AI_INPUT_DATA *data);
static void RTC_Time_scale(AI_INPUT_DATA *data);
static void Sensor_Data_to_Ai_Data(AI_INPUT_DATA *sensor_data);

/*safe functions */
static APP_STATUS Read_In_Sensor_Safe(void);
static APP_STATUS Read_Ex_Sensor_Safe(void);
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
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  AI_Init();
  AI_Get_InOutputs();
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim15);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  if(rtc_wakeup_event == 0U)
	  {
		  __WFI();
	  }

	  if (system_status == RAIN_DETECTED)
	  {
		  Motor_Rain_Close();
	  }else
	  {
		  /* in_sensor_read begin */
		  for(uint8_t retry = 0U; retry < MAX_CYCLE_FAILS; retry ++)
		  {
			  app_status = Read_In_Sensor_Safe();

			  if(app_status == APP_ERR)
			  {
				  cycle_fail_count ++;
			  }else
			  {
				  cycle_fail_count = 0U;
				  break;
			  }

		  }

		  if(cycle_fail_count == MAX_CYCLE_FAILS)
		  {
			  NVIC_SystemReset();
		  }
		  /* in_sensor_read end */

		  /* ex_sensor_read begin */
		  for(uint8_t retry = 0U; retry < MAX_CYCLE_FAILS; retry ++)
		  {
			  app_status = Read_Ex_Sensor_Safe();

			  if(app_status == APP_ERR)
			  {
				  cycle_fail_count ++;
			  }else
			  {
				  cycle_fail_count = 0U;
				  break;
			  }

		  }

		  if(cycle_fail_count == MAX_CYCLE_FAILS)
		  {
			  NVIC_SystemReset();
		  }
		  /* ex_sensor_read end */

		  /* get_sensor_data from in,ex temperature and humidity, current time begin */
		  Sensor_Data_to_Ai_Data(&ai_input_data);
		  /* get_sensor_data from in,ex temperature and humidity, current time end */

		  /* ai_update_sequence begin*/
		  AI_Update_Sequence(&ai_input_data);
		  /* ai_update_sequence end */
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
float Vpd_Calculator(uint8_t temperature, uint8_t humidity) {

	float svp;
	float vpd;

	/* float  uint8 data -> float */

	temperature = Temp;
	humidity = Humi;

	svp = 0.6108f * expf((17.27f * Temp) / (Temp + 237.3f));
	vpd = svp * (1.0f - Humi / 100.0f);

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

	if(sequence_initialized == 0)
	{
		AI_Init_Sequence();
		return;
	}

	if(new_data == NULL)
	{
	    return;
	}

	X_Scale(new_data);

	for(int i = 0; i < AI_NETWORK_IN_1_HEIGHT - 1; i++)
	{
		sequence[i] = sequence[i + 1];
	}

	sequence[AI_NETWORK_IN_1_HEIGHT - 1] = *new_data;


}

static void AI_Init_Sequence(void)
{
	memcpy(sequence ,inital_input, sizeof(inital_input));

	sequence_initialized = 1;
}

static float Y_Inverse_Scale(float scaled_value, float scale, float min) {

	return (scaled_value - min) / scale;
}

static void X_Scale(AI_INPUT_DATA *data) {
    data->ex_temp = data->ex_temp * X_EX_TEMP + X_EX_TEMP_MIN;

    data->ex_humi = data->ex_humi * X_EX_HUMI + X_EX_HUMI_MIN;

    data->in_temp = data->in_temp * X_IN_TEMP + X_IN_TEMP_MIN;

    data->in_humi = data->in_humi * X_IN_HUMI + X_IN_HUMI_MIN;

    data->hour_sin = data->hour_sin * X_HOUR_SIN + X_HOUR_SIN_MIN;

    data->hour_cos = data->hour_cos * X_HOUR_COS + X_HOUR_COS_MIN;

    data->month_sin = data->month_sin * X_MONTH_SIN + X_MONTH_SIN_MIN;

    data->month_cos = data->month_cos * X_MONTH_COS + X_MONTH_COS_MIN;

}

static void RTC_Time_scale(AI_INPUT_DATA *data) {

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    data->hour_sin =
        sinf(2.0f * M_PI * (float)sTime.Hours / 24.0f);

    data->hour_cos =
        cosf(2.0f * M_PI * (float)sTime.Hours / 24.0f);

    data->month_sin =
        sinf(2.0f * M_PI * (float)sDate.Month / 12.0f);

    data->month_cos =
        cosf(2.0f * M_PI * (float)sDate.Month / 12.0f);

}

/* safe functions begin */
static APP_STATUS Read_In_Sensor_Safe(void) {

	uint32_t current = HAL_GetTick();

	in_sensor_rx_ready = 0U;

	if (In_Sensor_Read() != HAL_OK)
	{
		return APP_ERR;
	}

	while(in_sensor_rx_ready == 0U)
	{
		__WFI();

		if(i2c_error_event || (HAL_GetTick() - current > IN_SENSOR_TIMEOUT_MS))
		{
			 i2c_error_event = 0U;
			 sensor_state = SENSOR_STATE_IDLE;
			 return APP_ERR;
		}
	}
	return APP_OK;
}

static APP_STATUS Read_Ex_Sensor_Safe(void) {
	uint32_t current = HAL_GetTick();

	ex_sensor_rx_ready = 0U;

	if (Ex_Sensor_Read() != HAL_OK)
	{
		return APP_ERR;
	}

	while(ex_sensor_rx_ready == 0U)
	{
		__WFI();

		if(i2c_error_event || (HAL_GetTick() - current > EX_SENSOR_TIMEOUT_MS))
		{
			i2c_error_event = 0U;
			sensor_state = SENSOR_STATE_IDLE;
			return APP_ERR;
		}
	}
	return APP_OK;

}

static void Sensor_Data_to_Ai_Data(AI_INPUT_DATA *sensor_data)
{
	ai_input_data.in_temp = in_temperature;
	ai_input_data.in_humi = in_humidity;
	ai_input_data.ex_temp = ex_temperature;
	ai_input_data.ex_humi = ex_humidity;

	RTC_Time_scale(&ai_input_data);
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
