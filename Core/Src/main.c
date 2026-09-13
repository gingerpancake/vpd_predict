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
#include "stm32l4xx_it.h"
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
	APP_OK = 0,
	APP_ERR
}APP_STATUS;

typedef enum {
	NORMAL,
	PRE_IDEAL_CUR_MAX_OUT,
	PRE_IDEAL_CUR_MIN_OUT,
	PRE_MAX_OUT_CUR_INC,
	PRE_MAX_OUT_CUR_DEC,
	PRE_MIN_OUT_CUR_INC,
	PRE_MIN_OUT_CUR_DEC,
	PRE_MAX_OUT_CUR_NMV,
	PRE_MIN_OUT_CUR_NMV
}VPD_STATUS;

APP_STATUS app_status;
ai_error err;
volatile SYSTEM_STATUS system_status = RAIN_STOP;
AI_INPUT_DATA ai_input_data;
VPD_STATUS vpd_status;
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

//x_scaler offset value
#define X_EX_TEMP_OFFSET			0.24786325f
#define	X_EX_HUMI_OFFSET			-0.125f
#define	X_IN_TEMP_OFFSET			-0.12121212f
#define	X_IN_HUMI_OFFSET			-0.33333333f
#define	X_HOUR_SIN_OFFSET			0.5f
#define	X_HOUR_COS_OFFSET			0.5f
#define	X_MONTH_SIN_OFFSET			0.5f
#define	X_MONTH_COS_OFFSET			0.66666667f

//y_scaler offset value
#define	Y_IN_TEMP_OFFSET			-0.12121212f
#define	Y_IN_HUMI_OFFSET			-0.33333333f

//limit temperature value
#define MAX_LIMIT_TEMP				35
#define MIN_LIMIT_TEMP				5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float ai_output_data[AI_NETWORK_OUT_1_SIZE];

ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

ai_buffer *ai_input;
ai_buffer *ai_output;

static uint8_t sequence_initialized = 0;

ai_handle network;

static AI_INPUT_DATA sequence[AI_NETWORK_IN_1_HEIGHT];

ai_u32	ai_err_code = 0x00;
ai_u32	ai_err_type = 0x00;

float fvpd = 0U;
float cvpd = 0U;
float pvpd = 0U;

float hour_sin;
float hour_cos;

float month_sin;
float month_cos;

static uint32_t cycle_fail_count = 0U;

static const float inital_input[AI_NETWORK_IN_1_SIZE] = {
    0.82478629f, 0.86363668f, 0.60606060f, 0.91999969f, 0.50000000f, 1.00000000f, 0.00000000f, 0.33333333f,
    0.82692304f, 0.85227304f, 0.60606060f, 0.91999969f, 0.62940952f, 0.98296291f, 0.00000000f, 0.33333333f,
    0.81837603f, 0.86363668f, 0.60606060f, 0.93333302f, 0.75000000f, 0.93301270f, 0.00000000f, 0.33333333f,
    0.80128202f, 0.93181852f, 0.60606060f, 0.93333302f, 0.85355339f, 0.85355339f, 0.00000000f, 0.33333333f,
    0.80341877f, 0.89772760f, 0.60606060f, 0.94666635f, 0.93301270f, 0.75000000f, 0.00000000f, 0.33333333f,
    0.79914527f, 0.89772760f, 0.57575757f, 0.94666635f, 0.98296291f, 0.62940952f, 0.00000000f, 0.33333333f,
    0.78846151f, 0.93181852f, 0.57575757f, 0.93333302f, 1.00000000f, 0.50000000f, 0.00000000f, 0.33333333f,
    0.79700851f, 0.92045488f, 0.66666666f, 0.83999971f, 0.98296291f, 0.37059048f, 0.00000000f, 0.33333333f,

    0.82692304f, 0.82954576f, 0.75757575f, 0.67999975f, 0.93301270f, 0.25000000f, 0.00000000f, 0.33333333f,
    0.86538458f, 0.75000028f, 0.87878787f, 0.46666647f, 0.85355339f, 0.14644661f, 0.00000000f, 0.33333333f,
    0.90598287f, 0.64772752f, 0.90909090f, 0.38666649f, 0.75000000f, 0.06698730f, 0.00000000f, 0.33333333f,
    0.93589739f, 0.56818204f, 0.90909090f, 0.39999982f, 0.62940952f, 0.01703709f, 0.00000000f, 0.33333333f,
    0.96794867f, 0.51136384f, 0.93939393f, 0.33333317f, 0.50000000f, 0.00000000f, 0.00000000f, 0.33333333f,
    0.97649568f, 0.50000020f, 0.96969696f, 0.30666651f, 0.37059048f, 0.01703709f, 0.00000000f, 0.33333333f,
    0.98076919f, 0.53409112f, 0.96969696f, 0.31999984f, 0.25000000f, 0.06698730f, 0.00000000f, 0.33333333f,
    0.97435893f, 0.53409112f, 0.96969696f, 0.33333317f, 0.14644661f, 0.14644661f, 0.00000000f, 0.33333333f,

    0.96153842f, 0.55681840f, 0.93939393f, 0.34666650f, 0.06698730f, 0.25000000f, 0.00000000f, 0.33333333f,
    0.98076919f, 0.51136384f, 0.84848484f, 0.50666646f, 0.01703709f, 0.37059048f, 0.00000000f, 0.33333333f,
    0.85683757f, 0.79545484f, 0.78787878f, 0.66666642f, 0.00000000f, 0.50000000f, 0.00000000f, 0.33333333f,
    0.88675210f, 0.77272756f, 0.75757575f, 0.73333307f, 0.01703709f, 0.62940952f, 0.00000000f, 0.33333333f,
    0.89743586f, 0.70454572f, 0.72727272f, 0.74666640f, 0.06698730f, 0.75000000f, 0.00000000f, 0.33333333f,
    0.90170936f, 0.65909116f, 0.72727272f, 0.69333308f, 0.14644661f, 0.85355339f, 0.00000000f, 0.33333333f,
    0.89529911f, 0.64772752f, 0.69696969f, 0.71999974f, 0.25000000f, 0.93301270f, 0.00000000f, 0.33333333f,
    0.87393159f, 0.68181844f, 0.66666666f, 0.78666639f, 0.37059048f, 0.98296291f, 0.00000000f, 0.33333333f,

    0.87393159f, 0.64772752f, 0.66666666f, 0.82666638f, 0.50000000f, 1.00000000f, 0.00000000f, 0.33333333f,
    0.85470082f, 0.69318208f, 0.63636363f, 0.82666638f, 0.62940952f, 0.98296291f, 0.00000000f, 0.33333333f,
    0.84401706f, 0.71590936f, 0.63636363f, 0.82666638f, 0.75000000f, 0.93301270f, 0.00000000f, 0.33333333f,
    0.84401706f, 0.69318208f, 0.63636363f, 0.82666638f, 0.85355339f, 0.85355339f, 0.00000000f, 0.33333333f,
    0.83974355f, 0.70454572f, 0.60606060f, 0.85333304f, 0.93301270f, 0.75000000f, 0.00000000f, 0.33333333f,
    0.83760680f, 0.69318208f, 0.60606060f, 0.85333304f, 0.98296291f, 0.62940952f, 0.00000000f, 0.33333333f,
    0.81837603f, 0.73863664f, 0.60606060f, 0.85333304f, 1.00000000f, 0.50000000f, 0.00000000f, 0.33333333f,
    0.83119655f, 0.71590936f, 0.63636363f, 0.83999971f, 0.98296291f, 0.37059048f, 0.00000000f, 0.33333333f,

    0.84401706f, 0.69318208f, 0.69696969f, 0.77333306f, 0.93301270f, 0.25000000f, 0.00000000f, 0.33333333f,
    0.87393159f, 0.68181844f, 0.75757575f, 0.62666643f, 0.85355339f, 0.14644661f, 0.00000000f, 0.33333333f,
    0.89529911f, 0.62500024f, 0.81818181f, 0.50666646f, 0.75000000f, 0.06698730f, 0.00000000f, 0.33333333f,
    0.93376064f, 0.60227296f, 0.81818181f, 0.53333312f, 0.62940952f, 0.01703709f, 0.00000000f, 0.33333333f,
    0.93162389f, 0.62500024f, 0.84848484f, 0.55999978f, 0.50000000f, 0.00000000f, 0.00000000f, 0.33333333f,
    0.93162389f, 0.59090932f, 0.84848484f, 0.50666646f, 0.37059048f, 0.01703709f, 0.00000000f, 0.33333333f,
    0.91666663f, 0.64772752f, 0.81818181f, 0.55999978f, 0.25000000f, 0.06698730f, 0.00000000f, 0.33333333f,
    0.91239312f, 0.65909116f, 0.81818181f, 0.58666644f, 0.14644661f, 0.14644661f, 0.00000000f, 0.33333333f,

    0.91452987f, 0.65909116f, 0.81818181f, 0.57333311f, 0.06698730f, 0.25000000f, 0.00000000f, 0.33333333f,
    0.91025637f, 0.67045480f, 0.78787878f, 0.65333309f, 0.01703709f, 0.37059048f, 0.00000000f, 0.33333333f,
    0.89316235f, 0.70454572f, 0.75757575f, 0.77333306f, 0.00000000f, 0.50000000f, 0.00000000f, 0.33333333f,
    0.88675210f, 0.72727300f, 0.72727272f, 0.81333305f, 0.01703709f, 0.62940952f, 0.00000000f, 0.33333333f,
    0.88675210f, 0.73863664f, 0.69696969f, 0.83999971f, 0.06698730f, 0.75000000f, 0.00000000f, 0.33333333f,
    0.87606834f, 0.77272756f, 0.69696969f, 0.85333304f, 0.14644661f, 0.85355339f, 0.00000000f, 0.33333333f,
    0.87393159f, 0.78409120f, 0.66666666f, 0.86666637f, 0.25000000f, 0.93301270f, 0.00000000f, 0.33333333f,
    0.86538458f, 0.78409120f, 0.66666666f, 0.86666637f, 0.37059048f, 0.98296291f, 0.00000000f, 0.33333333f,
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
float Vpd_Calculator(float temperature, float humidity);
VPD_STATUS Get_Vpd_State(float vpd);
static void Y_Inverse_Scale(float *scaled_value);
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
  MX_TIM2_Init();
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
	  while(rtc_wakeup_event == 0)
	  {
		  __WFI();
	  }

	  if (system_status == RAIN_DETECTED)
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

		  /* measuring currnet vpd */
		  if(pvpd == 0)
		  {
			  cvpd = Vpd_Calculator(in_temperature, in_humidity);
			  pvpd = cvpd;
		  }else
		  {
			  pvpd = cvpd;
			  cvpd = Vpd_Calculator(in_temperature, in_humidity);
		  }
		  /* measuring currnet vpd */

		  if(wakeup_num == 60)
		  {
		  /* get_sensor_data from in,ex temperature and humidity, current time begin */
		  Sensor_Data_to_Ai_Data(&ai_input_data);
		  /* get_sensor_data from in,ex temperature and humidity, current time end */

		  /* ai_update_sequence begin*/
		  AI_Update_Sequence(&ai_input_data);
		  /* ai_update_sequence end */

		  /* ai_run begin */
		  AI_Run();

		  Y_Inverse_Scale(ai_output_data);

		  fvpd = Vpd_Calculator(ai_output_data[0], ai_output_data[1]);
		  /* ai_run end */
		  wakeup_num = 0;
		  }

		  /* vpd status update */


		  if(fvpd > VPD_IDEAL_MAX)
		  {
			  if(cvpd > pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_INC;
			  }else if(cvpd < pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_DEC;
			  }else if (cvpd == pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_NMV;
			  }
		  }else if(fvpd < VPD_IDEAL_MIN)
		  {
			if(cvpd > pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_INC;
			}else if(cvpd < pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_DEC;
			}else if(cvpd == pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_NMV;
			}
		  }else if(VPD_IDEAL_MIN <= fvpd && fvpd <= VPD_IDEAL_MAX)
		  {
			  if(cvpd > VPD_IDEAL_MAX)
			  {
				  vpd_status = PRE_IDEAL_CUR_MAX_OUT;
			  }else if(cvpd < VPD_IDEAL_MIN)
			  {
				  vpd_status = PRE_IDEAL_CUR_MIN_OUT;
			  }else
			  {
				  vpd_status = NORMAL;
			  }
		  }
		  /* vpd status update */
		  Motor_Emergency_Close();
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

		  /* measuring currnet vpd */
		  if(pvpd == 0)
		  {
			  cvpd = Vpd_Calculator(in_temperature, in_humidity);
			  pvpd = cvpd;
		  }else
		  {
			  pvpd = cvpd;
			  cvpd = Vpd_Calculator(in_temperature, in_humidity);
		  }
		  /* measuring currnet vpd */

		  /* emergency(over limit temperature) motor control begin */
		  if(in_temperature > MAX_LIMIT_TEMP)
		  {
			  Motor_Emergency_Open();
		  }else if(in_temperature < MIN_LIMIT_TEMP)
		  {
			  Motor_Emergency_Close();
		  }
		  /* emergency(over limit temperature) motor control end */

		  if(wakeup_num == 60)
		  {
		  /* get_sensor_data from in,ex temperature and humidity, current time begin */
		  Sensor_Data_to_Ai_Data(&ai_input_data);
		  /* get_sensor_data from in,ex temperature and humidity, current time end */

		  /* ai_update_sequence begin*/
		  AI_Update_Sequence(&ai_input_data);
		  /* ai_update_sequence end */

		  /* ai_run begin */
		  AI_Run();

		  Y_Inverse_Scale(ai_output_data);

		  fvpd = Vpd_Calculator(ai_output_data[0], ai_output_data[1]);
		  /* ai_run end */
		  wakeup_num = 0;
		  }

		  /* vpd status update */


		  if(fvpd > VPD_IDEAL_MAX)
		  {
			  if(cvpd > pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_INC;
			  }else if(cvpd < pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_DEC;
			  }else if (cvpd == pvpd)
			  {
				  vpd_status = PRE_MAX_OUT_CUR_NMV;
			  }
		  }else if(fvpd < VPD_IDEAL_MIN)
		  {
			if(cvpd > pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_INC;
			}else if(cvpd < pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_DEC;
			}else if(cvpd == pvpd)
			{
				  vpd_status = PRE_MIN_OUT_CUR_NMV;
			}
		  }else if(VPD_IDEAL_MIN <= fvpd && fvpd <= VPD_IDEAL_MAX)
		  {
			  if(cvpd > VPD_IDEAL_MAX)
			  {
				  vpd_status = PRE_IDEAL_CUR_MAX_OUT;
			  }else if(cvpd < VPD_IDEAL_MIN)
			  {
				  vpd_status = PRE_IDEAL_CUR_MIN_OUT;
			  }else
			  {
				  vpd_status = NORMAL;
			  }
		  }
		  /* vpd status update */

		  /* motor logic begin */
		  switch(vpd_status) {
		  	  case NORMAL:
		  		  break;

		  	  case PRE_IDEAL_CUR_MAX_OUT:
		  		  Motor_Backward_Rotation();
		  		  break;

		  	  case PRE_IDEAL_CUR_MIN_OUT:
		  		  Motor_Forward_Rotation();
		  		  break;

		  	  case PRE_MAX_OUT_CUR_INC:
		  		  Motor_Backward_Rotation();
		  		  Motor_Backward_Rotation();
		  		  break;

		  	  case PRE_MAX_OUT_CUR_DEC:
		  		  Motor_Backward_Rotation();
		  		  break;

		  	  case PRE_MAX_OUT_CUR_NMV:
		  		  Motor_Backward_Rotation();
		  		  break;

		  	  case PRE_MIN_OUT_CUR_INC:
		  		  Motor_Forward_Rotation();
		  		  break;

		  	  case PRE_MIN_OUT_CUR_DEC:
		  		  Motor_Forward_Rotation();
		  		  Motor_Forward_Rotation();
		  		  break;

		  	  case PRE_MIN_OUT_CUR_NMV:
		  		  Motor_Forward_Rotation();

		  	  default:
		  		  break;
		  }
		  /* motor logic end */

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  rtc_wakeup_event = 0U;
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
float Vpd_Calculator(float temperature, float humidity) {
    float svp = 0.6108f * expf((17.27f * temperature) / (temperature + 237.3f));
    float vpd = svp * (1.0f - humidity / 100.0f);
    return vpd;
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
	ai_output[0].data = AI_HANDLE_PTR(ai_output_data);
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

static void Y_Inverse_Scale(float *scaled_value) {
	scaled_value[0] = (scaled_value[0] - Y_IN_TEMP_OFFSET) / Y_IN_TEMP;
	scaled_value[1] = (scaled_value[1] - Y_IN_HUMI_OFFSET) / Y_IN_HUMI;
}

static void X_Scale(AI_INPUT_DATA *data) {
    data->ex_temp = data->ex_temp * X_EX_TEMP + X_EX_TEMP_OFFSET;

    data->ex_humi = data->ex_humi * X_EX_HUMI + X_EX_HUMI_OFFSET;

    data->in_temp = data->in_temp * X_IN_TEMP + X_IN_TEMP_OFFSET;

    data->in_humi = data->in_humi * X_IN_HUMI + X_IN_HUMI_OFFSET;

    data->hour_sin = data->hour_sin * X_HOUR_SIN + X_HOUR_SIN_OFFSET;

    data->hour_cos = data->hour_cos * X_HOUR_COS + X_HOUR_COS_OFFSET;

    data->month_sin = data->month_sin * X_MONTH_SIN + X_MONTH_SIN_OFFSET;

    data->month_cos = data->month_cos * X_MONTH_COS + X_MONTH_COS_OFFSET;

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
