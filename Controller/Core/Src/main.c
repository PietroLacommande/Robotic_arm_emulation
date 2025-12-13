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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    double angle_base;
    double angle_a1;
    double angle_a2;
    uint16_t ccr_base;
    uint16_t ccr_a1;
    uint16_t ccr_a2;
} ArmAnglesCCR_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX_VALUE 4095 //(2^12bits) -1
#define REF_VOLTAGE 3.3f //volts
#define INITIAL_POSITION_SERVO 1500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float adcToVolts(uint32_t rawReading);
ArmAnglesCCR_t ArmAnglesCCR_centered(double x, double y, double z);
float readAdc();
void PwmStart(uint8_t channelNumber1, uint8_t channelNumber2, uint8_t channelNumber3);
void sendCCR(double x, double y, double z);
void readAngles();
void PwmStop(uint8_t channelNumber1, uint8_t channelNumber2, uint8_t channelNumber3);
void angleCorrectionMinus90();
int8_t verifyPosition(int8_t desiredBase, int8_t actualBase);
void initialPosition();
void turnOnLedGreen();
void turnOffLedGreen();
void activateGripper();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ArmAnglesCCR_t ccrAnlge;
int8_t tx_bytes_corrected[3] = {0,0,0}; // Tableau SPI à recevoir
uint8_t tx_bytes[3] = {0,0,0}; // Tableau SPI à recevoir

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  PwmStart(TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
  int8_t y_position= 65;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ccrAnlge = ArmAnglesCCR_centered(30,y_position,10);
	  turnOnLedGreen();
	  sendCCR(ccrAnlge.ccr_base,ccrAnlge.ccr_a1,ccrAnlge.ccr_a2);
	  HAL_Delay(100); //delai imposé pour laisser le temps a l'emulateur de prep les donnes
	  readAngles();
	  angleCorrectionMinus90();
	  int result = verifyPosition(30, tx_bytes_corrected[0]);
	  if(result ==-1){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		  //angle correction code TODO here
	  }

	  else if(result ==1){
		  activateGripper();
		  if(readAdc()>0.1){
			  initialPosition();
			  turnOffLedGreen();
//			  y_position *= (-1);
		  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
void activateGripper(){
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //activer la pince pour saisir objet
}
void turnOffLedGreen(){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //green light instead of red light to indicate motors stopped
}
void turnOnLedGreen(){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //green light instead of red light to indicate motors moving
}
void initialPosition(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, INITIAL_POSITION_SERVO);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, INITIAL_POSITION_SERVO);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, INITIAL_POSITION_SERVO);
}


int8_t verifyPosition(int8_t desiredBase, int8_t actualBase){
	float error = (float)desiredBase - (float)actualBase;
	if(error >=0.0 && error <10.0){
		return 1; //success
	}
	else {
		return -1; //error
	}
}


void angleCorrectionMinus90(){
	for(uint8_t i=0; i<sizeof(tx_bytes); i++){
		tx_bytes_corrected[i]=tx_bytes[i]-90; //ajustement de 90 degs
	}
}

void readAngles(){
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	 HAL_SPI_Receive(&hspi2, tx_bytes, 3, 500); //500ms delay
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}


void sendCCR(double x, double y, double z){
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, x); //PA10 -> PA8
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, y); //PA9 ->
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, z); //PA8
}

void PwmStart(uint8_t channelNumber1, uint8_t channelNumber2, uint8_t channelNumber3){
	  HAL_TIM_PWM_Start(&htim1, channelNumber1);
	  HAL_TIM_PWM_Start(&htim1, channelNumber2);
	  HAL_TIM_PWM_Start(&htim1, channelNumber3);
}

void PwmStop(uint8_t channelNumber1, uint8_t channelNumber2, uint8_t channelNumber3){
	  HAL_TIM_PWM_Stop(&htim1, channelNumber1);
	  HAL_TIM_PWM_Stop(&htim1, channelNumber2);
	  HAL_TIM_PWM_Stop(&htim1, channelNumber3);
}



float adcToVolts(uint32_t rawReading){
	float result = ((float)rawReading/(float)ADC_MAX_VALUE)*REF_VOLTAGE;
	return result;
}

float readAdc(){
	 HAL_ADC_Start(&hadc1); //TODO, pas bon
	 uint32_t adc_value = 0;
	 float adc_value_VOlts = 0;
	 if(HAL_ADC_PollForConversion(&hadc1, 10)== HAL_OK){
		 adc_value = HAL_ADC_GetValue(&hadc1);
		 adc_value_VOlts = adcToVolts(adc_value);
		 /* Restart ADC conversion */
		 HAL_ADC_Start(&hadc1);
	  }
	 return adc_value_VOlts;
}

ArmAnglesCCR_t ArmAnglesCCR_centered(double x, double y, double z)
{
    ArmAnglesCCR_t result;

    // ---- Paramètres physiques ----
    const double L = 75.0;          // longueur des segments (mm)
    const uint16_t CCR_MIN = 1000;  // CCR correspondant à -90°
    const uint16_t CCR_MAX = 2000;  // CCR correspondant à +90°
    const uint16_t CCR_CENTER = (CCR_MIN + CCR_MAX) / 2;
    const double CCR_SCALE = (CCR_MAX - CCR_MIN) / 180.0; // µs par degré

    // ---- Calcul des angles ----
    double b = atan2(y, x);               // angle base (radians)
    double l = sqrt(x*x + y*y);           // distance horizontale
    double h = sqrt(l*l + z*z);           // distance totale
    double phi = atan2(z, l);             // angle vertical de l'axe
    double theta = acos(h / (2.0 * L));   // loi du cos pour bras

    double a1 = phi + theta;              // angle bras 1 (radians)
    double a2 = phi - theta;              // angle bras 2 (radians)

    // conversion en degrés
    result.angle_base = b  * 180.0 / M_PI;
    result.angle_a1   = a1 * 180.0 / M_PI;
    result.angle_a2   = a2 * 180.0 / M_PI;

    // ---- Conversion en CCR centrés ----
    result.ccr_base = CCR_CENTER + (int16_t)(result.angle_base * CCR_SCALE);
    result.ccr_a1   = CCR_CENTER + (int16_t)(result.angle_a1 * CCR_SCALE);
    result.ccr_a2   = CCR_CENTER + (int16_t)(result.angle_a2 * CCR_SCALE);

    return result;
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
