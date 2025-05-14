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
#include "sr04.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
sr04_t sr04_1;
sr04_t sr04_2;
sr04_t sr04_3;

char txBuf[64];
int active=0;
float dist1av;
float dist2av;
float dist3av;
int count;
float diff_init;
float width;
float hold = 0;
float servo_test = 1300;
float servo_var_1 = 0;
float drivemotor_val = 0;
float drivemotor_test = 1500;
float bufferleft[]={4,4,4};
float bufferright[]={4,4,4};
float bufferfront[] = {4,4,4};
float bufferccr[]={1600,1600,1600};
float dist1=4;
float dist2=4;
float dist3=4;
float diff;
float error;
float angle;
float ccr=1600;
int buffercount;
float speedfront;
float resultantspeed = 1500;
int grabcoin;
int coingrabbed;
int likelihood;
int readycount;
float speed = 1565;
float time_readjust;
int canright;
int canleft;
float distcheck;
int wait;
int terminate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Sensor 1 uses PA12 as trig, PA11 as echo
  sr04_1.trig_port    = GPIOA;
  sr04_1.trig_pin     = GPIO_PIN_12;
  sr04_1.echo_htim    = &htim1;
  sr04_1.echo_channel = TIM_CHANNEL_4;

  // Sensor 2 uses PC9 as trig, PA10 as echo
  sr04_2.trig_port    = GPIOC;
  sr04_2.trig_pin     = GPIO_PIN_9;
  sr04_2.echo_htim    = &htim1;
  sr04_2.echo_channel = TIM_CHANNEL_3;

  // Sensor 3 uses PA9 as trig, PA8 as echo
  sr04_3.trig_port    = GPIOA;
  sr04_3.trig_pin     = GPIO_PIN_9;
  sr04_3.echo_htim    = &htim1;
  sr04_3.echo_channel = TIM_CHANNEL_1;

  sr04_init(&sr04_1);
  sr04_init(&sr04_2);
  sr04_init(&sr04_3);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  __HAL_TIM_ENABLE(&htim2);
  __HAL_TIM_ENABLE(&htim15);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  while (servo_var_1 < 10)
	  {
		  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,servo_test);
		  servo_test = servo_test + 50;
		  HAL_Delay(1000);
		  servo_var_1++;

	  }
	  */


	  while (active==0){
		  //idle
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
		  HAL_Delay(10);
	  }

	  //state 0 - initialisation
	  while (count<25){//take 10 measurements
		  	//Get distances for side sensors
		  	sr04_trigger(&sr04_1);
		  	sr04_trigger(&sr04_3);
		  	HAL_Delay(20);
		  	dist1 = sr04_1.distance / 2000.0f; // if the .distance is in mm
		  	dist3 = sr04_3.distance / 2000.0f;
		  	if (dist1>0.5){
		  		dist1av+=dist1;//accumulate totals
		  	}
		  	else {
		  		dist1av+=1;
		  	}
		  	if (dist3>0.5){
		  		dist3av+=dist3;
		  	}
		  	else {
		  		dist3av+=1;
		  	}
		  	sprintf(txBuf, "S1 distance: %.2f m, S3 distance: %.2f m\r\n", dist1, dist3);
		  	HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
		  	if (count==24){
		  		dist1av/=25;//get averages before leaving loop
		  		dist3av/=25;
		  		diff_init=dist1av-dist3av;
		  		width=(dist1av+dist3av)*0.75; //can play with this value to adjust responsiveness
		  		sprintf(txBuf, "S1 distance average: %.2f m, S3 distance average: %.2f m\r\n", dist1av, dist3av);
		  		HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
		  	}
		  	count++;
	  }

	if (coingrabbed==0){//state 1 - drive forwards
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(speed-5));
	}
	else {//state 4 - drive backwards
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
	}

	//state 2 and state 3 - readjust steering based on sensor measurements
	// Trigger Sensors
	sr04_trigger(&sr04_1);
	sr04_trigger(&sr04_2);
	sr04_trigger(&sr04_3);
	HAL_Delay(10); // Give it time to complete measuring

	//Determine distances
	dist1 = sr04_1.distance / 2000.0f; // if the .distance is in mm
	dist2=0;
 	//dist2 = sr04_2.distance / 2000.0f;
	dist3 = sr04_3.distance / 2000.0f;

	if (dist2<0.01){
		dist2=0.6;
	}

//	//measurement buffer to monitor orientation threshold
	buffercount++;
	if (buffercount==1){
		bufferright[0]=dist3;
		bufferleft[0]=dist1;
		bufferfront[0] = dist2;
		bufferccr[0]=ccr;
	}
	else if (buffercount==2){
		bufferright[1]=dist3;
		bufferleft[1]=dist1;
		bufferfront[1] = dist2;
		bufferccr[1]=ccr;
	}
	else if (buffercount==3){
		bufferright[2]=dist3;
		bufferleft[2]=dist1;
		bufferfront[2] =dist2;
		bufferccr[2]=ccr;
		buffercount=0;
//		if (bufferright[2]>bufferright[1]&&
//			bufferright[1]>bufferright[0]&&
//			bufferccr[0]<1550&&bufferccr[1]<1550&&bufferccr[2]<1550){
//			ccr=1700;
//		}
//		if (bufferleft[2]>bufferleft[1]&&
//			bufferleft[1]>bufferleft[0]&&
//			bufferccr[0]>1670&&bufferccr[1]>1670&&bufferccr[2]>1670){
//			ccr=1540;
//		}

	}
//	if (abs((bufferfront[0] - bufferfront[1])< 0.001)||
//		(abs(bufferfront[2] - bufferfront[1]) < 0.001)||
//		(abs(bufferfront[0]-bufferfront[2]) < 0.001)){
//		bufferfront[1]=5;
//		bufferfront[2]=6;
//		bufferfront[0]=4;
//	}

	//sets steering angle
	readycount++;
	if (readycount==40){
		diff=dist1-dist3;
		error=diff_init-diff;
		if (coingrabbed==0){
			angle=((error-width)/(-2*width))*100-50;//can play with this formula to adjust steering range
		}
		else {
			angle=((error-width)/(-2*width))*80-40;
		}
		if (wait<4){
			wait++;
		}
		else {
			ccr=0.01*angle*.2+4*angle+1600;
		}

		if (ccr>2100)
		{
			ccr=2100;
		}
		if (ccr<1300)
		{
			ccr=1300;
		}
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,ccr);
	}
	else if (readycount==49){
		readycount=0;
	}
	else if (readycount<40){
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
	}


	//state 3 - detection of can
	if (((bufferright[0] < 0.4 && bufferright[1] < 0.4) ||
		(bufferright[0] < 0.4 && bufferright[2] < 0.4) ||
		(bufferright[1] < 0.4 && bufferright[2] < 0.4))&&grabcoin==0&&coingrabbed==0) {
		bufferright[0] = 4;
		bufferright[1] = 4;
		bufferright[2] = 4;
		distcheck=dist3;
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
		//HAL_Delay(1000);
		//check that can is actually there
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		for (int i=0;i<50;i++)
		{
			sr04_trigger(&sr04_3);
			HAL_Delay(20);
			dist3 = sr04_3.distance / 2000.0f;
			if (dist3<0.4){
				likelihood++;
			}
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
		HAL_Delay(500);
		//if 2 measurements suggest can is there then position car to grab coin
		if (likelihood>1){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
			HAL_Delay(1500);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,speed);
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1300);
			time_readjust=700+distcheck*(300/0.4);
			HAL_Delay(time_readjust);//tune dynamically
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(speed-5));
			while (dist2>0.4&&terminate<20){
				terminate++;
				//sr04_trigger(&sr04_2);
				HAL_Delay(50);
				//dist2 = sr04_2.distance / 2000.0f;
				dist2=0.5;
			}
			terminate=0;
			grabcoin=1;
			canright=1;
			likelihood=0;
		}
	}

	if (((bufferleft[0] < 0.4 && bufferleft[1] < 0.4) ||
		(bufferleft[0] < 0.4 && bufferleft[2] < 0.4) ||
		(bufferleft[1] < 0.4 && bufferleft[2] < 0.4))&&grabcoin==0&&coingrabbed==0) {
		bufferleft[0] = 4;
		bufferleft[1] = 4;
		bufferleft[2] = 4;
		distcheck=dist1;
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
//		HAL_Delay(1000);
		//check that can is actually there
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
			for (int i=0;i<50;i++)
			{
				sr04_trigger(&sr04_1);
				HAL_Delay(20);
				dist1 = sr04_1.distance / 2000.0f;
				if (dist1<0.4){
					likelihood++;
				}
			}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
			HAL_Delay(500);
			//if 2 measurements suggest can is there then position car to grab coin
			if (likelihood>1){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
				HAL_Delay(1500);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,speed);
				__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,2100);
				//dist 1 can be in the range 0 to 0.4 after detecting a can. Readjustment time should be approximately in the range 600 to 1000 ms
				time_readjust=700+distcheck*(300/0.4);
				HAL_Delay(time_readjust);//tune dynamically
				__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(speed-5));
				while (dist2>0.3&&terminate<20){
					//sr04_trigger(&sr04_2);
					terminate++;
					HAL_Delay(50);
					//dist2 = sr04_2.distance / 2000.0f;
					dist2=0.5;
				}
				terminate=0;
				grabcoin=1;
				canleft=1;
				likelihood=0;
			}
	}

	if (((bufferfront[0] < 0.4 && bufferfront[0] > 0.1 && bufferfront[1] < 0.4 && bufferfront[1] > 0.1) ||
		(bufferfront[0] < 0.4 && bufferfront[0] > 0.1 && bufferfront[2] < 0.4 && bufferfront[2] > 0.1) ||
		(bufferfront[1] < 0.4 && bufferfront[1] > 0.1 && bufferfront[2] < 0.4 && bufferfront[2] > 0.1))&&grabcoin==0&&coingrabbed==0) {
		bufferfront[0] = 4;
		bufferfront[1] = 4;
		bufferfront[2] = 4;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
		for (int i=0;i<10;i++)
		{
			sr04_trigger(&sr04_2);
			dist2 = sr04_2.distance / 2000.0f;
			HAL_Delay(50);
		}
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		//__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
		//HAL_Delay(500);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1300);
		for (int i=0;i<20;i++)
		{
			sr04_trigger(&sr04_2);
			dist2 = sr04_2.distance / 2000.0f;
			HAL_Delay(50);
		}
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,2100);
		for (int i=0;i<20;i++)
		{
			sr04_trigger(&sr04_2);
			dist2 = sr04_2.distance / 2000.0f;
			HAL_Delay(50);
		}
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
		//HAL_Delay(200); //can add if more backing up is required
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,speed);
		wait=0;
		//check that can is actually there
//		for (int i=0;i<10;i++)
//				{
//					sr04_trigger(&sr04_2);
//					HAL_Delay(50);
//					dist2 = sr04_2.distance / 2000.0f;
//					if (dist2<0.5){
//						likelihood++;
//					}
//				}
//				//if 2 measurements suggest can is there then position car to grab coin
//				if (likelihood>3){
//					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,speed);
//					HAL_Delay(500);
//					grabcoin=1;
//					likelihood=0;
//				}
	}

	if (grabcoin==1){//can has been found and car has been positioned
		//stop and grab coin
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
		HAL_Delay(500);
		for (int i=0;i<39;i++){
			ccr=1300+i*20;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,ccr);
			HAL_Delay(200);
		}
		if (canright==1){
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1300);
		}
		else if (canleft==1){
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,2100);
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
		HAL_Delay(time_readjust);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
		coingrabbed=1;
		grabcoin=0;
		canright=0;
		canleft=0;
	}
	//Shauns speed controller front
	/*
	buffercountfront++;
	if (buffercountfront == 1)
	{
		//This makes [0] index the past value
		bufferfront[0] = dist2;
	}
	else if (buffercountfront == 2)
	{
		//This makes [1] index the current value
		bufferfront[1] = dist2;
		buffercountfront = 0;
		if (bufferfront [0] != bufferfront[1])
		{
			//Jakubs upper range is 1550 (full speed assumed)
			//So we will take 1550 - 1500 = 50. Divide this by max distance (4).
			speedfront = 12.5 * bufferfront[1];
			resultantspeed = speedfront + 1500;
			//Now sets speed of controller to that of calculated spped.
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,resultantspeed);

		}

	}
	*/

	sprintf(txBuf, "distance1: %.2f m, distance2: %.2f m, distance3: %.2f m\r\n,", dist1, dist2, dist3);
	HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
	sprintf(txBuf, "error: %.2f m, angle: %.2f m, ccr: %.2f m\r\n,", error, angle, ccr);
	HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
		//sprintf(txBuf, "Resultant speed: %.2f\r\n,", resultantspeed);
		//HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
}

//	if (dist1 < 1 && active==1)
//	{
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
//	sprintf(txBuf, "distance1: %.2f m, distance3: %.2f m, diff: %.2f m\r\n,", dist1, dist3, diff);
//	HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
//	sprintf(txBuf, "error: %.2f m, angle: %.2f m, ccr: %.2f m\r\n,", error, angle, ccr);
//	HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);

//	}
//	else
//	{
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1600);
//	}


//	if (active==1&&dist3 >= 1.00)
//	{
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1400);
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1650);
//		HAL_Delay(50);
//	} else {
//		//Should stop, assuming dist is less than 1m.
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
//	}
	//HAL_Delay(200);
  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  //Always make sure prescaler is 63, otherwise math does not work.
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 64000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 63;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 3333;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == GPIO_PIN_13) {
HAL_TIM_Base_Start_IT(&htim6);
} else {
__NOP();
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* Prevent unused argument(s) compilation warning */
//UNUSED(htim);

/* NOTE : This function should not be modified, when the callback is needed,
the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
*/
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)&&htim->Instance==TIM6){
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		active^=0x01;

		if (active==0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);

		}else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,speed);
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1600);
			grabcoin=0;
			coingrabbed=0;
			likelihood=0;
			dist1av=0;
			dist2av=0;
			dist3av=0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
			count=0;
		}
		HAL_TIM_Base_Stop_IT(&htim6);
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
