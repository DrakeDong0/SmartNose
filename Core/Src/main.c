/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  * Code written by Drake D. and Ivan K. for ECE198 our Project.
  * November 22, 2023
  *
  * This project records the gas PPM using MQ4, MQ135, and MQ136 sensors and
  * calculates the safety using safety regulations. Statistical principals like
  * measurement error and Z-Score are used to ensure a precise reading.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOA
#define LED_PORT2 GPIOB //6
#define LED_PIN  GPIO_PIN_5
#define LED_PIN2 GPIO_PIN_6
#define LED_PIN3 GPIO_PIN_7
#define LED_PIN4 GPIO_PIN_6
#define SW_PORT GPIOC



#define SW_PIN GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Initialize variables
    const int max_time = 15000;
    int time = 1000;
    bool clock = true;
    int button = 0;
    int mq4size = 0;
    int mq135size = 0;
    int mq136size = 0;
    int mq4data[1000];
    int mq135data[1000];
    int mq136data[1000];
    int mq4mean = 0;
    int mq135mean = 0;
    int mq136mean = 0;
    int mq4dev =  0;
    int mq135dev = 0;
    int mq136dev = 0;
    int mq4median = 0;
    int mq135median = 0;
    int mq136median = 0;
    int mq4moe = 0;
    int mq135moe = 0;
    int mq136moe = 0;
    int mq4sum = 0;
    int mq135sum = 0;
    int mq136sum = 0;

  void ADC_Select_CH0 (void)
  {
  	ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_0;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }

  void ADC_Select_CH1 (void)
  {
  	ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_1;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }
  void ADC_Select_CH4 (void)
  {
  	ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_1;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }

  void insert(int array[],int size){
        double value = array[size-1];
        int k = 0;
        for(k = size-1;(k>0) && (array[k-1] > value); k--){
            array[k] = array[k-1];
        }
        array[k] = value;
    }
    void insert_sort(int array[],int size){
        for(int k = 2; k <= size; k++){
            insert(array, k);
        }
    }
    int calculate_mean(int size, int array[]){
      int mean = 0;
      for(int k = 0; k < size; k++){
          mean += array[k];
      }
      mean = mean/size;
      return mean;
    }
    int calculate_dev(int size, int array[], int mean) {
        int dev = 0.0;

        for (int k = 0; k < size; k++) {
            dev += pow(array[k] - mean, 2);
        }

        dev = sqrt(dev / (size - 1));

        return dev;
    }
    void replace_zscores(int size, int array[], int mean, int dev, int median){
      int zscore = 0;
      for(int k = 0; k<size; k++){
          zscore = (array[k]-mean)/dev;
          if((zscore > 2) || (zscore < -2)){
              array[k] = median;
          }
      }
    }
    int calculate_median(int size, int array[]){
      int median = 0;
      for(int k = 0; k < size; k++){
          if((size%2) == 0){
              median = (array[(size/2)]+array[(size/2)-1])/2;
          }
          else{
              median = array[(size-1)/2];
          }
      }
      return median;
    }
    int sum(int array[], int size){
    	int sum = 0;
    	for(int k = 0; k<size; k++){
    	sum += array[k];
    	}
    }
    int calculate_moe(int sum, int mean, int size, int dev){
    	int tot = (sum-(mean*size))/dev;
    	int moe = tot*(dev/(sqrt(size)));
    	return moe;

    }
    Lcd_PortType ports[] = { GPIOC, GPIOB, GPIOA, GPIOA };
    Lcd_PinType pins[] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_6};
    Lcd_HandleTypeDef lcd;
    lcd = Lcd_create(ports, pins, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_4, LCD_4_BIT_MODE);
      for ( int x = 10; x > 0 ; x-- )
      {
      Lcd_clear(&lcd);
      Lcd_cursor(&lcd, 1,8);
      Lcd_int(&lcd, x);
        Lcd_cursor(&lcd, 0,1);
        Lcd_string(&lcd, "Sensors Warming Up!");
        HAL_Delay (1000);
        Lcd_clear(&lcd);
      }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Lcd_clear(&lcd);
  Lcd_cursor(&lcd, 0,1);
  Lcd_string(&lcd, "Press to Start");
  while(button == 0){
      if (HAL_GPIO_ReadPin(SW_PORT, SW_PIN) != GPIO_PIN_SET)

        {
         if(button == 0){

        	 button = 1;
        }
         else{
           button = 0;
           break;
         }
        }



  }
  while (clock && (button == 1))
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Lcd_clear(&lcd);
	      Lcd_cursor(&lcd, 1,7);
	      Lcd_int(&lcd, (time/1000));
	      Lcd_cursor(&lcd, 0,1);
	      Lcd_string(&lcd, "Reading inputs...");
	      ADC_Select_CH0();//MQ4
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 1000);
	      int x = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);
	      int mq4 = x + 300;
	      mq4data[mq4size] = mq4;
	      mq4size++;

	      ADC_Select_CH1();//MQ136
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 1000);
	      int y = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);
	      int mq136 = y-450;
	      if(mq136<0){
	        mq136 = mq136*(-1);
	      }
	      mq136data[mq136size] = mq136;
	      mq136size++;


	      ADC_Select_CH4();//MQ135
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 1000);
	      int z = HAL_ADC_GetValue(&hadc1);
	      int mq135 = z-250;
	      mq135data[mq135size] = mq135;
	      mq135size++;

	      HAL_ADC_Stop(&hadc1);

	      HAL_Delay (1000);
	      printf("MQ4: %d PPM | MQ136: %d PPM | MQ135: %d PPM \n", mq4, mq136, mq135);
	      if((mq4>1510)||(mq136>40)||(mq135>500)){
	    	  HAL_GPIO_WritePin(LED_PORT, LED_PIN,1);
	    	  }
	     else{
	    	 HAL_GPIO_WritePin(LED_PORT, LED_PIN,0);
	     }

	      time+= 1000;
	      if (time > max_time){
	        clock = false;
	      }


	    }
      	HAL_Delay(1000);
	    Lcd_clear(&lcd);
	    Lcd_cursor(&lcd, 0,1);
	    Lcd_string(&lcd, "Calculating... \n");
	    HAL_Delay(1000);

	  //  //calculate MQ4 Data
	    insert_sort(mq4data, mq4size);
	    mq4mean = calculate_mean(mq4size, mq4data);
	    mq4dev = calculate_dev(mq4size, mq4data, mq4mean);
	    mq4median = calculate_median(mq4size, mq4data);
	    replace_zscores(mq4size, mq4data, mq4mean, mq4dev, mq4median);
	    insert_sort(mq4data, mq4size);
	    //recalculate values
	    mq4mean = calculate_mean(mq4size, mq4data);
	    mq4dev = calculate_dev(mq4size, mq4data, mq4mean);
	    mq4median = calculate_median(mq4size, mq4data);
	    mq4sum = sum(mq4data, mq4size);
	    mq4moe = calculate_moe(mq4sum, mq4mean, mq4size, mq4dev);

	    //Calculate MQ135 Data
	    insert_sort(mq135data, mq135size);
	    mq135mean = calculate_mean(mq135size, mq135data);
	    mq135dev = calculate_dev(mq135size, mq135data, mq135mean);
	    mq135median = calculate_median(mq135size, mq135data);
	    replace_zscores(mq135size, mq135data, mq135mean, mq135dev, mq135median); insert_sort(mq135data, mq135size);
	     //recalculate values
	    mq135mean = calculate_mean(mq135size, mq135data);
	    mq135dev = calculate_dev(mq135size, mq135data, mq135mean);
	    mq135median = calculate_median(mq135size, mq135data);
	    mq135sum = sum(mq135data, mq135size);
	    mq135moe = calculate_moe(mq135sum, mq135mean, mq135size, mq135dev);

	    //Calculate MQ136 data
	    insert_sort(mq136data, mq136size);
	    mq136mean = calculate_mean(mq136size, mq136data);
	    mq136dev = calculate_dev(mq136size, mq136data, mq136mean);
	    mq136median = calculate_median(mq136size, mq136data);
	    replace_zscores(mq136size, mq136data, mq136mean, mq136dev, mq136median); insert_sort(mq136data, mq136size);
	    //recalculate values
	    mq136mean = calculate_mean(mq136size, mq136data);
	    mq136dev = calculate_dev(mq136size, mq136data, mq136mean);
	    mq136median = calculate_median(mq136size, mq136data);
	    mq136sum = sum(mq136data, mq136size);
	    mq136moe = calculate_moe(mq136sum, mq136mean, mq136size, mq136dev);

	    if ((mq136moe >= 10) || (mq135moe >= 10)|| (mq4moe >= 10)){ //check if measurement error is greater than 10%
	    	Lcd_clear(&lcd);
	    	Lcd_cursor(&lcd, 0,0);
	    	Lcd_string(&lcd, "Error too high");
	    	Lcd_cursor(&lcd, 1,0);
	    	Lcd_string(&lcd, "Try Again");
	    	exit(0);
	    }


	    printf("PRINTING DATA \n");//error checking by printing to console
	      for(int k = 0; k < mq136size; k++){
	        printf("MQ4: %d PPM | MQ136: %d PPM | MQ135: %d PPM \n", mq4data[k], mq136data[k], mq135data[k]);
	        printf("\n");
	      }

	    printf("MQ4 ->  Mean: %d, Deviation: %d, Median: %d \n", mq4mean, mq4dev, mq4median);
	    printf("MQ135 -> Mean: %d, Deviation: %d, Median: %d \n", mq135mean, mq135dev, mq135median);
	    printf("MQ136 -> Mean: %d, Deviation: %d, Median: %d \n", mq136mean, mq136dev, mq136median);
	    int gas = 0;
	    while (button == 1){
	    	if (HAL_GPIO_ReadPin(SW_PORT, SW_PIN) != GPIO_PIN_SET)

	    	        {
	    	         if(button == 0){

	    	        	 button = 1;
	    	        }
	    	         else{
	    	           button = 0;
	    	           break;
	    	         }
	    	}
			if (mq4median > 1500){
				gas = 4;
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "High Methane!");
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Mean PPM:");
				Lcd_cursor(&lcd, 1, 9);
				Lcd_int(&lcd, mq4mean);
				HAL_Delay(1500);
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "Deviation:");
				Lcd_cursor(&lcd, 0, 10);
				Lcd_int(&lcd, mq4dev);
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Median PPM:");
				Lcd_cursor(&lcd, 1,11);
				Lcd_int(&lcd, mq4median);
				HAL_Delay(1500);

			}
			if (mq136median > 50){
				gas = 136;
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "High H2S!");
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Mean PPM:");
				Lcd_cursor(&lcd, 1, 9);
				Lcd_int(&lcd, mq135mean);
				HAL_Delay(1500);
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "Deviation:");
				Lcd_cursor(&lcd, 0, 10);
				Lcd_int(&lcd, mq135dev);
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Median PPM:");
				Lcd_cursor(&lcd, 1,11);
				Lcd_int(&lcd, mq135median);
				HAL_Delay(1500);
			}
			if (mq135median > 500){
				gas = 135;
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "Poor Air Quality!");
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Mean PPM:");
				Lcd_cursor(&lcd, 1, 9);
				Lcd_int(&lcd, mq135mean);
				HAL_Delay(1500);
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "Deviation:");
				Lcd_cursor(&lcd, 0, 10);
				Lcd_int(&lcd, mq135dev);
				Lcd_cursor(&lcd, 1,0);
				Lcd_string(&lcd, "Median PPM:");
				Lcd_cursor(&lcd, 1,11);
				Lcd_int(&lcd, mq135median);
				HAL_Delay(1500);

			}
			if (gas == 0){
			 Lcd_clear(&lcd);
			 Lcd_cursor(&lcd, 0,1);
			 Lcd_string(&lcd, "No Spoilage");
			 Lcd_cursor(&lcd, 1,1);
			 Lcd_string(&lcd, "Detected! :)");
			}

	    	HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
	ITM_SendChar(*ptr++);
  }
  return len;
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
