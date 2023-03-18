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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//UART
char txData[13] = "#:25002783:$\n";
uint8_t rxdata[50];
char countarr[5];

int tickspr = 0;

//button vars
extern uint8_t state;
extern uint8_t power;
uint16_t cntA9=0;  //A9 is middle
uint16_t cntB8=0; //left B8
uint16_t cntA6=0;  //A6 is top
uint16_t cntA7=0;   //A7 right
uint16_t cntB9=0;  //B9 is bottom

uint8_t prevA9=1;
uint8_t prevB9=1;
uint8_t prevB8=1;
uint8_t prevA6=1;
uint8_t prevA7=1;



//adc slider
uint32_t val=0; // value of R (digital val 0 to around 4k)
float vin=0;
double vinadj=0;


//dac
double dacout=0;
uint32_t output_voltage = 0;


//mode
uint8_t MODE=0;
uint8_t emmode=0;


//pwm
uint16_t pwmval = 0;


//uart decoding

uint8_t statepara[4];
uint8_t par1[4];
uint8_t par2[4];

uint16_t stateval = 0 ;
uint16_t p1val =0;
uint16_t p2val =0;
uint16_t flagstat=0;
uint8_t uartmode=0;
uint8_t statusarr[19];

//uart
uint8_t rxReqBuf[7];
uint8_t rxSetBuf[19];
uint8_t globalsize;

//uart inerrupt conflict

uint8_t sliderold;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart){
//////
//////
////
////	//HAL_UART_Receive_IT(&huart2, rxdata, sizeof(rxdata));}
////
////
//////}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef*huart){
//
//
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



	void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

		if (huart->Instance == USART2){


			if (Size==7){
				memcpy(rxReqBuf,rxdata,Size);
				uartmode=2;

			} else if (Size==19) {

				memcpy(rxSetBuf,rxdata,Size);
		);

				uartmode=1;
				sliderold = vinadj;
			} else {
				globalsize = Size;
				uint8_t tBuf[21] = "Unrecognized Command\n";
				HAL_UART_Transmit(&huart2, tBuf, 21, 5);
				uartmode=0;
			}


		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxdata, 50);

		}
	}







/////BUTTTONSSSSSS
void buttonpressed(){

              if ((HAL_GetTick() - tickspr) >= 20){




          //button A9 middle
            	  	  if(prevA9==1 || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1 ){
                           if ((prevA9 == 1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0)){

                        	  if (power==1){
                        		  power=0;
                        		  resetall();


                        	  }
                        	  else {



                        		  power =1;
                        	  }

                               tickspr = HAL_GetTick();
                           }

                           if ((prevA9 == 0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1)){
                        	   tickspr = HAL_GetTick();
                           }

              prevA9 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);


            	  	  }

            	  	 //end A9 middle







            	  	//button A6 top
if((prevA6==1 || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 1) && power !=0){
     if ((prevA6 == 1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0)){
      cntA6++;


        if (cntA6==1000){
        cntA6=0;

        				}


        tickspr = HAL_GetTick();
            			}

        if ((prevA6 == 0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 1) ){
        	tickspr = HAL_GetTick();
        																}

prevA6 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);


}
            //end A6 top




	//button B9 bottom
if((prevB9==1 || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 1) && power !=0){
 if ((prevB9 == 1) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)){
  cntB9++;


    if (cntB9==1000){
    cntB9=0;

    				}


    tickspr = HAL_GetTick();
        			}

    if ((prevB9 == 0) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 1)){
    	tickspr = HAL_GetTick();
    																}

prevB9 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);


}
        //end B9 bottom









//button B8 left

   if((prevB8==1 || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1 ) && power !=0){
       if ((prevB8 == 1) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)){
    	   	   	 cntB8++;
    	   	   	 emmode=0;
            	 MODE = cntB8%3;



            	 //some stuff
            	 tickspr = HAL_GetTick();
            	  }

 if ((prevB8 == 0) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)){
            tickspr = HAL_GetTick();
           }

prevB8 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
modeset();

}

//end B8






   //button A7 right
if((prevA7==1 || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1 )&& power !=0){
if ((prevA7 == 1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0)){

    	 cntA7++;
    	 emmode++;
    	 	    		if(emmode==3){
    	 	    			emmode=0;
    	 	    		}
    	 tickspr = HAL_GetTick();
    	 snprintf(countarr, 5, "%03d\n", cntA7);
    	HAL_UART_Transmit(&huart2, countarr, 4, 1000);
    	  }

if ((prevA7 == 0) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1)){
    tickspr = HAL_GetTick();
   }

prevA7 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
modeset();

}

//end A7


}}

//end button code



void resetall(){
//reset vars


	 tickspr = 0;

	 cntA9=0;  //A9 is middle
	 cntB8=0; //left B8
	 cntA6=0;  //A6 is top
	 cntA7=0;   //A7 right
	 cntB9=0;  //B9 is bottom

	 prevA9=1;
	 prevB9=1;
	 prevB8=1;
	 prevA6=1;
	 prevA7=1;


	//adc slider
	 val=0; // value of R (digital val 0 to around 4k)
	 vin=0;
	 vinadj=0;


	//dac
	 dacout=0;
	 output_voltage = 0;


	//mode
	 MODE=0;
	 emmode=0;


	//pwm
	 uint16_t pwmval = 0;



//redo initializations




}






void dacset(){


	//start
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  HAL_ADC_Start(&hadc1);
	//adc

	//HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);   //check if NEEDED
	//adc read
	 val = HAL_ADC_GetValue(&hadc1);
	  vin = val * (3.3) / (pow(2,12)-1);
	  //scale ADC
	 vinadj = vin*(1.18 + -0.0436*vin + 1.72*(pow(vin,2))/1000);
	 //truncate adc
	 if (vinadj>3.28){
		 vinadj=3.30;
	 }


	  //DAC out:

	  //scale dac val
	  dacout = vinadj *( 1.01 + 0.0184*vinadj - 5.15*pow(vinadj,2)/1000);

	  //dac to digital value
	  if (flagstat!=1){
	  output_voltage = (uint32_t)((4095.0/3.30)*dacout);
	  }
	  //truncate digital out
	  if (output_voltage > 4095){
		  output_voltage =4095;
	  }


	  //output dac
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, output_voltage);



}

//end dac





void modeset(){

	if (MODE ==0){
		//PA10 position 1 from left
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	}

	if (MODE ==1){
		//PB4 position 3 from left
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);


	    emergencyset();  //if mode 2nd on else off

		//PB5 position 4 from left


		}


	if (MODE ==2){
		//PB10 position 2 from left
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

		}


}


void emergencyset(){
	if (emmode==2){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}



}

void pwmset(){

	  pwmval=(uint32_t)(((double)output_voltage)*1000/4095);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmval);

}


void uartdecode(){
	//size 14

	//param flags



	if (rxSetBuf[0] == '#' && rxSetBuf[17]=='$' && (rxSetBuf[18]=='\n' || rxSetBuf[18]==0b00001010 ) && rxSetBuf[2]=='M'){
		flagstat=1;
	}
	flagstat=1;



	memcpy(statepara, &rxSetBuf[13], 3);
	statepara[3] = '\0';
	stateval = ((double)atoi(statepara));


		memcpy(par1, &rxSetBuf[5], 3);
		par1[3] = '\0';
		p1val = ((double)atoi(par1));



		memcpy(par2, &rxSetBuf[9], 3);
			par2[3] = '\0';
			p2val = ((double)atoi(par2));







	//MODE 1

	if(rxSetBuf[3] == 'F' ){
	MODE =0;

	output_voltage =(uint32_t) (4095*(((double)atoi(par1))/512.0));

	dacset();
	pwmset();


	}

	//MODE 2
	if(rxSetBuf[3] == 'E' ){
	MODE =1;


		 output_voltage = p1val;
		dacset();
		pwmset();






	}


	//MODE 3
	if(rxSetBuf[3] == 'M' ){
	MODE =2;

	//adjust R channel


	}







}

void uartencode(){

	 // Initialize to all zeros

	uint8_t tBuf[21] = "inarrognized Command\n";
					HAL_UART_Transmit(&huart2, tBuf, 21, 5);
	// Use sprintf() to format the integer as a string and store it in the totransmit array
	sprintf(&statusarr[5], "%03d", p1val);
	sprintf(&statusarr[9], "%03d", p2val);
	sprintf(&statusarr[13], "%03d", stateval);
	statusarr[0] = '#';
	sprintf(&statusarr[1], "%03d", ':');
	statusarr[2] = 'M';
	statusarr[1] = ':';
	statusarr[4] = ':';
	statusarr[8] = ':';
	statusarr[12] = ':';
	statusarr[16] = ':';
	statusarr[17] = '$';
	statusarr[18] = 10;
	//sprintf(&statusarr[9], "%03d", p2val);

	if (MODE==0){

		statusarr[3] = 'F';
	}
	if (MODE==1){

		statusarr[3] = 'E';
	}
	if (MODE==2){

		statusarr[3] = 'M';
	}

	HAL_UART_Transmit(&huart2, statusarr, 19, 500);








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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart2, rxdata, sizeof(rxdata));
// HAL_Delay(250);
//  HAL_UART_Transmit_IT(&huart2, txData, 13);
  modeset();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

 //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxdata, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //continuous adc read outputted through dac - conditioned

	  if (power==1){
	  dacset();
	  //uartencode();

	  pwmset();

		  if (uartmode==1){
			  uartdecode();
			  uartmode=0;
		  }

		  if (uartmode==2){

			  uartencode();
			  uartmode=0;
	  }

		  if (flagstat==1 && (  abs((uint32_t)(1000.0*(sliderold - vinadj)))>15  || state ==1)){

			  flagstat=0;


		  }


	  //receive data




	  //testing if button was turned on





	  }

	  else{

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

	  }

	  if (state==1){
	 		  buttonpressed();
	 		  state=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
