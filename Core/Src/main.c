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
#include <math.h>
#include <stdlib.h>
#include "stdbool.h"
#include "iqs7211a_addresses.h"
#include "IQS7211A_init_AZP1189A3_v0.1.h"
#include "IQS7211A.h"
#include "hellothere.h"



//#include "stm32f4xx_hal_dma.h"
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
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//debug


extern uint8_t pressAndHoldFlag;
extern uint8_t SlideFlag;
extern uint8_t TapFlag;
extern uint32_t distance;
extern int diffX;
volatile int volconflag = 0;

int rchan=0;
int gchan=0;
int bchan=0;
uint8_t prevMODE =0;
uint8_t modechangedflag = 0;
extern double InitialdiffX;
extern double InitialdiffY;
extern double initialstate_intensity;
int rrr;
uint8_t stateok=0;
int max_r;

volatile double state_intensity=0;
double slidetime=0;
double calltime=0;
double sliderold1[6]={0};

//dma
int statett =0;
extern uint16_t prodNum;
uint8_t strobeon=0;
uint8_t ps=0; //power state replaces power
uint8_t dmaflag = 0 ;
int rcnt = 0;
extern uint8_t azoFlag;

uint8_t firston = 0;
int testflag=0;
//morsesend
int timuni = 512;
extern int32_t slideroldtrack ;
int reqwait = 0;
int reqwait2 = 0;
int voltlev =0;
int varno=0;
int len1=0;
int len2=0;
int len3=0;
extern uint8_t testmode;
uint8_t probincr=0;

int cyclesno;
char finale[4]="DEB";
uint32_t pwmvoltlev=0;

int sumdiff=0;
volatile int out_new=0;

int x1=0;
int x2=0;
int x3=0;
int x4=0;
int morsesent=0;
int ii=0;
int jj=0;


uint8_t databuf[5];


//morse
uint8_t flagstatcopy = 0;
int test1=0;
int test2=0;
int* output1 = NULL;
float cantruntime =0;
int* output2= NULL;
int* output3= NULL;
char defmes[4] = "SOS" ;
char message[4] = "SOS" ;
char c;
const char* morse = NULL;
int* output;

char* morseTable[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",
    "-----", ".----", "..---", "...--", "....-", ".....",
    "-....", "--...", "---..", "----."
};


//strobe

uint8_t alternator=0;
int dutycyc = 500;
uint32_t strobetime=0;
uint8_t strobeflag = 0;

int k=0;



//UART
char txData[13] = "#:25002783:$\n";
uint8_t rxdata[50];
char countarr[5];

int tickspr = 0;
int tickspr1 = 0;

//debug
uint8_t testopen;
uint8_t byvar;
uint8_t cntlp=0;
int hardfault1=0;
int stinlast =0;

//button vars
extern uint8_t state;
 uint8_t stateincr=0;
 int oldst= 0 ;

uint16_t cntA9=0;  //A9 is middle
uint16_t cntB8=0; //left B8
uint16_t cntA6=0;  //A6 is top
uint16_t cntA7=0;   //A7 right
uint16_t cntA1=0;  //B9 is bottom

uint8_t prevA9=1;
uint8_t prevA1=1;
uint8_t prevB8=1;
uint8_t prevA6=1;
uint8_t prevA7=1;

 uint8_t letbut=0;
 uint8_t ritbut=0;
 uint8_t botbut=0;
 uint8_t topbut=0;
 uint8_t midbut=0;



//adc slider
 uint32_t val=0; // value of R (digital val 0 to around 4k)


float vin=0;
double vinadj=0;


//dac
double dacout=0;
volatile uint32_t output_voltage = 0;


//mode
uint8_t MODE=0;
uint8_t emmode=0;


//pwm
uint16_t pwmval = 0;
int azoincroo=0;

double vinadjsum[3]= {0};
	  uint8_t io=0;

double vinadjav =0;





//uart decoding
uint8_t butencode=0;
uint8_t statepara[4];
uint8_t par1[4];
uint8_t par2[4];
uint8_t statepara_str[5];

uint16_t stateval = 0 ;
uint16_t p1val =0;
uint16_t p2bchan=0;
uint16_t p1timeval =0;

char p2val[4] = "000";
 uint16_t flagstat=0;
uint8_t uartmode=0;
uint8_t statusarr[19];

//uart
uint8_t rxReqBuf[7];
uint8_t rxSetBuf[19];
uint8_t globalsize;

extern uint16_t xCoord;
extern uint16_t yCoord;

//uart inerrupt conflict

double sliderold=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


void modeset();
void pwmset();
void dacset();
void emergencyset();
void uartencode();
void uartdecode();
void buttonpressed();
void strobeset();
void sendoutmorse();
void MODESWITCH();
void buttonselect();
void uartselect();
void poweroff();
void convertToMorse();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	   __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
	   //HAL_UART_Receive_IT(&huart2, (uint8_t *) rx_data, 1);

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	dmaflag=0;
	//firston=1;
	rcnt=0;
	//calltime = HAL_GetTick();
	 sliderold=vinadj;
	  if (huart->Instance == USART2) {
	        while (dmaflag != 1 && rcnt < Size+2) { // Loop while there are unread bytes in the buffer
	            if (rxdata[rcnt] == '$' && rcnt <= 5) {

	            	//req
	                memcpy(rxReqBuf, rxdata, 7);
	                uartmode = 2;
	                dmaflag = 1;

	            } else if (rxdata[rcnt] == '$' && rcnt> 5) {

	            	//set
	            	if (ps==0){
	            	ps=1;
	            	firston=1;

	            	}


	                memcpy(rxSetBuf, rxdata, 19);
	                uartmode = 1;

	                dmaflag = 1;

	            } else {
	                rcnt++;
	            }
	        }
	        //sliderold = vinadj;
	        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxdata, 50);
	        volconflag =0;
	    }
	}





void buttonpressed()  {

probincr++;
//button A9 middle
	 if (midbut ==0){
        if (ps==1){
        ps=0;
                  }

        else      {
        ps =1;
        butencode=1;
                  }
        stateok=0;

	 	 	 	 	  }



//button A6 top
     if (topbut==0)   {
      butencode=1;
      stateok=1;
            		  }
//end A6 top


//button B9 bottom

     if (botbut == 0) {
	 butencode=1;
	 stateok=1;
        			  }
 //end B9 bottom


//button B8 left
     if (letbut == 0) {
    	   butencode=1;
    	   ps=0;
    	   p1timeval=0;
    	   cntB8++;
    	   emmode=0;

           MODE = cntB8%3;
           ii=0;
           jj=0;
           rchan =128;
           gchan =128;
           bchan =128;
           stateok=1;
            	  	  }
//end B8 left


//button A7 right
     if (ritbut == 0){

		butencode=1;
        emmode++;
        p1timeval=0;

        if(emmode==3) {
          emmode=0;


    	 	    	   }

        stateok=1;}
//end A7 right

 modeset();

//end button code
}








void dacset(){

	strobeon=1;
	//start
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  HAL_ADC_Start_DMA(&hadc1, &val, 1);


	  vin = val * (3.3) / (pow(2,12)-1);

	  //scale ADC
	  //vinadj = vin*(1.18 - 0.0436*vin + 1.72*(pow(vin,2))/1000);
	  vinadj = vin*(1.17 - 0.0456*vin + 1.70*(pow(vin,2))/1000);


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

	  if (flagstat==1){
		  	output_voltage =(uint32_t) (4095*(((double)(stateval))/512.0));
	  }
	  //truncate digital out
	  if (output_voltage > 4095){
		  output_voltage =4095;
	  }
	  if  (alternator ==0 && strobeflag ==1){
	  		 //output_voltage =0;
		  strobeon =0;
	  	  }

	  if (MODE!=2){
	  //output dac

		  if (volconflag ==0 ){
			  initialstate_intensity = output_voltage;
		  }

		out_new = initialstate_intensity + InitialdiffX; // last state plus last difference
		if (out_new<0){
			out_new =0;

		}
		if (out_new>=4095){
					out_new =4095;

				}


	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, out_new*strobeon*ps);


	 // HAL_ADC_Stop(&hadc1)
	  }
	  HAL_ADC_Stop_DMA(&hadc1);
}

//end dac
void pwmset(){
	strobeon=1;
	if  (alternator ==0 && strobeflag ==1){
			  strobeon =0;
		  }

	  pwmval=(uint32_t)(((double)out_new)*1000.0/4095.0);
	  if (emmode !=2 && emmode !=1){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmval*strobeon*ps);
	  }

}




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
	if (emmode==2 || emmode==1){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}



}




void uartdecode(){
	//size 14
	ps=1;
	  probincr++;
	//param flags
	  firston =1;



	flagstat=1;

	flagstatcopy=1;

//set all values
	//mode is not stored - held in rxsetbuf

		memcpy(statepara, &rxSetBuf[5], 3); // state para is first
	statepara[3] = '\0';
	stateval = (uint16_t)(atoi(statepara));



		memcpy(par1, &rxSetBuf[9], 3);   //here is para1
		par1[3] = '\0';
			p1val = ((double)atoi(par1));
			p1timeval = p1val;


			memcpy(par2, &rxSetBuf[13], 3);   //here is para2
			par2[3] = '\0';
			//snprintf((char*)statepara_str, 5, "%s", (char*)statepara);
			strcpy(p2val, par2);
			p2bchan = ((double)atoi(par2));

			//p2val is a char array


			//if in morse:
			// default is sos on mode switch but then to whatever huart sends. 000 no change
			//






//NOTE statepar and par1 are switched when decoding. Con


	//MODE 1

	if(rxSetBuf[3] == 'F' ){
	MODE =0; //flash
	emmode=0;

	output_voltage =(uint32_t) (4095*(((double)(stateval))/512.0));
	out_new =(uint32_t) (4095*(((double)(stateval))/512.0));

	dacset();
	pwmset();


	}

	//MODE 2
	if(rxSetBuf[3] == 'E' ){
		output_voltage =(uint32_t) (4095*(((double)(stateval))/512.0));
		out_new =(uint32_t) (4095*(((double)(stateval))/512.0));

	MODE =1; //strobe

	if (p1val==0){

		//its in morse custom mode
		emmode=2;



		if (p2val[0] == '0' && p2val[1] == '0' && p2val[2] == '0'){

			message[3]= '\0';





			//do nothing idk
		}
		else{


			//declare my message array as para2
			for(int i=0;i<3;i++){
			message[i]= p2val[i];
			message[3]= '\0';

			}
			if ( p2val[0]=='0' && p2val[1]=='0' && p2val[2]=='0'){
				for(int i=0;i<3;i++){
							message[i]= defmes[i];
							message[3]= '\0';

							}

			}

		}


		}


	else if (p1val !=0){
		//probincr++;
					//its in strobe mode
					// output_voltage = stateval;
							dacset();
							pwmset();
							//customized for when mode =1
							emmode=0;

				}






	}


	//MODE 3
	if(rxSetBuf[3] == 'M' ){
	MODE =2;

	rchan =stateval;
	gchan = p1val;
	bchan = p2bchan;



	//adjust R channel


	}




firston=1;
	sliderold = vinadj;
	ps=1;
modeset();
}

void uartencode(){


//if flagstat 0??

		stateval = (uint32_t)((double)(out_new)*512.0/4095.0 + 0.5);

	if (emmode==0 && MODE ==1){ // if switched to manual input then  act accordingly flagstat==0 &&


		strcpy(p2val, "SOS");

	}

	if (MODE==1 && (emmode==1 )  && p2val[0]=='0' && p2val[1]=='0' && p2val[2]=='0'){
		 memcpy(p2val, defmes, 4);
	}

	if (MODE==1 && emmode==2 && p2val[0]=='0' && p2val[1]=='0' && p2val[2]=='0'){
		 memcpy(p2val, message, 4);

		}

	if (MODE ==2 ){
		stateval = rchan;
		p1timeval = gchan;
		sprintf(p2val, "%03d", bchan);



	}
	if (ps ==0 && MODE !=2){
		stateval =0;
	}

	// Use sprintf() to format the integer as a string and store it in the totransmit array

	sprintf((char*)(intptr_t)&statusarr[9], "%03d", p1timeval);
	//sprintf((char*)(intptr_t)&statusarr[13], "%03d", p2val);
	sprintf((char*)(intptr_t)&statusarr[13], "%s", p2val);
	//	sprintf(&statusarr[9], "%03d", p2val);
	sprintf(&statusarr[5], "%03d", stateval);
	statusarr[0] = '#';
	sprintf((char*)(intptr_t)&statusarr[1], "%03d", ':');
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




void strobeset(){

// Start the PWM signal
strobeflag =1;


if (alternator ==0){

dacset();
pwmset();
alternator =1;
}
else{

dacset();
pwmset();
alternator =0;

}


strobetime = HAL_GetTick();

strobeflag=0;



}
//declare

void convertToMorse() {


    for (int i = 0; i < 3; i++) {
        // Convert current character to uppercase
    	if (emmode==2){
         c = (message[i]);
    	}
    	if (emmode==1){
    		c = defmes[i];
    	}

        // Find Morse code for current character
        //const char* morse = NULL;
        if (c >= 'A' && c <= 'Z') {
            morse = morseTable[c - 'A']; //morse is current morse char ie S
        } else if (c >= '0' && c <= '9') {
            morse = morseTable[c - '0' + 26];
        }

        // Convert Morse code to binary and store in appropriate output array

        if (i == 0) {
    		len1 = strlen(morse) ;
        	output = output1 = (int*) malloc(strlen(morse) * sizeof(int)+1);




        } else if (i == 1) {
        	len2 = strlen(morse) ;
            output = output2 = (int*) malloc(strlen(morse) * sizeof(int)+1);
        } else {
        	len3 = strlen(morse) ;
            output = output3 = (int*) malloc(strlen(morse) * sizeof(int)+1);
        }
        //varno++;
         k = 0;
        while (*morse) {
            output[k++] = (*morse++ == '.') ? 0 : 1;
        }
        output[k] = '\0';



    }



}

void sendoutmorse(){


	if ((HAL_GetTick()-morsesent)>=reqwait){

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

		if ((HAL_GetTick()-morsesent)>=(reqwait+timuni + reqwait2)){

			voltlev = out_new;

			//fix here
			pwmvoltlev = ((double)voltlev)*1000/4095;
			reqwait2 =0;


	if (ii ==0){
		//output1[i]!='\0'
		if (output1[jj]==1){


			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);
			 //databuf[i] = '1';
			reqwait = 3*timuni;
			//srtflag =1; //wait extra 512ms at 0V

		}


		else{

			//databuf[i] = '0';

			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);
			reqwait = timuni;
		}
		jj++;
		if (jj==len1){
			reqwait2  = 2*timuni;

			ii++;
			jj=0;
		}

	}

	else if (ii ==1){
		//output1[i]!='\0'
		if (output2[jj]==1 ){


			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);

			reqwait = 3*timuni;

		}


		else{

			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);
			reqwait = timuni;
		}
		test1=jj;
		jj++;
		if (jj==len2){
			test2=jj;
			ii++;
			jj=0;
			reqwait2  = 2*timuni;
		}

	}

	else if (ii ==2){
		//output1[i]!='\0'
		if (output3[jj]==1){
			reqwait = 3*timuni;

			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);

		}


		else{
			//databuf[i] = '0';

			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltlev*ps);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmvoltlev*ps);
			reqwait = timuni;
		}
		jj++;
		if (jj==len3){
			ii=0;
			jj=0;
			reqwait2  = 6*timuni;
		}


			//HAL_UART_Transmit(&huart2, databuf, len1+1, 5);

	}
		morsesent = HAL_GetTick();

}}}

void buttonselect(){

	midbut = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
        		botbut = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        		letbut = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
        		ritbut = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
        		topbut =  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

  if ( letbut ==0 || botbut==0 ||  ritbut==0 || topbut==0 || midbut==0){
	  stateincr++;

  }
  else {
	  stateincr=0;
	  state=0;
  }


  	  if (stateincr>=170){
  		  stinlast= stateincr;
		  buttonpressed();
  	  stateincr=0;
  	state=0;
  	 tickspr = HAL_GetTick();
  	  }

}
//end func

void poweroff(){
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

	  }

//endpower

void MODESWITCH(){

		  if (MODE ==0){
				  dacset();
				  pwmset();  // SET PWM AND DAC BASED ON SLIDER VOLTAGE

					  }
			if (MODE ==1 ){

							if (emmode==0){
								//strobe flagstat==1 ||
								if(p1timeval !=0){
								if (HAL_GetTick()-strobetime>=p1timeval){
									  strobeset();
											  }}
								else if (flagstat==0 || p1timeval==0){
									p1timeval=512;

									if (HAL_GetTick()-strobetime>=512){
									strobeset();
																						  }

								}
							}

							else if (emmode ==1 || emmode ==2){

								//morse

								dacset();
								  pwmset();
						  convertToMorse();
								sendoutmorse();




							free(output1);
							free(output2);
							free(output3);
			}}

			if (MODE==2){
//				if (modechangedflag){
//					rchan =128;
//					gchan =128;
//					bchan =128;
//					modechangedflag=0;
//				}

				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);


			}

}
//end modeswitch

void uartselect(){
probincr += uartmode;
		if (uartmode==1){
			  	  	  //mode 1 is a set command

			  uartdecode();

				//uartencode();


			 uartmode=0;
		  }

		  if (uartmode==2){

			  //mode 2 is a request command


			  uartencode();
			  uartmode=0;  //mode 0 is no decode or encode
	  }

}

void hitthelights(){

	// x 0-  1792  y 0- 768
	int streng = (int)((float)yCoord/740.0*572);
	if (streng >512){
		streng =512;
	}

	if(xCoord <(1783.0/3) && xCoord >0){
		rchan = streng;  // r mapped pc8 3,3
		char* msg = "red channel: ";
		int len = strlen(msg);
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
		char statey[5];
								sprintf(&statey, "%03d\n", rchan);
								//HAL_UART_Transmit(&huart2, (uint8_t*)statey, 3, 100);
		//HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);

	}
	if(xCoord >(1783.0/3) && xCoord <(1783.0*2/3)){
			gchan = streng; // g mapped pa12 4,2
			char* msg = "green channel: ";
			int len = strlen(msg);
			//HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

			char statey[5];
			sprintf(&statey, "%03d\n", gchan);
			//HAL_UART_Transmit(&huart2, (uint8_t*)statey, 3, 100);

			//HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);

		}
	if(xCoord >(1783.0*2/3) && xCoord <(1783.0)){
			bchan =(int) streng;   // blue mapped pc6 tim3 chan 1

			char* msg = "blue channel: ";
			int len = strlen(msg);
			//HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
			//transmit message
			char statey[5];
						sprintf(&statey, "%03d\n", bchan);
						//HAL_UART_Transmit(&huart2, (uint8_t*)statey, 3, 100);
		//trans channel
		//trans newline

			//HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart2, rxdata, sizeof(rxdata));
HAL_Delay(150);
HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxdata, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
HAL_UART_Transmit(&huart2, (uint8_t*)txData, 13, 500);

//HAL_Delay(50);
//
//char buffer[] = "helloworld";
//strcpy(buffer, "helloworld");
//
//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 11, 500);

  modeset();
 // HAL_Delay(10);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // 42 33 31
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, &val, 1);
  //HAL_Delay(10);


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  	HAL_Delay(10);
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  //	HAL_Delay(10);
  	make_reset_certain();
  	set_trackpad_settings();
  //	HAL_Delay(10);
  	TP_ReATI();

  	//HAL_Delay(10);

firston =1;

//while (azoFlag==0){
//
//	continue;
//}
//azoFlag=0;
//acknowledgeReset();
//writeMM();
//TP_ReATI();
//uint16_t prodNum = getProductNum();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (MODE ==2){

			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)(((float)rchan)*ps*1000.0/512.0));
			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (int)(((float)bchan)*ps*1000.0/512.0));
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)(((float)gchan)*ps*1000.0/512.0));
	  }
	  else {
		  rchan=128;
		  bchan=128;
		  gchan=128;
			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)(((float)rchan)*0/512.0));
			 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (int)(((float)gchan)*0/512.0));
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)(((float)bchan)*0/512.0));
	  }
	  vinadjsum[io] = vinadj;
	  vinadjav = 	( vinadjsum[0]+vinadjsum[2]+vinadjsum[1])/(io+1);
	  io++;
	  if (io==3){
		  io=0;
		  vinadjsum[0]=0;
		  vinadjsum[1]=0;
		  vinadjsum[2]=0;
	  }


//	  if (prevMODE !=MODE){
//		  modechangedflag =1;
//	  }


	  //ensures slider old not set to 0 before vinadj initialized


	  //uart protocol set or request function

	  uartselect();

	  //find out which mode and call its operations
		  MODESWITCH();


	if (flagstat==0){
		sliderold=vinadj;
	}
	  if (ps==0){

		 poweroff();

	  }

	  //button press and operations



	// send out button press state
	  if (state==1 && HAL_GetTick()-tickspr>=28){  //button pressed
	buttonselect();


		 	  }
	  if (butencode==1){

		  probincr++;
		  uartencode();
		  butencode=0;
	  }


	  updateTrackpad();

	  if (TapFlag==1){
		  sliderold = vinadj;
		  stateok=0;
		  firston=1;
		 // cantruntime =HAL_GetTick();
		  if (MODE ==2 ){
			  //modes go from 0-2
			  hitthelights();

		  }
		  if (MODE ==1){
			  emmode++;
			        p1timeval=0;

			        if(emmode==3) {
			          emmode=0;

			    	 	    	   }
			       modeset();
		  }
		  flagstatcopy = 1;


		  TapFlag=0;
	  }

		  if (flagstat==1 && firston !=1 && (  abs((int32_t)(1000.0*
				  (sliderold - vinadjav)))>150  || ((stateok ==1) && state==1)))
				  {

			  flagstat=0;  //revert to analog in



		  }
		   rrr = (abs((int32_t)((int32_t)slideroldtrack - (int32_t)output_voltage) ));

		  if (  rrr>300  || flagstatcopy==1){

			  //sumdiff=0;
			 // resinitflag =1;
			  out_new = output_voltage;
			  initialstate_intensity = output_voltage;
			  InitialdiffX = 0;
			  volconflag=0;
			  flagstatcopy=0;
		  }

		  if (pressAndHoldFlag==1){
			  if (ps==1){
				  probincr++;
				  ps=0;

			  }
			  else{ps=1;}
			  pressAndHoldFlag=0;
		 	  }

//		  if (firston==1){
//			  azoFlag =1;
//		  }

		  firston=0;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB14 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
