

#include "main.h"
#include "iqs7211a_addresses.h"
#include "IQS7211A_init_AZP1189A3_v0.1.h"
#include "IQS7211A.h"
#include <stm32f3xx_hal.h>
#include "stdbool.h"
#include "hellothere.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
volatile uint8_t testmode=0;
extern int volconflag;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern uint8_t azoFlag;
extern int out_new;
IQS7211A_MEMORY_MAP IQSMemoryMap;
uint8_t gestures[2];
uint8_t Xco[2];
uint16_t Xcor = 0;
double startTime=0;
double duration=0;
volatile uint16_t xCoord = 0;
volatile uint16_t yCoord = 0;
volatile int32_t slideroldtrack = 0;

uint8_t transferBytes[30]; // Temporary array which holds the bytes to be transferred.

volatile uint8_t TapFlag = 0;
volatile uint8_t pressAndHoldFlag = 0;
volatile uint8_t PrevpressAndHoldFlag = 0;
volatile uint8_t DownpressFlag = 0;
volatile uint8_t PrevDownpressFlag = 0;
volatile uint8_t SlideFlag = 0;
volatile uint32_t distance;

volatile uint16_t InitialxCoord;
volatile uint16_t InitialyCoord;

extern uint32_t output_voltage;
volatile double initialstate_intensity;

volatile double InitialdiffX;
volatile double InitialdiffY;


void set_trackpad_settings() {

	uint8_t transferBytes[30];// Temporary array which holds the bytes to be transferred.

	/* Change the ATI Settings */
	/* Memory Map Position 0x30 - 0x3D */
	transferBytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
	transferBytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
	transferBytes[2] = TP_COMPENSATION_DIV_0;
	transferBytes[3] = TP_COMPENSATION_DIV_1;
	transferBytes[4] = TP_ATI_TARGET_0;
	transferBytes[5] = TP_ATI_TARGET_1;
	transferBytes[6] = TP_REF_DRIFT_LIMIT_0;
	transferBytes[7] = TP_REF_DRIFT_LIMIT_1;
	transferBytes[8] = TP_MIN_COUNT_REATI_0;
	transferBytes[9] = TP_MIN_COUNT_REATI_1;
	transferBytes[10] = REATI_RETRY_TIME_0;
	transferBytes[11] = REATI_RETRY_TIME_1;
	transferBytes[12] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
	transferBytes[13] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
	transferBytes[14] = ALP_COMPENSATION_DIV_0;
	transferBytes[15] = ALP_COMPENSATION_DIV_1;
	transferBytes[16] = ALP_ATI_TARGET_0;
	transferBytes[17] = ALP_ATI_TARGET_1;
	transferBytes[18] = ALP_LTA_DRIFT_LIMIT_0;
	transferBytes[19] = ALP_LTA_DRIFT_LIMIT_1;

	/* Change the ALP ATI Compensation */
	/* Memory Map Position 0x3A - 0x3D */
	transferBytes[20] = ALP_COMPENSATION_A_0;
	transferBytes[21] = ALP_COMPENSATION_A_1;
	transferBytes[22] = ALP_COMPENSATION_B_0;
	transferBytes[23] = ALP_COMPENSATION_B_1;
//	writeRandomBytes(IQS7211A_MM_TP_ATI_MIR, 24, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_TP_ATI_MIR,transferBytes, 24, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_TP_ATI_MIR, 1,
			transferBytes, 24, 5);

	/* Change the Report Rates and Timing */
	/* Memory Map Position 0x40 - 0x4A */
	transferBytes[0] = ACTIVE_MODE_REPORT_RATE_0;
	transferBytes[1] = ACTIVE_MODE_REPORT_RATE_1;
	transferBytes[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
	transferBytes[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
	transferBytes[4] = IDLE_MODE_REPORT_RATE_0;
	transferBytes[5] = IDLE_MODE_REPORT_RATE_1;
	transferBytes[6] = LP1_MODE_REPORT_RATE_0;
	transferBytes[7] = LP1_MODE_REPORT_RATE_1;
	transferBytes[8] = LP2_MODE_REPORT_RATE_0;
	transferBytes[9] = LP2_MODE_REPORT_RATE_1;
	transferBytes[10] = ACTIVE_MODE_TIMEOUT_0;
	transferBytes[11] = ACTIVE_MODE_TIMEOUT_1;
	transferBytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
	transferBytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
	transferBytes[14] = IDLE_MODE_TIMEOUT_0;
	transferBytes[15] = IDLE_MODE_TIMEOUT_1;
	transferBytes[16] = LP1_MODE_TIMEOUT_0;
	transferBytes[17] = LP1_MODE_TIMEOUT_1;
	transferBytes[18] = REF_UPDATE_TIME_0;
	transferBytes[19] = REF_UPDATE_TIME_1;
	transferBytes[20] = I2C_TIMEOUT_0;
	transferBytes[21] = I2C_TIMEOUT_1;
//	writeRandomBytes(IQS7211A_MM_ACTIVE_MODE_RR, 22, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_ACTIVE_MODE_RR,transferBytes, 22, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_ACTIVE_MODE_RR, 1,
			transferBytes, 22, 5);

	/* Change the System Settings */
	/* Memory Map Position 0x50 - 0x5B */
	transferBytes[0] = SYSTEM_CONTROL_0;
	transferBytes[1] = SYSTEM_CONTROL_1;
	transferBytes[2] = CONFIG_SETTINGS0;
	transferBytes[3] = CONFIG_SETTINGS1;
	transferBytes[4] = OTHER_SETTINGS_0;
	transferBytes[5] = OTHER_SETTINGS_1;
	transferBytes[6] = TRACKPAD_TOUCH_SET_THRESHOLD;
	transferBytes[7] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
	transferBytes[8] = ALP_THRESHOLD_0;
	transferBytes[9] = ALP_THRESHOLD_1;
	transferBytes[10] = OPEN_0_0;
	transferBytes[11] = OPEN_0_1;
	transferBytes[12] = ALP_SET_DEBOUNCE;
	transferBytes[13] = ALP_CLEAR_DEBOUNCE;
	transferBytes[14] = OPEN_1_0;
	transferBytes[15] = OPEN_1_1;
	transferBytes[16] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
	transferBytes[17] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
	transferBytes[18] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
	transferBytes[19] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
	transferBytes[20] = TRACKPAD_HARDWARE_SETTINGS_0;
	transferBytes[21] = TRACKPAD_HARDWARE_SETTINGS_1;
	transferBytes[22] = ALP_HARDWARE_SETTINGS_0;
	transferBytes[23] = ALP_HARDWARE_SETTINGS_1;
//	writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 24, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_SYSTEM_CONTROL,transferBytes, 24, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_SYSTEM_CONTROL, 1,
			transferBytes, 24, 5);

	/* Change the Trackpad Settings */
	/* Memory Map Position 0x60 - 0x69 */
	transferBytes[0] = TRACKPAD_SETTINGS_0_0;
	transferBytes[1] = TRACKPAD_SETTINGS_0_1;
	transferBytes[2] = TRACKPAD_SETTINGS_1_0;
	transferBytes[3] = TRACKPAD_SETTINGS_1_1;
	transferBytes[4] = X_RESOLUTION_0;
	transferBytes[5] = X_RESOLUTION_1;
	transferBytes[6] = Y_RESOLUTION_0;
	transferBytes[7] = Y_RESOLUTION_1;
	transferBytes[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
	transferBytes[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
	transferBytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
	transferBytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
	transferBytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
	transferBytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
	transferBytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
	transferBytes[15] = FINGER_SPLIT_FACTOR;
	transferBytes[16] = X_TRIM_VALUE_0;
	transferBytes[17] = X_TRIM_VALUE_1;
	transferBytes[18] = Y_TRIM_VALUE_0;
	transferBytes[19] = Y_TRIM_VALUE_1;
//	writeRandomBytes(IQS7211A_MM_TP_SETTINGS_0, 20, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_TP_SETTINGS_0,transferBytes, 20, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_TP_SETTINGS_0, 1,
			transferBytes, 20, 5);

	/* Change the ALP Settings */
	/* Memory Map Position 0x70 - 0x74 */
	transferBytes[0] = ALP_COUNT_FILTER_BETA_0;
	transferBytes[1] = OPEN_0;
	transferBytes[2] = ALP_LTA_BETA_LP1;
	transferBytes[3] = ALP_LTA_BETA_LP2;
	transferBytes[4] = ALP_SETUP_0;
	transferBytes[5] = ALP_SETUP_1;
	transferBytes[6] = ALP_TX_ENABLE_0;
	transferBytes[7] = ALP_TX_ENABLE_1;

	/* Change the Settings Version Numbers */
	/* Memory Map Position 0x74 - 0x75 */
	transferBytes[8] = MINOR_VERSION;
	transferBytes[9] = MAJOR_VERSION;
//	writeRandomBytes(IQS7211A_MM_ALP_COUNT_FILTER_BETA, 10, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_ALP_COUNT_FILTER_BETA,transferBytes, 10, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_ALP_COUNT_FILTER_BETA, 1,
			transferBytes, 10, 5);

	/* Change the Gesture Settings */
	/* Memory Map Position 0x80 - 0x8F */
	transferBytes[0] = GESTURE_ENABLE_0;
	transferBytes[1] = GESTURE_ENABLE_1;
	transferBytes[2] = TAP_TIME_0;
	transferBytes[3] = TAP_TIME_1;
	transferBytes[4] = TAP_DISTANCE_0;
	transferBytes[5] = TAP_DISTANCE_1;
	transferBytes[6] = HOLD_TIME_0;
	transferBytes[7] = HOLD_TIME_1;
	transferBytes[8] = SWIPE_TIME_0;
	transferBytes[9] = SWIPE_TIME_1;
	transferBytes[10] = SWIPE_X_DISTANCE_0;
	transferBytes[11] = SWIPE_X_DISTANCE_1;
	transferBytes[12] = SWIPE_Y_DISTANCE_0;
	transferBytes[13] = SWIPE_Y_DISTANCE_1;
	transferBytes[14] = SWIPE_ANGLE_0;
	transferBytes[15] = GESTURE_OPEN_0;
//	writeRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 16, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_GESTURE_ENABLE,transferBytes, 16, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_GESTURE_ENABLE, 1,
			transferBytes, 16, 5);

	/* Change the RxTx Mapping */
	/* Memory Map Position 0x90 - 0x9C */
	transferBytes[0] = RX_TX_MAP_0;
	transferBytes[1] = RX_TX_MAP_1;
	transferBytes[2] = RX_TX_MAP_2;
	transferBytes[3] = RX_TX_MAP_3;
	transferBytes[4] = RX_TX_MAP_4;
	transferBytes[5] = RX_TX_MAP_5;
	transferBytes[6] = RX_TX_MAP_6;
	transferBytes[7] = RX_TX_MAP_7;
	transferBytes[8] = RX_TX_MAP_8;
	transferBytes[9] = RX_TX_MAP_9;
	transferBytes[10] = RX_TX_MAP_10;
	transferBytes[11] = RX_TX_MAP_11;
	transferBytes[12] = RX_TX_MAP_12;
//	writeRandomBytes(IQS7211A_MM_RXTX_MAPPING_1_0, 13, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_RXTX_MAPPING_1_0,transferBytes, 13, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_RXTX_MAPPING_1_0, 1,
			transferBytes, 13, 5);
	/* Change the Allocation of channels into cycles 0-9 */
	/* Memory Map Position 0xA0 - 0xBD */
	transferBytes[0] = PLACEHOLDER_0;
	transferBytes[1] = CH_1_CYCLE_0;
	transferBytes[2] = CH_2_CYCLE_0;
	transferBytes[3] = PLACEHOLDER_1;
	transferBytes[4] = CH_1_CYCLE_1;
	transferBytes[5] = CH_2_CYCLE_1;
	transferBytes[6] = PLACEHOLDER_2;
	transferBytes[7] = CH_1_CYCLE_2;
	transferBytes[8] = CH_2_CYCLE_2;
	transferBytes[9] = PLACEHOLDER_3;
	transferBytes[10] = CH_1_CYCLE_3;
	transferBytes[11] = CH_2_CYCLE_3;
	transferBytes[12] = PLACEHOLDER_4;
	transferBytes[13] = CH_1_CYCLE_4;
	transferBytes[14] = CH_2_CYCLE_4;
	transferBytes[15] = PLACEHOLDER_5;
	transferBytes[16] = CH_1_CYCLE_5;
	transferBytes[17] = CH_2_CYCLE_5;
	transferBytes[18] = PLACEHOLDER_6;
	transferBytes[19] = CH_1_CYCLE_6;
	transferBytes[20] = CH_2_CYCLE_6;
	transferBytes[21] = PLACEHOLDER_7;
	transferBytes[22] = CH_1_CYCLE_7;
	transferBytes[23] = CH_2_CYCLE_7;
	transferBytes[24] = PLACEHOLDER_8;
	transferBytes[25] = CH_1_CYCLE_8;
	transferBytes[26] = CH_2_CYCLE_8;
	transferBytes[27] = PLACEHOLDER_9;
	transferBytes[28] = CH_1_CYCLE_9;
	transferBytes[29] = CH_2_CYCLE_9;
//	writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_0_9, 30, transferBytes, RESTART);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_CYCLE_SETUP_0_9,transferBytes, 30, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), IQS7211A_MM_CYCLE_SETUP_0_9, 1,
			transferBytes, 30, 5);

	/* Change the Allocation of channels into cycles 10-17 */
	/* Memory Map Position 0xB0 - 0xCA */
	transferBytes[0] = PLACEHOLDER_10;
	transferBytes[1] = CH_1_CYCLE_10;
	transferBytes[2] = CH_2_CYCLE_10;
	transferBytes[3] = PLACEHOLDER_11;
	transferBytes[4] = CH_1_CYCLE_11;
	transferBytes[5] = CH_2_CYCLE_11;
	transferBytes[6] = PLACEHOLDER_12;
	transferBytes[7] = CH_1_CYCLE_12;
	transferBytes[8] = CH_2_CYCLE_12;
	transferBytes[9] = PLACEHOLDER_13;
	transferBytes[10] = CH_1_CYCLE_13;
	transferBytes[11] = CH_2_CYCLE_13;
	transferBytes[12] = PLACEHOLDER_14;
	transferBytes[13] = CH_1_CYCLE_14;
	transferBytes[14] = CH_2_CYCLE_14;
	transferBytes[15] = PLACEHOLDER_15;
	transferBytes[16] = CH_1_CYCLE_15;
	transferBytes[17] = CH_2_CYCLE_15;
	transferBytes[18] = PLACEHOLDER_16;
	transferBytes[19] = CH_1_CYCLE_16;
	transferBytes[20] = CH_2_CYCLE_16;
	transferBytes[21] = PLACEHOLDER_17;
	transferBytes[22] = CH_1_CYCLE_17;
	transferBytes[23] = CH_2_CYCLE_17;
//	writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_10_17, 24, transferBytes, stopOrRestart);
//	HAL_I2C_Master_Transmit(&hi2c1, IQS7211A_MM_CYCLE_SETUP_10_17,transferBytes, 24, 100);
	while (azoFlag != 1)
		;
	azoFlag = 0;
//	HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_CYCLE_SETUP_10_17,1, transferBytes, 24, 5);
	HAL_I2C_Mem_Write(&hi2c1, 0x56 << 1, IQS7211A_MM_CYCLE_SETUP_10_17,
			I2C_MEMADD_SIZE_8BIT, transferBytes, 24, 100);

}

void make_reset_certain() {
	uint8_t transferBytes[2];// The array which will hold the bytes to be transferred.
	uint8_t software_reset[2];

	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), 0x10, 1, transferBytes, 2, 10);// read info flags
	if ((transferBytes[0] & 0x80) == 0b10000000) { //if reset bit is set
		//remember to acknowledge reset
		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), 0x50, 1, software_reset, 2, 10);
		software_reset[0] |= 0x80;
		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), 50, 1, software_reset, 1, 5);

	} else {
		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), 0x50, 1, software_reset, 2, 10);
		software_reset[1] |= 0x02;
		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), 50, 1, software_reset, 1, 5);//write the software reset bit

		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), 0x50, 1, software_reset, 2, 10);
		software_reset[0] |= 0x80;
		while (azoFlag != 1)
			;
		azoFlag = 0;
		HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), 50, 1, software_reset, 1, 5);

	}
}
//void flipY() {
//	uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
//
//	// First read the bytes at the memory address so that they can be preserved.
//	//readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes, RESTART);
//	azoFlag = 0;
//	while (!azoFlag) {
//	}
//	HAL_I2C_Mem_Read(&hi2c1, 0x56 << 1, 0x60,
//	I2C_MEMADD_SIZE_8BIT, transferBytes, 2, 100);
//	// Set the TP_EVENT_BIT in CONFIG_SETTINGS
//	transferBytes[0] |= 0b00000010;
//	// Write the bytes back to the device
//	//writeRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes, stopOrRestart);
//	azoFlag = 0;
//	while (!azoFlag) {
//	}
//	HAL_I2C_Mem_Write(&hi2c1, 0x56 << 1, 0x60,
//	I2C_MEMADD_SIZE_8BIT, transferBytes, 2, 100);
//}

void TP_ReATI() {
	uint8_t transferBytes[2]; // Array to store the bytes transferred.
							  // Use an array to be consistent with other methods in this class.
	//  readRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 1, transferByte, RESTART);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), 0x50, 1, transferBytes, 2, 5);
	// Mask the settings with the REDO_ATI_BIT.
	transferBytes[0] |= 0x20; // This is the bit required to start an ATI routine.
	// Write the new byte to the required device.
	//  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 1, transferByte, stopOrRestart);
	while (azoFlag != 1)
		;
	azoFlag = 0;
	HAL_I2C_Mem_Write(&hi2c1, (0x56 << 1), 0x50, 1, transferBytes, 2, 5);
//		  HAL_I2C_Mem_Write(&hi2c1, 0x56 << 1, IQS7211A_MM_SYSTEM_CONTROL,I2C_MEMADD_SIZE_8BIT, transferBytes, 1, 100);
	//	I2C_MEMADD_SIZE_8BIT, transferBytes, 24, 100);
}

// Read the info flags.
//	readRandomBytes(IQS7211A_MM_INFOFLAGS, 2, transferBytes, stopOrRestart);

//void restartTrackpad() {
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
//}
//
//
void updateTrackpad() {


	 if (azoFlag == 1) {
		//duration = HAL_GetTick() - startTime;

		uint8_t Gesture = 0;
		uint8_t xCoordLow = 0;
		uint8_t xCoordHigh = 0;
		uint8_t yCoordLow = 0;
		uint8_t yCoordHigh = 0;



		uint8_t transferBytes[10];
		HAL_I2C_Mem_Read(&hi2c1, (0x56 << 1), IQS7211A_MM_GESTURES,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) &transferBytes, 10,
				HAL_MAX_DELAY);
		Gesture = transferBytes[0];
		xCoordLow = transferBytes[6];
		xCoordHigh = transferBytes[7];
		yCoordLow = transferBytes[8];
		yCoordHigh = transferBytes[9];

		xCoord = (uint16_t) (xCoordLow);
		xCoord |= (uint16_t) (xCoordHigh << 8);
		yCoord = (uint16_t) (yCoordLow);
		yCoord |= (uint16_t) (yCoordHigh << 8);


		if ((xCoord != 65535) && (yCoord != 65535)) {


			yCoord=768-yCoord;

			char Coord_x[7];
			char Coord_y[7];

			sprintf(&Coord_x, "%05d  ", xCoord);
			sprintf(&Coord_y, "%05d\n", yCoord);


			char initCoX[7];
			char initCoY[7];

			if (PrevDownpressFlag == 0) {
				InitialxCoord = xCoord;
				InitialyCoord = yCoord;

					initialstate_intensity=out_new;

			}
			else if (PrevDownpressFlag == 1){
				volconflag = 1;
				slideroldtrack = output_voltage;
			}


			InitialdiffX = xCoord - InitialxCoord;
			InitialdiffY = yCoord - InitialyCoord;

			InitialdiffX=(InitialdiffX/1783)*4255.0;
			InitialdiffY=(InitialdiffY/745)*4095.0;

			distance = sqrt(pow((int)InitialdiffX, 2) + pow((int)InitialdiffY, 2));



			if (distance > 130) {

				SlideFlag=1;
			}




			if (Gesture == 0b00000001) {


				TapFlag = 1;
			}
			if (Gesture == 0b00000010) {
				if (pressAndHoldFlag == 0 && PrevpressAndHoldFlag == 0) {
					pressAndHoldFlag = 1;
				}
				PrevpressAndHoldFlag = 1;
			} else {
				PrevpressAndHoldFlag = 0;
			}

			PrevDownpressFlag = 1;
		} else {
			SlideFlag=0;
			PrevDownpressFlag = 0;
		}


	}
	azoFlag = 0;
}
