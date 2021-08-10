/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "u8g2.h"

#include "st7528.h"
#include "ctype.h"
#include "font5x7.h"
#include "font7x10.h"
#include "string.h"
#include "math.h"
#include "uart.h"
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskUSB */
osThreadId_t myTaskUSBHandle;
const osThreadAttr_t myTaskUSB_attributes = {
  .name = "myTaskUSB",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTaskInputBuff */
osThreadId_t myTaskInputBuffHandle;
const osThreadAttr_t myTaskInputBuff_attributes = {
  .name = "myTaskInputBuff",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTaskEthernet */
osThreadId_t myTaskEthernetHandle;
const osThreadAttr_t myTaskEthernet_attributes = {
  .name = "myTaskEthernet",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTaskUI */
osThreadId_t myTaskUIHandle;
const osThreadAttr_t myTaskUI_attributes = {
  .name = "myTaskUI",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTaskLCD */
osThreadId_t myTaskLCDHandle;
const osThreadAttr_t myTaskLCD_attributes = {
  .name = "myTaskLCD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskButtons */
osThreadId_t myTaskButtonsHandle;
const osThreadAttr_t myTaskButtons_attributes = {
  .name = "myTaskButtons",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskUART */
osThreadId_t myTaskUARTHandle;
const osThreadAttr_t myTaskUART_attributes = {
  .name = "myTaskUART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskSD */
osThreadId_t myTaskSDHandle;
const osThreadAttr_t myTaskSD_attributes = {
  .name = "myTaskSD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */



static u8g2_t u8g2;
u8x8_t u8x8;

// HAL RTC time and date pointer
RTC_DateTypeDef Date;
RTC_TimeTypeDef Time;

//static u8g2_t u8g2;


// various semafors for indication and timing control
uint8_t isWaitingForData = 1;
uint8_t isReadyForNext = 1;

uint8_t isTriggered = 0;
uint8_t isAdcFinished = 0;

uint8_t isEnterTriggered = 0;
uint8_t isEscTriggered = 0;
uint8_t isUpTriggered = 0;
uint8_t isDownTriggered = 0;
uint8_t isLeftTriggered = 0;
uint8_t isRightTriggered = 0;

uint32_t previousButtonPress = 0;
uint32_t buttonPressInterval = 50000;

/***********************************************************************************************************/
/*
// local variable (test use only)
uint8_t isLoggingToConsole = 1;
uint8_t isLoggingToSD = 0;

// in us
// measuring current all the time, for range change
uint32_t measuringInterval = 10; // can go as low as 10 us (actual times will be between 13-25 us), for reliable measuring interval go higher than 25 us

// in us
// interval for saving measured values to UART or SD card
uint32_t samplingInterval = 100;

double triggerLevel = 0.0001;

double offsetValue = 0;

uint8_t *pointer_isLoggingToConsole = &isLoggingToConsole;

uint8_t isAveraging = 0;

uint8_t isTriggerActive = 0;


*/
/***********************************************************************************************************/

uint8_t startOfMeasurement = 0;
uint8_t endOfMeasurement = 0;

uint8_t volatile isMeasuring = 0;
uint8_t *pointer_isMeasuring = &isMeasuring;



//#define DEBUG
#define DEBUG_CONSOLE_WRITE


uint32_t adcBuffer = 0;
uint32_t adcValue = 0;
uint8_t isAdcDone = 0;

/***************** FATfs ***********************/
USBH_HandleTypeDef hUSB_Host;
//HAL_SD_CardInfoTypedef SDCardInfo;


FATFS USBFatFs, SDFatFs;    /* File system objects logical drives */
FIL USBFile, SDFile;        /* File objects */
char USBpath[4], SDpath[4]; /* Flash disk and SD card logical drives paths */
uint8_t workBuffer[2*_MAX_SS];

FRESULT resUSB, resSD;                                   /* FatFs function common result codes */
uint32_t bytesWrittenUSB, bytesWrittenSD;                /* File write counts */
uint32_t bytesReadUSB, bytesReadSD;                      /* File read counts */
uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
uint8_t rtextUSB[512], rtextSD[512];                     /* File read buffers */



// SD CARD
FATFS fs;  // file system
FIL fil;  // file
FRESULT fresult;  // to store the result
char buffer[1024]; // to store data

UINT br, bw;   // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint8_t loggingFileName [20];


#define SECTOR_SIZE 512*8
#define LOGGING_FILE_RING_BUFFER_SIZE 16384

uint8_t txLogFile[LOGGING_FILE_RING_BUFFER_SIZE];
uint16_t txLogLen = SECTOR_SIZE;
ringbuff_t txLogFileRingBuffer;

/**************-FATfs *******************/

uint32_t timeWindowMeasuring = 0;
uint32_t timeWindowSampling = 0;

uint32_t measuringNumber = 0;
uint32_t cycleNumber = 0;

//uint8_t uartBufferTx[35];
uint8_t uartBufferTx[60];
uint8_t uartBufferTxJumbo[512];

uint8_t charBuffer [20];

uint8_t inputData[10];

uint8_t buffer4096 [4096];
uint16_t buffer4096CurrentPosition = 0;
uint8_t buffer4096FrameCount = 0;

uint32_t buffer4096MeasuringNumber [50];
double buffer4096MeasuredValue [50];


// interval between checking console and screen input
uint32_t consoleInteractionInterval = 250000;


uint32_t previousBlinkTime = 0;
uint32_t previousMeasuringTime = 0;
uint32_t previousSamplingTime = 0;
uint32_t previousConsoleInteraction = 0;

uint8_t data;
uint8_t currentRange = 0;
uint8_t rangeMode = 4;
uint8_t currentValuePosition = 0;
double previousValues[SAMPLES];
uint8_t previousValuesRange [SAMPLES];

uint8_t counter = 0;
uint8_t testValue = 0;

// linear regression
uint8_t dummyValues[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
float sumX = 0, sumX2 = 0, sumY = 0, sumXY = 0, a, b;


// ADC SPI buffers
uint8_t spiDataTx[4];
uint8_t spiDataRx[4];


// STM HAL return value
HAL_SPI_StateTypeDef returnValue;


// ADC regarding values
//double ADC_RESOLUTION = 0;
double measuredValue = 0;
double previousMeasuredValue = 0;
double measuredValueCurrent = 0;


// for I2C EEPROM, I2C OLED
I2C_HandleTypeDef* i2c_port = &hi2c4;
struct deviceSettings settings = {1, 0, 5, 100, 0, 0, 0.0001, 0.0 , 0, 0};

//double ADC_RESOLUTION = ((ADC_REF_VALUE*ADC_PGA*ADC_DIRECTION)/ADC_SCALE);
//double ADC_RESOLUTION = 0.01953125;
double ADC_RESOLUTION = 0.038146973;
double POT_A_RESOLUTION = ( POT_A_VALUE / POT_A_STEPS ) ;
double POT_B_RESOLUTION = ( POT_B_VALUE / POT_B_STEPS ) ;


// I2C EL. POT control
uint8_t i2cDeviceAddress = (0x50 << 1); // shifting address 1 bit to left..... address with all 3 jumpers to gnd == 0x50  01010 0000
uint8_t i2cWiperAdressA = 0x00;
uint8_t i2cWiperAdressB = 0x01;
// Pot A = Voltage control, Pot B = current limit control
uint8_t i2cWiperAdressPotA = 0x18;
uint8_t i2cWiperAdressPotB = 0x4E;

uint8_t i2cDataToWrite[3];
uint8_t i2cDataToRead[3];

// OLED SSD1106
//uint8_t i2cDeviceAddressOLED = (0x3C << 1);		// I2C OLED address eq. 0x78 without shift

// ST7528 LCD
uint8_t i2cDeviceAddressOLED = (0x3F << 1);		// I2C OLED address eq. 0x78 without shift
//uint8_t i2cDeviceAddressOLED = 0x3F;		// I2C OLED address eq. 0x78 without shift



uint8_t spiTxBuffer [201];
uint8_t spiRxBuffer [201];


HAL_SPI_StateTypeDef retValue;
HAL_SPI_StateTypeDef retValueSPI;

uint32_t spiErrorValue = 0;

#define RB_INPUT_SIZE 131072 //4096
#define RB_OUTPUT_SIZE 8192

// char buffer for ring buffer with capacity for up to 102 transmission=10ms
uint8_t inputBuffer [RB_INPUT_SIZE];

// with formatted data in string for output media etc flash, sd, uart
uint8_t	outputFormatterBuffer [RB_OUTPUT_SIZE];

ringbuff_t inputBuffer_RB;
ringbuff_t outputBuffer_RB;

uint32_t spiCounter = 0;
uint32_t spiCounterEnd = 0;


uint32_t errorCount = 0;
uint32_t okCount = 0;


#define SPI_PACKET_SIZE 120


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_RTC_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART7_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void vTaskUSB(void *argument);
void vTaskInputBuffer(void *argument);
void vTaskEthernet(void *argument);
void vTaskUi(void *argument);
void vTaskLcd(void *argument);
void vTaskButtons(void *argument);
void vTaskUart(void *argument);
void vTaskSd(void *argument);

/* USER CODE BEGIN PFP */
void MX_USB_HOST_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){

	spiCounter++;

	//HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

	//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 40);


	// add received data to input ring buffer for addition processing
	ringbuff_write(&inputBuffer_RB, spiRxBuffer, SPI_PACKET_SIZE);


	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	retValueSPI = HAL_SPI_TransmitReceive_DMA(&hspi4, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE);


	spiCounterEnd++;


	//HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);

	//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	//HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

	//sprintf(buffer, "[0]: %d, [1]: %d, [2]: %d, [3]: %d\n", spiRxBuffer[0], spiRxBuffer[1], spiRxBuffer[2], spiRxBuffer[3]);
	//send_uart(buffer);

}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){

	HAL_GPIO_WritePin(FPGA_RDY_PORT, FPGA_RDY_PIN, GPIO_PIN_RESET);
	//HAL_Delay(1);

	retValue = HAL_SPI_GetState(&hspi4);


	errorCount++;

	// 4 - OVR=overrun error, 16 - DMA error
	spiErrorValue = HAL_SPI_GetError(&hspi4);

	//HAL_SPI_DMAStop(&hspi3);
	//HAL_SPI_Abort(&hspi3);

	send_uart("SPI Error callback\n");
	sprintf(buffer, "SPI ERROR: %d\n", spiErrorValue);
	send_uart(buffer);

	//retValue = HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);

	if(retValue == HAL_SPI_STATE_BUSY_RX || retValue == HAL_SPI_STATE_BUSY_TX || retValue == HAL_SPI_STATE_BUSY_TX_RX || retValue == HAL_SPI_STATE_BUSY)
		send_uart("SPI transfer running\n");
	else{
		if(retValue == HAL_SPI_STATE_READY){
			send_uart("SPI transfer not running - ready\n");
			//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
		}
		if(retValue == HAL_SPI_STATE_RESET){
			send_uart("SPI transfer not running - reset\n");
			//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
		}
		if(retValue == HAL_SPI_STATE_ERROR){
			send_uart("SPI transfer not running - error\n");
			//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
		}
		if(retValue == HAL_SPI_STATE_ABORT){
			send_uart("SPI transfer not running - abort\n");
			//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
		}

		HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
		HAL_SPI_TransmitReceive_DMA(&hspi4, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE);
		HAL_GPIO_WritePin(FPGA_RDY_PORT, FPGA_RDY_PIN, GPIO_PIN_SET);

	}


}


// user button instrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	/*
	if (GPIO_Pin == ADC_RSV_PIN) {
			isAdcDone = 1;
	}
	*/

	if( ( __HAL_TIM_GET_COUNTER(&htim5) - previousButtonPress ) >=  buttonPressInterval ){
		previousButtonPress += ( ( __HAL_TIM_GET_COUNTER(&htim5) ) - previousButtonPress );


	if (GPIO_Pin == BUTTON_MEASURE_PIN) {

		if (isMeasuring == 0) {

			if( settings.powerSourceEnableMode == 1 ){
				  HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);
				  settings.powerSourceEnable == 1;
				  //send_uart3("Power Source: ENABLED\n");
			 }

			//isLoggingToConsole = 1;
			if(settings.isTriggerActive == 0)
				isMeasuring = 1;
			startOfMeasurement = 1;
			endOfMeasurement = 0;

			HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
			buzzerOn();

		} else {


			if( settings.powerSourceEnableMode == 1){
				  HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, RESET);
				  settings.powerSourceEnable == 0;
				  //send_uart3("Power Source: DISABLED\n");
			 }

			isMeasuring = 0;
			isTriggered = 0;
			startOfMeasurement = 0;
			endOfMeasurement = 1;

			HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
			buzzerOff();
		}

	}
	if (GPIO_Pin == ADC_ALARM_PIN && settings.isTriggerActive == 1) {
		//isMeasuring = 1;

	}

	if(GPIO_Pin == BUTTON_ENTER_PIN){

		isEnterTriggered = 1;
		//HAL_GPIO_TogglePin(LED_BLUE_PORT, LED_BLUE_PIN);

	}
	else if(GPIO_Pin == BUTTON_ESC_PIN){

		isEscTriggered = 1;
		//HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);

	}
	else if(GPIO_Pin == BUTTON_UP_PIN){

		isUpTriggered = 1;
			//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

	}
	else if (GPIO_Pin == BUTTON_DOWN_PIN){

		isDownTriggered = 1;
		//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	}
	else if (GPIO_Pin == BUTTON_LEFT_PIN){

		isLeftTriggered = 1;
		//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	}
	else if (GPIO_Pin == BUTTON_RIGHT_PIN){

		isRightTriggered = 1;
		//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	}
	else{
		//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
		//HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
	}


	}
	// when debounce active
	else{

		isDownTriggered = 0;
		isUpTriggered = 0;
		isLeftTriggered = 0;
		isRightTriggered = 0;


		//isEscTriggered = 0;
		//isEnterTriggered = 0;

	}


}



void setMeasuringRange (){

	if (rangeMode == 0){	// NA
		HAL_GPIO_WritePin(RANGE_SEL_1_PORT, RANGE_SEL_1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SEL_2_PORT, RANGE_SEL_2_PIN, GPIO_PIN_SET);
	}
	else if (rangeMode == 1){	// UA
		HAL_GPIO_WritePin(RANGE_SEL_1_PORT, RANGE_SEL_1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SEL_2_PORT, RANGE_SEL_2_PIN, GPIO_PIN_RESET);
	}
	else if (rangeMode == 2){	// MA
		HAL_GPIO_WritePin(RANGE_SEL_1_PORT, RANGE_SEL_1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SEL_2_PORT, RANGE_SEL_2_PIN, GPIO_PIN_SET);
	}
	else{	// AUTO
		HAL_GPIO_WritePin(RANGE_SEL_1_PORT, RANGE_SEL_1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SEL_2_PORT, RANGE_SEL_2_PIN, GPIO_PIN_RESET);
	}

}

// Enable buzzer
void buzzerOn(){
	//HAL_TIM_PWM_ConfigChannel(&htim1, &sCo, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	//htim1.Instance->CCR1 = 75;
}

// Disable buzzer
void buzzerOff(){
	//HAL_TIM_PWM_ConfigChannel(&htim1, &sCo, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	//HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
}

/* to send the data to the uart */
void send_uart(char *string) {
	uint16_t len = strlen(string);

	SCB_CleanDCache_by_Addr((uint32_t*)&string[0], len);

	//HAL_UART_Transmit(&huart3, (uint8_t*) string, len, 10); // transmit in blocking mode
	//HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, len); // transmit in non blocking

	//isReadyForNext = 0;
	//HAL_UART_Transmit_DMA(&huart6, (uint8_t*) string, len);
	HAL_UART_Transmit(&huart6, (uint8_t*) string, len, 50);
	//UARTAddToTxBuff(string, len);

	//while(isReadyForNext == 0);

	// usb test
	//USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *) string, len);
	//CDC_Transmit_FS((uint8_t*) string, len);
}

void send_uart2(char *string) {
	uint16_t len = strlen(string);

	//SCB_CleanDCache_by_Addr((uint32_t*)&string[0], len);

	//HAL_UART_Transmit(&huart3, (uint8_t*) string, len, 10); // transmit in blocking mode
	//HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, len); // transmit in non blocking
	//HAL_UART_Transmit_DMA(&huart7, (uint8_t*) string, len);

	UARTAddToTxBuff2(string, len);

	// usb test
	//USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *) string, len);
	//CDC_Transmit_FS((uint8_t*) string, len);
}

void send_uart3(char *string) {
	uint16_t len = strlen(string);

	//SCB_CleanDCache_by_Addr((uint32_t*)&string[0], len);
	//HAL_UART_Transmit_DMA(&huart3, (uint8_t*) string, len);
	//HAL_UART_Transmit_DMA(&huart7, (uint8_t*) string, len);
	UARTAddToTxBuff(string, len);

}


/* to find the size of data in the buffer */
int bufsize(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

/* to clear buffer */
void bufclear(void)  // clear buffer
{
	for (int i = 0; i < 1024; i++) {
		buffer[i] = '\0';
	}
}

// 1us base delay function
void microDelay(uint16_t delay) {

	__HAL_TIM_SET_COUNTER(&htim4, 0);

	while (__HAL_TIM_GET_COUNTER(&htim4) < delay);

}


HAL_StatusTypeDef eepromReadEEPROM(uint16_t address, uint8_t* MemTarget, uint16_t Size)
{
	uint16_t Counter = 0;
	HAL_StatusTypeDef Result = HAL_OK;
	while (Counter < Size && Result == HAL_OK)
	{
		uint16_t Diff = Size - Counter;

		if (Diff >= EEPROM_MAXPKT)
		{
			//Multi-Byte
			Result = HAL_I2C_Mem_Read(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
			Counter += EEPROM_MAXPKT;
		}
		else
		{
			//and the remaining ones...low packet size
			Result = HAL_I2C_Mem_Read(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], Diff, EEPROM_TIMEOUT);
			Counter += Diff;
		}
		HAL_Delay(EEPROM_WRITE / 2);
	}
	return Result;
}

HAL_StatusTypeDef eepromWriteEEPROM(uint16_t address, uint8_t* MemTarget, uint16_t Size)
{
	uint16_t Counter = 0;
	HAL_StatusTypeDef Result = HAL_OK;
	while (Counter < Size && Result == HAL_OK)
	{
		uint16_t Diff = Size - Counter;

		if (Diff >= EEPROM_MAXPKT)
		{
			//Multi-Byte
			Result = HAL_I2C_Mem_Write(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], EEPROM_MAXPKT, EEPROM_TIMEOUT);
			Counter += EEPROM_MAXPKT;
		}
		else
		{
			//and the remaining ones...low packet size
			Result = HAL_I2C_Mem_Write(i2c_port, EEPROM_ADDRESS,
					address + Counter, I2C_MEMADD_SIZE_16BIT,
					&MemTarget[Counter], Diff, EEPROM_TIMEOUT);
			Counter += Diff;
		}
		HAL_Delay(EEPROM_WRITE);
	}
	return Result;
}


//This is basically just a nicer wrapper
void eepromReadObject(struct{} *settings, int section)
{
	//Read a screen settings object from eeprom
	eepromReadEEPROM(section * EEPROM_SECTIONSIZE, (uint8_t*) settings, sizeof(settings));
}

void eepromSaveObject(struct{} *settings, int section)
{
	eepromWriteEEPROM(section * EEPROM_SECTIONSIZE, (uint8_t*) settings, sizeof(settings));
}

/*
struct deviceSettings{

	uint8_t isLoggingToConsole;
	uint8_t isLoggingToSD;
	uint32_t measuringInterval;
	uint32_t samplingInterval;
	uint8_t isAveraging;
	uint8_t isTriggerActive;
	double triggerLevel;
	double lastOffsetValue;
	uint16_t powerSourceVoltage;
	uint16_t powerSourceCurrent;
	uint8_t powerSourceEnable;
	uint8_t powerSourceEnableMode;
	uint8_t isLoggingToUSB;
	uint8_t isLoggingToEthernet;
	//double ADC_RESOLUTION;

};*/

void writeEEPROM_AT24(){

	uint8_t dataLength = 36;
	uint8_t dataBuffer [dataLength];
	uint64_t buffer64 = 0;


	dataBuffer[0] = settings.isLoggingToConsole;
	dataBuffer[1] = settings.isLoggingToSD;
	dataBuffer[2] = settings.measuringInterval>>24;
	dataBuffer[3] = settings.measuringInterval>>16;
	dataBuffer[4] = settings.measuringInterval>>8;
	dataBuffer[5] = settings.measuringInterval;
	dataBuffer[6] = settings.samplingInterval>>24;
	dataBuffer[7] = settings.samplingInterval>>16;
	dataBuffer[8] = settings.samplingInterval>>8;
	dataBuffer[9] = settings.samplingInterval;
	dataBuffer[10] = settings.isAveraging;
	dataBuffer[11] = settings.isTriggerActive;

	buffer64 = settings.triggerLevel*1000000000;
	dataBuffer[12] = buffer64>>56;
	dataBuffer[13] = buffer64>>48;
	dataBuffer[14] = buffer64>>40;
	dataBuffer[15] = buffer64>>32;
	dataBuffer[16] = buffer64>>24;
	dataBuffer[17] = buffer64>>16;
	dataBuffer[18] = buffer64>>8;
	dataBuffer[19] = buffer64;

	buffer64 = settings.lastOffsetValue*1000000000;
	dataBuffer[20] = buffer64>>56;
	dataBuffer[21] = buffer64>>48;
	dataBuffer[22] = buffer64>>40;
	dataBuffer[23] = buffer64>>32;
	dataBuffer[24] = buffer64>>24;
	dataBuffer[25] = buffer64>>16;
	dataBuffer[26] = buffer64>>8;
	dataBuffer[27] = buffer64;

	dataBuffer[28] = settings.powerSourceVoltage>>8;
	dataBuffer[29] = settings.powerSourceVoltage;

	dataBuffer[30] = settings.powerSourceCurrent>>8;
	dataBuffer[31] = settings.powerSourceCurrent;

	dataBuffer[32] = settings.powerSourceEnable;
	dataBuffer[33] = settings.powerSourceEnableMode;

	dataBuffer[34] = settings.isLoggingToUSB;
	dataBuffer[35] = settings.isLoggingToEthernet;

	//at24_HAL_WriteBytes(&hi2c4, i2cDeviceAddress, 0, &dataBuffer, dataLength);
	eepromWriteEEPROM(0, &dataBuffer, dataLength);


}

void readEEPROM_AT24(){

	uint8_t buffer8 = 0;
	uint32_t buffer32 = 0;
	uint16_t buffer16 = 0;
	uint64_t buffer64 = 0;

	uint8_t dataBuffer [64];

	//at24_HAL_ReadBytes(&hi2c4, i2cDeviceAddress, 0, &dataBuffer, 12);
	eepromReadEEPROM( 0, &dataBuffer, 64);

	settings.isLoggingToConsole = dataBuffer[0];
	settings.isLoggingToSD = dataBuffer[1];

	buffer32 = dataBuffer[5] | dataBuffer[4] << 8 | dataBuffer[3] << 16 | dataBuffer[2] << 24;
	settings.measuringInterval = buffer32;

	buffer32 = 0;
	buffer32 = dataBuffer[9] | dataBuffer[8] << 8 | dataBuffer[7] << 16 | dataBuffer[6] << 24;
	settings.samplingInterval = buffer32;

	settings.isAveraging = dataBuffer[10];
	settings.isTriggerActive = dataBuffer[11];

	buffer64 = 0;
	buffer64 = dataBuffer[19] | dataBuffer[18] << 8 | dataBuffer[17] << 16 | dataBuffer[16] << 24 | dataBuffer[15] << 32 | dataBuffer[14] << 40 | dataBuffer[13] << 48 | dataBuffer[12] << 56;
	settings.triggerLevel = buffer64;
	settings.triggerLevel /= 1000000000;

	buffer64 = 0;
	buffer64 = dataBuffer[27] | dataBuffer[26] << 8 | dataBuffer[25] << 16 | dataBuffer[24] << 24 | dataBuffer[23] << 32 | dataBuffer[22] << 40 | dataBuffer[21] << 48 | dataBuffer[20] << 56;
	settings.lastOffsetValue = buffer64;
	settings.lastOffsetValue /= 1000000000;

	buffer16 = 0;
	buffer16 = dataBuffer[29] | dataBuffer[28] << 8;
	settings.powerSourceVoltage = buffer16;

	buffer16 = 0;
	buffer16 = dataBuffer[31] | dataBuffer[30] << 8;
	settings.powerSourceCurrent = buffer16;

	settings.powerSourceEnable = dataBuffer[32];
	settings.powerSourceEnableMode = dataBuffer[33];

	settings.isLoggingToUSB = dataBuffer[34];
	settings.isLoggingToEthernet = dataBuffer[35];



}

void initEEPROM (){

	settings.isLoggingToConsole = 1;
	settings.isLoggingToSD = 0;
	settings.isLoggingToUSB = 0;
	settings.isLoggingToEthernet = 0;
	settings.measuringInterval = 5;
	settings.samplingInterval = 100;
	settings.isAveraging = 0;
	settings.isTriggerActive = 0;
	settings.triggerLevel = 0.0001;
	settings.lastOffsetValue = 0.0;
	settings.powerSourceVoltage = 2500;
	settings.powerSourceCurrent = 250;
	settings.powerSourceEnable = 1;
	settings.powerSourceEnableMode = 0;

	//eepromSaveObject(settings.isAveraging, 1);


}


// set voltage and current control I2C pots

/*
 * // Pot A = Voltage control, Pot B = current limit control
	uint8_t i2cWiperAdressPotA = 0x18;
	uint8_t i2cWiperAdressPotB = 0x4E;
 *
 */
void setPotenciomenters(){

	uint8_t temp8 = 0;
	double tempDouble = 0;
	double tempDouble2 = 0;

	// write mode LSB bite is low level, read mode LSB bite is high level
	uint8_t writeAddrA = (i2cWiperAdressPotA << 1) | 0x00 ;
	uint8_t readAddrA = (i2cWiperAdressPotA << 1) | 0x01 ;

	uint8_t writeAddrB = (i2cWiperAdressPotB << 1) | 0x00 ;
	uint8_t readAddrB = (i2cWiperAdressPotB << 1) | 0x01 ;

	/*Voltage settings*/
	tempDouble2 = settings.powerSourceVoltage;
	tempDouble = ( (tempDouble2 / 1000) / ( VOLTAGE_SENSE_CURRENT * LT3045_NUMBER ) ) - 4700;
	temp8 = tempDouble / POT_A_RESOLUTION;


	i2cDataToWrite [0] = 0x00;		// instruction 0x00 - RDAC, 0x20 - EEPROM, 0x40 - write protect
	i2cDataToWrite [1] = 0xFF - 1 - temp8;		// data 8 bits
	i2cDataToWrite [2] = 0;

	// voltage control - set value
	HAL_I2C_Master_Transmit(&hi2c4, writeAddrA, &i2cDataToWrite, 2, 1000);

	/*Current settings*/
	temp8 = 0;
	tempDouble = 0;
	tempDouble2 = 0;

	HAL_Delay(100);

	tempDouble2 = settings.powerSourceCurrent;
	tempDouble = ( 150 / tempDouble2 ) * 1000;
	tempDouble *= LT3045_NUMBER;
	tempDouble -= 330; /// by design 330 ohm inserted
	temp8 = tempDouble / POT_B_RESOLUTION;

	i2cDataToWrite [0] = 0x00;		// instruction 0x00 - RDAC, 0x20 - EEPROM, 0x40 - write protect
	i2cDataToWrite [1] = 0xFF - 1 - temp8;		// data 8 bits
	i2cDataToWrite [2] = 0;

	// current control - set value
	HAL_I2C_Master_Transmit(&hi2c4, writeAddrB, &i2cDataToWrite, 2, 1000);


	HAL_Delay(100);

}

void readPotenciometers(){

	// write mode LSB bite is low level, read mode LSB bite is high level
	uint8_t writeAddrA = (i2cWiperAdressPotA << 1) | 0x00 ;
	uint8_t readAddrA = (i2cWiperAdressPotA << 1) | 0x01 ;

	uint8_t writeAddrB = (i2cWiperAdressPotB << 1) | 0x00 ;
	uint8_t readAddrB = (i2cWiperAdressPotB << 1) | 0x01 ;

	i2cDataToWrite [0] = 0x00;		// instruction 0x00 - RDAC, 0x20 - EEPROM, 0x40 - write protect
	i2cDataToWrite [1] = 0x00;		// data 8 bits
	i2cDataToWrite [2] = 0x00;

	// voltage control - set value
	HAL_I2C_Master_Transmit(&hi2c4, readAddrA, &i2cDataToWrite, 1, 1000);

	HAL_I2C_Master_Receive(&hi2c4, readAddrA, &i2cDataToRead, 1, 1000);

	sprintf(uartBufferTx, "\nPot A - Voltage Control - read value: %d\n", i2cDataToRead[0]);
	send_uart3(uartBufferTx);

	HAL_Delay(100);

	i2cDataToWrite [0] = 0x00;		// instruction 0x00 - RDAC, 0x20 - EEPROM, 0x40 - write protect
	i2cDataToWrite [1] = 0x00;		// data 8 bits
	i2cDataToWrite [2] = 0;

	// current control - set value
	HAL_I2C_Master_Transmit(&hi2c4, readAddrB, &i2cDataToWrite, 1, 1000);

	HAL_I2C_Master_Receive(&hi2c4, readAddrB, &i2cDataToRead, 1, 1000);

	sprintf(uartBufferTx, "\nPot B - Current Control - read value: %d\n", i2cDataToRead[0]);
	send_uart3(uartBufferTx);

	HAL_Delay(100);


}




void addDataToBuffer(uint8_t *data[]){

	uint8_t len = strlen(data);

	for(uint8_t i = 0; i < len; i++){

		buffer4096[buffer4096CurrentPosition+1] = data[i];

	}

	buffer4096FrameCount++;
	buffer4096CurrentPosition +=len;



}


// testing sd card communication
void sd_card_test() {

	/* Mount SD Card */
	fresult = f_mount(&fs, "", 0);
	if (fresult != FR_OK) {
#ifdef DEBUG
		send_uart("error in mounting SD CARD...\n");
#endif
	} else {
#ifdef DEBUG
		send_uart("SD CARD mounted successfully...\n");
#endif
	}

	/*************** Card capacity details ********************/

	/* Check free space */
	("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);

#ifdef DEBUG
	send_uart(buffer);
#endif

	bufclear();
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);

#ifdef DEBUG
	send_uart(buffer);
#endif

	/************* The following operation is using PUTS and GETS *********************/

	/* Open file to write/ create a file if it doesn't exist */
	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	/* Writing text */
	fresult = f_puts("This data is from the First FILE\n\n", &fil);

	/* Close file */
	fresult = f_close(&fil);

#ifdef DEBUG
	send_uart("File1.txt created and the data is written \n");
#endif

	/* Open file to read */
	fresult = f_open(&fil, "file1.txt", FA_READ);

	/* Read string from the file */
	f_gets(buffer, &fil.fptr, &fil);

#ifdef DEBUG
	send_uart(buffer);
#endif

	/* Close file */
	f_close(&fil);

	bufclear();

	/**************** The following operation is using f_write and f_read **************************/

	/* Create second file with read write access and open it */
	fresult = f_open(&fil, "file2.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	/* Writing text */
	strcpy(buffer, "This is File 2 and it says Hello from controllerstech\n");

	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

#ifdef DEBUG
	send_uart("File2.txt created and data is written\n");
#endif

	/* Close file */
	f_close(&fil);

	// clearing buffer to show that result obtained is from the file
	bufclear();

	/* Open second file to read */
	fresult = f_open(&fil, "file2.txt", FA_READ);

	/* Read data from the file
	 * Please see the function details for the arguments */
	f_read(&fil, buffer, &fil.fptr, &br);

#ifdef DEBUG
	send_uart(buffer);
#endif

	/* Close file */
	f_close(&fil);

	bufclear();

	/*********************UPDATING an existing file ***************************/

	/* Open the file with write access */
	//fresult = f_open(&fil, "file2.txt", FA_OPEN_ALWAYS | FA_WRITE);
	/* Move to offset to the end of the file */
	//fresult = f_lseek(&fil, &fil.fptr);
	/* write the string to the file */
	//fresult = f_puts("This is updated data and it should be in the end \n", &fil);
	//f_close(&fil);
	/* Open to read the file */
	//fresult = f_open(&fil, "file2.txt", FA_READ);
	/* Read string from the file */
	//f_read(&fil, buffer, &fil.fptr, &br);
#ifdef DEBUG
	//send_uart(buffer);
#endif

	/* Close file */
	//f_close(&fil);
	//bufclear();
	/*************************REMOVING FILES FROM THE DIRECTORY ****************************/

	/*
	 fresult = f_unlink("/file1.txt");
	 if (fresult == FR_OK)
	 send_uart("file1.txt removed successfully...\n");

	 fresult = f_unlink("/file2.txt");
	 if (fresult == FR_OK)
	 send_uart("file2.txt removed successfully...\n");
	 */

	/* Unmount SDCARD */

	fresult = f_mount(NULL, "", 1);
	if (fresult == FR_OK) {

#ifdef DEBUG
		send_uart("SD CARD UNMOUNTED successfully...\n");
#endif
	}

}

void i2c_tests() {

	// I2C dig pot setting

	returnValue = HAL_I2C_IsDeviceReady(&hi2c4, i2cDeviceAddress, 3, 1000);

	if (returnValue == HAL_SPI_STATE_ERROR) {
#ifdef DEBUG
		send_uart("I2C test - error\n");
#endif
	} else {
#ifdef DEBUG
		send_uart("I2C test - OK\n");
#endif
	}

	//i2cDataToWrite[0] = i2cWiperAdressA;
	i2cDataToWrite[0] = 0x10;
	i2cDataToWrite[1] = 0x40;
	i2cDataToWrite[2] = 0x34;

	returnValue = HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, i2cDataToWrite, 2,
			1000);

	if (returnValue == HAL_SPI_STATE_ERROR) {
#ifdef DEBUG
		send_uart("I2C Write A - error\n");
#endif
	} else {
#ifdef DEBUG
		send_uart("I2C Write A - OK\n");
#endif
	}

	/*
	 i2cDataToWrite [0] = i2cWiperAdressB;
	 i2cDataToWrite [1] = 126;
	 i2cDataToWrite [2] = 126;

	 returnValue = HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, i2cDataToWrite, 2, 1000);


	 if(returnValue == HAL_SPI_STATE_ERROR)
	 send_uart("I2C Write B - error\n");
	 else
	 send_uart("I2C Write B - OK\n");
	 //returnValue = HAL_I2C_Master_Receive(&hi2c4, i2cDeviceAddress, i2cDataToRead, 3, 1000);
	 */

	// i2C reading back
	uint8_t dataToSend[2];
	dataToSend[0] = 0x10;
	dataToSend[1] = i2cDeviceAddress;

	i2cDataToRead[0] = 0;
	i2cDataToRead[1] = 0;
	i2cDataToRead[2] = 0;

	returnValue = HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, dataToSend, 2,
			1000);

	returnValue = HAL_I2C_Master_Receive(&hi2c4, i2cDeviceAddress, i2cDataToRead, 3,
			1000);

	if (returnValue == HAL_SPI_STATE_ERROR) {
#ifdef DEBUG
		send_uart("I2C Read - error\n");
#endif
	} else {
#ifdef DEBUG
		send_uart("I2C Read - OK\n");
#endif
	}

	sprintf(uartBufferTx, "%d - %d - %d\n", i2cDataToRead[0], i2cDataToRead[1],
			i2cDataToRead[2]);

#ifdef DEBUG
	send_uart(uartBufferTx);
#endif

}

// logging to SD card
void saveToSD (char *string){


	if(startOfMeasurement == 1){
	/*
		// getting time and date for formatting as name of new logged file
		HAL_RTC_GetTime(&hrtc, &Time, FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &Date, FORMAT_BCD);

		// file name format "20YYMMDD_HHMM" example "20200120_1022"
		sprintf(loggingFileName, "20%2d%2d%2d_%2d%2d.txt", Date.Year, Date.Month, Date.Date, Time.Hours,Time.Minutes);
		//uint8_t len = strlen(loggingFileName);
	*/
		/* Mount SD Card */
		fresult = f_mount(&fs, "", 0);
		if (fresult != FR_OK) {
			isMeasuring = 0;
			send_uart("!!! Sampling stopped !!!\n");

			//#ifdef DEBUG
				return send_uart("!!! ERROR: mounting SD CARD !!!\n");
			//#endif
		} else {
			#ifdef DEBUG
				send_uart("SD CARD mounted successfully...\n");
			#endif
		}


		startOfMeasurement == 0;

	}

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);


	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);

	#ifdef DEBUG
		bufclear();
		sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);
		send_uart(buffer);
	#endif

	if( free_space < strlen(string) ){
		isMeasuring = 0;
		send_uart("!!! Sampling stopped !!!\n");

		return send_uart("!!! ERROR: SD card capacity !!!\n");

	}


	/* Open file to write/ create a file if it doesn't exist */
	fresult = f_open(&fil, loggingFileName,	FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	/* Move to offset to the end of the file */
	fresult = f_lseek(&fil, &fil.fptr);

	/* write the string to the file */
	fresult = f_puts(string, &fil);

	/* Close file */
	f_close(&fil);

	//bufclear();




}


// function to set selected range ON/OFF
// ON --> selection = 1
// OFF --> selection = 0
void setRangeNA(uint8_t selection){

	if(selection){
		// nA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);
		// NA_TRANS port E pin 1 HIGH
		GPIOE->ODR |= (1<<1);
		// AS4 port E pin 3 LOW
		GPIOE->ODR &= ~(1<<3);

	}
	else{
		// nA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		// NA_TRANS port E pin 1 LOW
		GPIOE->ODR &= ~(1<<1);
		// AS4 port E pin 3 HIGH
		GPIOE->ODR |= (1<<3);

	}

}

void setRangeUA (uint8_t selection){

	if(selection){
		// uA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_RESET);
		// UA_TRANS port E pin 0 HIGH
		GPIOE->ODR |= (1<<0);
		// AS3 port E pin 5 LOW
		GPIOE->ODR &= ~(1<<5);

	}
	else{
		// uA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
		// UA_TRANS port E pin 0 LOW
		GPIOE->ODR &= ~(1<<0);
		// AS3 port E pin 5 HIGH
		GPIOE->ODR |= (1<<5);

	}
}

void setRangeMA (uint8_t selection){

	if(selection){
		// mA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_RESET);
		// MA_TRANS port B pin 9 HIGH
		GPIOB->ODR |= (1<<9);
		// AS2 port E pin 4 LOW
		GPIOE->ODR &= ~(1<<4);

	}
	else{
		// mA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		// MA_TRANS port B pin 9 LOW
		GPIOB->ODR &= ~(1<<9);
		// AS2 port E pin 4 HIGH
		GPIOE->ODR |= (1<<4);

	}
}


// simple range changing , no linear regression, based on measured value and hard limits
void change_range(float measuredValue) {

	// change from nA to uA
	if (currentRange == 0 & measuredValue >= RANGE_UPPER_LIMIT_NA) {
		currentRange = 1;

		setRangeNA(0);
		setRangeUA(1);
		setRangeMA(0);
		/*

		// mA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		// MA_TRANS port B pin 9 LOW
		GPIOB->ODR &= ~(1<<9);
		// AS2 port E pin 4 HIGH
		GPIOE->ODR |= (1<<4);

		// uA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_RESET);
		// UA_TRANS port E pin 0 HIGH
		GPIOE->ODR |= (1<<0);
		// AS3 port E pin 5 LOW
		GPIOE->ODR &= ~(1<<5);

		// nA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		// NA_TRANS port E pin 1 LOW
		GPIOE->ODR &= ~(1<<1);
		// AS4 port E pin 3 HIGH
		GPIOE->ODR |= (1<<3);

		*/

	}

	// change from uA to mA
	if (currentRange == 1 & measuredValue >= RANGE_UPPER_LIMIT_UA) {
	//else if (currentRange == 1 & measuredValue >= RANGE_UPPER_LIMIT_UA) {
		currentRange = 2;

		setRangeNA(0);
		setRangeUA(0);
		setRangeMA(1);
		/*

		// mA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_RESET);
		// MA_TRANS port B pin 9 HIGH
		GPIOB->ODR |= (1<<9);
		// AS2 port E pin 4 LOW
		GPIOE->ODR &= ~(1<<4);

		// uA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
		// UA_TRANS port E pin 0 LOW
		GPIOE->ODR &= ~(1<<0);
		// AS3 port E pin 5 HIGH
		GPIOE->ODR |= (1<<5);


		// nA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		// NA_TRANS port E pin 1 LOW
		GPIOE->ODR &= ~(1<<1);
		// AS4 port E pin 3 HIGH
		GPIOE->ODR |= (1<<3);

		*/


	}

	// change from mA to uA
	else if (currentRange == 2 & measuredValue <= RANGE_LOWER_LIMIT_MA) {
		currentRange = 1;

		setRangeNA(0);
		setRangeUA(1);
		setRangeMA(0);
		/*

		// mA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		// MA_TRANS port B pin 9 LOW
		GPIOB->ODR &= ~(1<<9);
		// AS2 port E pin 4 HIGH
		GPIOE->ODR |= (1<<4);

		// uA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_RESET);
		// UA_TRANS port E pin 0 HIGH
		GPIOE->ODR |= (1<<0);
		// AS3 port E pin 5 LOW
		GPIOE->ODR &= ~(1<<5);

		// nA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		// NA_TRANS port E pin 1 LOW
		GPIOE->ODR &= ~(1<<1);
		// AS4 port E pin 3 HIGH
		GPIOE->ODR |= (1<<3);

		*/

	}

	// change from uA to nA
	else if (currentRange == 1 & measuredValue <= RANGE_LOWER_LIMIT_UA) {
		currentRange = 0;

		setRangeNA(1);
		setRangeUA(0);
		setRangeMA(0);
		/*

		// mA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		// MA_TRANS port B pin 9 LOW
		GPIOB->ODR &= ~(1<<9);
		// AS2 port E pin 4 HIGH
		GPIOE->ODR |= (1<<4);

		// uA range OFF
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
		// UA_TRANS port E pin 0 LOW
		GPIOE->ODR &= ~(1<<0);
		// AS3 port E pin 5 HIGH
		GPIOE->ODR |= (1<<5);


		// nA range ON
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);
		// NA_TRANS port E pin 1 HIGH
		GPIOE->ODR |= (1<<1);
		// AS4 port E pin 3 LOW
		GPIOE->ODR &= ~(1<<3);

		*/

	}

	/*
	 else{
	 // default setting for ranges
	 // mA range OFF
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

	 // uA range OFF
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

	 // nA range ON
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);

	 }
	 */

}

// checking current value agains range limits and adjusting range selection
void check_range_linear(float measuredValue, float rangeChangeRatio) {

	//if (previousValues [SAMPLES - 1] < measuredValue && measuredValue < RANGE_LOWER_LIMIT_UA){}

	// change from nA to uA
	if (currentRange == 0 && (measuredValue >= RANGE_UPPER_LIMIT_NA && rangeChangeRatio >= RANGE_UPPER_CHANGE_RATIO_NA)|| measuredValue >= RANGE_UPPER_TOTAL_LIMIT_NA) {
		currentRange = 1;

		// mA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

		// uA range ON
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_SET);

		// nA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);

	}

	// change from uA to mA
	else if (currentRange == 1 && (measuredValue >= RANGE_UPPER_LIMIT_UA && rangeChangeRatio >= RANGE_UPPER_CHANGE_RATIO_UA)|| measuredValue >= RANGE_UPPER_TOTAL_LIMIT_UA) {
		currentRange = 2;

		// mA range ON
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_SET);

		// uA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

		// nA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);

	}

	// change from mA to uA
	else if (currentRange == 2 && (measuredValue <= RANGE_LOWER_LIMIT_MA && rangeChangeRatio <= RANGE_LOWER_CHANGE_RATIO_MA)|| measuredValue <= RANGE_LOWER_TOTAL_LIMIT_MA) {
		currentRange = 1;

		// mA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

		// uA range ON
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_SET);

		// nA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);

	}

	// change from uA to nA
	else if (currentRange == 1 && (measuredValue <= RANGE_LOWER_LIMIT_UA && rangeChangeRatio <= RANGE_LOWER_CHANGE_RATIO_UA)|| measuredValue <= RANGE_LOWER_TOTAL_LIMIT_UA) {
		currentRange = 0;

		// mA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

		// uA range OFF
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

		// nA range ON
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_SET);

	}
	/*
	 else{
	 // default setting for ranges
	 // mA range OFF
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

	 // uA range OFF
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

	 // nA range ON
	 HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);


	 }
	 */

}

void adc_write_data(uint8_t command, uint8_t regAdrr, uint8_t dataValueMS, uint8_t dataValueLS){

	isWaitingForData = 1;

	uint8_t bufferTx [4];
	uint8_t bufferRx [4];

	bufferTx [0] = command;				// write 8 bit command word
	bufferTx [1] = regAdrr;				// write 8 bit register address
	bufferTx [2] = dataValueMS;			// 16 bit register value --> 8 MSB bit into register
	bufferTx [3] = dataValueLS;			// 16 bit register value --> 8 LSB bit into register

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	microDelay(50);

	HAL_SPI_TransmitReceive_IT(&hspi1, bufferTx, bufferRx, 4);
	//HAL_SPI_TransmitReceive_DMA(&hspi1, bufferTx, bufferRx, 4);

	while (isWaitingForData > 0);

	microDelay(50);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	microDelay(50);

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	microDelay(50);

	HAL_SPI_TransmitReceive_IT(&hspi1, bufferTx, bufferRx, 4);
	//HAL_SPI_TransmitReceive_DMA(&hspi1, bufferTx, bufferRx, 4);

	while (isWaitingForData > 0);

	microDelay(50);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	microDelay(50);


}

// sequence to config ADC after reset or power up
uint8_t adc_config() {

	isWaitingForData = 1;

	// SETTING ADC PGA AND WORKING RANGE
	adc_write_data(ADC_WRITE, ADC_RANGE_SEL_REG, 0x0, ADC_RANGE_UNIDIR_125REF_INT_REF);
/*
	spiDataTx[0] = ADC_WRITE;									// write 8 bit command word
	spiDataTx[1] = ADC_RANGE_SEL_REG;							// write 8 bit register address
	spiDataTx[2] = 0;											// 16 bit register value --> 8 MSB bit into register
	spiDataTx[3] = ADC_RANGE_UNIDIR_125REF_INT_REF;				// 16 bit register value --> 8 LSB bit into register

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, spiDataTx, 4, 1000);
	HAL_Delay(5);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);
	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, spiDataRx, 4, 1000);
	HAL_Delay(5);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	//HAL_SPI_Receive(&hspi1, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_IT(&hspi1, spiDataTx, spiDataRx, 4);

	while (isWaitingForData > 0);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	//HAL_GPIO_TogglePin(SPI1_CS_GPIO_Port, SPI1_CS_GPIO_Port);
	//HAL_Delay(100);
	microDelay(5);

	// send dummy data and read output register
	spiDataTx[0] = 0x00;	// 0000 0000
	spiDataTx[1] = 0x00;	// 0000 0000
	spiDataTx[2] = 0x00;	// 0000 0000
	spiDataTx[3] = 0x00;	// 0000 0000

	isWaitingForData = 1;

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	//returnValue = HAL_SPI_Receive(&hspi1, spiDataRx, 4, 1000);
	//HAL_SPI_TransmitReceive(&hspi1, spiDataTx, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_IT(&hspi1, spiDataTx, spiDataRx, 4);

	while (isWaitingForData > 0);
	//HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(100);
/*
	microDelay(100);

	// to GPI LOW
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_TogglePin(SPI1_CS_GPIO_Port, SPI1_CS_GPIO_Port);

	//returnValue = HAL_SPI_Receive(&hspi1, spiDataRx, 4, 1000);
	returnValue = HAL_SPI_TransmitReceive(&hspi1, spiDataTx, spiDataRx, 4, 1000);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
*/

}

void adc_reset(){

	// to GPIO LOW
	HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_RESET);
	microDelay(5);
	// to GPIO HIGH
	HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_SET);

	HAL_Delay(25);

	send_uart3("ADC resetted\n");

}

void adc_compensateOffset(){

	double result = 0;
	uint16_t offsetSamples = 500;

	HAL_Delay(50);

	// stabilize components anf flush buffer
	for(uint8_t i = 0; i < offsetSamples/10; i++){
		adc_sample();
		//result += previousValues[SAMPLES-1];
		microDelay(100);
	}

	for(uint16_t i = 0; i < offsetSamples; i++){
		adc_sample();
		//result += measuredValue; //previousValues[SAMPLES-1];
		result += previousValues[(currentValuePosition + SAMPLES - 1)%SAMPLES];
		microDelay(50);
	}

	settings.lastOffsetValue = result / offsetSamples;

	sprintf(uartBufferTx, "ADC offset: %.12f\n", settings.lastOffsetValue);
	send_uart3(uartBufferTx);


}
void adc_sample_internal(){

	/*
	isAdcDone = 0;

	HAL_ADC_Start_DMA(&hadc1, &adcValue, 1);
	while(isAdcDone  == 0);
	*/
	//HAL_ADC_Start_IT (&hadc1);

	//isAdcDone = 0;
	//while(isAdcDone == 0);

	//adcValue = adcBuffer;

	measuredValue = adcValue;

	//measuredValue = (measuredValue * (0.09375));
	measuredValue = (measuredValue * 0.8056640625);
	//measuredValue = ( measuredValue * ( (ADC_REF_VALUE*ADC_PGA*2) / ADC_SCALE ) );

	// adding bidirectional offset value
	//measuredValue = measuredValue - (ADC_REF_VALUE * ADC_PGA*ADC_BIDIRECTIONAL);
	// shifting from mV to V value
	measuredValue /= 1000;
	// adding measured DC offset value
	//measuredValue -= offsetValue;
	// shift voltage to 0 - 5V
	measuredValue = measuredValue * 1.51;

/*
	// move all values to left to make space for new value
	for (uint8_t i = 0; (i + 1) < SAMPLES; i++) {
		previousValues[i] = previousValues[i + 1];
	}
*/
	// change range filling values
	//previousValues[SAMPLES - 1] = measuredValue;
	previousValues[currentValuePosition % SAMPLES] = measuredValue;
	currentValuePosition++;



}

// NEW ADC TI ADS8910 18bit, 1Msps, differencial
void adc_sample() {

	isWaitingForData = 1;
	uint32_t receivedValue = 0;
	isAdcDone = 0;

	// dummy data to clock data out of ADC = No operation command
	spiDataTx[0] = 0x00;	// 0000 1000
	spiDataTx[1] = 0x00;	// 0000 0000
	spiDataTx[2] = 0x00;	// 0000 0000
	//spiDataTx[3] = 0x00;	// 0000 0000

	// starting conversion
	//HAL_GPIO_WritePin(ADC_CONV_PORT, ADC_CONV_PIN, GPIO_PIN_SET);
	// CONV port B pin 5
	// SET HIGH
	GPIOB->ODR |= (1<<5);
	//microDelay(1);

	//HAL_GPIO_WritePin(ADC_CONV_PORT, ADC_CONV_PIN, GPIO_PIN_RESET);
	// CONV port B pin 5
	// SET LOW
	GPIOB->ODR &= ~(1<<5);

	// RVS port D pin 5 check for transition LOW-->HIGH
	//while( (GPIOD->IDR & 0x20) != 1);
	while(HAL_GPIO_ReadPin(ADC_RSV_PORT, ADC_RSV_PIN) != GPIO_PIN_SET);
	//while(isAdcDone == 0);

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOD->ODR &= ~(1<<6);

	//HAL_SPI_TransmitReceive(&hspi1, spiDataTx, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiDataTx, spiDataRx, 3);
	//HAL_SPI_TransmitReceive_IT(&hspi1, spiDataTx, spiDataRx, 4);
	//HAL_SPI_Receive_DMA(&hspi1, spiDataRx, 4);

	while (isWaitingForData > 0);
	//while (HAL_DMA_GetState(&hdma_spi1_rx) == HAL_DMA_STATE_RESET);

	// to CS HIGH
 	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
 	// CS port D pin 6
 	// SET HIGH
 	GPIOD->ODR |= (1<<6);

	//shifting received byte data into final value
 	// ADS8690  // ADS8910
	receivedValue = (spiDataRx[2] >> 6) | (spiDataRx[1] << 2)	| (spiDataRx[0] << 10);

	measuredValue = receivedValue;

	if(receivedValue <= 131071){
		measuredValue = receivedValue;
		measuredValue = (measuredValue * ADC_RESOLUTION);
		// adding bidirectional offset value
		//measuredValue = measuredValue - (ADC_REF_VALUE * ADC_PGA*ADC_BIDIRECTIONAL);
		// shifting from mV to V value
		measuredValue /= 1000;
		//measuredValue *=2.186;
		// adding measured DC offset value
		measuredValue -= settings.lastOffsetValue;
	}
	else{
		measuredValue = receivedValue - 0x1FFFF;

		measuredValue = measuredValue * ADC_RESOLUTION; //(-ADC_REF_VALUE + temp);
		measuredValue = measuredValue - ADC_REF_VALUE;
		// shifting from mV to V value
		measuredValue /= 1000;
		// adding measured DC offset value
		measuredValue -= settings.lastOffsetValue;

	}

/*
	// move all values to left to make space for new value
	for (uint8_t i = 0; (i + 1) < SAMPLES; i++) {
		previousValues[i] = previousValues[i + 1];
	}
*/

	// change range filling values
	//previousValues[SAMPLES - 1] = measuredValue;
	previousValues[currentValuePosition % SAMPLES] = measuredValue;
	previousValuesRange[currentValuePosition % SAMPLES]= currentRange;
	currentValuePosition++;


}


// OLD ADC TI ADS8691 18bit, 1Msps, with buffer, single ended
void adc_sample_ads8691() {

	isWaitingForData = 1;
	uint32_t receivedValue = 0;
	isAdcDone = 0;

	// dummy data to clock data out of ADC = No operation command
	spiDataTx[0] = 0x00;	// 0000 1000
	spiDataTx[1] = 0x00;	// 0000 0000
	spiDataTx[2] = 0x00;	// 0000 0000
	//spiDataTx[3] = 0x00;	// 0000 0000

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOD->ODR &= ~(1<<6);

	// to CS HIGH
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	// CS port D pin 6
	// SET HIGH
	GPIOD->ODR |= (1<<6);

	// RVS port D pin 5 check for transition LOW-->HIGH
	//while( (GPIOD->IDR & 0x20) != 1);
	while(HAL_GPIO_ReadPin(ADC_RSV_PORT, ADC_RSV_PIN) != GPIO_PIN_SET);
	//while(isAdcDone == 0);

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOD->ODR &= ~(1<<6);

	//HAL_SPI_TransmitReceive(&hspi1, spiDataTx, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiDataTx, spiDataRx, 3);
	//HAL_SPI_TransmitReceive_IT(&hspi1, spiDataTx, spiDataRx, 4);
	//HAL_SPI_Receive_DMA(&hspi1, spiDataRx, 4);

	while (isWaitingForData > 0);
	//while (HAL_DMA_GetState(&hdma_spi1_rx) == HAL_DMA_STATE_RESET);

	// to CS HIGH
 	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
 	// CS port D pin 6
 	// SET HIGH
 	GPIOD->ODR |= (1<<6);

	//shifting received byte data into final value
 	// ADS8690  // ADS8910
	receivedValue = (spiDataRx[2] >> 6) | (spiDataRx[1] << 2)	| (spiDataRx[0] << 10);

	measuredValue = receivedValue;
	//measuredValue = (measuredValue * (0.09375));
	measuredValue = (measuredValue * ADC_RESOLUTION);
	//measuredValue = ( measuredValue * ( (ADC_REF_VALUE*ADC_PGA*2) / ADC_SCALE ) );

	// adding bidirectional offset value
	measuredValue = measuredValue - (ADC_REF_VALUE * ADC_PGA*ADC_BIDIRECTIONAL);
	// shifting from mV to V value
	measuredValue /= 1000;
	// adding measured DC offset value
	measuredValue -= settings.lastOffsetValue;

/*
	// move all values to left to make space for new value
	for (uint8_t i = 0; (i + 1) < SAMPLES; i++) {
		previousValues[i] = previousValues[i + 1];
	}
*/

	// change range filling values
	//previousValues[SAMPLES - 1] = measuredValue;
	previousValues[currentValuePosition % SAMPLES] = measuredValue;
	previousValuesRange[currentValuePosition % SAMPLES]= currentRange;
	currentValuePosition++;


}

void ranges() {

	// dummy reading ADC
	//previousValues [SAMPLES-1] = HAL_ADC_GetValue(&hadc1);


	if(AUTORANGE_MODE == 1){
		// implements linear regression, harder to calculate

		// calculate linear regression

		sumX = 0;
		sumY = 0;
		sumX2 = 0;
		sumXY = 0;

		/* OLD function with shaking array
		for (uint8_t i = 0; i <= SAMPLES; i++) {
			sumX = sumX + dummyValues[i];
			sumX2 = sumX2 + dummyValues[i] * dummyValues[i];
			sumY = sumY + previousValues[i];
			sumXY = sumXY + previousValues[i] * previousValues[i];
		}
		*/

		for (uint8_t i = 0; i <= SAMPLES; i++) {
			sumX = sumX + dummyValues[i];
			sumX2 = sumX2 + dummyValues[i] * dummyValues[i];
			sumY = sumY + previousValues[(currentValuePosition + SAMPLES - i)%SAMPLES];
			sumXY = sumXY + previousValues[(currentValuePosition + SAMPLES - i)%SAMPLES] * previousValues[(currentValuePosition + SAMPLES - i)%SAMPLES];
		}

		// Calculating a and b

		b = (SAMPLES * sumXY - sumX * sumY) / (SAMPLES * sumX2 - sumX * sumX);
		a = (sumY - b * sumX) / SAMPLES;

		// checking range, linear regressions
		//check_range_linear(previousValues[SAMPLES - 1], b);
		check_range_linear(previousValues[(currentValuePosition + SAMPLES - 1)%SAMPLES], b);
	}
	else{

		if(rangeMode == 4){
			change_range((float)previousValues[ (currentValuePosition + SAMPLES - 1)%SAMPLES]);
		}
		else if(rangeMode == 0){
			// nA range only
			currentRange = 0;
			setRangeNA(1);
			setRangeUA(0);
			setRangeMA(0);
		}
		else if(rangeMode == 1){
			// uA range only
			currentRange = 1;
			setRangeNA(0);
			setRangeUA(1);
			setRangeMA(0);
		}
		else if(rangeMode == 2){
			// mA range only
			currentRange = 2;
			setRangeNA(0);
			setRangeUA(0);
			setRangeMA(1);
		}
		else{
			change_range((float)previousValues[ (currentValuePosition + SAMPLES - 1)%SAMPLES]);
		}



	}


	//change_range((float)previousValues[ (currentValuePosition + SAMPLES - 1)%SAMPLES]);


}

// convert measured voltage and its range to final current value
double previousValueToCurrent (uint8_t index){

	// nA
	if (previousValuesRange[(currentValuePosition + SAMPLES - index)%SAMPLES] == 0){
		return (previousValues[(currentValuePosition + SAMPLES - index)%SAMPLES] / 1000000);
	}
	// uA
	else if (previousValuesRange[(currentValuePosition + SAMPLES - index)%SAMPLES] == 1){
		return (previousValues[(currentValuePosition + SAMPLES - index)%SAMPLES] / 1000);
	}
	// mA
	else if (previousValuesRange[(currentValuePosition + SAMPLES - index)%SAMPLES] == 2){
		return (previousValues[(currentValuePosition + SAMPLES - index)%SAMPLES]);
	}
	else{
		// leaving as is indicating error during operation
		return (previousValues[(currentValuePosition + SAMPLES - index)%SAMPLES]);
	}

	return (previousValues[(currentValuePosition + SAMPLES - index)%SAMPLES]);


}

double averaging (){

	double resultValue = 0;

	if(settings.isAveraging == 1){

		if(settings.samplingInterval >= 100){

			for (uint8_t i = 1;  i < 4; i++)
				//resultValue += previousValues[SAMPLES - i];
				//resultValue += previousValues[(currentValuePosition + SAMPLES - i)%SAMPLES];
				resultValue = previousValueToCurrent((uint8_t)i);

			resultValue /= 3;

		}
		if(settings.samplingInterval >= 200){

			for (uint8_t i = 1;  i < 7; i++)
				//resultValue += previousValues[SAMPLES - i];
				//resultValue += previousValues[(currentValuePosition + SAMPLES - i)%SAMPLES];
				resultValue = previousValueToCurrent((uint8_t)i);

			resultValue /= 6;

		}



	}
	else{

		//resultValue = previousValues[SAMPLES-1];
		//resultValue = previousValues[(currentValuePosition + SAMPLES - 1)%SAMPLES];
		resultValue = previousValueToCurrent((uint8_t)1);
	}


	return resultValue;



}



// program will check every measuring period if sample value is greater or equal to trigger level
// and also if trigger menu is activated plus if it was activated before --> if all check in it then starts measurement
// TODO - possibly implement HW trigger (ADC8691) allows settings High and Low + hysteresis values and send interrupt when exceeding values
void triggerMenu(){

	//if(previousValues[SAMPLES-1] >= triggerLevel && isTriggered == 0 && isTriggerActive == 1){
	if(averaging() >= settings.triggerLevel && isTriggered == 0 && settings.isTriggerActive == 1){

		isTriggered = 1;
		isMeasuring = 1;

	}


}

// convert 1 Byte decimal value to 1 byte hex value
uint8_t decimalToHex(uint8_t decimal){

	uint8_t highValue = 0;
	uint8_t lowValue = 0;
	uint8_t result = 0;

	lowValue = decimal%16;

	while(decimal >=16){
		highValue++;
		decimal -=16;
	}

	result = lowValue | highValue << 4;

	return result;


}
// convert 1 Byte hex value to 1 Byte decimal value
uint8_t hexToDecimal (uint8_t hex){

	uint8_t highValue = hex >> 4;
	uint8_t lowValue = hex<<4;
	lowValue = lowValue >> 4;
	uint8_t result = 0;

	result = highValue * 16 + lowValue;

	return result;



}


// compares expected command with received command word
// if they match then return 1, otherwise 0
/* OLD function
uint8_t compareInputCommand( uint8_t *commandWord [], uint8_t *inputWord[]){

	uint8_t commandLen = strlen(commandWord);
	uint8_t inputLen = strlen(inputWord);

	if(commandLen == inputLen){
		if(commandWord[0] == inputWord[0] && commandWord[1] == inputWord[1] && commandWord[2] == inputWord[2])
			return atoi(inputWord);
			//return 1;
	}
	else{
		send_uart("ERR-Command length mismatch\n");
		return 999;
	}

	return 999;


}*/

uint16_t convertInputToInt (uint8_t expectedLength, uint8_t *inputWord){

	uint8_t inputLen = strlen(inputWord);

	if (expectedLength + 2 >= inputLen) {
		return atoi(inputWord);
	} else {
		//send_uart("\nERR-Command length mismatch\n");
		return 999;
	}

	return 999;



}



// return command numeric value, otherwise it returns 999 as error
/* OLD functino
uint16_t convertInputToInt (uint8_t expectedLength, uint8_t *inputWord){

	uint8_t inputLen = strlen(inputWord);

	if (expectedLength == inputLen) {
		return atoi(inputWord);
	} else {
		send_uart("ERR-Command length mismatch\n");
		return 999;
	}

	return 999;



}*/

void flushUart(){

	uint8_t dummy = 0;

	while(UART_RX_AVAILABLE_BYTE() > 0)
		UART_RX_Read(&dummy, 1);

}

void printSettingsValues(){

	//send_uart("\nSettings\n");
	uint8_t txBuffer [1024];
	sprintf(txBuffer, "\nSettings\n Logging to console: %d\n Logging to SD card: %d\n Logging to USB flash drive: %d\n Logging to Ethernet: %d\n Measuring interval [us]: %d\n Sampling interval [us]: %d\n Averaging function enabled: %d\n Trigger function eneabled: %d\n"
			" Trigger level [A]: %1.12f\n Offset value [mV]: %1.12f\n Power source - Voltage [mV]: %d\n Power source - Current limit [mA]: %d\n Power source enabled: %d\n Power source enable mode: %d\n",
			settings.isLoggingToConsole,
			settings.isLoggingToSD,
			settings.isLoggingToUSB,
			settings.isLoggingToEthernet,
			settings.measuringInterval,
			settings.samplingInterval,
			settings.isAveraging,
			settings.isTriggerActive,
			settings.triggerLevel,
			settings.lastOffsetValue,
			settings.powerSourceVoltage,
			settings.powerSourceCurrent,
			settings.powerSourceEnable,
			settings.powerSourceEnableMode);
	send_uart3(txBuffer);



}


void printLastMeasuredValues(){

	uint8_t txBuffer [100];

	send_uart3("\nLast measured values\n");
	sprintf(txBuffer, "Buffer contains: %d samples\n", SAMPLES);
	send_uart3(txBuffer);

	for(uint8_t i = SAMPLES; i > 0; i--){
		sprintf(txBuffer,"Measured value %3d [A]: %1.12f\n", (i+SAMPLES-1%SAMPLES), previousValueToCurrent(i));
		send_uart3(txBuffer);


	}
}

void printCurrentTimeDate(){

	uint8_t txBuffer [250];

	/* Get the RTC current Time and Date */
	 HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);

	/* Display time Format: hh:mm:ss */
	/* Display date Format: dd-mm-yy */
	sprintf(txBuffer,"Time [hh:mm:ss] and Date [dd-mm-yy]\n %02d:%02d:%02d_%02d-%02d-%2d\n",Time.Hours, Time.Minutes, Time.Seconds, Date.Date, Date.Month, 2000 + Date.Year);

	send_uart3(txBuffer);

}


// function to get requested number of bytes/char from UART with timeout capabilities
/* OLD function
uint8_t getConsoleInput(uint8_t *buffer [], uint8_t commandLength, uint32_t timeoutValue){

	// set flags
	isWaitingForData = 1;
	uint32_t consoleInputTime = 0;
	// reset timer
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	// null buffer
	for(uint8_t i = 0; i < 10; i++)
		inputData[i] = 0;

	//HAL_UART_Receive_DMA(&huart6, buffer, commandLength);
	HAL_UART_Receive_IT(&huart6, inputData, commandLength);
	while(isWaitingForData == 1 && __HAL_TIM_GET_COUNTER(&htim14) < timeoutValue );
	consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

	// timeout report
	if(isWaitingForData == 1 || consoleInputTime > timeoutValue){
		return 99;
	}

	return 0;



}*/

// function to read defined number of chars from uart ring buffer
// if timeout value will be 0 then it will act as blocking function waiting for atleast 1 char otherwise it will wait for defined number of chars or timeout
uint8_t getConsoleInput(uint8_t *buffer, uint8_t commandLength, uint32_t timeoutValue){

	// set flags
	//isWaitingForData = 1;
	uint32_t consoleInputTime = 0;
	// reset timer
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	// null buffer
	for(uint8_t i = 0; i < 10; i++)
		buffer[i] = 0;
	// blocking and timeout
	if(timeoutValue > 1){

		while (UART_RX_AVAILABLE_BYTE() < 1	&& __HAL_TIM_GET_COUNTER(&htim14) < timeoutValue);
		uint8_t byte = UART_RX_AVAILABLE_BYTE();
		//while(UART_RX_AVAILABLE_BYTE() < commandLength+1);
		consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

		UART_RX_Read(buffer, commandLength + 2);
		//send_uart(dataToRead);
		flushUart();
		//dataToRead = UART_RX_AVAILABLE_BYTE();
		//send_uart(dataToRead);
		//HAL_UART_Receive_DMA(&huart6, buffer, commandLength);
		//HAL_UART_Receive_IT(&huart6, inputData, commandLength);
		//while(isWaitingForData == 1 && __HAL_TIM_GET_COUNTER(&htim14) < timeoutValue );

		//consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

		// timeout report
		if (UART_RX_AVAILABLE_BYTE() < 1 || consoleInputTime > timeoutValue) {
			return 99;
		}

		return 0;

		// blocking, no timeout
	} else if(timeoutValue == 1) {
		//flushUart();
		/*
		 while(UART_RX_AVAILABLE_BYTE() < 1  && __HAL_TIM_GET_COUNTER(&htim14) < timeoutValue);
		 uint8_t byte = UART_RX_AVAILABLE_BYTE();
		 //while(UART_RX_AVAILABLE_BYTE() < commandLength+1);
		 consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

		 UART_RX_Read(buffer, commandLength+2);
		 //send_uart(dataToRead);
		 flushUart();
		 //dataToRead = UART_RX_AVAILABLE_BYTE();
		 //send_uart(dataToRead);
		 //HAL_UART_Receive_DMA(&huart6, buffer, commandLength);
		 //HAL_UART_Receive_IT(&huart6, inputData, commandLength);
		 //while(isWaitingForData == 1 && __HAL_TIM_GET_COUNTER(&htim14) < timeoutValue );

		 //consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

		 // timeout report
		 if(UART_RX_AVAILABLE_BYTE() < 1 || consoleInputTime > timeoutValue){
		 return 99;
		 }

		 return 0;
		 */
		while(UART_RX_AVAILABLE_BYTE() < 1);
		uint8_t byte = UART_RX_AVAILABLE_BYTE();
		//while(UART_RX_AVAILABLE_BYTE() < commandLength+1);
		consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);
		UART_RX_Read(buffer, commandLength + 2);
		//send_uart(dataToRead);
		flushUart();

		return 0;



	}
	// non blocking, no timeout
	else{
		if (UART_RX_AVAILABLE_BYTE() > 0) {
			uint8_t byte = UART_RX_AVAILABLE_BYTE();
			//while(UART_RX_AVAILABLE_BYTE() < commandLength+1);
			consoleInputTime = __HAL_TIM_GET_COUNTER(&htim14);

			UART_RX_Read(buffer, commandLength + 2);
			//send_uart(dataToRead);
			flushUart();

		} else
			return 99;

		return 0;

	}

	return 0;


}

/*
*	Measuring settings submenu structure - basic submenu structure will be printed when first accessed
*		[201] *Show measuring settings submenu
*		[202] *Set measuring period
*		[203] *Set sampling period
*		[204] *Enabling averaging
*		[205] *Enabling trigger menu
*		[206] *Set trigger threshold
*		[888] *Go to main menu
*		[999] *Go to main menu
*/

void consoleMeasuringInterface(){

	uint16_t temp16 = 0;

	// print menu structure
	send_uart3("\nMeasuring settings menu\n[201] Show measuring settings menu\n[202] Set measuring period\n[203] Set sampling period\n[204] Enable averaging function\n[205]Enable trigger function\n[206] Set trigger threshold\n"
			"[888] Go to Main menu\n[999] Go to Main menu\n");

	// get input command
	uint8_t stateFlag = getConsoleInput(inputData, 3, 1);

	// timeout or no data handler
	if (stateFlag == 99) {
		//send_uart("Timeout");
		//microDelay(50);
		return;
	}

	switch (convertInputToInt(3, inputData)) {
		// show measuring settings menu
		case 201:
			return consoleMeasuringInterface();
			break;
		// set measuring period
		case 202:
			temp16 = 0;
			send_uart3("\nEnter desired measuring interval in us (anywhere between 1us - 1000000us/1sec) or enter 0 to exit without change\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 1 & temp16 <= 1000000){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"New entered value is: %d us\n", temp16);
				send_uart3(tempBuf);
				settings.measuringInterval = temp16;
			}
			else if(temp16 == 0){
				send_uart3("\nMeasuring period: UNCHANGED\n");
			}
			else{
				send_uart3("\nEntered value if out of range\n");
			}
			return consoleMeasuringInterface();
			break;
		// set sampling period
		case 203:
			temp16 = 0;
			send_uart3("\nEnter desired sampling interval in us (anywhere between 1us - 10000000us/10sec) or enter 0 to exit without change\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 1 & temp16 <= 10000000){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"New entered value is: %d us\n", temp16);
				send_uart3(tempBuf);
				settings.samplingInterval = temp16;
			}
			else if(temp16 == 0){
				send_uart3("\nSampling period: UNCHANGED\n");
			}
			else{
				send_uart3("\nEntered value if out of range\n");
			}
			return consoleMeasuringInterface();
			break;
		// enable averaging
		case 204:
			send_uart3("\nDo you want to enable averaging function\n 1 - YES, ENABLE\n 2 - NO, DISABLE\n 0 - EXIT without change\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.isAveraging = 1;
				send_uart3("\nAVERAGING FUNCTION: ENABLED\n");
				break;
			case 2:
				settings.isAveraging = 0;
				send_uart3("\nAVERAGING FUNCTION: DISABLED\n");
				break;
			default:
				send_uart3("\nAVERAGING FUNCTION: NO CHANGE\n");
				break;
			}
			return consoleMeasuringInterface();
			break;
		// enable trigger
		case 205:
			send_uart3("\nDo you want to enable trigger function\n 1 - YES, ENABLE\n 2 - NO, DISABLE\n 0 - EXIT without change\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.isTriggerActive = 1;
				send_uart3("\nTRIGGER FUNCTION: ENABLED\n");
				break;
			case 2:
				settings.isTriggerActive = 0;
				send_uart3("\nTRIGGER: DISABLED\n");
				break;
			default:
				send_uart3("\nTRIGGER: NO CHANGE\n");
				break;
			}
			return consoleMeasuringInterface();
			break;
		// set trigger level
		case 206:
			temp16 = 0;
			send_uart3("\nEnter desired trigger menu threashold in multiples of nA (anywhere between 5nA - 4A) or enter 0 to exit without change\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 5 & temp16 <= 4000000){
				uint8_t tempBuf [50];
				settings.samplingInterval = temp16;
				settings.samplingInterval /= 1000000000;
				sprintf(tempBuf,"New entered value is: %1.12f A\n", settings.samplingInterval);
				send_uart3(tempBuf);

			}
			else if(temp16 == 0){
				send_uart3("\nSampling period: UNCHANGED\n");
			}
			else{
				send_uart3("\nEntered value if out of range\n");
			}
			return consoleMeasuringInterface();
			break;

			// main menu + print main menu structure
		case 888:
			send_uart3("\nGoing back to Main menu\n");
			return consoleInterface(123);
			break;
		// main menu + print main menu structure
		case 999:
			send_uart3("\nGoing back to Main menu\n");
			return consoleInterface(123);
			break;

		default:
			send_uart3("\nUNDEFINED SELECTION\n");
			return consoleMeasuringInterface();
			break;
	}

	return consoleMeasuringInterface();

}

/*
*	RTC settings submenu structure
*		[302] *Show RTC settings submenu
*		[321] *Show current time and date
*		[322] *Set Time - Hours, Minutes
*		[323] *Set Date - Day
*		[324] *Set Date - Month
*		[325] *Set Date - Year
*		[888] *Go to Device settings menu
*		[999] *Go to main menu
*/

// menu for setting RTC clock
void consoleRTCSettings(){

	uint16_t temp16 = 0;
	// print menu structure
	send_uart3("\nRTC settings menu\n[302] Show RTC settings menu\n[321] Show current time and date\n[322] Set Time - Hours, Minutes\n[323] Set Date - Day\n[324] Set Date - Month\n[325] Set Date - Year\n"
			"[888] Go to Device settings menu\n[999] Go to Main menu\n");

	/* Get the RTC current Time and Date */
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);

	// get input command
	uint8_t stateFlag = getConsoleInput(inputData, 3, 1);

	// timeout or no data handler
	if (stateFlag == 99) {
		//send_uart("Timeout");
		//microDelay(50);
		return;
	}

	switch (convertInputToInt(3, inputData)) {
		// show rtc settings menu
		case 302:
			return consoleRTCSettings();
			break;
		// show current time and date
		case 321:
			printCurrentTimeDate();
			return consoleRTCSettings();
			break;
		// set time - hours, minutes
		case 322:
			temp16 = 0;
			send_uart3("\nEnter time, first hours in 24h format then when prompted minutes and finally seconds\n\n ENTER hours\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 0 & temp16 <= 24){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"  Entered time - hours: %d\n", temp16);
				send_uart3(tempBuf);
				Time.Hours = temp16;

				temp16 = 0;
				send_uart3("\n ENTER minutes\n");
				getConsoleInput(&inputData, 10, 1);
				temp16 = convertInputToInt(10, &inputData);
				if(temp16 >= 0 & temp16 <= 60){
					sprintf(tempBuf,"  Entered time - minutes: %d\n", temp16);
					send_uart3(tempBuf);
					Time.Minutes = temp16;

					temp16 = 0;
					send_uart3("\n ENTER seconds\n");
					getConsoleInput(&inputData, 10, 1);
					temp16 = convertInputToInt(10, &inputData);
					if(temp16 >= 0 & temp16 <= 60){
						sprintf(tempBuf,"  Entered time - seconds: %d\n", temp16);
						send_uart3(tempBuf);
						Time.Seconds = temp16;

						Time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
						Time.StoreOperation = RTC_STOREOPERATION_RESET;
						//if (HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BCD) != HAL_OK)
						if (HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN) != HAL_OK)
						{
							send_uart3("ERROR - SAVING TIME\n");
						}
						else
							HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					}
				}
			}
			else{
				send_uart3("\nEntered value if out of range\n Time not set");
			}
			return consoleRTCSettings();
			break;
		// set date - day
		case 323:
			temp16 = 0;
			send_uart3("\nEnter date - day in format from 1 to 31\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 1 & temp16 <= 31){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"  Entered day: %d\n", temp16);
				send_uart3(tempBuf);
				Date.Date = decimalToHex((uint8_t)temp16);
				//Date.Date = temp16;
				if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
				{
					send_uart3("ERROR - SAVING DATE\n");
				}
				else
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
			}
			else{
				send_uart3("\nEntered value if out of range\n Date not set");
			}
			return consoleRTCSettings();
			break;
		// set date - month
		case 324:
			send_uart3("\nEnter number corresponding to month(1 - January, 2 - February, ...)\n");
			getConsoleInput(&inputData, 10, 1);
			switch (convertInputToInt(10, &inputData)) {
				case 1:
					Date.Month = RTC_MONTH_JANUARY;
					send_uart3("Entered month: January / 1\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 2:
					Date.Month = RTC_MONTH_FEBRUARY;
					send_uart3("Entered month: February / 2\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 3:
					Date.Month = RTC_MONTH_MARCH;
					send_uart3("Entered month: March / 3\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 4:
					Date.Month = RTC_MONTH_APRIL;
					send_uart3("Entered month: April / 4\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 5:
					Date.Month = RTC_MONTH_MAY;
					send_uart3("Entered month: May / 5\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 6:
					Date.Month = RTC_MONTH_JUNE;
					send_uart3("Entered month: June / 6\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 7:
					Date.Month = RTC_MONTH_JULY;
					send_uart3("Entered month: July / 7\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 8:
					Date.Month = RTC_MONTH_AUGUST;
					send_uart3("Entered month: August / 8\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 9:
					Date.Month = RTC_MONTH_SEPTEMBER;
					send_uart3("Entered month: September / 9\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 10:
					Date.Month = RTC_MONTH_OCTOBER;
					send_uart3("Entered month: October / 10\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 11:
					Date.Month = RTC_MONTH_NOVEMBER;
					send_uart3("Entered month: November / 11\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				case 12:
					Date.Month = RTC_MONTH_DECEMBER;
					send_uart3("Entered month: December / 12\n");
					if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
					{
						send_uart3("ERROR - SAVING DATE\n");
					}
					else
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
					break;
				default:
					send_uart3("\nEntered value if out of range\n Date not set");
					break;
			}
			return consoleRTCSettings();
			break;
		// set date - year
		case 325:
			temp16 = 0;
			send_uart3("\nEnter date - year in format of last 2 digits (ex. 2020 means entering 20)\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 1 & temp16 <= 31){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"  Entered year: %d\n", 2000 + temp16);
				send_uart3(tempBuf);
				Date.Year = decimalToHex((uint8_t)temp16);
				if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
				{
					send_uart3("ERROR - SAVING DATE\n");
				}
				else
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
			}
			else{
				send_uart3("\nEntered value if out of range\n Date not set");
			}
			return consoleRTCSettings();
			break;

		// device settings menu
		case 888:
			send_uart3("\nGoing back to Device settings menu\n");
			return consoleDeviceInterface();
			break;
		// main menu
		case 999:
			send_uart3("\nGoing back to Main menu\n");
			return consoleInterface(123);
			break;

		default:
			send_uart3("\nUNDEFINED SELECTION\n");
			return consoleRTCSettings();
			break;
	}


	return consoleRTCSettings();


}



/*
*	Device settings submenu structure
*		[301] *Show device settings submenu
*		[302] *Setting RTC (Time and Date)
*		[303] *Choosing storing method (UART x SD card x USB flash) // NOT DECIDED WHICH
*		[304] *Check SD card and print info about card
*		[305] *Check USB flash drive and print info about drive // NOT SURE YET
*		[306] *Perform offset calibration
*		[307] *Perform zero offset value
*		[308] *Do device selfcheck test // NOT DECIDED YET
*		[309] *Adjust voltage for power source
*		[310] *Adjust current limit for power source
*		[311] *Enable power source
*		[312] *Behaviour mode of power source
*		[888] *Go to main menu
*		[999] *Go to main menu
*/

// subfunction to hanhle console inputs for device settings menu
void consoleDeviceInterface(){

	uint16_t temp16 = 0;

	// print menu structure
	send_uart3("\nDevice settings menu\n[301] Show device settings menu\n[302] RTC settings\n[303] Choosing storing method\n[304] Check SD card\n[305] Check USB Flash drive\n[306] Perform offset calibration\n[307] Perform zero offset calibration\n"
			"[308] Perform device selfcheck\n[309] Adjust power source - voltage\n[310] Adjust power source - current limit\n[311] Enable power source\n[312] Set power source enabling mode\n[888] Go to Main menu\n[999] Go to Main menu\n");


	// get input command
	uint8_t stateFlag = getConsoleInput(inputData, 3, 1);

	// timeout or no data handler
	if(stateFlag == 99){
		return;
	}

	switch(convertInputToInt(3, inputData)){
		// show device settings menu
		case 301:
			send_uart3("\nDevice settings menu\n[301] Show device settings menu\n[302] RTC settings\n[303] Choosing storing method\n[304] Check SD card\n[305] Check USB Flash drive\n[306] Perform offset calibration\n[307] Perform zero offset calibration\n"
						"[308] Perform device selfcheck\n[309] Adjust power source - voltage\n[310] Adjust power source - current limit\n[311] Enable power source\n[312] Set power source enabling mode\n[888] Go to Main menu\n[999] Go to Main menu\n");
			return consoleDeviceInterface();
			break;
		// go to RTC settings submenu
		case 302:
			return consoleRTCSettings();
			break;
		// choose storing method
		case 303:
			send_uart3("\nChoose storing method\n\n 1 - COM port\n 2 - SD Card\n 3 - USB Flash Drive\n 0 - Exit, no change\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.isLoggingToConsole = 1;
				settings.isLoggingToSD = 0;
				settings.isLoggingToUSB = 0;
				send_uart3("\nStoring method: COM port\n");
				break;
			case 2:
				settings.isLoggingToConsole = 0;
				settings.isLoggingToSD = 1;
				settings.isLoggingToUSB = 0;
				send_uart3("\nStoring method: SD Card\n");
				break;
			case 3:
				settings.isLoggingToConsole = 0;
				settings.isLoggingToSD = 0;
				settings.isLoggingToUSB = 1;
				//isLoggingToConsole = 1;
				//isLoggingToSD = 1;
				send_uart3("\nStoring method: USB Flash Drive\n");
				break;
			case 0:
				send_uart3("\nStoring method: UNCHANGED\n");
				break;
			default:
				send_uart3("\nStoring method: UNDEFINED\n");
				break;
			}
			return consoleDeviceInterface();
			break;
		// check SD card
		case 304:
			send_uart3("\nChecking SD card\n");

			return consoleDeviceInterface();
			break;
		// check USB flash drive
		case 305:
			send_uart3("\nChecking USB flash drive\n");

			return consoleDeviceInterface();
			break;
		// perform DC offset calibration
		case 306:
			send_uart3("\nDo you really want to perform offset calibration (You will lose previous DC offset calibration value)\n 1 - YES, PERFORM CALIBRATION\n 2 - NO, I DONT WANT TO\n"
					" !! BEFORE YOU ANSWER !!\n UNPLUG all measured devices from device (inc. measuring plugs) and then start procedure)\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				send_uart3("\nOffset calibration - started\n");
				adc_compensateOffset();
				send_uart3("\nOffset calibration - finished\n");
				writeEEPROM_AT24();
				microDelay(5);
				readEEPROM_AT24();
				printSettingsValues();
				break;
			case 2:
				send_uart3("\nOffset value: UNCHANGED\n");
				break;
			default:
				send_uart3("\nOffset value: UNCHANGED\n");
				break;
			}

			return consoleDeviceInterface();
			break;
		// zero DC offset value / disable calibration
		case 307:
			send_uart3("\nDo you really want to zero offset calibration value (You will lose DC offset calibration)\n 1 - YES, ZERO DC OFFSET\n 2 - NO, I DONT WANT TO\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.lastOffsetValue = 0.0;
				writeEEPROM_AT24();
				microDelay(5);
				readEEPROM_AT24();
				printSettingsValues();
				send_uart3("\nDC offset value: ZEROED\n");
				break;
			case 2:
				send_uart3("\nDC offset value: UNCHANGED\n");
				break;
			default:
				send_uart3("\nDC offset value: UNCHANGED\n");
				break;
			}
			break;
		// perform device selfcheck
		case 308:
			send_uart3("\nDevice selfcheck - started\n");

			send_uart3("\nDevice selfcheck - finished\n");
			return consoleDeviceInterface();
			break;
		// adjust power source voltage
		case 309:
			temp16 = settings.powerSourceVoltage;
			send_uart3("\nEnter desired power source voltage in mV (anywhere between 500 mV - 5500 mV) or enter 0 to exit without change\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 500 & temp16 <= 5500){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"New entered value is: %d mV\n", temp16);
				send_uart3(tempBuf);
				settings.powerSourceVoltage = temp16;
				setPotenciomenters();

			}
			else if(temp16 == 0){
				send_uart3("\nPower source - Voltage: UNCHANGED\n");
			}
			else{
				send_uart3("\nEntered value if out of range\n");
			}
			return consoleDeviceInterface();
			break;
		// adjust power source current limit
		case 310:
			temp16 = settings.powerSourceCurrent;
			send_uart3("\nEnter desired power source current limit in mA (anywhere between 10 mA - 500 mA) or enter 0 to exit without change\n");
			getConsoleInput(&inputData, 10, 1);
			temp16 = convertInputToInt(10, &inputData);
			if(temp16 >= 10 & temp16 <= 500){
				uint8_t tempBuf [50];
				sprintf(tempBuf,"New entered value is: %d mA\n", temp16);
				send_uart3(tempBuf);
				settings.powerSourceCurrent = temp16;
				setPotenciomenters();

			}
			else if(temp16 == 0){
				send_uart3("\nPower source - Current: UNCHANGED\n");
			}
			else{
				send_uart3("\nEntered value if out of range\n");
			}
			return consoleDeviceInterface();
			break;
		// enable power source
		case 311:
			send_uart3("\nDo you want to enable power source\n 1 - YES, ENABLE\n 2 - NO, DISABLE\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.powerSourceEnable = 1;
				HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);
				send_uart3("\nPower source: ENABLED\n");
				break;
			case 2:
				settings.powerSourceEnable = 0;
				send_uart3("\nPower source: DISABLED\n");
				HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, RESET);
				break;
			default:
				send_uart3("\nPower source: NO CHANGE\n");
				break;
			}
			return consoleDeviceInterface();
			break;
		// choose power source enable mode
		case 312:
			send_uart3("\nChoose behaviour of power source\n 1 - Enable when starting measuring\n 2 - Enable when starting device\n 3 - Leave to user (manually enable/disable)\n 4 - Exit without change\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				settings.powerSourceEnableMode = 1;
				send_uart3("\nPower source enable mode: Enable when starting measuring\n");
				break;
			case 2:
				settings.powerSourceEnableMode = 2;
				send_uart3("\nPower source enable mode: Enable when starting device\n");
				HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);
				break;
			case 3:
				settings.powerSourceEnableMode = 0;
				send_uart3("\nPower source enable mode: Enable by user\n");
				break;
			default:
				send_uart3("\nPower source enable mode: NO CHANGE\n");
				break;

			}
			return consoleDeviceInterface();
			break;

		// main menu + print main menu structure
		case 888:
			send_uart3("\nGoing back to Main menu\n");
			return consoleInterface(123);
			break;

		default:
			send_uart3("\nUNDEFINED SELECTION\n");
			return consoleDeviceInterface();
			break;
	}

	return consoleDeviceInterface();


}


/*
 *	Interface for control command receive/transmit including settings various parameters such as sampling interval, storing method, etc
 *
 *	command structure is Host will send 3 Bytes/Chars in format of 3 numbers which represent menu item, slave will then reply with request for entering user input
 *
 *	Default menu will be send to PC when connected to serial or on request by sending "101" command
 *	When Host sends request for start measuring through console device will start measuring and stop responding to any console request because of insufficient computing power
 *	only way to stop measuring will be through HW button on device
 *	To go from submenu to main menu send command "999" or send command ""888" to  go one level higher in menu structure
 *
 *	Default menu structure
 *		[101] *Show default menu
 *		[102] *Show current settings
 *		[103] *Show last measured values
 *		[104] *Read EEPROM
 *		[105] *Write EEPROM
 *		[106] *Initializi EEPROM
 *		[111] *Start measuring with currently applied and saved settings
 *		[201] *Go to measuring settings submenu
 *		[301] *Go to device settings submenu
 *
 *	Measuring settings submenu structure - basic submenu structure will be printed when first accessed
 *		[201] *Show measuring settings submenu
 *		[202] *Set measuring period
 *		[203] *Set sampling period
 *		[204] *Enabling averaging
 *		[205] *Enabling trigger menu
 *		[206] *Set trigger threshold
 *		[888] *Go to main menu
 *		[999] *Go to main menu
 *
 *	Device settings submenu structure
 *		[301] *Show device settings submenu
 *		[302] *Setting RTC (Time and Date)
 *		[303] *Choosing storing method (UART x SD card x USB flash) // NOT DECIDED WHICH
 *		[304] *Check SD card and print info about card
 *		[305] *Check USB flash drive and print info about drive // NOT SURE YET
 *		[306] *Perform offset calibration
 *		[307] *Perform zero offset value
 *		[308] *Do device selfcheck test // NOT DECIDED YET
 *		[309] *Adjust voltage for power source
 *		[310] *Adjust current limit for power source
 *		[311] *Enable power source
 *		[312] *Behaviour mode of power source
 *		[888] *Go to main menu
 *		[999] *Go to main menu
 *
 *	RTC settings submenu structure
 *		[302] *Show RTC settings submenu
 *		[321] *Show current time and date
 *		[322] *Set Time - Hours, Minutes
 *		[323] *Set Date - Day
 *		[324] *Set Date - Month
 *		[325] *Set Date - Year
 *		[888] *Go to Device settings menu
 *		[999] *Go to main menu
 *
 *
 */
void consoleInterface(uint8_t flag){

	uint8_t stateFlag = 0;

	if(flag == 0){
		/*
		isWaitingForData = 1;

		for(uint8_t i = 0; i < sizeof(inputData); i++)
			inputData[i] = 0;

		//HAL_UART_Receive_DMA(&huart6, inputData, 3);
		HAL_UART_Receive_IT(&huart6, inputData, 3);
		while(isWaitingForData == 1);
		*/
		//send_uart("flag0");
		//microDelay(50);
		microDelay(1);

	}
	else if(flag == 11){
		send_uart3("Input timeout");
		return;
	}
	else if(123){
		send_uart3("[101] Show default menu\n[102] Show current settings\n[103] Show last measured values\n[104] Read EEPROM\n[105] Write EEPROM\n[106] Initialize EEPROM\n"
				"[111] Start measuring\n[201] Go to measuring settings\n[301] Go to device settings\n");
		return;
	}


	// get input command
	stateFlag = getConsoleInput(inputData, 3, 0);

	// timeout or no data handler
	if(stateFlag == 99){
		return;
	}

	switch(convertInputToInt(3, inputData)){
	//switch (convertInputToInt(3, &getConsoleInput(3))){
	//switch (convertInputToInt(3, &inputData)) {
		// Show default menu
		case 101:
			send_uart3("[101] Show default menu\n[102] Show current settings\n[103] Show last measured values\n[104] Read EEPROM\n[105] Write EEPROM\n[106] Initialize EEPROM\n"
							"[111] Start measuring\n[201] Go to measuring settings\n[301] Go to device settings\n");
			break;
		// Show measuring settings menu
		case 201:
			consoleMeasuringInterface();
			break;
		// Show device settings menu
		case 301:
			consoleDeviceInterface();
			break;
		// Show current settings
		case 102:
			printSettingsValues();
			break;
		// show last measured values
		case 103:
			printLastMeasuredValues();
			break;
		// read EEPROM
		case 104:
			readEEPROM_AT24();
			printSettingsValues();
			send_uart3("\n EEPROM read\n");
			break;
		// write EEPROM
		case 105:
			writeEEPROM_AT24();
			microDelay(5);
			readEEPROM_AT24();
			printSettingsValues();
			send_uart3("\n EEPROM written and read back\n");
			break;
		// initialize EEPROM
		case 106:
			send_uart3("\nDou you really want to initialize EEPROM (You will lose all user settings)?\n\n1 - YES, INIT AND SAVE\n2 - NO, I DONT WANT TO\n");
			getConsoleInput(&inputData, 1, 1);
			switch (convertInputToInt(1, &inputData)) {
			case 1:
				initEEPROM();
				writeEEPROM_AT24();
				microDelay(5);
				readEEPROM_AT24();
				printSettingsValues();
				send_uart3("\n\nEEPROM INITIALIZED\n");
				break;
			case 2:
				send_uart3("\nEEPROM NOT INITIALIZED\n");
				break;
			default:
				send_uart3("\nEEPROM NOT INITIALIZED\n");
				break;
			}
			break;
		// start measuring
		case 111:
			send_uart3("\n Measuring - started\n");
			if (isMeasuring == 0) {
				//isLoggingToConsole = 1;
				if (settings.isTriggerActive == 0)
					isMeasuring = 1;
				startOfMeasurement = 1;
				endOfMeasurement = 0;

				HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
			} else {
				isMeasuring = 0;
				isTriggered = 0;
				startOfMeasurement = 0;
				endOfMeasurement = 1;

				HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
			}
			break;
		// timeout error output
		case 99:
			send_uart3("\nERR-Input timeout\n");
			send_uart3("[101] Show default menu\n[102] Show current settings\n[103] Show last measured values\n[104] Read EEPROM\n[105] Write EEPROM\n[106] Initialize EEPROM\n"
							"[111] Start measuring\n[201] Go to measuring settings\n[301] Go to device settings\n");
			break;
		// all outher input combination output
		default:
			send_uart3("\nERR-Invalid input\n");
			send_uart3("[101] Show default menu\n[102] Show current settings\n[103] Show last measured values\n[104] Read EEPROM\n[105] Write EEPROM\n[106] Initialize EEPROM\n"
							"[111] Start measuring\n[201] Go to measuring settings\n[301] Go to device settings\n");
			break;
	}





}


void drawScreen (){

	u8x8_ClearDisplay(&u8x8);

	sprintf(charBuffer, "%d", settings.isAveraging);
	u8x8_DrawGlyph(&u8x8, 0, 1, charBuffer[0]);

	sprintf(charBuffer, "%d", settings.isLoggingToConsole);
	u8x8_DrawGlyph(&u8x8, 2, 1, charBuffer[0]);

	sprintf(charBuffer, "%d", settings.isLoggingToSD);
	u8x8_DrawGlyph(&u8x8, 4, 1, charBuffer[0]);

	sprintf(charBuffer, "%d", settings.isTriggerActive);
	u8x8_DrawGlyph(&u8x8, 6, 1, charBuffer[0]);

	//sprintf(charBuffer, "%.3f", settings.ADC_RESOLUTION);
	//u8x8_DrawString(&u8x8, 8, 1, charBuffer);

	sprintf(charBuffer, "%d", counter);
	u8x8_DrawGlyph(&u8x8, 0, 2, charBuffer[0]);

}

uint8_t gpioState (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){

	GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	if(GPIO_PIN_RESET == pinState)
		return 1;
	else
		return 0;

}



uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr){
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    //microDelay(arg_int*10);
    break;
  case U8X8_MSG_GPIO_DC:
    //HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
	//HAL_Delay(1);
    break;
  case U8X8_MSG_GPIO_RESET:
    //HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
	isDownTriggered=0;
	isEnterTriggered=0;
	isEscTriggered=0;
	isUpTriggered=0;
	isLeftTriggered=0;
	isRightTriggered=0;
	microDelay(1);
	//HAL_Delay(1);
	break;
  case U8X8_MSG_GPIO_MENU_SELECT:
	u8x8_SetGPIOResult(u8x8, isEnterTriggered);
	//u8x8_SetGPIOResult(u8x8, gpioState(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN));
	break;
  case U8X8_MSG_GPIO_MENU_NEXT:
	u8x8_SetGPIOResult(u8x8, isUpTriggered);
	break;
  case U8X8_MSG_GPIO_MENU_PREV:
	u8x8_SetGPIOResult(u8x8, isDownTriggered);
	break;
  case U8X8_MSG_GPIO_MENU_UP:
  	u8x8_SetGPIOResult(u8x8, isUpTriggered);
  	//u8x8_SetGPIOResult(u8x8, gpioState(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN));
  	break;
  case U8X8_MSG_GPIO_MENU_DOWN:
  	u8x8_SetGPIOResult(u8x8, isDownTriggered);
  	//u8x8_SetGPIOResult(u8x8, gpioState(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN));
  	break;
  case U8X8_MSG_GPIO_MENU_HOME:
	u8x8_SetGPIOResult(u8x8, isEscTriggered);
	//u8x8_SetGPIOResult(u8x8, gpioState(BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN));
	break;
  default:
	u8x8_SetGPIOResult(u8x8, 1);			// default return value
	break;
  }

  return 1;
}

uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;

  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
      {
    	  buffer[buf_idx++] = *data;
    	  data++;
    	  arg_int--;
      }
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
    	HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin, GPIO_PIN_SET);
    	// Hardware display reset
    	HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin, GPIO_PIN_RESET);
    	HAL_Delay(1);	// The minimum reset "L" pulse width (tRW) is 1us at VDD=3.3V and 2us at VDD=1.8V
    	HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin, GPIO_PIN_SET);
    	HAL_Delay(2); // The maximum reset duration (tR) is 1us at VDD=3.3V and 2us at VDD=1.8V
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      //buffer[buf_idx++] = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      //i2c_transfer(u8x8_GetI2CAddress(u8x8) >> 1, buf_idx, buffer);
      //HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, (uint8_t *) arg_ptr, arg_int, 1000);
      //HAL_I2C_Master_Transmit(&hi2c4, (uint16_t) i2cDeviceAddressOLED, &buffer[0], buf_idx, 1000);
      HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i2cDeviceAddressOLED, &buffer[0], buf_idx, 5);
      //HAL_I2C_Master_Transmit_DMA(&hi2c4, (uint16_t) i2cDeviceAddressOLED, &buffer[0], buf_idx);
      microDelay(1);
      break;
    default:
      return 0;
  }
  return 1;
}


/*
uint8_t u8x8_byte_sw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){

	uint8_t *data;

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, (uint8_t *) arg_ptr, arg_int, 1000);
		//HAL_I2C_Master_Transmit(&hi2c4, DevAddress, pData, Size, Timeout)
		break;

	case U8X8_MSG_BYTE_INIT:
		//i2c_init(u8x8);
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		//i2c_start(u8x8);
		HAL_I2C_Master_Transmit(&hi2c4, i2cDeviceAddress, (uint8_t *) arg_ptr, arg_int, 1000);
		//i2c_write_byte(u8x8, u8x8_GetI2CAddress(u8x8));
		//i2c_write_byte(u8x8, 0x078);
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		//i2c_stop(u8x8);
		break;
	default:
		return 0;
	}
	return 1;
}
*/


uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8)
{
  uint8_t pin_state;
  uint8_t result_msg = 0;	/* invalid message, no event */

  HAL_Delay(5);

  if(isEnterTriggered == 1){
	  //isEnterTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
	  return U8X8_MSG_GPIO_MENU_SELECT;

  }
  else if(isEscTriggered == 1){
	  //isEscTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
	  return U8X8_MSG_GPIO_MENU_HOME;

  }
  else if(isLeftTriggered== 1){
  	  //isLeftTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
  	  return U8X8_MSG_GPIO_MENU_PREV;

  }
  else if(isRightTriggered == 1){
  	  //isRightTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
  	  return U8X8_MSG_GPIO_MENU_NEXT;

  }/*
  else if(HAL_GPIO_ReadPin(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN) == GPIO_PIN_RESET){
	  return U8X8_MSG_GPIO_MENU_PREV;
  }
  else if(HAL_GPIO_ReadPin(BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN) == GPIO_PIN_RESET){
  	  return U8X8_MSG_GPIO_MENU_NEXT;
  }*/
  else if(isUpTriggered == 1){
	  //isUpTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
  	  return U8X8_MSG_GPIO_MENU_UP;

  }
  else if(isDownTriggered == 1){
	  //isDownTriggered = 0;
	  isDownTriggered = 0;
	  isEnterTriggered = 0;
	  isEscTriggered = 0;
	  isUpTriggered = 0;
	  isLeftTriggered = 0;
	  isRightTriggered = 0;
  	  return U8X8_MSG_GPIO_MENU_DOWN;

  }
  else{
	  return 0;
  }



  return result_msg;
}

void screen_u8x8_test(){

	 //u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	//u8g2_Setup_sh1106_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_stm32_gpio_and_delay);
	u8x8_Setup(&u8x8, u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_i2c,
			u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	u8x8_InitDisplay(&u8x8);
	//u8x8_d_sh1106_128x64_noname(&u8x8, msg, arg_int, arg_ptr)
	u8x8_SetPowerSave(&u8x8, 0);
	u8x8_ClearDisplay(&u8x8);
	//u8x8_SetFont(&u8x8, u8x8_font_amstrad_cpc_extended_r);
	u8x8_SetFont(&u8x8, u8x8_font_5x7_f);
	//u8g2_InitDisplay(&u8g2);
	//u8g2_SetPowerSave(&u8g2, 0);

	u8x8_DrawString(&u8x8, 0, 0, "Hello World!");

	drawScreen();
	counter++;
	HAL_Delay(1500);

	settings.isAveraging = 1;

	drawScreen();
	counter++;
	HAL_Delay(1500);
	// save current settings data to eeprom
	//eepromSaveObject(settings.isAveraging, 1);

	drawScreen();
	counter++;
	HAL_Delay(1500);

	//initEEPROM();

	drawScreen();
	counter++;
	HAL_Delay(1500);

	// read eeprom for settings
	//eepromReadObject(settings.isAveraging, 1);

	drawScreen();
	counter++;
	HAL_Delay(1500);


}

void screen_u8g2_test(){


	//u8x8_Setup(&u8x8, u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_i2c, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	//u8x8_Setup(u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_i2c, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	//u8g2_Setup_sh1106_i2c_128x64_noname_2(&u8g2, 0, u8x8_byte_sw_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]
	u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]  //u8x8_byte_sw_i2c
	//u8g2_SetI2CAddress(&u8g2, i2cDeviceAddress);
	//uint8_t memoryBuffer [8*u8g2_GetBufferTileHeight(&u8g2)*u8g2_GetBufferTileHeight(&u8g2)];

	//u8g2_SetBufferPtr(&u8g2, &memoryBuffer);

	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetFontMode(&u8g2, 1);
	u8g2_SetFontDirection(&u8g2, 0);
	u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);		//u8g2_font_4x6_tf

	u8g2_ClearBuffer(&u8g2);

	//u8g2_drawStr(0,15,"Hello World!");
	u8g2_DrawStr(&u8g2,2 , 7, "test");
	//u8g2_DrawGlyph(u8g2, x, y, encoding)
	u8g2_DrawLine(&u8g2, 0, 20, 128, 20);
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(2000);

	uint8_t buffer [50];

	sprintf(buffer, "Value before: %d", testValue);
	send_uart3(buffer);
	u8g2_UserInterfaceInputValue(&u8g2, "Source voltage: "," " , &testValue, 0, 255, 3, " V");

	sprintf(buffer, "Value after: %d", testValue);
	send_uart3(buffer);

	HAL_Delay(10000);

	sprintf(buffer, "Value before: %d", testValue);
	send_uart3(buffer);
	u8g2_UserInterfaceInputValue(&u8g2, "Source voltage: "," " , &testValue, 0, 255, 3, " V");

	sprintf(buffer, "Value after: %d", testValue);
	send_uart3(buffer);


	u8g2_UserInterfaceSelectionList(&u8g2, "MENU", 1, "settings\nstart measuring\nfactory reset\nexit");


	//u8g2.userInterfaceInputValue("Select Voltage", "DAC= ", &v, 0, 5, 1, " V");

	//u8g2_FirstPage(&u8g2);

	//u8x8_InitDisplay(&u8x8);
	//u8x8_d_sh1106_128x64_noname(&u8x8, msg, arg_int, arg_ptr)
	//u8x8_SetPowerSave(&u8x8, 0);
	//u8x8_ClearDisplay(&u8x8);
	//u8x8_SetFont(&u8x8, u8x8_font_amstrad_cpc_extended_r);
	//u8x8_SetFont(&u8x8, u8x8_font_5x7_f);




}

void screenInfoPanel(){

	//u8g2_ClearDisplay(&u8g2);
	//u8g2_SetFontMode(&u8g2, 1);
	//u8g2_SetFontDirection(&u8g2, 0);
	//u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);		//u8g2_font_4x6_tf

	u8g2_ClearBuffer(&u8g2);

	u8g2_DrawStr(&u8g2, 15 , 7, "InfoPanel");

	//u8g2_DrawStr(&u8g2, 70 , 7, "Range: Auto");
	u8g2_DrawStr(&u8g2, 70 , 7, "Range: ");
	if(rangeMode == 0)
		u8g2_DrawStr(&u8g2, 98 , 7, "nA");
	else if(rangeMode == 1)
		u8g2_DrawStr(&u8g2, 98 , 7, "uA");
	else if(rangeMode == 2)
		u8g2_DrawStr(&u8g2, 98 , 7, "mA");
	else
		u8g2_DrawStr(&u8g2, 98 , 7, "Auto");

	//u8g2_DrawGlyph(u8g2, x, y, encoding)
	u8g2_DrawLine(&u8g2, 0, 10, 128, 10);
	u8g2_DrawLine(&u8g2, 64, 0, 64, 10);

	sprintf(buffer, "OUT voltage [mV]: %d", settings.powerSourceVoltage);
	u8g2_DrawStr(&u8g2, 10 , 20, buffer);
	sprintf(buffer, "OUT current limit [mA]: %d", settings.powerSourceCurrent);
	u8g2_DrawStr(&u8g2, 10 , 30, buffer);
	sprintf(buffer, "Sampling interval [us]: %d", settings.samplingInterval);
	u8g2_DrawStr(&u8g2, 10 , 40, buffer);

	//u8g2_DrawStr(&u8g2, 10 , 50, "Storing method: NA");
	u8g2_DrawStr(&u8g2, 10 , 50, "Storing method: ");
	if(settings.isLoggingToConsole == 1)
		u8g2_DrawStr(&u8g2, 74 , 50, "PC");
	else if(settings.isLoggingToSD == 1)
		u8g2_DrawStr(&u8g2, 74 , 50, "SD");
	else if(settings.isLoggingToUSB == 1)
		u8g2_DrawStr(&u8g2, 74 , 50, "USB");
	else if(settings.isLoggingToEthernet == 1)
		u8g2_DrawStr(&u8g2, 74 , 50, "Eth");

	u8g2_SendBuffer(&u8g2);
	//HAL_Delay(50);



}

/*
Device settings submenu structure
 *		[301] *Show device settings submenu
 *		[302] *Setting RTC (Time and Date) - DIF SUBMENU
 *		[303] *Choosing storing method (UART x SD card x USB flash) // NOT DECIDED WHICH
 *		[304] *Check SD card and print info about card
 *		[305] *Check USB flash drive and print info about drive // NOT SURE YET
 *		[306] *Perform offset calibration
 *		[307] *Perform zero offset value
 *		[308] *Do device selfcheck test // NOT DECIDED YET
 *		[309] *Adjust voltage for power source - DIF SUBMENU
 *		[310] *Adjust current limit for power source - DIF SUBMENU
 *		[311] *Enable power source - DIF SUBMENU
 *		[312] *Behaviour mode of power source - DIF SUBMENU
 *		[888] *Go to main menu
 *		[999] *Go to main menu
*/

void screenDeviceSettings(){

	/*
	uint8_t buffer [50];

	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawStr(&u8g2, 20, 35, "Device Settings");
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(1000);
	u8g2_UserInterfaceInputValue32(&u8g2, "Test uint32", " ", &settings.samplingInterval, 100, 50000, 5, " us");

	sprintf(buffer, "Test uint32: %d", settings.samplingInterval);
	send_uart(buffer);
	HAL_Delay(10000);
	*/

	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);

	uint32_t temp32 = 0;

	switch (u8g2_UserInterfaceSelectionList(&u8g2, "Device menu", 1,"Storing method\nCheck SD card\nCheck USB drive\nOffset calibration\nNull offset\nBack to Main")) {
	// exit back to main menu
	case 0:
		isEnterTriggered = 1;
		return screenInterface();
	// storing method
	case 1:
		temp32 = u8g2_UserInterfaceSelectionList(&u8g2, "Storing method", 1,"COM port\nSD Card\nUSB Drive\nEthernet\nExit");
		if (temp32 == 1) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "STORE METHOD: COM Port");
			u8g2_SendBuffer(&u8g2);
			settings.isLoggingToConsole = 1;
			settings.isLoggingToSD = 0;
			settings.isLoggingToUSB = 0;
			settings.isLoggingToEthernet = 0;
			HAL_Delay(1500);
		} else if (temp32 == 2) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "STORE METHOD: SD Card");
			u8g2_SendBuffer(&u8g2);
			settings.isLoggingToConsole = 0;
			settings.isLoggingToSD = 1;
			settings.isLoggingToUSB = 0;
			settings.isLoggingToEthernet = 0;
			HAL_Delay(1500);
		} else if (temp32 == 3) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "STORE METHOD: USB Drive");
			u8g2_SendBuffer(&u8g2);
			settings.isLoggingToConsole = 0;
			settings.isLoggingToSD = 0;
			settings.isLoggingToUSB = 1;
			settings.isLoggingToEthernet = 0;
			//settings.isLoggingToConsole = 1;
			//settings.isLoggingToSD = 1;
			HAL_Delay(1500);
		} else if (temp32 == 3) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "STORE METHOD: Ethernet");
			u8g2_SendBuffer(&u8g2);
			settings.isLoggingToConsole = 0;
			settings.isLoggingToSD = 0;
			settings.isLoggingToUSB = 0;
			settings.isLoggingToEthernet = 1;
			//settings.isLoggingToConsole = 1;
			//settings.isLoggingToSD = 1;
			HAL_Delay(1500);
		} else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "STORE METHOD: UNCHANGED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenDeviceSettings();
		break;
	// check sd card
	case 2:
		u8g2_ClearDisplay(&u8g2);
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawStr(&u8g2, 20, 35, "SD CARD: OK");
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(1500);
		return screenDeviceSettings();
		break;
	// check usb flash drive
	case 3:
		u8g2_ClearDisplay(&u8g2);
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawStr(&u8g2, 20, 35, "USB DRIVE: OK");
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(1500);
		return screenDeviceSettings();
		break;
	// perform zero offset calibration
	case 4:
		if (u8g2_UserInterfaceMessage(&u8g2, "Offset calibration", NULL, NULL," YES \n NO ") == 1) {
			adc_compensateOffset();
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "OFFSET: CALIBRATED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		} else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 15, 35, "OFFSET: UNCHANGED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenDeviceSettings();
		break;
	// zero offset value
	case 5:
		if (u8g2_UserInterfaceMessage(&u8g2, "Zero offset value", NULL, NULL," YES \n NO ") == 1) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "OFFSET: ZEROED");
			u8g2_SendBuffer(&u8g2);
			settings.lastOffsetValue = 0.0;
			HAL_Delay(1500);
		} else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 15, 35, "OFFSET: UNCHANGED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenDeviceSettings();
		break;
	// default handle
	default:
		isEnterTriggered = 1;
		return screenInterface();
		break;

	}



}

/*
*	Measuring settings submenu structure - basic submenu structure will be printed when first accessed
 *		[201] *Show measuring settings submenu
 *		[202] *Set measuring period
 *		[203] *Set sampling period
 *		[204] *Enabling averaging
 *		[205] *Enabling trigger menu
 *		[206] *Set trigger threshold
 *		[888] *Go to main menu
 *		[999] *Go to main menu
 */
void screenMeasuringSettings(){

	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	/*
	u8g2_DrawStr(&u8g2, 20, 35, "Measuring Settings");
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(1000);
	*/

	uint32_t temp32 = 0;

	switch(u8g2_UserInterfaceSelectionList(&u8g2, "Measuring menu", 1, "Current settings\nMeasuring period\nSampling period\nEnable averaging\nEnable trigger\nTrigger threshold\nBack to Main")){
	case 0:
		isEnterTriggered = 1;
		return screenInterface();

	// current settings
	case 1:
		screenInfoPanel();
		break;
	// measuring period
	case 2:
		u8g2_UserInterfaceInputValue32(&u8g2, "Measuring period", "", &settings.measuringInterval, 1, 100000, 5, " us");
		return screenMeasuringSettings();
		break;
	// sampling period
	case 3:
		u8g2_UserInterfaceInputValue32(&u8g2, "Sampling period", "", &settings.samplingInterval, 1, 50000, 5, " us");
		return screenMeasuringSettings();
		break;
	// enable averaging
	case 4:
		if (u8g2_UserInterfaceMessage(&u8g2, "Enable averaging function", NULL, NULL," YES \n NO ") == 1) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "AVERAGING: ENABLED");
			u8g2_SendBuffer(&u8g2);
			settings.isAveraging = 1;
			HAL_Delay(1500);
		}
		else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 15, 35, "AVERAGING: DISABLED");
			u8g2_SendBuffer(&u8g2);
			settings.isAveraging = 0;
			HAL_Delay(1500);
		}
		return screenMeasuringSettings();
		break;
	// enable trigger
	case 5:
		if (u8g2_UserInterfaceMessage(&u8g2, "Enable trigger function", NULL, NULL," YES \n NO ") == 1) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "TRIGGER: ENABLED");
			u8g2_SendBuffer(&u8g2);
			settings.isTriggerActive = 1;
			HAL_Delay(1500);
		}
		else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 15, 35, "TRIGGER: DISABLED");
			u8g2_SendBuffer(&u8g2);
			settings.isTriggerActive = 0;
			HAL_Delay(1500);
		}
		return screenMeasuringSettings();
		break;
	// trigger level
	case 6:
		u8g2_UserInterfaceInputValue32(&u8g2, "Trigger level", "x100 nA", &temp32, 1, 10000000, 5, " nA");
		settings.triggerLevel = temp32*100;
		settings.triggerLevel /= 10000000;
		return screenMeasuringSettings();
		break;
	default:
		isEnterTriggered = 1;
		return screenInterface();
		break;

	}


}

/*
*		[309] *Adjust voltage for power source
*		[310] *Adjust current limit for power source
*		[311] *Enable power source
*		[312] *Behaviour mode of power source
*/

void screenPowerSourceControl(){

	/*
	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawStr(&u8g2, 15, 35, "Power Source Control");
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(1000);
	*/
	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);


	uint32_t temp32 = 0;

	switch (u8g2_UserInterfaceSelectionList(&u8g2, "Power source menu", 1,"Adjust voltage\nAdjust current limit\nEnable source\nEnable mode\nBack to Main")) {
	// exit back to main menu
	case 0:
		isEnterTriggered = 1;
		return screenInterface();
	// adjust voltage
	case 1:
		temp32 = u8g2_UserInterfaceInputValue32(&u8g2, "Source voltage", "",&settings.powerSourceVoltage, 500, 5500, 5, " mV");
		if(temp32 == 1){
			setPotenciomenters();
		}
		return screenPowerSourceControl();
		break;
	// adjust current limit
	case 2:
		temp32 = u8g2_UserInterfaceInputValue32(&u8g2, "Source current limit", "",&settings.powerSourceCurrent, 10, 500, 5, " mA");
		if(temp32 == 1){
			setPotenciomenters();
		}
		return screenPowerSourceControl();
		break;
	// enable source
	case 3:
		if (u8g2_UserInterfaceMessage(&u8g2, "Enable power souce", NULL, NULL," YES \n NO ") == 1) {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "SOURCE: ENABLED");
			u8g2_SendBuffer(&u8g2);
			HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);
			HAL_Delay(1500);
		}
		else {
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 15, 35, "SOURCE: DISABLED");
			u8g2_SendBuffer(&u8g2);
			HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, RESET);
			HAL_Delay(1500);
		}
		return screenPowerSourceControl();
		break;
	// enable mode
	case 4:
		temp32 = u8g2_UserInterfaceSelectionList(&u8g2, "Enable mode", 1,"Enable on measuring\nEnable on boot\nManual mode\nExit");
		if(temp32 == 1){
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "ENABLE: START MEAS");
			u8g2_SendBuffer(&u8g2);
			settings.powerSourceEnableMode = 1;
			HAL_Delay(1500);
		}
		else if(temp32 == 2){
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "ENABLE: DURING BOOT");
			u8g2_SendBuffer(&u8g2);
			settings.isAveraging = 2;
			HAL_Delay(1500);
			HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);

		}
		else if(temp32 == 3){
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "ENABLE: MANUAL");
			u8g2_SendBuffer(&u8g2);
			settings.isAveraging = 0;
			HAL_Delay(1500);
		}
		else{
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "ENABLE: NO CHANGE");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
			//settings.powerSourceEnableMode = 1;
		}
		return screenPowerSourceControl();
		break;
	// default handle
	default:
		isEnterTriggered = 1;
		return screenInterface();
		break;

	}


}

/**	RTC settings submenu structure
 *		[302] *Show RTC settings submenu
 *		[321] *Show current time and date
 *		[322] *Set Time - Hours, Minutes
 *		[323] *Set Date - Day
 *		[324] *Set Date - Month
 *		[325] *Set Date - Year
 *		[888] *Go to Device settings menu
 *		[999] *Go to main menu
 */

void screenRtcSettings(){

	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);

	/* Get the RTC current Time and Date */
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);

	uint32_t temp32 = 0;

	switch (u8g2_UserInterfaceSelectionList(&u8g2, "RTC menu", 1,"Set Time\nSet Date - Days\nSet Date - Month\nSet Date - Year\nBack to Main")) {
	// exit back to main menu
	case 0:
		isEnterTriggered = 1;
		return screenInterface();
	// set time - hours, minutes, seconds
	case 1:
		temp32 = 0;
		if(u8g2_UserInterfaceInputValue32(&u8g2, "Enter time - hours", "1 .. 24",&Time.Hours, 1, 24, 5, " h") == 1){
			if(u8g2_UserInterfaceInputValue32(&u8g2, "Enter time - minutes", "0 .. 59",&Time.Minutes, 0, 59, 5, " min") == 1){
				if(u8g2_UserInterfaceInputValue32(&u8g2, "Enter time - seconds", "0 .. 59",&Time.Seconds, 0, 59, 5, " sec") == 1){

					Time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					Time.StoreOperation = RTC_STOREOPERATION_RESET;
					if (HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN) != HAL_OK)
					{
						u8g2_ClearDisplay(&u8g2);
						u8g2_ClearBuffer(&u8g2);
						u8g2_DrawStr(&u8g2, 20, 35, "TIME: ERROR");
						u8g2_SendBuffer(&u8g2);
						HAL_Delay(1500);
					}
					else{
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register

						u8g2_ClearDisplay(&u8g2);
						u8g2_ClearBuffer(&u8g2);
						u8g2_DrawStr(&u8g2, 20, 35, "TIME: SAVED");
						u8g2_SendBuffer(&u8g2);
						HAL_Delay(1500);
					}

				}
				else{
					u8g2_ClearDisplay(&u8g2);
					u8g2_ClearBuffer(&u8g2);
					u8g2_DrawStr(&u8g2, 20, 35, "TIME: NOT SAVED");
					u8g2_SendBuffer(&u8g2);
					HAL_Delay(1500);

				}

			}
			else{
				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "TIME: NOT SAVED");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);
			}
		}
		else{
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "TIME: NOT SAVED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);

		}

		return screenRtcSettings();
		break;
	// set date day
	case 2:
		if(u8g2_UserInterfaceInputValue32(&u8g2, "Set date - day", "1 .. 31",&Date.Date, 1, 31, 5, "") == 1){

			if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
			{
				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "DAY: ERROR");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);
			}
			else{
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register

				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "DAY: SAVED");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);

			}

		}
		else{
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "DAY: NOT SAVED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenRtcSettings();
		break;
	// set date month
	case 3:
		if(u8g2_UserInterfaceInputValue32(&u8g2, "Set date - month", "1 .. 12",&Date.Month, 1, 12, 5, "") == 1){

			if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
			{
				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "MONTH: ERROR");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);
			}
			else{
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register

				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "MONTH: SAVED");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);

			}

		}
		else{
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "MONTH: NOT SAVED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenRtcSettings();
		break;
	// set date year
	case 4:
		if(u8g2_UserInterfaceInputValue32(&u8g2, "Set date - year", "20XX 0 .. 100",&Date.Year, 0, 100, 5, "") == 1){

			if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
			{
				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "YEAR: ERROR");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);
			}
			else{
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register

				u8g2_ClearDisplay(&u8g2);
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 20, 35, "YEAR: SAVED");
				u8g2_SendBuffer(&u8g2);
				HAL_Delay(1500);

			}

		}
		else{
			u8g2_ClearDisplay(&u8g2);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 20, 35, "YEAR: NOT SAVED");
			u8g2_SendBuffer(&u8g2);
			HAL_Delay(1500);
		}
		return screenRtcSettings();
		break;
	// default handle
	default:
		isEnterTriggered = 1;
		return screenInterface();
		break;

	}




}

/*
 *	Interface for control command receive/transmit including settings various parameters such as sampling interval, storing method, etc
 *
 *	command structure is Host will send 3 Bytes/Chars in format of 3 numbers which represent menu item, slave will then reply with request for entering user input
 *
 *	Default menu will be send to PC when connected to serial or on request by sending "101" command
 *	When Host sends request for start measuring through console device will start measuring and stop responding to any console request because of insufficient computing power
 *	only way to stop measuring will be through HW button on device
 *	To go from submenu to main menu send command "999" or send command ""888" to  go one level higher in menu structure
 *
 *	Default menu structure
 *		[101] *Show default menu
 *		[102] *Show current settings
 *		[103] *Show last measured values
 *		[104] *Read EEPROM
 *		[105] *Write EEPROM
 *		[106] *Initializi EEPROM
 *		[111] *Start measuring with currently applied and saved settings
 *		[201] *Go to measuring settings submenu
 *		[301] *Go to device settings submenu
 *
 *	Measuring settings submenu structure - basic submenu structure will be printed when first accessed
 *		[201] *Show measuring settings submenu
 *		[202] *Set measuring period
 *		[203] *Set sampling period
 *		[204] *Enabling averaging
 *		[205] *Enabling trigger menu
 *		[206] *Set trigger threshold
 *		[888] *Go to main menu
 *		[999] *Go to main menu
 *
 *	Device settings submenu structure
 *		[301] *Show device settings submenu
 *		[302] *Setting RTC (Time and Date)
 *		[303] *Choosing storing method (UART x SD card x USB flash) // NOT DECIDED WHICH
 *		[304] *Check SD card and print info about card
 *		[305] *Check USB flash drive and print info about drive // NOT SURE YET
 *		[306] *Perform offset calibration
 *		[307] *Perform zero offset value
 *		[308] *Do device selfcheck test // NOT DECIDED YET
 *		[309] *Adjust voltage for power source
 *		[310] *Adjust current limit for power source
 *		[311] *Enable power source
 *		[312] *Behaviour mode of power source
 *		[888] *Go to main menu
 *		[999] *Go to main menu
 *
 *	RTC settings submenu structure
 *		[302] *Show RTC settings submenu
 *		[321] *Show current time and date
 *		[322] *Set Time - Hours, Minutes
 *		[323] *Set Date - Day
 *		[324] *Set Date - Month
 *		[325] *Set Date - Year
 *		[888] *Go to Device settings menu
 *		[999] *Go to main menu
 *
 *
 */
void screenInterface(){

	//u8g2_ClearDisplay(&u8g2);
	//u8g2_SetFontMode(&u8g2, 1);
	//u8g2_SetFontDirection(&u8g2, 0);
	//u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);		//u8g2_font_4x6_tf


	//isDownTriggered = 0;
	//isUpTriggered = 0;
	isLeftTriggered = 0;
	isRightTriggered = 0;
	isEscTriggered = 0;

	if(isUpTriggered){
		isUpTriggered = 0;
		rangeMode++;

		if(rangeMode > 4)
			rangeMode = 0;
	}
	if(isDownTriggered){
		isDownTriggered = 0;
		rangeMode--;

		if(rangeMode < 0)
			rangeMode = 4;

	}

	if(isEnterTriggered == 1){
		isEnterTriggered = 0;
		u8g2_ClearBuffer(&u8g2);


		switch(u8g2_UserInterfaceSelectionList(&u8g2, "MENU", 1, "Device Settings\nMeasuring settings\nStart measuring\nPower source control\nRTC Settings\nBack to InfoPanel")){
			// back to info panel
			case 0:
				screenInfoPanel();
				break;
			// device settings
			case 1:
				screenDeviceSettings();
				break;
			// measuring settings
			case 2:
				screenMeasuringSettings();
				break;
			// Start measuring
			case 3:
				if(u8g2_UserInterfaceMessage(&u8g2, "Start measuring", NULL, NULL, " YES \n NO ") == 1){
					if (isMeasuring == 0) {
						//isLoggingToConsole = 1;
						if (settings.isTriggerActive == 0)
							isMeasuring = 1;
						startOfMeasurement = 1;
						endOfMeasurement = 0;

						HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

						u8g2_ClearDisplay(&u8g2);
						u8g2_ClearBuffer(&u8g2);
						u8g2_DrawStr(&u8g2, 20, 35, "Measuring STARTED!");
						u8g2_SendBuffer(&u8g2);

					} else {
						isMeasuring = 0;
						isTriggered = 0;
						startOfMeasurement = 0;
						endOfMeasurement = 1;

						HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);

						u8g2_ClearDisplay(&u8g2);
						u8g2_ClearBuffer(&u8g2);
						u8g2_DrawStr(&u8g2, 20, 35, "Measuring STOPPED!");
						u8g2_SendBuffer(&u8g2);
					}
				}
				else{
					u8g2_ClearDisplay(&u8g2);
					u8g2_ClearBuffer(&u8g2);
					u8g2_DrawStr(&u8g2,15 , 35, "Start measuring CANCELED!");
					u8g2_SendBuffer(&u8g2);
				}
				break;
			// Power source control
			case 4:
				screenPowerSourceControl();
				break;
			// rtc settings
			case 5:
				screenRtcSettings();
				break;
			default:
				screenInfoPanel();
				break;


		}



	}

	else{
		screenInfoPanel();
	}




}

void cleanBuffer(uint8_t *buffer, uint16_t bufferSize){

	for(uint16_t i = 0; i < bufferSize; i++ )
		buffer[i] = 0;
}


// function to initialize device
void deviceInit(){

	// init 32 bit TIM5 in normal mode for 1 us delay
	HAL_TIM_Base_Start(&htim5);
	// init 16 bit TIM4 in normal mode for 1 us delay
	HAL_TIM_Base_Start(&htim4);

	// init 16 bit TIM14 for timeout interrupt
	HAL_TIM_Base_Start(&htim14);

	// unit indication LED
	HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);

	// default setting for ranges
	// mA range OFF
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_RESET);

	// uA range OFF
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

	// nA range ON
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_SET);

	// GND range ON
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_GND_PORT, RANGE_SELECT_PIN_AS_GND, GPIO_PIN_RESET);

	// set adc conv pin to default/low level
	HAL_GPIO_WritePin(ADC_CONV_PORT, ADC_CONV_PIN, GPIO_PIN_RESET);

	/***************** set mA range on -- TEST only ********************************/
	currentRange = 2;

	// mA range ON
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_MA_PORT, RANGE_SELECT_PIN_AS_MA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_MA_PORT, RANGE_SELECT_PIN_TRANS_MA, GPIO_PIN_SET);

	// uA range OFF
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_UA_PORT, RANGE_SELECT_PIN_AS_UA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_UA_PORT, RANGE_SELECT_PIN_TRANS_UA, GPIO_PIN_RESET);

	// nA range OFF
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_AS_NA_PORT, RANGE_SELECT_PIN_AS_NA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RANGE_SELECT_PIN_TRANS_NA_PORT, RANGE_SELECT_PIN_TRANS_NA, GPIO_PIN_RESET);

	// USB OTG Power Enable
	send_uart3("USB OTG POWER - ENABLED\n");
	HAL_GPIO_WritePin(USB_OTG_POWER_EN_PORT, USB_OTG_POWER_EN_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	// OLED init
	//u8g2_Setup_sh1106_i2c_128x64_noname_2(u8g2, rotation, u8x8_byte_sw_i2c, u8x8_stm32_gpio_and_delay_cb); // [page buffer, size = 256 bytes]
	//u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_stm32_gpio_and_delay_cb);
	//u8g2_InitDisplay(&u8g2);
	//u8g2_SetPowerSave(&u8g2, 0);
	// SCREEN INIT

	//u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]  //u8x8_byte_sw_i2c
	//u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R2, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]  //u8x8_byte_sw_i2c // 180 degree rotation
	u8g2_Setup_st7528_i2c_nhd_c160100_f(&u8g2, U8G2_R2, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = full page XYZ bytes]  //u8x8_byte_sw_i2c
	u8g2_InitDisplay(&u8g2);

	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_SetFlipMode(&u8g2, 1);
	u8g2_ClearDisplay(&u8g2);
	//u8g2_Clear(&u8g2);
	u8g2_SetFontMode(&u8g2, 0);
	u8g2_SetContrast(&u8g2, 80);
	u8g2_SetFontDirection(&u8g2, 0);
	u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);//u8g2_font_4x6_tf		//u8g2_font_6x10_tf
	u8g2_ClearBuffer(&u8g2);
	//ST7528_Init();
	//ST7528_Contrast(ST7528_RREG_72, ST7528_BIAS_11, 10);
	//ST7528_Clear();
	//ST7528_SetYDir(SCR_INVERT_OFF);
	//u8g2_DrawStr(&u8g2, 10, 10, "Hello World!");
	//u8g2_DrawLine(&u8g2, 5, 5, 155, 15);
	u8g2_SendBuffer(&u8g2);
	u8g2_UpdateDisplay(&u8g2);
	HAL_Delay(100);
	/*

	 u8x8_Setup(&u8x8, u8x8_d_st7528_nhd_c160100, u8x8_cad_ssd13xx_i2c, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	 u8x8_InitDisplay(&u8x8);
	 u8x8_SetPowerSave(&u8x8, 0);
	 u8x8_SetFlipMode(&u8x8, 1);
	 u8x8_ClearDisplay(&u8x8);
	 u8x8_SetFlipMode(&u8x8, 0);
	 u8x8_SetContrast(&u8x8, 80);
	 u8x8_SetFont(&u8x8, u8x8_font_5x7_f);
	 u8x8_DrawString(&u8x8, 10, 10, "Hello world");
	 u8x8_RefreshDisplay(&u8x8);
	 */
	/*
	 // test alt lib for ST7528i
	 ST7528_Init();
	 ST7528_Contrast(ST7528_RREG_72, ST7528_BIAS_11, 10);
	 ST7528_Clear();
	 ST7528_SetYDir(SCR_INVERT_OFF);
	 ST7528_Flush();

	 LCD_Rect(5, 5, 155, 95, 15); //line1
	 //LCD_Rect(3, 5, 155, 48, 0); //lne2 oblast chzbneho vzkresleni
	 //LCD_Rect(3, 49, 155, 64, 0);  //line3
	 //LCD_Rect(3, 65, 155, 80, 0); //line4
	 LCD_PutStr(40, 40, "ASDFG:\nTEST", &Font5x7);
	 LCD_Circle(30, 44, 4, 15);
	 ST7528_Flush();
	 HAL_Delay(500);
	 */

	u8g2_DrawStr(&u8g2, 20, 20, "Current logger");
	u8g2_DrawStr(&u8g2, 20, 40, "Firmware version 1.0.0");
	u8g2_SendBuffer(&u8g2);
	u8g2_UpdateDisplay(&u8g2);

	HAL_Delay(1000);

	// init uart ring buffers
	UARTRXInit();
	UARTTXInit();


	// initialize RB buffers
	ringbuff_init(&inputBuffer_RB, inputBuffer, RB_INPUT_SIZE);
	ringbuff_init(&outputBuffer_RB, outputFormatterBuffer, RB_OUTPUT_SIZE);


	//HAL_ADC_Start_DMA(&hadc1, &adcBuffer, 1);
	//HAL_ADC_Start_IT(&hadc1);

	//screenInfoPanel();
	//screenInterface();
	//send_uart3("\ntest print\n");

	//sd_card_test();

	/***********************************************/
	// device config load
	/*
	 send_uart3("EEPROM - DATA WRITE - START\n");
	 microDelay(50);
	 writeEEPROM_AT24();
	 HAL_Delay(5);
	 send_uart3("EEEPROM - DATA WRITE - DONE\n");
	 microDelay(50);
	 */
	send_uart3("EEPROM - DATA READ - START\n");
	microDelay(50);

	readEEPROM_AT24();
	printSettingsValues();

	send_uart3("EEPROM - DATA READ - DONE\n");
	microDelay(50);

	// config ADC
	// reset pin
	HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_SET);

	//adc_config();

	microDelay(500);

	// measure adc offset
	//adc_compensateOffset();
	microDelay(50);

	//settings.lastOffsetValue = -0.04;

	//u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	//u8g2_DrawStr(&u8g2, 0, 15, "Hello World!");
	//u8g2_DrawCircle(&u8g2, 64, 40, 10, U8G2_DRAW_ALL);

	HAL_RTC_Init(&hrtc);

	// getting time and date for formatting as name of new logged file
	HAL_RTC_GetTime(&hrtc, &Time, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, FORMAT_BIN);
	// file name format "20YYMMDD_HHMM" example "20200120_1022" = 20.1.2020 10:22
	sprintf(uartBufferTx, "\n%2d%02d%02d_%02d%02d.txt\n", 2000 + Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes);

	send_uart3(uartBufferTx);
	microDelay(50);
	//HAL_RTC_SetTime(&hrtc, &Time, FORMAT_BCD);
	//HAL_RTC_SetDate(&hrtc, &Date, FORMAT_BCD);

	send_uart3("Power source - Init - START\n");
	microDelay(50);

	setPotenciomenters();

	readPotenciometers();

	if ((settings.powerSourceEnableMode == 0 & settings.powerSourceEnable == 1)
			| settings.powerSourceEnableMode == 2) {
		HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, SET);
		send_uart3("Power Source: ENABLED\n");
	} else {
		HAL_GPIO_WritePin(PS_EN_PORT, PS_EN_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, RESET);
		send_uart3("Power Source: DISABLED\n");
	}

	// test power source for output voltage
	/*
	 *
	 for(uint8_t i = 0; i < 18; i++){
	 settings.powerSourceVoltage = i*15;
	 setPotenciomenters();
	 sprintf(uartBufferTx, "\nEL pot values: %2d\n", settings.powerSourceVoltage);
	 send_uart3(uartBufferTx);
	 HAL_Delay(1000);

	 }
	 */

	send_uart3("Power source - Init - DONE\n");
	microDelay(50);

	send_uart3("\nINITDONE\n");
	send_uart2("\nINITDONE\n");
	microDelay(50);



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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_UART7_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /*HAL_Delay(250);

  HAL_RTC_SetTime(&hrtc, &Time, FORMAT_BCD);
  HAL_RTC_SetDate(&hrtc, &Date, FORMAT_BCD);
*/
  HAL_Delay(500);

  deviceInit();

  // initialize spi transfer buffers
  for(uint8_t i = 0; i < sizeof(spiTxBuffer); i++)
	  spiTxBuffer[i] = 0;

  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskUSB */
  //myTaskUSBHandle = osThreadNew(vTaskUSB, NULL, &myTaskUSB_attributes);

  /* creation of myTaskInputBuff */
  myTaskInputBuffHandle = osThreadNew(vTaskInputBuffer, NULL, &myTaskInputBuff_attributes);

  /* creation of myTaskEthernet */
  //myTaskEthernetHandle = osThreadNew(vTaskEthernet, NULL, &myTaskEthernet_attributes);

  /* creation of myTaskUI */
  myTaskUIHandle = osThreadNew(vTaskUi, NULL, &myTaskUI_attributes);

  /* creation of myTaskLCD */
  myTaskLCDHandle = osThreadNew(vTaskLcd, NULL, &myTaskLCD_attributes);

  /* creation of myTaskButtons */
  myTaskButtonsHandle = osThreadNew(vTaskButtons, NULL, &myTaskButtons_attributes);

  /* creation of myTaskUART */
  myTaskUARTHandle = osThreadNew(vTaskUart, NULL, &myTaskUART_attributes);

  /* creation of myTaskSD */
  //myTaskSDHandle = osThreadNew(vTaskSd, NULL, &myTaskSD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */


  send_uart("\n\nRTOS OS STARTED\n\n");
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  send_uart("\n\nI WILL NEVER GET HERE! RTOS KERNEL STOPPED!!!\n\n");

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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
  hi2c1.Init.Timing = 0x6000030D;
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
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x6000030D;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x32;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x8;
  sDate.Year = 0x14;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 108;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 54000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0xFFFF-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 3000000;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_8;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 3000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ASW1_Pin|ASW4_Pin|ASW2_Pin|ASW3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PE6_Pin|LED_BLUE_Pin|LED_GREEN_Pin|RANGE_UA_Pin
                          |RANGE_NA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC13_Pin|PC2_Pin|PC3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA0_Pin|PA3_Pin|PS_EN_Pin|USB_OTG_FS_VBUS_Pin
                          |LCD_RST_Pin|PA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|ADC_CONV_Pin|RANGE_MA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDMMC1_WP_GPIO_Port, SDMMC1_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ASW1_Pin ASW4_Pin ASW2_Pin ASW3_Pin
                           LED_GREEN_Pin RANGE_UA_Pin RANGE_NA_Pin */
  GPIO_InitStruct.Pin = ASW1_Pin|ASW4_Pin|ASW2_Pin|ASW3_Pin
                          |LED_GREEN_Pin|RANGE_UA_Pin|RANGE_NA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE6_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = PE6_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13_Pin PC2_Pin PC3_Pin */
  GPIO_InitStruct.Pin = PC13_Pin|PC2_Pin|PC3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0_Pin PA3_Pin PS_EN_Pin LCD_RST_Pin
                           PA15_Pin */
  GPIO_InitStruct.Pin = PA0_Pin|PA3_Pin|PS_EN_Pin|LCD_RST_Pin
                          |PA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OverCurrent_Pin BUTTON_MEASURING_Pin Extra_GPIO_Pin BUTTON_DOWN_Pin
                           BUTTON_PREV_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OverCurrent_Pin|BUTTON_MEASURING_Pin|Extra_GPIO_Pin|BUTTON_DOWN_Pin
                          |BUTTON_PREV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin ADC_CONV_Pin ADC_RST_Pin RANGE_MA_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|ADC_CONV_Pin|ADC_RST_Pin|RANGE_MA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_NEXT_Pin BUTTON_UP_Pin BUTTON_ESC_Pin BUTTON_ENTER_Pin
                           PD3_Pin PD4_Pin */
  GPIO_InitStruct.Pin = BUTTON_NEXT_Pin|BUTTON_UP_Pin|BUTTON_ESC_Pin|BUTTON_ENTER_Pin
                          |PD3_Pin|PD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC1_CD_Pin ADC_RVS_Pin */
  GPIO_InitStruct.Pin = SDMMC1_CD_Pin|ADC_RVS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC1_WP_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SDMMC1_WP_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  //MX_USB_HOST_Init();

  /* init code for LWIP */
  //MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
	send_uart("Task Default Task Init\n");

	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for (;;) {

		HAL_GPIO_TogglePin(LED_BLUE_PORT, LED_BLUE_PIN);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTaskUSB */
/**
* @brief Function implementing the myTaskUSB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskUSB */
void vTaskUSB(void *argument)
{
  /* USER CODE BEGIN vTaskUSB */
	send_uart("Task USB Init\n");

	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();


	MX_USB_HOST_Init();

	/* Infinite loop */
	for (;;) {

		MX_USB_HOST_Process();
		vTaskDelayUntil(&xLastWakeTime, xFrequency/10);

	}
  /* USER CODE END vTaskUSB */
}

/* USER CODE BEGIN Header_vTaskInputBuffer */
/**
* @brief Function implementing the myTaskInputBuff thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskInputBuffer */
void vTaskInputBuffer(void *argument)
{
  /* USER CODE BEGIN vTaskInputBuffer */
	send_uart("Task Input Buffer Init\n");

	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();

	send_uart("Task Input Buffer Init\n");

	uint8_t tempBuffer [128];

	uint8_t temp8 = 0;

	uint8_t charBuffer [128];

	uint32_t currentBufferSize = 0;

	double voltage = 0;
	double currentValue = 0;

	HAL_SPI_TransmitReceive_DMA(&hspi4, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE);
	//HAL_SPI_TransmitReceive(&hspi3, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE, 100);
	//HAL_SPI_TransmitReceive(&hspi3, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE, 100);
	//HAL_SPI_TransmitReceive(&hspi3, spiTxBuffer, spiRxBuffer, SPI_PACKET_SIZE, 100);
	HAL_GPIO_WritePin(FPGA_RDY_PORT, FPGA_RDY_PIN, GPIO_PIN_SET);


	vTaskDelayUntil(&xLastWakeTime, xFrequency/10);

	//ringbuff_reset(&inputBuffer);
	ringbuff_read(&inputBuffer_RB, tempBuffer, SPI_PACKET_SIZE);

	//HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);

	uint32_t temp32 = 0;

	uint16_t displayCounter = 512;
	uint32_t counterStats = 0;

	/* Infinite loop */
	for (;;) {

		//send_uart("Task Input Buffer Cycle\n");
		currentBufferSize = ringbuff_get_full(&inputBuffer_RB);

		if (counterStats >= 1024) {

			sprintf(charBuffer, "InBuff S: %d\n", currentBufferSize);
			send_uart(charBuffer);

			sprintf(charBuffer, "	Cnt OK-NOK-SPIs-SPIe-SPIHal %d-%d-%d-%d-%d\n", okCount, errorCount, spiCounter, spiCounterEnd, retValueSPI);
			send_uart(charBuffer);

			counterStats = 0;

		}

		counterStats++;

		if (currentBufferSize >= 40) {

			//send_uart("Task Input Buffer Data Ready\n");
			okCount++;

			// read 4 Bytes = 1 measurement
			ringbuff_read(&inputBuffer_RB, tempBuffer, 40);

			/*
			 if (tempBuffer[0] == 4)
			 voltage = (voltage*0.0195*1000000000);
			 else if (tempBuffer[0] == 2)
			 voltage = (voltage*0.0195*1000000);
			 else if (tempBuffer[0] == 1)
			 voltage = (voltage*0.0195*1000);
			 */

			temp32 = 0;

			if (displayCounter >= 25) {
				temp32 = (tempBuffer[1] << 16) | (tempBuffer[2] << 8) | (tempBuffer[3]);
				voltage = temp32;
				voltage = (voltage * 0.0195);

				if ((tempBuffer[0] >> 5) == 2)
					currentValue = (voltage / 1000000000);
				else if ((tempBuffer[0] >> 5) == 1)
					currentValue = (voltage / 1000000);
				else if ((tempBuffer[0] >> 5) == 0)
					currentValue = (voltage / 1000);
				else
					currentValue = (voltage);

				//sprintf(charBuffer, "	ADC: %d-V: %.2f-C: %6.12f\n", temp32, voltage, currentValue);
				sprintf(charBuffer, "	ADC: %d-V: %.2f-C: %6.12f-R: %d\n", temp32, voltage, currentValue, (tempBuffer[0] >> 5));
				//sprintf(charBuffer, "	Received Data %d-%d-%d-%d --- ADC VALUE: %d --- Voltage: %6.2f\n", tempBuffer[0], tempBuffer[1], tempBuffer[2], tempBuffer[3], temp32, voltage);
				send_uart(charBuffer);
				displayCounter = 0;
			}

			displayCounter++;

		} else {
			//send_uart("Task Input Buffer NO Data\n");
			retValue = HAL_SPI_GetState(&hspi4);

			if (retValue == HAL_SPI_STATE_BUSY_RX
					|| retValue == HAL_SPI_STATE_BUSY_TX
					|| retValue == HAL_SPI_STATE_BUSY_TX_RX
					|| retValue == HAL_SPI_STATE_BUSY) {
				//send_uart("SPI transfer running\n");
			} else {
				if (retValue == HAL_SPI_STATE_READY) {
					send_uart("SPI transfer not running - ready\n");
					//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
				}
				if (retValue == HAL_SPI_STATE_RESET) {
					send_uart("SPI transfer not running - reset\n");
					//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
				}
				if (retValue == HAL_SPI_STATE_ERROR) {
					send_uart("SPI transfer not running - error\n");
					//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
				}
				if (retValue == HAL_SPI_STATE_ABORT) {
					send_uart("SPI transfer not running - abort\n");
					//HAL_SPI_TransmitReceive_DMA(&hspi3, spiTxBuffer, spiRxBuffer, 4);
				}

				HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
				errorCount++;

			}

			vTaskDelayUntil(&xLastWakeTime, xFrequency / 10);

		}

	}
  /* USER CODE END vTaskInputBuffer */
}

/* USER CODE BEGIN Header_vTaskEthernet */
/**
* @brief Function implementing the myTaskEthernet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskEthernet */
void vTaskEthernet(void *argument)
{
  /* USER CODE BEGIN vTaskEthernet */
	send_uart("Task Ethernet Init\n");

	//MX_LWIP_Init();

	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END vTaskEthernet */
}

/* USER CODE BEGIN Header_vTaskUi */
/**
* @brief Function implementing the myTaskUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskUi */
void vTaskUi(void *argument)
{
  /* USER CODE BEGIN vTaskUi */
	send_uart("Task UI Init\n");



	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END vTaskUi */
}

/* USER CODE BEGIN Header_vTaskLcd */
/**
* @brief Function implementing the myTaskLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLcd */
void vTaskLcd(void *argument)
{
  /* USER CODE BEGIN vTaskLcd */
	send_uart("Task LCD Init\n");


	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();


	//u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]  //u8x8_byte_sw_i2c
	//u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R2, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = 256 bytes]  //u8x8_byte_sw_i2c // 180 degree rotation
	u8g2_Setup_st7528_i2c_nhd_c160100_f(&u8g2, U8G2_R2, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay); //[page buffer, size = full page XYZ bytes]  //u8x8_byte_sw_i2c
	u8g2_InitDisplay(&u8g2);

	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_SetFlipMode(&u8g2, 1);
	u8g2_ClearDisplay(&u8g2);
	//u8g2_Clear(&u8g2);
	u8g2_SetFontMode(&u8g2, 0);
	u8g2_SetContrast(&u8g2, 80);
	u8g2_SetFontDirection(&u8g2, 0);
	u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);//u8g2_font_4x6_tf		//u8g2_font_6x10_tf
	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawStr(&u8g2, 10, 10, "Hello World!");
	//u8g2_DrawLine(&u8g2, 5, 5, 155, 15);
	u8g2_SendBuffer(&u8g2);
	u8g2_UpdateDisplay(&u8g2);


	/* Infinite loop */
	for (;;) {

		screenInterface();

		setMeasuringRange();

		vTaskDelayUntil(&xLastWakeTime, xFrequency/10);

	}
  /* USER CODE END vTaskLcd */
}

/* USER CODE BEGIN Header_vTaskButtons */
/**
* @brief Function implementing the myTaskButtons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskButtons */
void vTaskButtons(void *argument)
{
  /* USER CODE BEGIN vTaskButtons */
	send_uart("Task Buttons Init\n");

	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END vTaskButtons */
}

/* USER CODE BEGIN Header_vTaskUart */
/**
* @brief Function implementing the myTaskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskUart */
void vTaskUart(void *argument)
{
  /* USER CODE BEGIN vTaskUart */
	send_uart("Task UART Init\n");

	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();


	/* Infinite loop */
	for (;;) {

		consoleInterface(0);
		setMeasuringRange();

		vTaskDelayUntil(&xLastWakeTime, xFrequency/10);

	}
  /* USER CODE END vTaskUart */
}

/* USER CODE BEGIN Header_vTaskSd */
/**
* @brief Function implementing the myTaskSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskSd */
void vTaskSd(void *argument)
{
  /* USER CODE BEGIN vTaskSd */
	send_uart("Task SD Init\n");

	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END vTaskSd */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
	  HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
	  HAL_Delay(250);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
