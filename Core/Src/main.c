/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARDUINO_CS_PORT      GPIOA
#define ARDUINO_CS_PIN     GPIO_PIN_6

#define DEFAULT_STARTUP_VAL (0x80)
#define USE_DAC 1
#define RECORDING_SIZE_MIC 1000
#define DFSDM_BUFFER_SIZE 1000
#define AUDIO_BUFFER_SIZE 1000
#define HEADER_SIZE 44

#define MAX_RECORDING_LENGTH 128044

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// General Program
int error = 0;

// Microphone
int recording_audio = 0;
int finished_recording = 0;
int16_t recording[RECORDING_SIZE_MIC];
int32_t dfsdm_buffer[DFSDM_BUFFER_SIZE * 2];

int mic_transfer_complete = 0;
int mic_half_transfer = 0;
int transfer_position = 0;
int start_recording_process = 0;

uint8_t header_data[] = {
    0x52, 0x49, 0x46, 0x46, 0x24, 0xE8, 0x03, 0x00,
    0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20,
    0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x80, 0x3E, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00,
    0x02, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61,
    0x00, 0xE8, 0x03, 0x00
};

int bytes_written_to_file = 0;
// AudioPlayback
int audio_playing = 0;
int audio_position = 44;
int song_length = 0;
int song_length_total = 0;
int buffer_half = 0;
int buffer_complete = 0;
int samples_played = 44;

int16_t playback_buffer[AUDIO_BUFFER_SIZE*2];


char *recordingPath = "";
char *LIGHTONAUDIO = "/sounds/light_on.wav";
char *LIGHTOFFAUDIO = "/sounds/light_off.wav";
char *DOOROPENAUDIO = "/sounds/door_open.wav";
char *DOORCLOSEAUDIO = "/sounds/door_close.wav";
char *MICON = "/sounds/listen_on.wav";
char *MICOFF = "/sounds/stop_listen.wav";

start_playback_process = 0;

UINT bytesRead;
UINT bytesWritten;
UINT bytes_to_read = AUDIO_BUFFER_SIZE * 2;
UINT bytes_to_record = RECORDING_SIZE_MIC * 2;

//SD Card Variables
FATFS       FatFs;
FRESULT     fres;
FIL 		file;
uint32_t 	totalSpace, freeSpace;
BYTE* 		loaded_song;

//UART Variables
uint8_t data[64];
int data_ready = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void start_recording_from_mic(){
	if (!recording_audio){
		printf("Starting Recording Process\r\n");
		if (f_open(&file, "recording1.wav", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
			printf("Error opening file for writing.\n\r");
		}
		if (f_write(&file, header_data, sizeof(header_data), &bytesWritten) != FR_OK) {
			printf("Error writing header to file.\n");
		}
		recording_audio = 1;
		start_recording_process = 0;
	}
}
void start_audio_playback(){
	if (!audio_playing){
		open_SD_card_song(recordingPath);
		read_SD_card_song_initial();
		printf("Starting Playback \r\n");
		audio_playing = 1;
		start_playback_process = 0;
		song_length_total = song_length;
		LL_TIM_EnableIT_UPDATE(TIM2);
		LL_TIM_EnableCounter(TIM2);
		NVIC_EnableIRQ(TIM2_IRQn);
	}
}
void mount_SD_card(void){

    //Mount the SD Card
	fres = f_mount(&FatFs, "", 1);    //1=mount now
	if (fres != FR_OK){
		printf("No SD Card found : (%i)\r\n", fres);
        return;
	}
	printf("SD Card Mounted Successfully\r\n");

    //Read the SD Card Total size and Free Size
    FATFS *pfs;
    DWORD fre_clust;

    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

    printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\r\n", totalSpace, freeSpace);

}

void unmount_SD_card(void){
	f_mount(NULL, "", 0);
	printf("SD Card Unmounted Successfully\r\n");
}

void read_SD_card_song_initial(){
    if (f_read(&file, &playback_buffer[0], AUDIO_BUFFER_SIZE*2*2, &bytesRead) == FR_OK) {
            // Check if the expected number of bytes were read
    	if (bytesRead == AUDIO_BUFFER_SIZE*2*2) {
            	printf("Audio Loaded, %d bytes.\r\n", bytesRead);
            	audio_position += AUDIO_BUFFER_SIZE*2*2;
    	} else {
                // Handle the case where not all bytes were read
    	}
	}

}

void read_SD_card_song_at_position(int buffer_position){
	if ( audio_position + AUDIO_BUFFER_SIZE * 2 <= song_length_total){
		if (f_read(&file, &playback_buffer[buffer_position], bytes_to_read, &bytesRead) == FR_OK) {
						// Check if the expected number of bytes were read
			if (bytesRead == AUDIO_BUFFER_SIZE * 2) {
				audio_position += AUDIO_BUFFER_SIZE*2;
				//printf("Audio_position: %d\r\n", audio_position);
			} else {
							// Handle the case where not all bytes were read
			}
		}
	}
}


void open_SD_card_song(const char *mypath){

	FILINFO fileInfo;
	const TCHAR *songpath = _T(mypath);

	fres = f_open(&file, songpath, FA_READ);
	if (fres != FR_OK) {
		printf("Failed opening file\r\n");
		return;
	}

    fres = f_stat(songpath, &fileInfo);
    if (fres == FR_OK) {
        // Print file size
    	printf("File size: %lu bytes\r\n", fileInfo.fsize);
    	song_length = fileInfo.fsize;
    }
    printf("path: %s\r\n", songpath);

}

void close_SD_card_song(void){
	fres = f_close(&file);
	if (fres != FR_OK) {
		return;
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  // UART Interrupt Init
  // LIGHT
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, data, 64);

  // DOOR
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, data, 64);


  HAL_UARTEx_ReceiveToIdle_IT(&huart1, data, 64);

  // set status of light to off on start (Opposite than LED due to relay)
  HAL_GPIO_WritePin(lightStatus_GPIO_Port, lightStatus_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(doorStatus_GPIO_Port, doorStatus_Pin, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Initilize SD Card and Play start up Sound*/

  mount_SD_card();

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x7FF);

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1,dfsdm_buffer,DFSDM_BUFFER_SIZE * 2) != HAL_OK){
	  printf("Failed to start DFSDM");
  }


  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (start_playback_process && !audio_playing){
		  start_audio_playback();
	  }
	  if (start_recording_process && !start_playback_process && !audio_playing){
		  start_recording_from_mic();
	  }
	  if (recording_audio){
		  /*
		  if ((transfer_position == RECORDING_SIZE_MIC)){
			  __HAL_DMA_DISABLE(&hdma_dfsdm1_flt1);

			  printf("\r\n\n");
			  for (int i = 0; i < RECORDING_SIZE_MIC; i++){
				  printf("%04X ", ((recording[i] & 0xFF) << 8) | ((recording[i] >> 8) & 0xFF));
			  }
			  printf("\r\n\n");

			  return 1;
		  }
		  */
		  if (finished_recording || bytes_written_to_file >= MAX_RECORDING_LENGTH){
			  printf("Finished Recording Audio\r\n");
			  recording_audio = 0;
			  finished_recording = 0;
			  mic_half_transfer = 0;
			  mic_transfer_complete = 0;
			  bytes_written_to_file = 0;
			  f_close(&file);

			  HAL_UART_Transmit(&huart3, header_data, HEADER_SIZE, HAL_MAX_DELAY);
			  int bytes_read = 44;
			  uint8_t block[10000];
			  open_SD_card_song("recording1.wav");
			  while (bytes_read < 128044){
				  if (f_read(&file, &block[0], 10000, &bytesRead) == FR_OK) {
				 	HAL_UART_Transmit(&huart3, block, 10000, HAL_MAX_DELAY);
				 	//printf("Sent Data to Arduino\r\n");
				 	bytes_read += 10000;
				  }
			  }
			  f_close(&file);
		  }
		  if(mic_half_transfer){
			  for (int i = 0; i < DFSDM_BUFFER_SIZE; i++){
				  recording[i] = SaturaLH((dfsdm_buffer[i] >> 8), -32768, 32767);
			  }
			  if (f_write(&file, recording, bytes_to_record, &bytesWritten) != FR_OK) {
				  printf("Error Writing To File 1.\n");
				  f_close(&file);
				  return 1;
			  }
			  bytes_written_to_file+= DFSDM_BUFFER_SIZE * 2;
			  mic_half_transfer = 0;
		  }
		  else if (mic_transfer_complete){
			  for (int i = DFSDM_BUFFER_SIZE; i < DFSDM_BUFFER_SIZE * 2; i++){
				  recording[i - DFSDM_BUFFER_SIZE] = SaturaLH((dfsdm_buffer[i] >> 8), -32768, 32767);
			  }
			  if (f_write(&file, recording, bytes_to_record, &bytesWritten) != FR_OK) {
				  printf("Error Writing to File 2.\n");
				  f_close(&file);
				  return 1;
			  }
			  bytes_written_to_file+= DFSDM_BUFFER_SIZE * 2;
			  mic_transfer_complete = 0;
		  }
	  }
	  if (audio_playing){
		  if (audio_position >= song_length_total - AUDIO_BUFFER_SIZE * 2){
			  NVIC_DisableIRQ(TIM2_IRQn);
			  LL_TIM_DisableCounter(TIM2);
			  printf("Song Finished at %d\r\n", audio_position);
			  f_close(&file);
			  audio_playing = 0;
			  audio_position = 44;
			  samples_played = 44;
			  buffer_half = 0;
			  buffer_complete = 0;
		  }
		  if (buffer_half){
			  read_SD_card_song_at_position(0);
			  buffer_half = 0;
		  }
		  if (buffer_complete){
			  read_SD_card_song_at_position(AUDIO_BUFFER_SIZE);
			  buffer_complete = 0;
		  }
	  }
	  if (error){
		  printf("There has been an error\r\n Terminating Program\r\n");
		  unmount_SD_card();
		  return -1;
	  }

  }
  unmount_SD_card();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 125;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 31;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x02;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, doorStatus_Pin|lightStatus_Pin|NANOcsPIn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ChipSelectSD_GPIO_Port, ChipSelectSD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userControl_Pin */
  GPIO_InitStruct.Pin = userControl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userControl_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : doorStatus_Pin lightStatus_Pin NANOcsPIn_Pin */
  GPIO_InitStruct.Pin = doorStatus_Pin|lightStatus_Pin|NANOcsPIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ChipSelectSD_Pin */
  GPIO_InitStruct.Pin = ChipSelectSD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ChipSelectSD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Regular conversion complete callback.
  * @note   In interrupt mode, user has to read conversion value in this function
            using HAL_DFSDM_FilterGetRegularValue.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*
	if (!audio_playing){
	  open_SD_card_song(recordingPath);
	  read_SD_card_song_initial();
	  printf("Starting Playback \r\n");
	  audio_playing = 1;
	  song_length_total = song_length;
	  LL_TIM_EnableIT_UPDATE(TIM2);
	  LL_TIM_EnableCounter(TIM2);
	  NVIC_EnableIRQ(TIM2_IRQn);
	}
	*/

/*
	HAL_GPIO_WritePin(ARDUINO_CS_PORT, ARDUINO_CS_PIN, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, header_data, HEADER_SIZE, HAL_MAX_DELAY) != HAL_OK){
		printf("SPI Transfer Failed\r\n");
		HAL_GPIO_WritePin(ARDUINO_CS_PORT, ARDUINO_CS_PIN, GPIO_PIN_SET);
		error = 1;
	} else {
		printf("Sent message\r\n");
		HAL_GPIO_WritePin(ARDUINO_CS_PORT, ARDUINO_CS_PIN, GPIO_PIN_SET);
	}
*/
}


void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if (recording_audio){
		mic_half_transfer = 1;
	}

}
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if( recording_audio){
		mic_transfer_complete = 1;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (strncmp((char*)data, "MIC_ON", strlen("MIC_ON")) == 0) {
		printf("MIC_ON\r\n");
		recordingPath = MICON;
		start_playback_process = 1;
		start_recording_process = 1;
	}
	if (strncmp((char*)data, "LIGHT_ON", strlen("LIGHT_ON")) == 0) {
		printf("LIGHT_ON\r\n");
		recordingPath = LIGHTONAUDIO;
		start_playback_process = 1;
	}
	if (strncmp((char*)data, "LIGHT_OFF", strlen("LIGHT_OFF")) == 0) {
		printf("LIGHT_OFF\r\n");
		recordingPath = LIGHTOFFAUDIO;
		start_playback_process = 1;
	}
	if (strncmp((char*)data, "DOOR_OPEN", strlen("DOOR_OPEN")) == 0) {
		printf("DOOR_OPEN\r\n");
		recordingPath = DOOROPENAUDIO;
		start_playback_process = 1;
	}
	if (strncmp((char*)data, "DOOR_CLOSE", strlen("DOOR_CLOSE")) == 0) {
		printf("DOOR_CLOSE\r\n");
		recordingPath = DOORCLOSEAUDIO;
		start_playback_process = 1;
	}
	HAL_UARTEx_ReceiveToIdle_IT(huart, data, 64);
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
