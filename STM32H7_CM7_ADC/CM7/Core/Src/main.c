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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream0_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel0_dma1_stream0_tc_1;
MDMA_LinkNodeTypeDef node_mdma_channel0_dma1_stream0_tc_2;
MDMA_HandleTypeDef hmdma_mdma_channel1_sw_0;
MDMA_LinkNodeTypeDef node_mdma_channel1_sw_1;
MDMA_LinkNodeTypeDef node_mdma_channel1_sw_2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RAM_D2_NL  __attribute__((section(".ram_d2_noload"), aligned(32)))



// Interleaved DMA buffer: [CH6,CH5, CH6,CH5, ...]
#define ADC_BUF_LEN   1024u              // must be multiple of 4
#define CH_SAMPLES    (ADC_BUF_LEN/2u)   // 512 samples *per channel*

RAM_D2_NL  volatile uint16_t ch5_buf[CH_SAMPLES] = {0};
RAM_D2_NL volatile uint16_t ch6_buf[CH_SAMPLES];

RAM_D2_NL volatile uint16_t ch5_data_mdma[CH_SAMPLES] = {0};
RAM_D2_NL volatile uint16_t ch6_data_mdma[CH_SAMPLES] = {0};

// Flags to know when data is ready
volatile uint8_t ch_pair_half_ready = 0;   // first 256 samples per channel
volatile uint8_t ch_pair_full_ready = 0;   // full 512 samples per channel


// base: pointer to start of interleaved half-buffer
// count: number of half-words in this block (must be even)
// dst_index: where to start writing in ch5_data/ch6_data
static inline void deinterleave_block(volatile uint16_t *base,
                                      uint32_t count,
                                      volatile uint16_t *dst5,
                                      volatile uint16_t *dst6,
                                      uint32_t dst_index)
{
    // Even = CH6, Odd = CH5  (Rank1=CH6, Rank2=CH5)
    for (uint32_t i = 0; i < count; i += 2) {
        dst6[dst_index] = base[i + 0];  // CH6
        dst5[dst_index] = base[i + 1];  // CH5
        dst_index++;
    }
}



// ---- Results (read these from main loop) ----
volatile uint16_t ch5_avg_half = 0;   // latest half-buffer average for CH5
volatile uint16_t ch6_avg_half = 0;   // latest half-buffer average for CH6
volatile uint16_t ch5_avg_full = 0;   // average over the whole buffer (optional)
volatile uint16_t ch6_avg_full = 0;
volatile uint8_t  half_ready = 0;     // 1 when half result updated
volatile uint8_t  full_ready = 0;     // 1 when full result updated

volatile uint8_t mdma_transfer_done = 0;
volatile uint8_t mdma_error_flag = 0;

// Helper: average one interleaved block: [CH6,CH5,CH6,CH5,...]
// base  : pointer to first sample in the block
// count : number of half-words in the block (must be even)
static inline void avg_interleaved_block(volatile  uint16_t *base, uint32_t count,
                                         uint16_t *avg_ch5, uint16_t *avg_ch6)
{
    uint32_t sum5 = 0, sum6 = 0;
    // even = CH6, odd = CH5  (because Rank1=CH6, Rank2=CH5)
    for (uint32_t i = 0; i < count; i += 2) {
        sum6 += base[i + 0];
        sum5 += base[i + 1];
    }
    uint32_t pairs = count >> 1;      // count/2 samples per channel
    *avg_ch5 = (uint16_t)(sum5 / pairs);
    *avg_ch6 = (uint16_t)(sum6 / pairs);
}

/* Two-sample buffer in rank order: [0]=CH10(PC0), [1]=CH16(PA0) */
RAM_D2_NL volatile uint16_t adc_buf[1024] = {0};
static inline void ADC_ReadBuffer_Safely(void)
{
    /* CM7 D-cache: invalidate before CPU reads the DMA-updated buffer */
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc_buf, sizeof(adc_buf));
}

int ADC_DMA_Half_Flag = 0;
int ADC_DMA_Full_Flag, MDMA_Block_Flag, MDMA_Buffer_Flag = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	  if (hadc->Instance == ADC1) {

		//  SCB_InvalidateDCache_by_Addr((uint32_t*)adc_buf, sizeof(adc_buf));
		  ADC_DMA_Full_Flag += 1;
		      ch_pair_full_ready = 1;   // full 512 samples/channel now ready
		      full_ready = 1;
	    }

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {

	  if (hadc->Instance == ADC1) {

		//  SCB_InvalidateDCache_by_Addr((uint32_t*)adc_buf, sizeof(adc_buf));

		  ADC_DMA_Half_Flag += 1;
		    ch_pair_half_ready = 1;   // you now have the first 256 samples/channel
		 		     half_ready = 1;

	    }

}

uint32_t mdma_blocks, mdma_buffers = 0;

void HAL_MDMA_BufferCpltCallback(MDMA_HandleTypeDef *hdma) // “buffer done” (BFTCIF)
{
	 // MDMA wrote the destinations: make CPU see the data
	MDMA_Buffer_Flag +=1;
	    mdma_buffers++;
}
void HAL_MDMA_BlockCpltCallback(MDMA_HandleTypeDef *hdma) // “block done”  (BTIF)
{
	MDMA_Block_Flag += 1;
	 mdma_blocks++;

}

HAL_StatusTypeDef ADC1_Status;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  assert(hdma_adc1.Instance == DMA1_Stream0);   // <— must be true


  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);

  ADC1_Status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 1024) != HAL_OK) {
      Error_Handler();
  }

  HAL_MDMA_Start_IT(&hmdma_mdma_channel1_sw_0,
                    (uint32_t)&adc_buf[0],      // CH6 starts at even index
                    (uint32_t)&ch6_buf[0],
                    2,                           // BlockDataLength = 2 bytes (half-word)
                    512);                        // BlockCount     = 512 elements


//  HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream0_tc_0,
//                    (uint32_t)&adc_buf[0],      // CH6 starts at even index
//                    (uint32_t)&ch6_buf[0],
//                    2,                           // BlockDataLength = 2 bytes (half-word)
//                    512);                        // BlockCount     = 512 elements


 // HAL_MDMA_GenerateSWRequest(&hmdma_mdma_channel0_dma1_stream0_tc_0);  // <— should fire your MDMA IRQ

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */
  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");
  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_YELLOW);
  BSP_LED_On(LED_RED);
  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_MDMA_GenerateSWRequest(&hmdma_mdma_channel1_sw_0);
	  HAL_Delay(1000);

	   if (half_ready) {
			  // First half: adc_buf[0 .. (ADC_BUF_LEN/2 - 1)]
				 		     avg_interleaved_block(&adc_buf[0], ADC_BUF_LEN/2,
				 		                           (uint16_t*)&ch5_avg_half, (uint16_t*)&ch6_avg_half);
	       half_ready = 0;
	       // ch5_avg_half / ch6_avg_half contain averages of the last half-buffer
	       // ...use or print them...
	   }

	   if (full_ready) {
					     //half_ready = 0;
//
//			     	     // Optional: whole-buffer averages (cheap: just sum both halves)
//			     	     uint16_t a5 = 0, a6 = 0;
//			     	     avg_interleaved_block(&adc_buf[0], ADC_BUF_LEN/2, &a5, &a6);
//			     	     uint16_t b5 = 0, b6 = 0;
//			     	     avg_interleaved_block(&adc_buf[ADC_BUF_LEN/2], ADC_BUF_LEN/2, &b5, &b6);
//			     	     ch5_avg_full = (uint16_t)(((uint32_t)a5 + b5) / 2u);
//			     	     ch6_avg_full = (uint16_t)(((uint32_t)a6 + b6) / 2u);


	       full_ready = 0;
	       // ch5_avg_full / ch6_avg_full contain averages over the full 1024-sample buffer
	   }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_dma1_stream0_tc_0
  *   node_mdma_channel0_dma1_stream0_tc_1
  *   node_mdma_channel0_dma1_stream0_tc_2
  *   hmdma_mdma_channel1_sw_0
  *   node_mdma_channel1_sw_1
  *   node_mdma_channel1_sw_2
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */
  MDMA_LinkNodeConfTypeDef nodeConfig;

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_dma1_stream0_tc_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_dma1_stream0_tc_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.TransferTriggerMode = MDMA_FULL_TRANSFER;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.BufferTransferLength = 512;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBlockAddressOffset = 2;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBlockAddressOffset = 0;

  hmdma_mdma_channel0_dma1_stream0_tc_0.XferBlockCpltCallback =  HAL_MDMA_BlockCpltCallback;
  hmdma_mdma_channel0_dma1_stream0_tc_0.XferBufferCpltCallback =  HAL_MDMA_BufferCpltCallback;

  if (HAL_MDMA_Init(&hmdma_mdma_channel0_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_dma1_stream0_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_FULL_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 2;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[0];
  nodeConfig.DstAddress = (uint32_t)&ch6_buf[0];
  nodeConfig.BlockDataLength = 2;
  nodeConfig.BlockCount = 512;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_dma1_stream0_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_dma1_stream0_tc_1 */

  /* USER CODE END mdma_channel0_dma1_stream0_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_dma1_stream0_tc_0, &node_mdma_channel0_dma1_stream0_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_FULL_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 2;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[1];
  nodeConfig.DstAddress = (uint32_t)&ch5_buf[0];
  nodeConfig.BlockDataLength = 2;
  nodeConfig.BlockCount = 512;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_dma1_stream0_tc_2, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_dma1_stream0_tc_2 */

  /* USER CODE END mdma_channel0_dma1_stream0_tc_2 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_dma1_stream0_tc_0, &node_mdma_channel0_dma1_stream0_tc_2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel0_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel1 */
  /* Configure MDMA request hmdma_mdma_channel1_sw_0 on MDMA_Channel1 */
  hmdma_mdma_channel1_sw_0.Instance = MDMA_Channel1;
  hmdma_mdma_channel1_sw_0.Init.Request = MDMA_REQUEST_SW;
  hmdma_mdma_channel1_sw_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel1_sw_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel1_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel1_sw_0.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  hmdma_mdma_channel1_sw_0.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  hmdma_mdma_channel1_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  hmdma_mdma_channel1_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  hmdma_mdma_channel1_sw_0.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  hmdma_mdma_channel1_sw_0.Init.BufferTransferLength = 512;
  hmdma_mdma_channel1_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel1_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel1_sw_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel1_sw_0.Init.DestBlockAddressOffset = 0;


  hmdma_mdma_channel1_sw_0.XferBlockCpltCallback =  HAL_MDMA_BlockCpltCallback;
  hmdma_mdma_channel1_sw_0.XferBufferCpltCallback =  HAL_MDMA_BufferCpltCallback;

  if (HAL_MDMA_Init(&hmdma_mdma_channel1_sw_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_SW;
  nodeConfig.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[0];
  nodeConfig.DstAddress = (uint32_t)&ch6_buf[0];
  nodeConfig.BlockDataLength = 2;
  nodeConfig.BlockCount = 512;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel1_sw_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel1_sw_1 */

  /* USER CODE END mdma_channel1_sw_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_sw_0, &node_mdma_channel1_sw_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_SW;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[1];
  nodeConfig.DstAddress = (uint32_t)&ch5_buf[0];
  nodeConfig.BlockDataLength = 2;
  nodeConfig.BlockCount = 512;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel1_sw_2, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel1_sw_2 */

  /* USER CODE END mdma_channel1_sw_2 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_sw_0, &node_mdma_channel1_sw_2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel1_sw_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
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
