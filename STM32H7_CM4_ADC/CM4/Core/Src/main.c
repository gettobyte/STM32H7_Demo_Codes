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
#define ADC_BUF_LEN 1024u
#define CH_SAMPLES    (ADC_BUF_LEN/2u)   // 512 samples *per channel*

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream0_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel0_dma1_stream0_tc_1;
MDMA_LinkNodeTypeDef node_mdma_channel0_dma1_stream0_tc_2;
/* USER CODE BEGIN PV */

__attribute__((aligned(32))) uint16_t adc_buf[1024] = {0};


__attribute__((aligned(32))) volatile uint16_t ch5_data[CH_SAMPLES];
__attribute__((aligned(32))) volatile uint16_t ch6_data[CH_SAMPLES];

__attribute__((aligned(32))) volatile uint16_t ch5_data_mdma[CH_SAMPLES];
__attribute__((aligned(32))) volatile uint16_t ch6_data_mdma[CH_SAMPLES];

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



// Helper: average one interleaved block: [CH6,CH5,CH6,CH5,...]
// base  : pointer to first sample in the block
// count : number of half-words in the block (must be even)
static inline void avg_interleaved_block( uint16_t *base, uint32_t count,
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC2)
  {
	     ch_pair_full_ready = 1;   // full 512 samples/channel now ready
	     full_ready = 1;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	 if (hadc->Instance == ADC2)
	  {
			 ch_pair_half_ready = 1;   // you now have the first 256 samples/channel
		     half_ready = 1;
	  }
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t sum(uint32_t a, uint32_t b)
{
	return (a+b);
}
uint32_t diff(uint32_t a, uint32_t b)
{
	if (a>b)
		return (a-b);
	else
		return (b-a);
}

uint32_t mult(uint32_t a, uint32_t b)
{
	return (a*b);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t add_result, sub_result, mult_result;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* ADC calibration */
   if (HAL_ADCEx_Calibration_Start(&hadc2,
                                   ADC_CALIB_OFFSET,
                                   ADC_SINGLE_ENDED) != HAL_OK)
   {
     Error_Handler();
   }

   /* Start ADC2 in DMA circular mode */
   if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buf, 1024) != HAL_OK)
   {
     Error_Handler();
   }

   HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream0_tc_0,
                     (uint32_t)&adc_buf[1],      // CH5 starts at odd index
                     (uint32_t)&ch5_data[0],
                     1,                           // BlockDataLength = 2 bytes (half-word)
                     512);                        // BlockCount     = 512 elements

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	   if (ch_pair_half_ready)
	   {
	  // First half of the interleaved buffer: adc_buf[0 .. 511]
		// -> 256 samples per channel into [0..255]
//		deinterleave_block(&adc_buf[0], ADC_BUF_LEN/2u,
//						   ch5_data, ch6_data, 0u);

		avg_interleaved_block(&adc_buf[0], ADC_BUF_LEN/2,
													   (uint16_t*)&ch5_avg_half, (uint16_t*)&ch6_avg_half);

   ch_pair_half_ready = 0;
   // ch5_data[0..255] and ch6_data[0..255] just filled
   // e.g., compute quick stats on the first half
		   }

		   if (ch_pair_full_ready) {

			   // Second half of the interleaved buffer: adc_buf[512 .. 1023]
			 		      // -> 256 samples per channel into [256..511]
//			 		      deinterleave_block(&adc_buf[ADC_BUF_LEN/2u], ADC_BUF_LEN/2u,
//			 		                         ch5_data, ch6_data, CH_SAMPLES/2u);

			 		     uint16_t a5 = 0, a6 = 0;
						 avg_interleaved_block(&adc_buf[0],ADC_BUF_LEN/2, &a5, &a6);
						 uint16_t b5 = 0, b6 = 0;
						 avg_interleaved_block(&adc_buf[ADC_BUF_LEN/2], ADC_BUF_LEN/2, &b5, &b6);
						 ch5_avg_full = (uint16_t)(((uint32_t)a5 + b5) / 2u);
						 ch6_avg_full = (uint16_t)(((uint32_t)a6 + b6) / 2u);


		       ch_pair_full_ready = 0;

		   }





	  add_result = sum(32,48);
	  sub_result = diff(50, 25);
	  mult_result = mult(20,10);

	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.BufferTransferLength = 512;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_8BEATS;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_16BEATS;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBlockAddressOffset = 2;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBlockAddressOffset = 0;
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
  nodeConfig.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_8BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_8BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 2;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[1];
  nodeConfig.DstAddress = (uint32_t)&ch5_data[0];
  nodeConfig.BlockDataLength = 1;
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
  nodeConfig.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_RIGHT;
  nodeConfig.Init.BufferTransferLength = 512;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_8BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_16BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 2;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = (uint32_t)&adc_buf[0];
  nodeConfig.DstAddress = (uint32_t)&ch6_data[0];
  nodeConfig.BlockDataLength = 1;
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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
