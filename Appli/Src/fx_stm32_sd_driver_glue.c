/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

#include "fx_stm32_sd_driver.h"
#include "main.h"
#include "stm32n6570_discovery_sd.h"

TX_SEMAPHORE sd_tx_semaphore;
TX_SEMAPHORE sd_rx_semaphore;

/* sv: use BSP glue code, not HAL */
#define FX_STM32_SD_USE_HAL    0

#if (FX_STM32_SD_USE_HAL == 1)
SD_HandleTypeDef hsd1;
#endif

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief Initializes the SD IP instance
* @param UINT instance SD instance to initialize
* @retval 0 on success error value otherwise
*/
INT fx_stm32_sd_init(UINT instance)
{
  INT ret = 0;

  /* USER CODE BEGIN PRE_FX_SD_INIT */
  UNUSED(instance);
  /* USER CODE END PRE_FX_SD_INIT */

  printf("FXG INIT SD %d\r\n", instance);

#if (FX_STM32_SD_USE_HAL == 1)
  hsd1.Instance = SDMMC2;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
#else
  printf("FXG INIT SD2 %d\r\n", instance);
  HAL_SD_MspInit(&hsd_sdmmc[0]);

  BSP_SD_Init(instance);
#endif

  /* USER CODE BEGIN POST_FX_SD_INIT */
//  HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
//  HAL_SD_ConfigSpeedBusOperation(&hsd1, SDMMC_SPEED_MODE_AUTO);
//  printf("CARD TYPE %d speed %d \r\n", hsd1.SdCard.CardType, hsd1.SdCard.CardSpeed );

  /* USER CODE END POST_FX_SD_INIT */

  return ret;
}

/**
* @brief Deinitializes the SD IP instance
* @param UINT instance SD instance to deinitialize
* @retval 0 on success error value otherwise
*/
INT fx_stm32_sd_deinit(UINT instance)
{
  INT ret = 0;

  /* USER CODE BEGIN PRE_FX_SD_DEINIT */
  UNUSED(instance);
  /* USER CODE END PRE_FX_SD_DEINIT */
#if (FX_STM32_SD_USE_HAL == 1)
  if(HAL_SD_DeInit(&hsd1) != HAL_OK)
  {
    ret = 1;
  }
#else
  BSP_SD_DeInit(instance);
#endif

  /* USER CODE BEGIN POST_FX_SD_DEINIT */

  /* USER CODE END POST_FX_SD_DEINIT */

  return ret;
}

/**
* @brief Check the SD IP status.
* @param UINT instance SD instance to check
* @retval 0 when ready 1 when busy
*/
INT fx_stm32_sd_get_status(UINT instance)
{
  INT ret = 0;

  /* USER CODE BEGIN PRE_GET_STATUS */
  UNUSED(instance);
  /* USER CODE END PRE_GET_STATUS */

#if (FX_STM32_SD_USE_HAL == 1)
  if(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
#else
  if(BSP_SD_GetCardState(instance) != SD_TRANSFER_OK)
#endif
  {
    ret = 1;
  }

  /* USER CODE BEGIN POST_GET_STATUS */

  /* USER CODE END POST_GET_STATUS */

  return ret;
}

/**
* @brief Read Data from the SD device into a buffer.
* @param UINT instance SD IP instance to read from.
* @param UINT *buffer buffer into which the data is to be read.
* @param UINT start_block the first block to start reading from.
* @param UINT total_blocks total number of blocks to read.
* @retval 0 on success error code otherwise
*/
INT fx_stm32_sd_read_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks)
{
  INT ret = 0;
  /* USER CODE BEGIN PRE_READ_BLOCKS */
  UNUSED(instance);
  /* USER CODE END PRE_READ_BLOCKS */

#if (FX_STM32_SD_USE_HAL == 1)
  if(HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks) != HAL_OK)
  {
    ret = 1;
  }
#else
  if (BSP_SD_ReadBlocks_DMA( instance, buffer, start_block, total_blocks)!= BSP_ERROR_NONE)
  {
    ret = 1;
  }
#endif

  /* USER CODE BEGIN POST_READ_BLOCKS */

  /* USER CODE END POST_READ_BLOCKS */

  return ret;
}

/**
* @brief Write data buffer into the SD device.
* @param UINT instance SD IP instance to write into.
* @param UINT *buffer buffer to write into the SD device.
* @param UINT start_block the first block to start writing into.
* @param UINT total_blocks total number of blocks to write.
* @retval 0 on success error code otherwise
*/
INT fx_stm32_sd_write_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks)
{
  INT ret = 0;
  /* USER CODE BEGIN PRE_WRITE_BLOCKS */
  UNUSED(instance);
  /* USER CODE END PRE_WRITE_BLOCKS */

#if (FX_STM32_SD_USE_HAL == 1)
  if(HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks) != HAL_OK)
  {
    ret = 1;
  }
#else
    if (BSP_SD_WriteBlocks_DMA( instance, buffer, start_block, total_blocks)!= BSP_ERROR_NONE)
    {
      ret = 1;
    }
#endif

  /* USER CODE BEGIN POST_WRITE_BLOCKS */

  /* USER CODE END POST_WRITE_BLOCKS */

  return ret;
}

#if (FX_STM32_SD_USE_HAL == 1)
/**
* @brief SD DMA Tx Transfer completed callbacks
* @param instance the sd instance
* @retval None
*/
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  /* USER CODE BEGIN PRE_TX_CMPLT */
//	printf("ack\r\n");
  /* USER CODE END PRE_TX_CMPLT */

//printf("a %d %d %d\r\n", hsd->State, HAL_SD_GetCardState(hsd) , hsd->TxXferSize);
  tx_semaphore_put(&sd_tx_semaphore);
  /* USER CODE BEGIN POST_TX_CMPLT */

  /* USER CODE END POST_TX_CMPLT */
}
/**
  * @brief  Initializes the SD MSP.
  * @param  hsd  SD handle
  * @retval None
  */
#else

/**
  * @brief BSP Tx Transfer completed callbacks
  * @param  instance     SD instance
  * @retval None
  */
void BSP_SD_WriteCpltCallback(uint32_t instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(instance);
  tx_semaphore_put(&sd_tx_semaphore);
}
#endif


void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
    GPIO_InitTypeDef gpio_init_structure = {0};

    HAL_PWREx_EnableVddIO5();
    /* Enable SDMMC clock */
    __HAL_RCC_SDMMC2_CLK_ENABLE();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Common GPIO configuration */
    gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull      = GPIO_PULLUP;
    gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_structure.Alternate = GPIO_AF11_SDMMC2;

    /* D2-CLK-CMD-D0-D1*/
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  | GPIO_PIN_5 ;
    HAL_GPIO_Init(GPIOC, &gpio_init_structure);

    /* D3*/
    gpio_init_structure.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOE, &gpio_init_structure);

    /* NVIC configuration for SDMMC2 interrupts */
    HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
}

/**
  * @brief  DeInitializes the SD MSP.
  * @param  hsd  SD handle
  * @retval None
  */
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef gpio_init_structure;

  if(hsd->Instance==SDMMC2)
  {
    /* Disable NVIC for SDIO interrupts */
    HAL_NVIC_DisableIRQ(SDMMC2_IRQn);

    /* GPIOC configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  | GPIO_PIN_5 ;
    HAL_GPIO_DeInit(GPIOC, gpio_init_structure.Pin);

    /* GPIOD configuration */
    gpio_init_structure.Pin = GPIO_PIN_4;
    HAL_GPIO_DeInit(GPIOD, gpio_init_structure.Pin);

    /* Disable SDMMC2 clock */
    __HAL_RCC_SDMMC2_CLK_DISABLE();
  }
}



void SDMMC2_IRQHandler(void)
{
#if (FX_STM32_SD_USE_HAL == 1)
  HAL_SD_IRQHandler(&hsd_sdmmc[0]);
#else
  BSP_SD_IRQHandler(0);
#endif
}


#if (FX_STM32_SD_USE_HAL == 1)
/**
* @brief SD DMA Rx Transfer completed callbacks
* @param instance the sd instance
* @retval None
*/
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  /* USER CODE BEGIN PRE_RX_CMPLT */

  /* USER CODE END PRE_RX_CMPLT */

  tx_semaphore_put(&sd_rx_semaphore);

  /* USER CODE BEGIN POST_RX_CMPLT */

  /* USER CODE END POST_RX_CMPLT */
}

/* USER CODE BEGIN 1 */
#else

/**
  * @brief BSP Rx Transfer completed callbacks
  * @param  instance     SD instance
  * @retval None
  */
void BSP_SD_ReadCpltCallback(uint32_t instance)
{
  /* Prevent unused argument(s) compilation warning */
	tx_semaphore_put(&sd_rx_semaphore);
}

#endif




/* USER CODE END 1 */
