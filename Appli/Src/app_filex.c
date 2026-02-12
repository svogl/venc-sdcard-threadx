
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_filex.c
 * @author  MCD Application Team
 * @brief   FileX applicative file
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
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_sd.h"
#include "stm32n6xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* Main thread stack size */
#define FX_APP_THREAD_STACK_SIZE 4 * 1024
/* Main thread priority */
#define FX_APP_THREAD_PRIO 10

/* USER CODE BEGIN PD */
#define DEFAULT_QUEUE_LENGTH 1280
#define SD_DETECT_Pin GPIO_PIN_12
#define SD_DETECT_GPIO_Port GPION
#define SD_DETECT_EXTI_IRQn EXTI12_IRQn

//typedef enum {
//	NO_CARD = 0,
//	CARD_INSERTED,
//	FILE_OPENED
//} FxThreadState;
//

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */

TX_MUTEX q_mutex;


TX_THREAD fx_app_thread;
uint8_t fx_thread_stack[FX_APP_THREAD_STACK_SIZE];


/* Buffer for FileX FX_MEDIA sector cache. */
ALIGN_32BYTES(
    uint32_t fx_sd_media_memory[60 * FX_STM32_SD_DEFAULT_SECTOR_SIZE /
                                sizeof(uint32_t)]) __NON_CACHEABLE;

/* Define FileX global data structures.  */
FX_MEDIA sdio_disk;

/* USER CODE BEGIN PV */

/* Define FileX global data structures.  */
FX_FILE the_video_file;
/* Define ThreadX global data structures.  */
TX_QUEUE tx_msg_queue;
ULONG queue_buf[DEFAULT_QUEUE_LENGTH];
/* USER CODE END PV */

//#define FIFO_SIZE (4*1024*1024)
#define FIFO_SIZE (256*1024)

////////////////
////////////////
//////////////// FIRST COME, FIRST SERVE!
////////////////
////////////////

TX_MUTEX f_mutex;

typedef struct  {
	uint32_t start; // pointer to start of buffer
	uint32_t end; // pointer to end of buffer
	uint32_t empty[2];// align next to 16b
	uint8_t data[FIFO_SIZE];
} fifo_buf;

__attribute__ ((section (".psram_bss"))) // keep in psram
//__attribute__((section(".noncacheable")))
__attribute__ ((aligned (32)))
fifo_buf sd_fifo = { 0,0, {0,0}, ""};


static int fifo_clear(fifo_buf* fifo) {
	fifo->start = 0;
	fifo->end = 0;
	fifo->empty[0] = 0xcafebabe;
	fifo->empty[0] = 0xdeadbeef;
	fifo->data[0] = 0;
	return 0;
}

static int fifo_contains(const fifo_buf* fifo ) {
	if (fifo->end < fifo->start) {
		printf("ERROR! BUFFFER BUG s=%d e=%d \r\n",fifo->start, fifo->end);
		return 0;
	}
//	printf("avail %d\r\n", fifo->end - fifo->start);
	return fifo->end - fifo->start ;
}
static int fifo_available(const fifo_buf* fifo ) { return FIFO_SIZE - fifo->end	; }

int fifo_enq(fifo_buf* fifo, uint8_t* data, int size) {
	if (fifo->end + size > FIFO_SIZE) { // buffer full
		printf("ERROR! BUFFFER FULLLL e=%d s=%d \r\n",fifo->end, size);
		return -1;
	}
	// enqueue data into fifo:
	memcpy(&fifo->data[fifo->end], data, size);
	fifo->end += size;

	return 0;
}

////////////////
////////////////
//////////////// FIRST COME, FIRST SERVE!
////////////////
////////////////

/* Private function prototypes -----------------------------------------------*/
/* Main thread entry function.  */
void fx_app_thread_func(ULONG thread_input);

/* USER CODE BEGIN PFP */
static UINT SD_IsDetected(uint32_t Instance);
static VOID media_close_callback(FX_MEDIA *media_ptr);


// thread state machine state & state-handling functions:
FxThreadState state = NO_CARD;

/** initialize the driver & sdcard info*/
static int state_open_card();

static int state_open_file(char* fname);

/** de-init sdcard */
static int state_close_sdcard();

static int state_write_data();


/* USER CODE END PFP */

/**
 * @brief  Application FileX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT VENC_FileX_Init(void) {
  UINT ret = FX_SUCCESS;

  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */

  /* USER CODE BEGIN MX_FileX_Init */

  // HW init


  /* Create the message queue */
  ret = tx_queue_create(&tx_msg_queue, "sd_event_queue", 1, (VOID *)queue_buf,
                        DEFAULT_QUEUE_LENGTH * sizeof(ULONG));

  /* Check message queue creation */
  if (ret != FX_SUCCESS) {
    return TX_QUEUE_ERROR;
  }

  ret = tx_mutex_create(&(q_mutex), "Write Q Mutex", TX_NO_INHERIT);
  if (ret != FX_SUCCESS) {
	return TX_MUTEX_ERROR;
  }

  ret = tx_mutex_create(&(f_mutex), "FIFO Mutex", TX_NO_INHERIT);
  if (ret != FX_SUCCESS) {
	return TX_MUTEX_ERROR;
  }

  /* USER CODE END MX_FileX_Init */
//	printf("VENC_INIT SD %d\r\n", 0);
//	BSP_SD_Init(0);

  /* USER CODE BEGIN MX_FileX_Init 1*/


  /* Create the main thread.  */
  ret = tx_thread_create(&fx_app_thread,
			FX_APP_THREAD_NAME, fx_app_thread_func,
			0, fx_thread_stack, FX_APP_THREAD_STACK_SIZE,
            8,  // priority
			  8,  // preempt threshold
			  TX_APP_THREAD_TIME_SLICE,
			  TX_AUTO_START);

//			FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD,
//			FX_APP_THREAD_TIME_SLICE, FX_APP_THREAD_AUTO_START);

  /* Check main thread creation */
  if (ret != FX_SUCCESS) {
	printf("THREAD CREATION FAILED!!!!\r\n");
    return TX_THREAD_ERROR;
  }

  /* USER CODE END MX_FileX_Init 1*/

  /* Initialize FileX.  */
  fx_system_initialize();

  fifo_clear(&sd_fifo);
  return ret;
}



ULONG s_msg = DATA_AVAILABLE;

void notify_data_available(){
	s_msg = DATA_AVAILABLE;
	tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
}

void notify_close(){
	s_msg = CLOSE_FILE;
	tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
}



void fx_app_thread_func(ULONG thread_input) {
  UINT sd_status = FX_SUCCESS;

  ULONG r_msg;

  char fname[64];
  int iter=100;

  if (SD_IsDetected(FX_STM32_SD_INSTANCE) == HAL_OK) {
    /* SD card is already inserted, place the info into the queue */
	s_msg = CARD_STATUS_CHANGED;
	tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
    printf("SD_IsDetected? %d\r\n", sd_status);
  }

  /* Infinite Loop */
  for (;;) {
	  // visual state indication..:
	BSP_LED_Off(LED_GREEN);
	BSP_LED_Off(LED_RED);

	/* We wait here for a valid SD card insertion event, if it is not inserted
     * already */
    while (1) {
		r_msg=0;
      while (_tx_queue_receive(&tx_msg_queue, &r_msg,
                              TX_TIMER_TICKS_PER_SECOND / 2) != TX_SUCCESS) {
        /* Toggle GREEN LED to indicate idle state after a successful operation
         */
//        if (last_status == CARD_STATUS_CONNECTED) {
//          BSP_LED_Off(LED_GREEN);
//        }
    	  int sd_det = SD_IsDetected(FX_STM32_SD_INSTANCE);
    	  if (sd_det) {
    	      BSP_LED_On(LED_GREEN);
    	  } else {
    	      BSP_LED_Off(LED_GREEN);
    	  }
// TODO: enable this block once the sd_det is indicating the right state.
//    	  printf("sd_det %d\r\n", sd_det);
    	  // state handling:
//    	  if (state == NO_CARD && sd_det) {
//    		  r_msg = CARD_STATUS_CHANGED; // notify of card insert event
//    		  break;
//    	  }
//    	  if (state != NO_CARD && !sd_det) {
//    		  // card ejected. bail out.
//    		  r_msg = CARD_STATUS_CHANGED; // notify of card insert event
//    		  break;
//    	  }
      }
      unsigned ret=FX_SUCCESS;

//      printf("NOTI %d %d\r\n", r_msg, state);
      switch (r_msg) {
      case DATA_AVAILABLE:
    	  // data should be available: if file is open, dequeue and write; otherwise
    	  // push buffers back to free queue
    	  if (state == FILE_OPENED) {
    	      BSP_LED_On(LED_GREEN);

    	      ret = state_write_data();
    	  } else {
    		  // file not open -> shuffle q objects back to the free q
    		  struct qentry* e = deq(writeQ);
    		  if (e){
    			  enq(freeQ, e);
    		  }
			  BSP_LED_On(LED_RED);
    		  ; // or complain
    	  }
    	  break;

      case CARD_STATUS_CHANGED:
          printf("TDX STAT %08x\r\n", r_msg);
    	  if (state == NO_CARD) {
    		  // card inserted...
    		  ret = state_open_card();
              printf("TDX STAT copen? %d\r\n", ret);
    		  if (ret == FX_SUCCESS) {
    			  // get next filename
    			  snprintf(fname, sizeof(fname), "vid-%03d.mp4", iter++);

        		  ret = state_open_file(fname);
        		  if (ret == FX_SUCCESS) {
                      printf("TDX STAT fopen! %d %s\r\n", ret, fname);
        			  state = FILE_OPENED;
        		  } else {
        			  printf("FAILED TO OPEN FILE %s\r\n",fname);
        		  }
    		  } else {
    			  // else open failed, disk full,.... stay in state
    			  printf("FAILED TO OPEN CARD\r\n");
    		  }
    	  } else {
    		  // card might have been removed
    		  ret = state_close_sdcard();
    		  if (ret == FX_SUCCESS) {
    			  ; // ok
    		  }  // error unmounting? things should be closed anyway
    		  state = NO_CARD;
			  BSP_LED_On(LED_RED);
    	  }
    	  break;
      case CLOSE_FILE:
    	  VENC_FileX_close();
    	  break;
      default:
    	  printf("FIXME! UNKNOWN MSG %u\r\n", r_msg);
    	  break;
      }
    }
  }

  /* USER CODE END fx_app_thread_func 1 */
}


/** state machine state handling functions
 *
 */

/** initialize the sdcard */
static int state_open_card()
{
  UINT sd_status=FX_SUCCESS;
  printf("COPEN\r\n");

  // debounce
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);

  if (SD_IsDetected(FX_STM32_SD_INSTANCE) != HAL_OK) {
	  // no card yet, abort.
		printf("fx_media_open NO CARD%d\r\n", sd_status);
	  return FX_NOT_FOUND;
  }

  /* Open the SD disk driver */
  sd_status = fx_media_open(&sdio_disk, FX_SD_VOLUME_NAME, fx_stm32_sd_driver,
							(VOID *)FX_NULL, (VOID *)fx_sd_media_memory,
							sizeof(fx_sd_media_memory));

  /* Check the media open sd_status */
  if (sd_status != FX_SUCCESS) {
	printf("fx_media_open ERROR %d\r\n", sd_status);
	return sd_status;
  }
  // status ok, register callback
  fx_media_close_notify_set(&sdio_disk, media_close_callback);


  {
	  int instance = 0;
	  SD_HandleTypeDef* hsd = &hsd_sdmmc[instance];

	  HAL_SD_CardStateTypeDef s = HAL_SD_GetCardState(hsd);
	  printf("S %d e? %d\r\n", s, hsd->ErrorCode);

	  HAL_SD_CardInfoTypeDef cinfo;

	  HAL_SD_CardCIDTypeDef cid;
	  HAL_SD_CardCSDTypeDef csd;
	  HAL_SD_CardStatusTypeDef cStatus;

	  // get come more card info:
	  HAL_SD_GetCardInfo(hsd, &cinfo);
	  HAL_SD_GetCardCID(hsd, &cid);
	  HAL_SD_GetCardCSD(hsd, &csd);
	  HAL_SD_GetCardStatus(hsd, &cStatus);

	  printf("CardInfo type %u, version %u, class %u, spd %u\r\n",
			  cinfo.CardType,
			  cinfo.CardVersion,
			  cinfo.Class,
			  cinfo.CardSpeed
			  );
	  printf("Inst PWR %08x CLKCR %08x \r\n",
			  hsd->Instance->POWER,
			  hsd->Instance->CLKCR
			  );
	//  $10 = {CardType = 0, CardVersion = 1, Class = 0, RelCardAdd = 0, BlockNbr = 0, BlockSize = 0, LogBlockNbr = 0, LogBlockSize = 0, CardSpeed = 876165616}
  }

  printf("COPENED\r\n");
  return FX_SUCCESS;
}

static void state_write_notify(struct FX_FILE_STRUCT *file) {
//	printf("WROTE to %s\r\n", file->fx_file_name);
}

static int state_open_file(char* fname)
{
	UINT sd_status=FX_SUCCESS;
	// assert media is open!

	printf("FOPEN\r\n");

	sd_status = fx_file_delete(&sdio_disk, fname);
	if (sd_status != FX_SUCCESS) {
		/* Check for an already created status. This is expected on the
		second pass of this loop!  */
		if (sd_status != FX_NOT_FOUND) {
		  /* Delete error, call error handler.  */
		  return sd_status;
		}
	}

	sd_status = fx_file_create(&sdio_disk, fname);
	/* Check the create status.  */
	if (sd_status != FX_SUCCESS) {
		printf("fx_file_create ERROR %d\r\n", sd_status);
		return sd_status;
	}

	/* Open the file.  */
	sd_status = fx_file_open(&sdio_disk, &the_video_file, fname, FX_OPEN_FOR_WRITE);

	if (sd_status != FX_SUCCESS) {
		/* Error opening file, call error handler, complain, something...  */
		return sd_status;
	}
	printf("FOPENED %d\r\n", sd_status);

	the_video_file.fx_file_write_notify = state_write_notify;

	return sd_status; // success
}

/** de-init sdcard */
static int state_close_sdcard()
{
	printf("FCLOSE\r\n");
	// close file
	fx_file_close(&the_video_file);

	// close driver
	fx_media_close(&sdio_disk);
	return FX_SUCCESS;
}

// 512, effectively:
#define BLOCK_SIZE FX_STM32_SD_DEFAULT_SECTOR_SIZE


/// flush fifo contents to sd card, move rest to front, reset counters.
int fifo_drain(FX_FILE* file, fifo_buf* fifo) {
	int count = fifo_contains(fifo);
	int blocks = count	/BLOCK_SIZE;
	int write_size = blocks * BLOCK_SIZE;

	// write n full blocks to disk
//	uint32_t t1 = HAL_GetTick();

//	printf("FIF> %ld %d\r\n", fifo->start, write_size);

	int status = fx_file_write(file, &fifo->data[fifo->start], write_size);

	//	uint32_t t2 = HAL_GetTick();
//	printf("FIF< %ld %d td %ld\r\n", fifo->start, write_size, (t2-t1));

	// TODO: abort on error
	if (status != FX_SUCCESS) {
		printf("WRITE ERROR %d\r\n", status);
		return status;
	}

	// advance buffer by written size
	fifo->start += write_size; // points to beginning of fresh data.

	if (fifo->start >= FIFO_SIZE) {
		fifo_clear(fifo);
		return status;
	}

	int tail = fifo->end - write_size; // the last few bytes..:

	// fifo_compact() - move the remaining bytes to front:
	if ( tail && fifo->start > 0) {
		// TODO: this is not elegant, rather wrap around buffers; on the other
		// hand, DMA transfer penalty could be higher than copying a few bytes.
		// move data to beginning.
		memcpy(&fifo->data[0], &fifo->data[fifo->start], tail);
		fifo->start = 0;
		fifo->end = tail;
	}
	if (!tail) { // sent everything -> reset fifo.
		fifo_clear(fifo);
	}
	return status;
}

// write to fifo & dump to disk if possible.
// wraps around fifo as necessary

int fifo_write(FX_FILE* file, fifo_buf* fifo, uint8_t* data, int size)
{
	UINT status=0;
	while (size > 0) {
		int enq_size = size;
		int available = fifo_available(fifo);

		if (available < size) {
			enq_size = available;
		}

		// enqueue data into fifo:
		fifo_enq(fifo, data, enq_size);

		size -= enq_size; // move forward
		data += enq_size;

		int delta = fifo_contains(fifo);

		// drain buffer if enough data has been collected:
		if (delta > FIFO_SIZE / 2) { // 50% threshold
//		if (delta > BLOCK_SIZE) { // write as soon as possible
			fifo_drain(file, fifo);
		}
	}

	return status;
}

// forcibly flush the remaining bytes in the buffer and reset the fifo.
int fifo_flush(FX_FILE* file, fifo_buf* fifo)
{
	if (fifo->start == fifo->end) {
		// fifo empty, nothing to do.
		return 0;
	}

	UINT status = fx_file_write(file, &fifo->data[fifo->start], fifo->end - fifo->start);

	// reset fifo:
	fifo_clear(fifo);

	return status;
}


static int state_write_data()
{
	struct qentry* entry = deq(writeQ);
	UINT status = 0;

	if (entry == NULL) { // should not happen!
		printf("write_data - empty writeQ.\r\n");
		return FX_SUCCESS; // ignore for now...
	}
	uint32_t t1 = HAL_GetTick();

	status = fifo_write(&the_video_file, &sd_fifo, (uint8_t*)entry->data, entry->size);

	uint32_t t2 = HAL_GetTick();
	printf("write %ld %ld %ld\r\n", entry->idx, entry->size, (t2-t1));

	enq(freeQ, entry);

	return status;
}


/* USER CODE BEGIN 1 */

UINT VENC_FileX_write(CHAR *data, LONG size) {
	UINT status;
	/* Write the given data to the file.  */
    uint32_t t1 = HAL_GetTick();

    status = fifo_write(&the_video_file, &sd_fifo, (uint8_t*)data, size);

	uint32_t t2 = HAL_GetTick();
	printf("WRT %ld %ld\r\n", size, (t2-t1));

  return status;
}

UINT VENC_FileX_close(void)
{
	printf("CLOSING! \r\n");

	fifo_flush(&the_video_file, &sd_fifo);

	printf("CLOSING!2 \r\n");

	/* Close the test file.  */
	UINT status = fx_file_close(&the_video_file);
	/* Check the file close status.  */
	if (status != FX_SUCCESS) {
		printf("CLOSING! E %d %d \r\n", __LINE__, status);
		/* Error closing the file, call error handler.  */
		//return status; // make sure we flush & close the media anyway
	}

	status = fx_media_flush(&sdio_disk);
	/* Check the media flush  status.  */
	if (status != FX_SUCCESS) {
		printf("CLOSING! E %d %d \r\n", __LINE__, status);
		/* Error closing the file, call error handler.  */
		return status;
	}
	/* Close the media.  */
	status = fx_media_close(&sdio_disk);

	printf("CLOSED! %d\r\n", status);

	return status;
}

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param Instance  SD Instance
 * @retval Returns if SD is detected or not
 */
static UINT SD_IsDetected(uint32_t Instance) {
  UINT ret;
//  return BSP_SD_IsDetected(Instance) ? HAL_OK : HAL_ERROR;

  if (Instance >= 1) {
    ret = HAL_ERROR;
  } else {
    /* Check SD card detect pin */
    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin) == GPIO_PIN_SET) {
      ret = HAL_ERROR;
    } else {
      ret = HAL_OK;
    }
  }

  return ret;
}

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI
 * line.
 * @retval None
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  ULONG s_msg = CARD_STATUS_CHANGED;

  printf("EXTI %d\r\n", GPIO_Pin);

  if (GPIO_Pin == SD_DETECT_Pin) {
    tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
  }
}

/**
 * @brief  Media close notify callback function.
 * @param  media_ptr: Media control block pointer
 * @retval None
 */
static VOID media_close_callback(FX_MEDIA *media_ptr) {
  state = CARD_INSERTED; // not file opened, in ancy case

  printf("media_close_callback %s \r\n", media_ptr->fx_media_name);

}

/* USER CODE END 1 */
