
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_filex.h
  * @author  MCD Application Team
  * @brief   FileX applicative header file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FILEX_H__
#define __APP_FILEX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fx_api.h"
#include "fx_stm32_sd_driver.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


//////////////////////////
//////////////////////////
////////////////////////// 2 Q or not 2 Q... API:
//////////////////////////
//////////////////////////

/** queue entry struct. */
struct qentry {
	struct qentry* next; // points to next or NULL at end.
	int32_t idx; // buffer array index; int8 would be ok, but this will be 32-bit aligned anyway
	int32_t size; /// number of bytes of data that are in use
	int32_t data_len; /// size of the data buffer
	int32_t fc; /// frame counter
	int32_t ts; /// timestamp (haltick of arrival)
	uint8_t* data; /// pointer to the actual buffer data
};

/// check if q has data elements
static inline int qAvailable(const struct qentry* queue) { return queue->next != NULL; };

extern int enq(struct qentry* queue, struct qentry* ent);

/// dequeue - remove first entry. call in no-irq context to be atomic or guard!
/// @return entry pointer or NULL if empty
extern struct qentry* deq(struct qentry* queue);

/* available queues: freeQ -> unused buffers, writeQ -> buffers to be written to disk */
extern struct qentry* freeQ;
extern struct qentry* writeQ;

/* mutex to guard access to the queues. lock for enq/deq operations */
extern TX_MUTEX q_mutex;


/* Message content*/
typedef enum {
	DATA_AVAILABLE = 3, /* queued data... */
	OPEN_FILE = 5, /* open file */
	CLOSE_FILE = 7, /* close file, re-open... */
	CARD_STATUS_CHANGED = 9, /* card pulled or inserted -> detect pin irq*/
	MOUNT_CARD = 11, /* simulate card detect irq - mount */
	UMOUNT_CARD, /* simulate card detect irq - unmount*/
} FXMessageType;


typedef enum {
	NO_CARD = 0,
	CARD_INSERTED,
	FILE_OPENED
} FxThreadState;

extern FxThreadState state;

extern void notify_data_available();

// mount sdcard
void notify_mount();
// unmount sdcard
void notify_umount();

// open next file
extern void notify_open();
// close file
extern void notify_close();


//////////////////////////
//////////////////////////
////////////////////////// 2 Q or not 2 Q... /API
//////////////////////////
//////////////////////////

extern UINT SD_IsDetected(uint32_t Instance);
extern void init_detect_pin();


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TX_APP_THREAD_TIME_SLICE                5
#define TX_APP_THREAD_PRIO                      5

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT VENC_FileX_Init(void);
/* USER CODE BEGIN EFP */
UINT VENC_FileX_write(CHAR * data, LONG size);
UINT VENC_FileX_close(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* Main thread Name */
#ifndef FX_APP_THREAD_NAME
  #define FX_APP_THREAD_NAME "FileX app thread"
#endif

/* Main thread time slice */
#ifndef FX_APP_THREAD_TIME_SLICE
  #define FX_APP_THREAD_TIME_SLICE TX_NO_TIME_SLICE
#endif

/* Main thread auto start */
#ifndef FX_APP_THREAD_AUTO_START
  #define FX_APP_THREAD_AUTO_START TX_AUTO_START
#endif

/* Main thread preemption threshold */
#ifndef FX_APP_PREEMPTION_THRESHOLD
  #define FX_APP_PREEMPTION_THRESHOLD FX_APP_THREAD_PRIO
#endif

/* fx sd volume name */
#ifndef FX_SD_VOLUME_NAME
  #define FX_SD_VOLUME_NAME "STM32_SDIO_DISK"
#endif

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
#ifdef __cplusplus
}
#endif
#endif /* __APP_FILEX_H__ */
