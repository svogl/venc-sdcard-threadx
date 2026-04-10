/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @brief   This project is a HAL template project for STM32N6xx devices.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

#include "ewl.h"
#include "h264encapi.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6xx_hal_sd.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_sd.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_xspi.h"
#include "stm32n6570_discovery_camera.h"
#include "stm32_lcd.h"
#include "tx_api.h"
#include "app_filex.h"

//void SDMMC1_IRQHandler(void)
//{
//	while(0) {}
//}

/** @addtogroup Templates
 * @{
 */

/** @addtogroup HAL
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define FRAMERATE 30
/* number of frames to film and encode */
#define VIDEO_FRAME_NB 100
#define USE_SD_AS_OUTPUT 1

/* Private macro -------------------------------------------------------------*/

/* enable trace if possible */
#if USE_COM_LOG || defined(TERMINAL_IO)
#define TRACE_MAIN(...) printf(__VA_ARGS__)
#else
#define TRACE_MAIN(...)
#endif /* USE_COM_LOG || defined(TERMINAL_IO) */

/* Private variables ---------------------------------------------------------*/

uint16_t * pipe_buffer[2];
volatile uint8_t buf_index_changed = 0;

H264EncIn encIn= {0};
H264EncOut encOut= {0};
H264EncInst encoder= {0};
H264EncConfig cfg= {0};

uint32_t output_size = 0;
uint32_t img_addr = 0;

volatile int32_t cam_frame_counter = 0;

EWLLinearMem_t outbuf;

// luma 473600
// chroma 236800
// 1420800


__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t ewl_pool[1640000];// __NON_CACHEABLE;


uint8_t tx_main_heap[4096];
TX_BYTE_POOL byte_pool;
TX_THREAD main_thread;

//__attribute__ ((section (".psram_bss")))
//__attribute__ ((aligned (32)))
//uint32_t output_buffer[800*600/8];// __NON_CACHEABLE;

static int frame_nb = VIDEO_FRAME_NB+1; // by default, be inactive

// button interface
static void handleInteraction();
static void init_sensor_pins();


static TX_SEMAPHORE button_semaphore;

//////////////////////////
//////////////////////////
////////////////////////// 2 Q or not 2 Q...
//////////////////////////
//////////////////////////

#define NUM_BUFS 3
#define BUF_SIZE (160*1024)

// static memory block that is used for buffers
//__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (8)))
uint8_t out_buffers[NUM_BUFS][BUF_SIZE] __NON_CACHEABLE;

// pointer to buffer
uint8_t* tx_q[NUM_BUFS] = {0,};
// writable size in bytes to transfer:
int32_t out_buffers_len[NUM_BUFS] = {0, };


// pre-allocated list of queue entries; initialized in initq
struct qentry raw_entries[NUM_BUFS] = {
		{ NULL, },
};

// the queues use a dummy element.
struct qentry freeQhead = { .idx = -1, .next = NULL};
struct qentry writeQhead = { .idx = -1, .next = NULL};

struct qentry* freeQ = &freeQhead;
struct qentry* writeQ = &writeQhead;

int initq() {
	for (int i=0;i<NUM_BUFS; i++) {
		raw_entries[i].next = NULL;
		raw_entries[i].idx = i;
		raw_entries[i].data_len = BUF_SIZE;
		raw_entries[i].size = -1;
		raw_entries[i].data = out_buffers[i];

		enq(freeQ, &raw_entries[i]);
	}
	return 0;
}

/// dequeue - remove first entry. call in no-irq context to be atomic or guard!
/// @return entry pointer or NULL if empty
struct qentry* deq(struct qentry* queue)
{
	tx_mutex_get(&q_mutex, TX_WAIT_FOREVER);

	if (!queue->next) {
		tx_mutex_put(&q_mutex);
		return NULL;
	}
	struct qentry* ent = queue->next;
	queue->next = ent->next;

	if (ent)
		ent->next = NULL; // sanitize, just in case.

	tx_mutex_put(&q_mutex);
	return ent;
}

/// enqueue an element at the end of the queue.
int enq(struct qentry* queue, struct qentry* ent)
{
	tx_mutex_get(&q_mutex, TX_WAIT_FOREVER);

	struct qentry* iter = queue;
	// go to the end of the q
	while (iter ->next) {
		iter  = iter->next;
	}
	iter->next = ent;
	ent->next = NULL; // sanitize, just in case.

	tx_mutex_put(&q_mutex);
	return 0;
}


//////////////////////////
//////////////////////////
//////////////////////////
//////////////////////////
//////////////////////////

//////////////////////////
////////////////////////// RTC


RTC_HandleTypeDef hrtc;
static void MX_RTC_Init(void); /// init rtc hw

static void RTC_CalendarShow(void); ///
static void RTC_InitTime(void); /// set a fixed date/time

// RTC status:
uint8_t aShowTime[16] = "hh:mm:ss";
uint8_t aShowTimeStamp[16] = "hh:mm:ss";
uint8_t aShowDate[16] = "mm-dd-yyyy";
uint8_t aShowDateStamp[16] = "mm-dd-yyyy";
__IO uint8_t  RTCStatus = 0;

////////////////////////// /RTC
//////////////////////////


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static int encoder_hw_init(uint32_t width, uint32_t height, uint32_t * output_buffer);
static int encoder_prepare(uint32_t width, uint32_t height, uint32_t * output_buffer);
static int encode_frame(struct qentry* ent);
static int encoder_end(void);
static void MPU_Config(void);

/* change save stream and read frame for preferred in/output */
static int save_stream(uint32_t offset, uint32_t * buf, size_t size);
static int flush_out_buffer(void);
void main_thread_func(ULONG arg);

/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Save an encoded buffer fragment at the given offset.
 * @param  offset
 * @param  buf  pointer to the buffer to save
 * @param  size  size (in bytes) of the buffer to save
 * @retval err error code. 0 On success.
 */
int save_stream(uint32_t offset, uint32_t * buf, size_t size){
	int ret = 0;
	uint32_t t1 = HAL_GetTick();
#if USE_SD_AS_OUTPUT
	ret = enqueue_file_data((char*)buf,size);
#else
	return 0;
#endif
	uint32_t t2 = HAL_GetTick();
	printf("save_stream %d %d %d\r\n", ret, size, (t2-t1));
	return ret;
}

static int flush_out_buffer(void){
#if USE_SD_AS_OUTPUT
	notify_close();
#else
	return 0;
#endif
}

int det = 0xface;
int userBState = 0;
int tampBState = 0;

int pir_irq_occured = 0;

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	/* enable MPU configuration to create non cacheable sections */
	MPU_Config();

	/* Enable DCache */
	//  SCB_EnableDCache();
	/* Enable ICache */
	//  SCB_EnableICache();

	//  SystemCoreClockUpdate();

	/* Initialize the HAL timebase (eg. SysTick) */
	HAL_Init();

	init_detect_pin();

	init_sensor_pins();

	// rtc:
	MX_RTC_Init();
	RTC_CalendarShow();
	RTC_InitTime();
	RTC_CalendarShow();


	det = SD_IsDetected(0);

	/* oscillator and PLL already configured. Configure periph clocks */
	SystemClock_Config();

	/* Check expected frequency */
	/* UART log */
#if USE_COM_LOG
	COM_InitTypeDef COM_Init;

	/* Initialize COM init structure */
	COM_Init.BaudRate   = 115200;
	COM_Init.WordLength = COM_WORDLENGTH_8B;
	COM_Init.StopBits   = COM_STOPBITS_1;
	COM_Init.Parity     = COM_PARITY_NONE;
	COM_Init.HwFlowCtl  = COM_HWCONTROL_NONE;

	BSP_COM_Init(COM1, &COM_Init);

	if (BSP_COM_SelectLogPort(COM1) != BSP_ERROR_NONE)
	{
		TRACE_MAIN("failed to set up log port\n");
		Error_Handler();
	}
#endif
	BSP_PB_Init(BUTTON_USER1, BUTTON_MODE_EXTI);
	BSP_PB_Init(BUTTON_TAMP, BUTTON_MODE_EXTI);

	det = SD_IsDetected(0);


	printf("---------------- BOOT %d\r\n",det);

	tx_kernel_enter();
}

void BSP_PB_Callback (Button_TypeDef Button)
{
	int state = -1;

	switch (Button) {
	case BUTTON_USER1:
		state = BSP_PB_GetState(Button);
		break;
	case BUTTON_TAMP:
		state = BSP_PB_GetState(Button);
		break;
	default:
		break;
	}
	printf("BTN CB for %d s= %d\r\n", Button, state);
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
void tx_application_define(void *first_unused_memory)
{

	/* Create btn event semaphore.  */
	if (tx_semaphore_create(&button_semaphore, "button sema", 0) != TX_SUCCESS)
	{
		printf("COULD NOT SET UP SEMA!\r\n");
		//		return TX_SEMAPHORE_ERROR;
	}


	void *thread_stack_pointer;
	tx_byte_pool_create(&byte_pool, "byte pool", tx_main_heap, sizeof(tx_main_heap));
	tx_byte_allocate(&byte_pool,
			&thread_stack_pointer, 4000, TX_NO_WAIT);

	(void) tx_thread_create(&main_thread,
			"main_thread",
			main_thread_func, 0,
			thread_stack_pointer, 4000,
			8,  // priority
			8,  // preempt threshold
			TX_APP_THREAD_TIME_SLICE,
			//					  TX_DONT_START);
			TX_AUTO_START);


}

/// state handling - check inputs, trigger recording
static int32_t then = 0;
static int32_t thenD = 0;

static FxThreadState myState = NO_CARD;

/// set to 1 to switch on LCD subsys
static int lcd_enabled = 0;

void main_thread_func(ULONG arg){
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	/* set all required IPs as secure privileged */
	__HAL_RCC_RIFSC_CLK_ENABLE();
	RIMC_MasterConfig_t RIMC_master = {0};
	RIMC_master.MasterCID = RIF_CID_1;
	RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_SDMMC2, &RIMC_master);
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1 , &RIMC_master);
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2 , &RIMC_master);
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC  , &RIMC_master);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_SDMMC2 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI    , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

	/* initialize LEDs to signal processing is ongoing */
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);

	TRACE_MAIN("CPU frequency    : %lu\n", HAL_RCC_GetCpuClockFreq() / 1000000);
	TRACE_MAIN("sysclk frequency : %lu\n", HAL_RCC_GetSysClockFreq() / 1000000);
	TRACE_MAIN("pclk5 frequency  : %lu\n", HAL_RCC_GetPCLK5Freq() / 1000000);


	if (lcd_enabled) { // lcd util stuff
		//	  BSP_LCD_LayerConfig_t LayerConfig = {0};
		//	  /* Preview layer Init */
		//	  LayerConfig.X0          = 0;
		//	  LayerConfig.Y0          = 0;
		//	  LayerConfig.X1          = LCD_DEFAULT_WIDTH;
		//	  LayerConfig.Y1          = LCD_DEFAULT_HEIGHT;
		//	  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_RGB565;
		//	  LayerConfig.Address     = (uint32_t) 0x34050000;

		//	  BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &LayerConfig);

		//	  LayerConfig.Address = (uint32_t) 0x3413A600; /* External XSPI1 PSRAM */

		//	  BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &LayerConfig);


		UTIL_LCD_SetFuncDriver(&LCD_Driver);
		UTIL_LCD_SetLayer(LTDC_LAYER_1);
		//  UTIL_LCD_Clear(0x00000000);
		UTIL_LCD_SetFont(&Font20);
		UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
	}

	/*** External RAM and NOR Flash *********************************************/
	BSP_XSPI_RAM_Init(0);
	BSP_XSPI_RAM_EnableMemoryMappedMode(0);

	//  BSP_XSPI_NOR_Init_t NOR_Init;
	//  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
	//  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
	//  BSP_XSPI_NOR_Init(0, &NOR_Init);
	//  BSP_XSPI_NOR_EnableMemoryMappedMode(0);

	// from here on, psram is available...:
	initq();

	/* initialize ext flash interface and driver */

	VENC_FileX_Init();


	/* Initialize camera */
	if(BSP_CAMERA_Init(0, CAMERA_R2592x1944, CAMERA_PF_RAW_RGGB10) != BSP_ERROR_NONE){
		Error_Handler();
	}
	// 800x480x2 = 0xBB800 == 768.000 --> 0x3410b800
	//       --> end: 0x341C7000
	// 800x600x2 = 0xEA600 == 960.000 --> 0x3413A600
	//       --> end: 0x34224C00 -> check linker script
	//           size: 0x19B400 == 1684480 bytes == 1645k
	// RAM END:
	// Axisram6 start 0x34350000 sz 448kb = 0x70000  --> END = 0x343C0000
	//
	// todo - define bpp and use these instead of the abs. adresses.
	// n.b. in the .ld file, the RAM start must be FRAME_BASE + 2* FRAME_SIZE.
#define FRAME_BASE 0x34050000
#define FRAME_SIZE (  LCD_DEFAULT_WIDTH * LCD_DEFAULT_HEIGHT * 2 )
#define FRAME_0 (FRAME_BASE)
#define FRAME_1 (FRAME_BASE + (FRAME_SIZE))
	/* start camera acquisition */
	if(BSP_CAMERA_DoubleBufferStart(0, (uint8_t *)(0x34050000),(uint8_t *)(0x3413A600), CAMERA_MODE_CONTINUOUS)!= BSP_ERROR_NONE){
		Error_Handler();
	}

	/* Initialize LCD */
	if (lcd_enabled) {
		int err = BSP_LCD_InitEx(0, LCD_ORIENTATION_LANDSCAPE, LCD_PIXEL_FORMAT_RGB565, LCD_DEFAULT_WIDTH, LCD_DEFAULT_HEIGHT);
		if(err){
			TRACE_MAIN("error initializing LCD : %d\n", err);
			Error_Handler();
		}
		BSP_LCD_SetLayerAddress(0, 0, 0x34050000);
	}

	/* initialize VENC */
	LL_VENC_Init();

	/* initialization done. Turn on the LEDs */

	ULONG s_msg = DATA_AVAILABLE;

	encoder_hw_init(800,600,(uint32_t*)0x3413A600);

	while (1) {
		// TODO: introduce proper state handling.

		while (frame_nb < VIDEO_FRAME_NB) {
			if (state == FILE_OPENED && myState != FILE_OPENED) {
				// file opened -> init encoder
				printf("STARTING ENCODER \r\n");
				// todo: start encoding on sensor event.

				/* initialize encoder software for camera feed encoding */
				encoder_prepare(800,600,(uint32_t*)0x3413A600);

				myState = FILE_OPENED;
			}


			if(buf_index_changed){
				/* new frame available */
				buf_index_changed = 0;

				if(BSP_CAMERA_BackgroundProcess() != BSP_ERROR_NONE)
				{
					Error_Handler();
				}

				if (myState == FILE_OPENED) {
					struct qentry* ent = deq(freeQ);

					if (!ent) {
						// no free buffers available - silently fail over
						// and wait for the next frame.
					} else {

						int ret = encode_frame(ent);
						frame_nb++;
						if (ret < 0) {
							printf("ENC ERROR. %d STOP.\r\n\r\n", ret);
							break;
						}

						//						TRACE_MAIN("ENQ writeQ %d - %d i %d s %d states %ld %ld\r\n", frame_nb, ent->fc, ent->idx, ent->size,
						//								myState, state);

						enq(writeQ, ent);
						notify_data_available();
					}
				} else {
					//				  printf("c %lu\r\n", cam_frame_counter );
				}

				// if we're near the end of recording and we had some activity, continue to record
				if (frame_nb > VIDEO_FRAME_NB - 10 && pir_irq_occured > 0) {
					printf("main() detected %d PIR events; continuing...\r\n", pir_irq_occured);
					pir_irq_occured = 0;
					frame_nb = 1;
				}
				handleInteraction();
			} else {
				tx_thread_sleep(1);
			}

		}

		handleInteraction();

		if (frame_nb == VIDEO_FRAME_NB) {

			/* after encoding a certain nb of frames, close file & flush buffers */
			encoder_end();
			flush_out_buffer();
			myState = CARD_INSERTED;
			frame_nb++;
		}

		//tx_thread_sleep(1000); // todo replace with irq result
		if (tx_semaphore_get(&button_semaphore, TX_WAIT_FOREVER) == TX_SUCCESS)
		{

		}
	}


	/* program ended */
	while(1);
}


static void handleInteraction()
{
	int32_t now = HAL_GetTick();
	if (now - then > 100) { // 2 times a sec, check the gpios

		int bUser = BSP_PB_GetState(BUTTON_USER1);
		int bTamp = BSP_PB_GetState(BUTTON_TAMP);
		int sd = BSP_SD_IsDetected(0);
		int pir = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

		if (now - thenD > 1000) { // every something, report
			printf("c %lu bu %d bt %d pir %d sd %d S: %d %d\r\n", cam_frame_counter , bUser, bTamp , pir, sd,
					state, myState);
			thenD = now;
		}

		// SENSOR INPUT
		if (userBState && !bUser) {
			// trigger on falling edge -
			printf("USER BLIP!\r\n");

			// -> activity; prolong recording interval
			if (myState == FILE_OPENED) {
				printf("CAPTURING, but reseting frame counter from %d!\r\n", frame_nb);
				frame_nb=0;
			} else {
				printf("IDLE, starting up!\r\n");
				notify_open();

				// allow some time for file to open
				tx_thread_sleep(50);

				frame_nb = 0;
				state = FILE_OPENED;
			}
			tx_semaphore_put(&button_semaphore);
		}

		// simulate eject / mount request
		if (tampBState && !bTamp) {
			// trigger on falling edge
			printf("TAMP BLIP! s=%d \r\n", state);

			if (state == NO_CARD) { // simulate insert
				printf("TAMP BLIP!\r\n");
				notify_mount();
			} else if (state == CARD_INSERTED) {
				notify_umount();
			}

			if (myState == FILE_OPENED) {
				frame_nb = VIDEO_FRAME_NB; // triggers close code outside
			}
		}


		userBState = bUser;
		tampBState = bTamp;
		//
		then = now;
	}
}

static int encoder_hw_init(uint32_t width, uint32_t height, uint32_t * output_buffer)
{
	H264EncRet ret;
	height /= 16; // round down to macroblock size
	height *= 16;
	H264EncPreProcessingCfg preproc_cfg = {0};

	/* software workaround for Linemult triggering VENC interrupt. Make it happen as little as possible */
	MODIFY_REG(DCMIPP->P1PPCR, DCMIPP_P1PPCR_LINEMULT_Msk,DCMIPP_MULTILINE_128_LINES);

	frame_nb = 0;
	/* Step 1: Initialize an encoder instance */
	/* set config to 1 ref frame */
	cfg.refFrameAmount = 1;
	/* 30 fps frame rate */
	cfg.frameRateDenom = 1;
	cfg.frameRateNum = FRAMERATE;
	/* Image resolution */
	cfg.width = width;
	cfg.height = height;
	/* Stream type */
	cfg.streamType = H264ENC_BYTE_STREAM;
	// do not use the double-buffer for HRD; we're saving to disk & saving memory.
	cfg.viewMode = H264ENC_BASE_VIEW_SINGLE_BUFFER;

	/** single buffer allocations for 800x592:
	  EWL ALLOC 473600
	  EWL ALLOC 236800
	  EWL ALLOC 236800
	  EWL ALLOC 168
	  EWL ALLOC 48256
	  EWL ALLOC 103600
	  EWL ALLOC 928
	  total:  1100152 == 1.04Mb
	 */
	/* encoding level*/
	/*See API guide for level depending on resolution and framerate*/
	cfg.level = H264ENC_LEVEL_2_2;
	cfg.svctLevel = 0;

	cfg.level = H264ENC_LEVEL_3_1; // for higher resolution...

	/* Output buffer size */
	outbuf.size = cfg.width * cfg.height;

	ret = H264EncInit(&cfg, &encoder);
	if (ret != H264ENC_OK)
	{
		TRACE_MAIN("error initializing encoder %d\n", ret);
		return -1;
	}

	/* set format conversion for preprocessing */
	ret = H264EncGetPreProcessing(encoder, &preproc_cfg);
	if(ret != H264ENC_OK){
		TRACE_MAIN("error getting preproc data\n");
		return -1;
	}
	preproc_cfg.inputType = H264ENC_RGB565;
	ret = H264EncSetPreProcessing(encoder, &preproc_cfg);
	if(ret != H264ENC_OK){
		TRACE_MAIN("error setting preproc data\n");
		return -1;
	}

	{ // if rate control
		H264EncRateCtrl rateCtrl;
		ret = H264EncGetRateCtrl(encoder, &rateCtrl );
		if(ret != H264ENC_OK){
			TRACE_MAIN("error get ratectl data\n");
			return -1;
		}
		rateCtrl.pictureRc = 0;
		rateCtrl.mbRc = 0;
		rateCtrl.gopLen = 8;
		rateCtrl.bitPerSecond = 5*1024*1024; // 1M bps
		ret = H264EncSetRateCtrl(encoder, &rateCtrl );
		if(ret != H264ENC_OK){
			TRACE_MAIN("error set ratectl data\n");
			return -1;
		}
	}

	return 0;
}


static int encoder_prepare(uint32_t width, uint32_t height, uint32_t * output_buffer)
{
	H264EncRet ret;


	/*assign buffers to input structure */
	encIn.pOutBuf = output_buffer;
	encIn.busOutBuf = (uint32_t) output_buffer;
	encIn.outBufSize = width * height/2;

	/* create stream */
	ret = H264EncStrmStart(encoder, &encIn, &encOut);
	if (ret != H264ENC_OK)
	{
		TRACE_MAIN("error starting stream\n");
		return -1;
	}

	/* save the stream header */
	if (save_stream(output_size, encIn.pOutBuf,  encOut.streamSize))
	{
		TRACE_MAIN("error saving stream\n");
		//    return -1;
	}
	TRACE_MAIN("stream started. saved %d bytes\n", encOut.streamSize);
	output_size+= encOut.streamSize;

	frame_nb=0;
	return 0;
}

#define N_PRINTABLE_CHARS 64
char buffer[N_PRINTABLE_CHARS + 1] = { '\0', };

static int print_timer(uint8_t* img_addr) {
	if(!img_addr){
		TRACE_MAIN("Error : NULL image address");
		return -1;
	}

	//	BSP_LCD_SetLayerAddress(0, 0, img_addr);
	//	snprintf(buffer, sizeof(buffer), "%06u", cam_frame_counter);
	//	UTIL_LCD_DisplayStringAt(4, 16, buffer, LEFT_MODE);

	int x = cam_frame_counter;
	int stride = LCD_DEFAULT_WIDTH * 2 /*bpp*/ ;
	uint8_t* line0= img_addr + (4+0)* stride;
	uint8_t* line1= img_addr + (4+1)* stride;
	uint8_t* line2= img_addr + (4+2)* stride;
	uint8_t* line3= img_addr + (4+3)* stride;
	int xofs=0;
	while (x) {
		if (x & 1) {
			for (int w=0;w<4;w++) {
				line0[w+xofs] = 0xff;
				line0[w+xofs+1] = 0xff;
				line1[w+xofs] = 0xff;
				line1[w+xofs+1] = 0xff;
				line2[w+xofs] = 0xff;
				line2[w+xofs+1] = 0xff;
				line3[w+xofs] = 0xff;
				line3[w+xofs+1] = 0xff;
			}
		} else {
			for (int w=0;w<4;w++) {
				line0[w+xofs] = 0x00;
				line0[w+xofs+1] = 0x00;
				line1[w+xofs] = 0x00;
				line1[w+xofs+1] = 0x00;
				line2[w+xofs] = 0x00;
				line2[w+xofs+1] = 0x00;
				line3[w+xofs] = 0x00;
				line3[w+xofs+1] = 0x00;
			}
		}
		xofs+=4;
		x >>= 1;
	}
}

/** runs the video encoder on a single frame.
 * */
static int encode_frame(struct qentry* ent){
	int ret = H264ENC_FRAME_READY;
	if(!img_addr){
		TRACE_MAIN("Error : NULL image address");
		return -1;
	}
	ent->fc = cam_frame_counter;
	ent->ts = HAL_GetTick();

	//  printf("enc p %d -> %d %d\r\n", ent->idx, frame_nb, cam_frame_counter);

	//  print_timer(img_addr);

	encIn.pOutBuf = (u32*)ent->data;
	encIn.busOutBuf = (uint32_t) ent->data;
	encIn.outBufSize = ent->data_len;

	if (! (frame_nb & 0x07) || frame_nb ==0 )
	{
		/* if frame is the first : set as intra coded */
		encIn.timeIncrement = 0;
		encIn.codingType = H264ENC_INTRA_FRAME;
	}
	else
	{
		/* if there was a frame previously, set as predicted */
		encIn.timeIncrement = 1;
		encIn.codingType = H264ENC_PREDICTED_FRAME;
	}
	encIn.ipf = H264ENC_REFERENCE_AND_REFRESH;
	encIn.ltrf = H264ENC_REFERENCE;
	/* set input buffers to structures */
	encIn.busLuma = img_addr;
	ret = H264EncStrmEncode(encoder, &encIn, &encOut, NULL, NULL, NULL);

	ent->size = encOut.streamSize;

	//  printf("ENCed %d %d %d\r\n", ret, encIn.codingType, encOut.streamSize);

	switch (ret)
	{
	case H264ENC_FRAME_READY:
		// the actual write operation is performed by the filex thread
		/*save stream - done outside by queueing... */
		output_size += encOut.streamSize;
		break;
	case H264ENC_INVALID_ARGUMENT:
		TRACE_MAIN("invalid argument!\r\n");
		break;
	case H264ENC_OUTPUT_BUFFER_OVERFLOW:
		TRACE_MAIN("output buffer overflow!\r\n");
		break;
	case H264ENC_SYSTEM_ERROR:
		TRACE_MAIN("fatal error while encoding\r\n");
		break;
	default:
		TRACE_MAIN("error encoding frame %d : %d\n", frame_nb, ret);
		break;
	}
	return ret;
}


static int encoder_end(void){
	int ret = H264EncStrmEnd(encoder, &encIn, &encOut);
	TRACE_MAIN("done encoding %d frames. size : %u\n",frame_nb ,output_size);
	if (ret != H264ENC_OK)
	{
		return -1;
	}
	else
	{
		/* save stream tail */
		if (save_stream(output_size, encIn.pOutBuf,  encOut.streamSize))
		{
			TRACE_MAIN("error saving stream\n");
			return -1;
		}
		output_size+=encOut.streamSize;
	}

	return 0;
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            CPU Clock source               = IC1
 *            System bus Clock source        = IC2
 *            SYSCLK(Hz)                     = 400000000
 *            HCLK(Hz)                       = 200000000
 *            AHB Prescaler                  = 2
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            APB4 Prescaler                 = 1
 *            APB5 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 64000000
 *            PLL1 State                     = ON
 *            PLL2 State                     = ON
 *            PLL3 State                     = BYPASS
 *            PLL4 State                     = ON
 * @retval None
 */
static void SystemClock_Config(void)
{
#if defined(DEBUG)
	/* if in debug mode, clock is not configured when entering app. Configure it here */
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};

	/* Configure the system Power Supply */
	if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
	{
		Error_Handler();
	}

	/* Get current CPU/System buses clocks configuration */
	/* and if necessary switch to intermediate HSI clock */
	/* to ensure target clock can be set                 */
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
	if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
			(RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
	{
		RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
		RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler();
		}
	}

	/* HSI selected as PLL1 source                             */
	/* PLL1 output = ((HSI/PLLM)*PLLN)/PLLP1/PLLP2             */
	/*             = ((64000000/8)*100)/1/1                    */
	/*             = (8000000*100)/1/1                         */
	/*             = 800000000 (800 MHz)                       */
	/* PLL2 output = HSI (64 MHz)                              */
	/* PLL3 output = HSI (64 MHz)                              */
	/* PLL4 output = HSI (64 MHz)                              */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL1.PLLM = 4;
	RCC_OscInitStruct.PLL1.PLLN = 75;
	RCC_OscInitStruct.PLL1.PLLP1 = 1;
	RCC_OscInitStruct.PLL1.PLLP2 = 1;
	RCC_OscInitStruct.PLL1.PLLFractional = 0;


	// PLL2: 64 x 125 / 8 = 1000MHz
	RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL2.PLLM = 8;
	RCC_OscInitStruct.PLL2.PLLFractional = 0;
	RCC_OscInitStruct.PLL2.PLLN = 125;
	RCC_OscInitStruct.PLL2.PLLP1 = 1;
	RCC_OscInitStruct.PLL2.PLLP2 = 1;

	// PLL3:bypass
	RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_BYPASS;
	RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;

	// PLL4: 64 x 40 / 32 = 80MHz
	RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL4.PLLM = 32;
	RCC_OscInitStruct.PLL4.PLLFractional = 0;
	RCC_OscInitStruct.PLL4.PLLN = 40;
	RCC_OscInitStruct.PLL4.PLLP1 = 1;
	RCC_OscInitStruct.PLL4.PLLP2 = 1;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/* Select PLL1 outputs as CPU and System bus clock source */
	/* CPUCLK (sysa_ck) = ic1_ck = PLL1 output/ic1_divider = 800 MHz */
	/* SYSCLK AXI (sysb_ck) = ic2_ck = PLL1 output/ic2_divider = 400 MHz */
	/* SYSCLK NPU (sysc_ck) = ic6_ck = PLL1 output/ic6_divider = 800 MHz */
	/* SYSCLK AXISRAM3/4/5/6 (sysd_ck) = ic11_ck = PLL1 output/ic11_divider = 800 MHz */
	/* Configure the HCLK, PCLK1, PCLK2, PCLK4 and PCLK5 clocks dividers */
	/* HCLK = ic2_ck = PLL1 output/HCLK divider = 200 MHz */
	/* PCLKx = HCLK / PCLKx divider = 200 MHz */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK  | \
			RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4  | RCC_CLOCKTYPE_PCLK5);
	RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
	RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
	RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
	RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC6Selection.ClockDivider = 2;
	RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC11Selection.ClockDivider = 2;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
	RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
#endif
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC2;
	PeriphClkInit.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_IC4;
	PeriphClkInit.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
	PeriphClkInit.ICSelection[RCC_IC4].ClockDivider = 6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		while (1);
	}
}

void BSP_CAMERA_FrameEventCallback(uint32_t instance)
{
	UNUSED(instance);
	/* swap buffers and signal new frame*/
	img_addr = DCMIPP->P1STM0AR;
	cam_frame_counter++;

	if (lcd_enabled) {
		BSP_LCD_SetLayerAddress(0, 0, img_addr);
		BSP_LCD_Reload(0, BSP_LCD_RELOAD_VERTICAL_BLANKING);
	}

	buf_index_changed = 1;
}

HAL_StatusTypeDef MX_LTDC_ClockConfig(LTDC_HandleTypeDef *hltdc)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hltdc);

	HAL_StatusTypeDef         status = HAL_OK;
	RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};

	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	RCC_PeriphCLKInitStruct.LtdcClockSelection = RCC_LTDCCLKSOURCE_IC16;
	RCC_PeriphCLKInitStruct.ICSelection[RCC_IC16].ClockSelection = RCC_ICCLKSOURCE_PLL4;
	RCC_PeriphCLKInitStruct.ICSelection[RCC_IC16].ClockDivider = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
	{
		status = HAL_ERROR;
	}

	return status;
}

/****************** RTC ****************************/

/**
  * @brief RTC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hrtc->Instance==RTC)
  {
    /* USER CODE BEGIN RTC_MspInit 0 */

    /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    __HAL_RCC_RTC_CLK_ENABLE();
    /* USER CODE BEGIN RTC_MspInit 1 */

    /* USER CODE END RTC_MspInit 1 */

  }

}

/**
  * @brief RTC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* USER CODE BEGIN RTC_MspDeInit 0 */

    /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTCAPB_CLK_DISABLE();
    __HAL_RCC_RTC_CLK_DISABLE();
    /* USER CODE BEGIN RTC_MspDeInit 1 */

    /* USER CODE END RTC_MspDeInit 1 */
  }

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

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_SecureStateTypeDef secureState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  secureState.rtcSecureFull = RTC_SECURE_FULL_YES;
  secureState.backupRegisterStartZone2 = RTC_BKP_DR0;
  secureState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_SecureModeSet(&hrtc, &secureState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

static void RTC_InitTime(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	sTime.Hours = 0x13;
	sTime.Minutes = 0x32;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JUNE;
	sDate.Date = 0x04;
	sDate.Year = 0x26;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

}

/**
 * @brief  Display the current time and date.
 *   showtime : pointer to buffer
 *   showdate : pointer to buffer
 * @retval None
 */
static void RTC_CalendarShow(void)
{
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	/* Display time Format : hh:mm:ss */
	sprintf((char *)aShowTime, "%.2d:%.2d:%.2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	printf("%s \r\n",aShowTime);

	/* Display date Format : mm-dd-yy */
	sprintf((char *)aShowDate, "%.2d-%.2d-%.2d", sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);
	printf("%s \r\n",aShowDate);
}

static void RTC_PrintTimestamp(char *buf, int size)
{
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	/* Display time Format : hh:mm:ss */
	snprintf(buf, size, "%04d%02d%02d-%02d%02d%02d",
			2000 + sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date,
			stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	printf("TS [%s]\r\n",buf);
}

/****************** /RTC ***************************/





void Error_Handler(void)
{
	BSP_LED_Off(LED_GREEN);
	while (1)
	{
		BSP_LED_Toggle(LED_RED);
		HAL_Delay(250);
	}
}

static void MPU_Config(void)
{
	MPU_Region_InitTypeDef default_config = {0};
	MPU_Attributes_InitTypeDef attr_config = {0};
	uint32_t primask_bit = __get_PRIMASK();
	__disable_irq();

	/* disable the MPU */
	HAL_MPU_Disable();

	/* create an attribute configuration for the MPU */
	attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
	attr_config.Number = MPU_ATTRIBUTES_NUMBER0;

	HAL_MPU_ConfigMemoryAttributes(&attr_config);

	/* Create a non cacheable region */
	/*Normal memory type, code execution unallowed */
	default_config.Enable = MPU_REGION_ENABLE;
	default_config.Number = MPU_REGION_NUMBER0;
	default_config.BaseAddress = __NON_CACHEABLE_SECTION_BEGIN;
	default_config.LimitAddress =  __NON_CACHEABLE_SECTION_END-1;
	default_config.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	default_config.AccessPermission = MPU_REGION_ALL_RW;
	default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
	HAL_MPU_ConfigRegion(&default_config);

	/* enable the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

	/* Exit critical section to lock the system and avoid any issue around MPU mechanisme */
	__set_PRIMASK(primask_bit);
}


static EXTI_HandleTypeDef hpb_exti_user;
static EXTI_HandleTypeDef hpb_exti_tamp;
static EXTI_HandleTypeDef hpb_exti_pir;
static EXTI_HandleTypeDef hpb_exti_sd_det;

// GPIO E0 .. TAMP button
void EXTI0_IRQHandler(void) {
	printf("EXTI0\r\n");

	HAL_EXTI_ClearPending(&hpb_exti_tamp, EXTI_TRIGGER_FALLING);

//	handleInteraction();
	tx_semaphore_put(&button_semaphore);
}

// GPIO 11 IRQ handler --> PIR interrupt!
void EXTI11_IRQHandler(void) {

//	EXTI->FPR1 = 0x800; // clear interrupt.
	// or use HAL_EXTI_ClearPending(hexti, Edge);
	HAL_EXTI_ClearPending(&hpb_exti_pir, EXTI_TRIGGER_FALLING);

	int level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
	printf("EXTI11 PIR is %x\r\n", level);

	pir_irq_occured++;

	if (myState == FILE_OPENED) {
		printf("CAPTURING, but reseting frame counter from %d!\r\n", frame_nb);
		frame_nb=0;
	} else {
		printf("IDLE, starting up!\r\n");
		notify_open();

		// allow some time for file to open
		tx_thread_sleep(50);

		frame_nb = 0;
		state = FILE_OPENED;
	}

	tx_semaphore_put(&button_semaphore);
}

// some docs on EXTI / GPIO IRQs (not necessarily related to the n6):
// https://wiki.st.com/stm32mcu/wiki/Getting%20started%20with%20EXTI
// https://community.st.com/t5/stm32-mcus/how-to-place-and-execute-stm32-code-in-sram-memory-with/ta-p/49528

// GPIO N12 IRQ handler --> SD detect pin
void EXTI12_IRQHandler(void) {
	printf("EXTI12 SD Detect\r\n");

	if (HAL_EXTI_GetPending(&hpb_exti_sd_det, EXTI_TRIGGER_FALLING)) {
		HAL_EXTI_ClearPending(&hpb_exti_sd_det, EXTI_TRIGGER_FALLING);
	}
	if (HAL_EXTI_GetPending(&hpb_exti_sd_det, EXTI_TRIGGER_RISING)) {
		HAL_EXTI_ClearPending(&hpb_exti_sd_det, EXTI_TRIGGER_RISING);
	}

	notify_card_change();
}

// GPIO C13  USER button
void EXTI13_IRQHandler(void) {
	printf("EXTI13\r\n");
	HAL_EXTI_ClearPending(&hpb_exti_user, EXTI_TRIGGER_FALLING);

	tx_semaphore_put(&button_semaphore);
//	handleInteraction();
}


static void init_sensor_pins()
{
	GPIO_InitTypeDef gpio_init_structure;

	__HAL_RCC_GPIOC_CLK_ENABLE();

	// PIR sensor on PC11

	/* Configure Interrupt mode for SD detection pin PN12 */
	gpio_init_structure.Pin     = GPIO_PIN_11;
	gpio_init_structure.Pull    = GPIO_NOPULL;
	gpio_init_structure.Speed   = GPIO_SPEED_FREQ_LOW;
	gpio_init_structure.Mode    = GPIO_MODE_INPUT;

	HAL_GPIO_Init(GPIOC, &gpio_init_structure);


	/* PIR EXTI interrupt init*/

	/* EXTI interrupt init; only one pin per exti line supported! */

#define BUTTON_PIR_EXTI_IRQn 			EXTI11_IRQn
#define BUTTON_PIR_EXTI_LINE            EXTI_LINE_11

	(void)HAL_EXTI_GetHandle(&hpb_exti_pir, BUTTON_PIR_EXTI_LINE);
	//	(void)HAL_EXTI_RegisterCallback(&hpb_exti_pir,  HAL_EXTI_COMMON_CB_ID, PIR_Sensor_EXTI_Callback);

	const EXTI_ConfigTypeDef extiConfig = {
			.Line = BUTTON_PIR_EXTI_LINE,
			.Mode = EXTI_MODE_INTERRUPT,
			.Trigger = EXTI_TRIGGER_FALLING,
			.GPIOSel = EXTI_GPIOC
	};

	HAL_EXTI_SetConfigLine(&hpb_exti_pir, &extiConfig);

	HAL_NVIC_SetPriority(BUTTON_PIR_EXTI_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(BUTTON_PIR_EXTI_IRQn);


	HAL_EXTI_GetHandle(&hpb_exti_tamp, EXTI_LINE_0);    // TAMP
	HAL_EXTI_GetHandle(&hpb_exti_sd_det, EXTI_LINE_12); // SD Detect
	HAL_EXTI_GetHandle(&hpb_exti_user, EXTI_LINE_13);   // USER


#define SD_DET_EXTI_IRQn 			EXTI12_IRQn
#define SD_DET_EXTI_LINE            EXTI_LINE_12


	const EXTI_ConfigTypeDef sdExtiConfig = {
			.Line = SD_DET_EXTI_LINE,
			.Mode = EXTI_MODE_INTERRUPT,
			.Trigger = EXTI_TRIGGER_RISING_FALLING,
			.GPIOSel = EXTI_GPION
	};

	HAL_EXTI_SetConfigLine(&hpb_exti_sd_det, &sdExtiConfig);

	HAL_NVIC_SetPriority(SD_DET_EXTI_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(SD_DET_EXTI_IRQn);

}


void EWLPoolChoiceCb(u8 **pool_ptr, size_t *size)
{
	*pool_ptr = ewl_pool;
	*size = sizeof(ewl_pool);
}

void EWLPoolReleaseCb(u8 **pool_ptr)
{
	UNUSED(pool_ptr);
}

/********************** threadx low power stuff *******************/
/**
 * @brief  App_ThreadX_LowPower_Enter
 * @param  None
 * @retval None
 */
void App_ThreadX_LowPower_Enter(void)
{
	/* USER CODE BEGIN  App_ThreadX_LowPower_Enter */
	//  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	BSP_LED_Toggle(LED_RED);
	/* Enter to the stop mode */
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	/* USER CODE END  App_ThreadX_LowPower_Enter */
}

/**
 * @brief  App_ThreadX_LowPower_Exit
 * @param  None
 * @retval None
 */
void App_ThreadX_LowPower_Exit(void)
{
	/* USER CODE BEGIN  App_ThreadX_LowPower_Exit */
	BSP_LED_On(LED_RED);
	/* Reconfigure the system clock*/
	HAL_RCC_DeInit();
	SystemClock_Config();
	/* USER CODE END  App_ThreadX_LowPower_Exit */
}




#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file  pointer to the source file name
 * @param  line  assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	TRACE_MAIN("assert failed at line %lu of file %s\n", line, file);
	/* Infinite loop */
	while (1)
	{
	}
}
#endif
