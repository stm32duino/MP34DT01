/**
******************************************************************************
* @file    mp34dt01_Driver.h
* @author  Central Labs updated by WI6LABS
* @version V2.0.0
* @date    10-July-2017
* @brief   This file provides the Audio driver for the STM32 IOT discovery kit
*******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MP34DT01_DRIVER_H
#define __MP34DT01_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PeripheralPins.h"

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
 * @{
 */

/** @addtogroup MP34DT01_L4_DRIVER
 * @{
 */

/** @addtogroup MP34DT01_L4_DRIVER_AUDIO_L4
 * @{
 */

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Types
 * @{
 */

/**
 * @brief   HP filter state structure definition
 */
typedef struct {
	int32_t Z;
	int32_t oldOut;
	int32_t oldIn;
}HP_FilterState_TypeDef;


/**
 * @brief   Microphone internal structure definition
 */
typedef struct {
	uint32_t MicChannels; /*!< Specifies the number of channels */

	uint32_t Sampling_Freq; /*!< Specifies the desired sampling frequency */

	HP_FilterState_TypeDef HP_Filters[4]; /*!< HP filter state for each channel*/

	uint16_t * PCM_Data; /*!< Takes track of the external PCM data buffer as passed by the user in the start function*/

} MP34DT01_HandlerTypeDef;

/**
 * @}
 */


/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Constants
 * @{
 */

/* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_IN_IRQ_PREPRIO          0   /* Select the preemption priority level(0 is the highest) */

/*------------------------------------------------------------------------------
 CONFIGURATION: Audio Driver Configuration parameters
 ------------------------------------------------------------------------------*/

  /* Audio status definition */
#ifndef AUDIO_OK
#define AUDIO_OK                            ((uint8_t)0)
#endif

#ifndef AUDIO_ERROR
#define AUDIO_ERROR                         ((uint8_t)1)
#endif

#ifndef AUDIO_TIMEOUT
#define AUDIO_TIMEOUT                       ((uint8_t)2)
#endif

#define DEFAULT_AUDIO_IN_VOLUME               64

/**
 * @}
 */

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Variables
 * @{
 */
extern DMA_HandleTypeDef               hdma_dfsdmReg_FLT[];

/**
 * @}
 */

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Macros
 * @{
 */

 #define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define OverSampling(__FREQUENCY__) \
((__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 128 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 64 \
        : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 64 \
            : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 64  \
              : (__FREQUENCY__ == AUDIO_FREQUENCY_96K) ? 32 : 32)

#define ClockDivider(__FREQUENCY__) \
              ((__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 17 \
                  : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 17 \
                      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 24 \
                          : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 16  \
                            : (__FREQUENCY__ == AUDIO_FREQUENCY_96K) ? 16 : 16)
/**
 * @}
 */

  /** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Constants
  * @{
  */
#define MAX_CH_NUMBER           (4)
#define MAX_FS                  (96000)
#define MAX_SAMPLES_PER_CH      ((MAX_FS/1000)*2)

/* AUDIO FREQUENCY */
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)
#endif

/* DFSDM Configuration defines */
#define AUDIO_IN_DFSDM_1st_FILTER                               DFSDM_Filter0
#define AUDIO_IN_DFSDM_2st_FILTER                               DFSDM_Filter1
#define AUDIO_IN_DFSDM_3rd_FILTER                               DFSDM_Filter2
#define AUDIO_IN_DFSDM_4th_FILTER                               DFSDM_Filter3

/* I2S DMA Stream Rx definitions */
#define AUDIO_IN_DFSDM_DMA_CLK_ENABLE()                         __HAL_RCC_DMA1_CLK_ENABLE()

#define AUDIO_IN_DFSDM_DMA_1st_CHANNEL                          DMA1_Channel4
#define AUDIO_IN_DFSDM_DMA_2nd_CHANNEL                          DMA1_Channel5
#define AUDIO_IN_DFSDM_DMA_3rd_CHANNEL                          DMA1_Channel6
#define AUDIO_IN_DFSDM_DMA_4th_CHANNEL                          DMA1_Channel7

#define AUDIO_IN_DFSDM_DMA_1st_CH_IRQn                          DMA1_Channel4_IRQn
#define AUDIO_IN_DFSDM_DMA_2nd_CH_IRQn                          DMA1_Channel5_IRQn
#define AUDIO_IN_DFSDM_DMA_3rd_CH_IRQn                          DMA1_Channel6_IRQn
#define AUDIO_IN_DFSDM_DMA_4th_CH_IRQn                          DMA1_Channel7_IRQn

#define AUDIO_IN_DFSDM_DMA_PERIPH_DATA_SIZE                     DMA_PDATAALIGN_WORD
#define AUDIO_IN_DFSDM_DMA_MEM_DATA_SIZE                        DMA_MDATAALIGN_WORD

#define AUDIO_IN_DFSDM_DMA_1st_CH_IRQHandler                    DMA1_Channel4_IRQHandler
#define AUDIO_IN_DFSDM_DMA_2nd_CH_IRQHandler                    DMA1_Channel5_IRQHandler
#define AUDIO_IN_DFSDM_DMA_3rd_CH_IRQHandler                    DMA1_Channel6_IRQHandler
#define AUDIO_IN_DFSDM_DMA_4th_CH_IRQHandler                    DMA1_Channel7_IRQHandler

/**
 * @}
 */

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               1

   /** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Functions
  * @{
  */
uint8_t MP34DT01_SetVolume(uint8_t Volume);
uint8_t MP34DT01_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr, PinName ckout, PinName datin);
uint8_t MP34DT01_DeInit(PinName ckout, PinName datin);
uint8_t MP34DT01_Record(uint16_t* pbuf, uint32_t size);
uint8_t MP34DT01_Stop(void);
uint8_t MP34DT01_Pause(void);
uint8_t MP34DT01_Resume(void);
uint8_t MP34DT01_ClockConfig(uint32_t AudioFreq, void *Params);




/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
 It is called into this driver when the current buffer is filled to prepare the next
 buffer pointer and its size. */
void MP34DT01_TransferComplete_CallBack(void);
void MP34DT01_HalfTransfer_CallBack(void);
void MP34DT01_Error_Callback(void);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MP34DT01_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
