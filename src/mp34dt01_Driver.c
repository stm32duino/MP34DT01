/**
******************************************************************************
* @file    mp34dt01_Driver.c
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

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mp34dt01_Driver.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup MP34DT01_L4_DRIVER
* @{
*/

/** @addtogroup MP34DT01_L4_DRIVER_AUDIO_L4
* @brief This file provides set of firmware functions to manage MEMS microphones
*        initialization on STM32L4xx-Nucleo Kit from STMicroelectronics.
* @{
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Types
* @{
*/

/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Defines
* @{
*/

/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Macros
* @{
*/

/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Variables
* @{
*/

static MP34DT01_HandlerTypeDef MP34DT01_Handler;
DFSDM_Channel_HandleTypeDef haudio_in_dfsdmchannel[4];
DFSDM_Filter_HandleTypeDef haudio_in_dfsdmfilter[4];
DMA_HandleTypeDef hdma_dfsdmReg_FLT[4];
int32_t RecBuff[MAX_CH_NUMBER][MAX_SAMPLES_PER_CH * N_MS_PER_INTERRUPT];
static uint16_t AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;

/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Function_Prototypes
* @{
*/

static void DFSDMx_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static uint8_t DFSDMx_Init(uint32_t AudioFreq, uint32_t ChnlNbr,
                            PinName CKOUT, PinName DATIN);
static uint8_t DFSDMx_DeInit(PinName CKOUT, PinName DATIN);
static void DFSDMx_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

static uint8_t dfsdm_enable_clock(DFSDM_Channel_TypeDef *channel_instance);
static DFSDM_Channel_TypeDef *dfsdm_next_channel(PinName DATIN, uint8_t index);


/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Exported_Functions
* @{
*/

/**
* @brief  Initializes audio acquisition.
* @param  AudioFreq: Audio frequency to be configured for the peripherals.
* 		  Possible values are 8000, 16000, 32000, 48000 OR 96000 Hz
* @param  BitRes: Not used in this release.
* @param  ChnlNbr: Number of channel to be configured.
* @param  ckout: pin number of clock out signal from DFSDSM module
* @param  datin: pin number of data in signal to DFSDM module
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t MP34DT01_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr,
                      PinName ckout, PinName datin)
{
  UNUSED(BitRes);
  uint32_t ret = AUDIO_ERROR;

  /*Set Structure for internal state*/
  MP34DT01_Handler.MicChannels = ChnlNbr;
  MP34DT01_Handler.Sampling_Freq = AudioFreq;

  MP34DT01_ClockConfig(MP34DT01_Handler.Sampling_Freq, NULL);

  /* SAI data transfer preparation:
  Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
  ret = DFSDMx_Init(AudioFreq, ChnlNbr, ckout, datin);

  /* Return AUDIO_OK when all operations are correctly done */
  return ret;
}

/**
* @brief  DeInitializes the audio peripheral.
* @retval None
*/
uint8_t MP34DT01_DeInit(PinName ckout, PinName datin)
{
  return DFSDMx_DeInit(ckout, datin);
}

/**
* @brief  Clock Config.
* @param  Params: additional parameters where required
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @note   This API is called by MP34DT01_Init()
*         Being __weak it can be overwritten by the application
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
__weak uint8_t MP34DT01_ClockConfig(uint32_t AudioFreq, void *Params)
{
  UNUSED(Params);
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  switch (AudioFreq) {
  case AUDIO_FREQUENCY_8K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 37;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 17;
    break;
  }
  case AUDIO_FREQUENCY_16K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 37;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 17;
    break;
  }
  case AUDIO_FREQUENCY_32K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  case AUDIO_FREQUENCY_48K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  case AUDIO_FREQUENCY_96K: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  default: {
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
    break;
  }
  }

  /* Configure PLLSAI prescalers */
  /* Please note that some of these parameters must be consistent with
  the parameters of the main PLL */
  RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  RCC_ExCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M = 6;

  if(HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;
}

/**
* @brief  Starts audio recording.
* @param  * pbuf: Buffer that will contain 1 ms of PCM for each microphone.
Its dimension must be equal to (in uint16_t words):
((PCM sampling frequency)/1000 * Channels)
* @param  size: Not used in this driver.
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01_Record(uint16_t* pbuf, uint32_t size)
{
  UNUSED(size);
  int32_t counter = 0;
  MP34DT01_Handler.PCM_Data = pbuf;

  for (counter = MP34DT01_Handler.MicChannels; counter > 0; counter --)
  {
    if (HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdmfilter[counter-1],
                                                   (int32_t*) RecBuff[counter-1],
                                                   MP34DT01_Handler.Sampling_Freq / 1000 * 2 * N_MS_PER_INTERRUPT))
    {
      return AUDIO_ERROR;
    }
  }
  return AUDIO_OK;
}

/**
* @brief  Stops audio recording.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01_Stop(void)
{
  int32_t counter = 0;
  for (counter = MP34DT01_Handler.MicChannels; counter > 0; counter --)
  {
    if (HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdmfilter[counter-1]))
    {
      return AUDIO_ERROR;
    }
  }
  /* Return 0 if all operations are OK */
  return AUDIO_OK;
}


/**
* @brief  Pauses the audio file stream.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01_Pause(void)
{
  int32_t counter = 0;
  for (counter = MP34DT01_Handler.MicChannels; counter > 0; counter --)
  {
    if (HAL_OK != HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdmfilter[counter-1]))
    {
      return AUDIO_ERROR;
    }
  }
  /* Return 0 if all operations are OK */
  return AUDIO_OK;
}

/**
* @brief  Resumes the audio file stream.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01_Resume(void)
{
  int32_t counter = 0;
  for (counter = MP34DT01_Handler.MicChannels; counter > 0; counter --)
  {
    if (HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdmfilter[counter-1],
                                                   (int32_t*) RecBuff[counter-1],
                                                   MP34DT01_Handler.Sampling_Freq / 1000 * 2))
    {
      return AUDIO_ERROR;
    }
  }
  return AUDIO_OK;
}


/**
* @brief  Controls the audio in volume level.
* @param  Volume: Volume level to be set. This value has the same behaviour of the
Volume parameter of the PDM to PCM software decimation library. Other
strategies are possible in order to control the volume, for example
to act on the right bit shift amount of the DFSDM peripheral
Values must be in the range from 0 to 64
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01_SetVolume(uint8_t Volume)
{
  AudioInVolume = Volume;
  return AUDIO_OK;
}

/**
* @brief  Audio IN Error callback function
* @param  None
* @retval None
*/
__weak void MP34DT01_Error_Callback(void)
{
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/**
* @}
*/

/** @defgroup MP34DT01_L4_DRIVER_AUDIO_L4_Private_Functions
* @{
*/

/******************************************************************************
Static Functions
*******************************************************************************/

/**
* @brief  Initializes the DFSDM peripheral clock.
* @param  channel_instance: Channel instance used.
* @retval 1 if DFSDM1 or 2 if DFSDM2 else 0
*/
static uint8_t dfsdm_enable_clock(DFSDM_Channel_TypeDef *channel_instance)
{
  uint8_t tmp;

  switch((uint32_t)channel_instance) {
    case ((uint32_t)DFSDM1_Channel0):
    case ((uint32_t)DFSDM1_Channel1):
    case ((uint32_t)DFSDM1_Channel2):
    case ((uint32_t)DFSDM1_Channel3):
#ifdef DFSDM1_Channel4
    case ((uint32_t)DFSDM1_Channel4):
    case ((uint32_t)DFSDM1_Channel5):
    case ((uint32_t)DFSDM1_Channel6):
    case ((uint32_t)DFSDM1_Channel7):
#endif // DFSDM1_Channel4
      __HAL_RCC_DFSDM1_CLK_ENABLE();
      tmp = 1;
    break;

#ifdef DFSDM2_Channel0
    case ((uint32_t)DFSDM2_Channel0):
    case ((uint32_t)DFSDM2_Channel1):
    case ((uint32_t)DFSDM2_Channel2):
    case ((uint32_t)DFSDM2_Channel3):
#ifdef DFSDM2_Channel4
    case ((uint32_t)DFSDM2_Channel4):
    case ((uint32_t)DFSDM2_Channel5):
    case ((uint32_t)DFSDM2_Channel6):
    case ((uint32_t)DFSDM2_Channel7):
#endif // DFSDM2_Channel4
      __HAL_RCC_DFSDM2_CLK_ENABLE();
      tmp = 2;
    break;
#endif // DFSDM2_Channel0

    default:
      tmp = 0;
    break;
  }

  return tmp;
}

/**
* @brief  Initializes the DFSDM peripheral clock.
* @PinName  PinName: DATIN pin attached to a DFSDM channel in.
* @retval Channel instance pointer if correct, else NULL pointer
*/
static DFSDM_Channel_TypeDef *dfsdm_next_channel(PinName DATIN, uint8_t index)
{
  DFSDM_Channel_TypeDef *hchan;
  uint32_t function = pinmap_function(DATIN, PinMap_DFSDM_DATIN);

  if(function == (uint32_t)NC)
    return NULL;

  uint8_t channel = STM_PIN_CHANNEL(function);
  uint8_t module = dfsdm_enable_clock(pinmap_peripheral(DATIN, PinMap_DFSDM_DATIN));

  if(module == 1) {
    while(index > 0) {
      if(channel == 0) {
#ifdef DFSDM1_Channel4
        channel = 7;
#else
        channel = 3;
#endif
      } else {
        channel--;
      }
      index--;
    }

    switch(channel) {
      case 0:
        hchan = DFSDM1_Channel0; break;
      case 1:
        hchan = DFSDM1_Channel1; break;
      case 2:
        hchan = DFSDM1_Channel2; break;
      case 3:
        hchan = DFSDM1_Channel3; break;
  #ifdef DFSDM1_Channel4
      case 4:
        hchan = DFSDM1_Channel4; break;
      case 5:
        hchan = DFSDM1_Channel5; break;
      case 6:
        hchan = DFSDM1_Channel6; break;
      case 7:
        hchan = DFSDM1_Channel7; break;
  #endif // DFSDM1_Channel4
      default: hchan = NULL; break;
    }
  }
#ifdef DFSDM2_Channel0
  else if(module == 2) {
    while(index > 0) {
      if(channel == 0) {
#ifdef DFSDM2_Channel4
        channel = 7;
#else
        channel = 3;
#endif
      } else {
        channel--;
      }
      index--;
    }

    switch(channel) {
      case 0:
        hchan = DFSDM2_Channel0; break;
      case 1:
        hchan = DFSDM2_Channel1; break;
      case 2:
        hchan = DFSDM2_Channel2; break;
      case 3:
        hchan = DFSDM2_Channel3; break;
#ifdef DFSDM2_Channel4
      case 4:
        hchan = DFSDM2_Channel4; break;
      case 5:
        hchan = DFSDM2_Channel5; break;
      case 6:
        hchan = DFSDM2_Channel6; break;
      case 7:
        hchan = DFSDM2_Channel7; break;
#endif // DFSDM2_Channel4
      default: hchan = NULL; break;
    }
  }
#endif //DFSDM2_Channel0
  else {
    return NULL;
  }

  return hchan;
}

/**
* @brief  Initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
* @param  AudioFreq: Audio frequency to be used to set correctly the DFSDM peripheral.
* @retval AUDIO_OK if correct communication, else wrong communication
*/
static uint8_t DFSDMx_Init(uint32_t AudioFreq, uint32_t ChnlNbr,
                           PinName CKOUT, PinName DATIN) {

  uint16_t shift_amount = 0;
  uint32_t SincOrder = 0;
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_TypeDef *port;

  DFSDM_Channel_TypeDef *dfsdm_ckout = pinmap_peripheral(CKOUT, PinMap_DFSDM_CKOUT);
  DFSDM_Channel_TypeDef *dfsdm_datin = pinmap_peripheral(DATIN, PinMap_DFSDM_DATIN);

  if((dfsdm_ckout == NP || dfsdm_datin == NP)) {
    printf("ERROR: DFSDM pins mismatch\n");
    return AUDIO_ERROR;
  }

  if(OverSampling(AudioFreq) == 32)
  {
    shift_amount = 5;
    SincOrder = DFSDM_FILTER_SINC5_ORDER;
  }
  else if(OverSampling(AudioFreq) == 128)
  {
    shift_amount = 8;
    SincOrder = DFSDM_FILTER_SINC4_ORDER;
  }
  else if(OverSampling(AudioFreq) == 64)
  {
    shift_amount = 10;
    SincOrder = DFSDM_FILTER_SINC5_ORDER;
  }

  /*####AUDIO CH 1: DFSDM CHANNEL 2####*/
  haudio_in_dfsdmchannel[0].Init.OutputClock.Activation   = ENABLE;
  haudio_in_dfsdmchannel[0].Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  /* Set the DFSDM clock OUT audio frequency configuration */
  haudio_in_dfsdmchannel[0].Init.OutputClock.Divider      = ClockDivider(AudioFreq);
  haudio_in_dfsdmchannel[0].Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  haudio_in_dfsdmchannel[0].Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  haudio_in_dfsdmchannel[0].Init.Input.Pins               = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  haudio_in_dfsdmchannel[0].Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_RISING;
  haudio_in_dfsdmchannel[0].Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  haudio_in_dfsdmchannel[0].Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  haudio_in_dfsdmchannel[0].Init.Awd.Oversampling         = 10;
  haudio_in_dfsdmchannel[0].Init.Offset                   = 0;
  haudio_in_dfsdmchannel[0].Init.RightBitShift            = shift_amount;
  haudio_in_dfsdmchannel[0].Instance                      = dfsdm_datin;

  /* Enable DFSDM clock */
  dfsdm_enable_clock(dfsdm_datin);

  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  port = set_GPIO_Port_Clock(STM_PORT(CKOUT));
  GPIO_InitStruct.Pin = STM_GPIO_PIN(CKOUT);
  GPIO_InitStruct.Mode = STM_PIN_MODE(pinmap_function(CKOUT,PinMap_DFSDM_CKOUT));
  GPIO_InitStruct.Pull = STM_PIN_PUPD(pinmap_function(CKOUT,PinMap_DFSDM_CKOUT));
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = STM_PIN_AFNUM(pinmap_function(CKOUT,PinMap_DFSDM_CKOUT));
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  port = set_GPIO_Port_Clock(STM_PORT(DATIN));
  GPIO_InitStruct.Pin = STM_GPIO_PIN(DATIN);
  GPIO_InitStruct.Mode = STM_PIN_MODE(pinmap_function(DATIN,PinMap_DFSDM_DATIN));
  GPIO_InitStruct.Pull = STM_PIN_PUPD(pinmap_function(DATIN,PinMap_DFSDM_DATIN));
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = STM_PIN_AFNUM(pinmap_function(DATIN,PinMap_DFSDM_DATIN));
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdmchannel[0]) != HAL_DFSDM_CHANNEL_STATE_RESET)
  {
    return AUDIO_ERROR;
  }

  if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdmchannel[0]))
  {
    return AUDIO_ERROR;
  }

  /*####AUDIO CH 1: FILTER 0####*/
  haudio_in_dfsdmfilter[0].Init.RegularParam.Trigger         = DFSDM_FILTER_SW_TRIGGER;
  haudio_in_dfsdmfilter[0].Init.RegularParam.FastMode        = ENABLE;
  haudio_in_dfsdmfilter[0].Init.RegularParam.DmaMode         = ENABLE;
  haudio_in_dfsdmfilter[0].Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  haudio_in_dfsdmfilter[0].Init.InjectedParam.ScanMode       = DISABLE;
  haudio_in_dfsdmfilter[0].Init.InjectedParam.DmaMode        = DISABLE;
  haudio_in_dfsdmfilter[0].Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  haudio_in_dfsdmfilter[0].Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  haudio_in_dfsdmfilter[0].Init.FilterParam.SincOrder        = SincOrder;
  /* Set the DFSDM Filters Oversampling to have correct sample rate */
  haudio_in_dfsdmfilter[0].Init.FilterParam.Oversampling     = OverSampling(AudioFreq);
  haudio_in_dfsdmfilter[0].Init.FilterParam.IntOversampling  = 1;
  haudio_in_dfsdmfilter[0].Instance                          = AUDIO_IN_DFSDM_1st_FILTER;

  if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdmfilter[0]) == HAL_DFSDM_FILTER_STATE_RESET)
  {
    /* Init the DFSDM Filter */
    DFSDMx_FilterMspInit(&haudio_in_dfsdmfilter[0]);
  }

  if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdmfilter[0]))
  {
    return AUDIO_ERROR;
  }

  /* Configure injected channel */
  if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdmfilter[0], DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON))
  {
    return AUDIO_ERROR;
  }

  if (ChnlNbr > 1)
  {
    /*####AUDIO CH 2: DFSDM CHANNEL 1####*/
    haudio_in_dfsdmchannel[1].Init.OutputClock.Activation   = ENABLE;
    haudio_in_dfsdmchannel[1].Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
    /* Set the DFSDM clock OUT audio frequency configuration */
    haudio_in_dfsdmchannel[1].Init.OutputClock.Divider      = ClockDivider(AudioFreq);
    haudio_in_dfsdmchannel[1].Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    haudio_in_dfsdmchannel[1].Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
    haudio_in_dfsdmchannel[1].Init.Input.Pins               = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
    haudio_in_dfsdmchannel[1].Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_FALLING;
    haudio_in_dfsdmchannel[1].Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    haudio_in_dfsdmchannel[1].Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
    haudio_in_dfsdmchannel[1].Init.Awd.Oversampling         = 10;
    haudio_in_dfsdmchannel[1].Init.Offset                   = 0;
    haudio_in_dfsdmchannel[1].Init.RightBitShift            = shift_amount;
    haudio_in_dfsdmchannel[1].Instance                      = dfsdm_next_channel(DATIN, 1);

    if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdmchannel[1]) != HAL_DFSDM_CHANNEL_STATE_RESET)
    {
      return AUDIO_ERROR;
    }
    if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdmchannel[1]))
    {
      return AUDIO_ERROR;
    }
    /*####AUDIO CH 2: FILTER 1####*/
    haudio_in_dfsdmfilter[1].Init.RegularParam.Trigger         = DFSDM_FILTER_SYNC_TRIGGER;
    haudio_in_dfsdmfilter[1].Init.RegularParam.FastMode        = ENABLE;
    haudio_in_dfsdmfilter[1].Init.RegularParam.DmaMode         = ENABLE;
    haudio_in_dfsdmfilter[1].Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
    haudio_in_dfsdmfilter[1].Init.InjectedParam.ScanMode       = ENABLE;
    haudio_in_dfsdmfilter[1].Init.InjectedParam.DmaMode        = ENABLE;
    haudio_in_dfsdmfilter[1].Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
    if (ChnlNbr == 1)
    {
      haudio_in_dfsdmfilter[1].Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_RISING_EDGE;
    }
    else
    {
      haudio_in_dfsdmfilter[1].Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
    }
    haudio_in_dfsdmfilter[1].Init.FilterParam.SincOrder        = SincOrder;
    /* Set the DFSDM Filters Oversampling to have correct sample rate */
    haudio_in_dfsdmfilter[1].Init.FilterParam.Oversampling     = OverSampling(AudioFreq);
    haudio_in_dfsdmfilter[1].Init.FilterParam.IntOversampling  = 1;
    haudio_in_dfsdmfilter[1].Instance                          = AUDIO_IN_DFSDM_2st_FILTER;

    if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdmfilter[1]) == HAL_DFSDM_FILTER_STATE_RESET)
    {
      /* Init the DFSDM Filter */
      DFSDMx_FilterMspInit(&haudio_in_dfsdmfilter[1]);
    }

    if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdmfilter[1]))
    {
      return AUDIO_ERROR;
    }

    /* Configure injected channel */
    if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdmfilter[1], DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON))
    {
      return AUDIO_ERROR;
    }
  }

  if (ChnlNbr > 2)
  {
    /*####AUDIO CH 3: CHANNEL 7####*/
    haudio_in_dfsdmchannel[2].Init.OutputClock.Activation   = ENABLE;
    haudio_in_dfsdmchannel[2].Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
    /* Set the DFSDM clock OUT audio frequency configuration */
    haudio_in_dfsdmchannel[2].Init.OutputClock.Divider      = ClockDivider(AudioFreq);
    haudio_in_dfsdmchannel[2].Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    haudio_in_dfsdmchannel[2].Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
    haudio_in_dfsdmchannel[2].Init.Input.Pins               = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
    haudio_in_dfsdmchannel[2].Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_RISING;
    haudio_in_dfsdmchannel[2].Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    haudio_in_dfsdmchannel[2].Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
    haudio_in_dfsdmchannel[2].Init.Awd.Oversampling         = 10;
    haudio_in_dfsdmchannel[2].Init.Offset                   = 0;
    haudio_in_dfsdmchannel[2].Init.RightBitShift            = shift_amount;
    haudio_in_dfsdmchannel[2].Instance                      = dfsdm_next_channel(DATIN, 2);

    if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdmchannel[2]) != HAL_DFSDM_CHANNEL_STATE_RESET)
    {
      return AUDIO_ERROR;
    }
    if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdmchannel[2]))
    {
      return AUDIO_ERROR;
    }

    /*####AUDIO CH 3: FILTER 2####*/
    haudio_in_dfsdmfilter[2].Init.RegularParam.Trigger         = DFSDM_FILTER_SYNC_TRIGGER;
    haudio_in_dfsdmfilter[2].Init.RegularParam.FastMode        = ENABLE;
    haudio_in_dfsdmfilter[2].Init.RegularParam.DmaMode         = ENABLE;
    haudio_in_dfsdmfilter[2].Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
    haudio_in_dfsdmfilter[2].Init.InjectedParam.ScanMode       = DISABLE;
    haudio_in_dfsdmfilter[2].Init.InjectedParam.DmaMode        = DISABLE;
    haudio_in_dfsdmfilter[2].Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
    haudio_in_dfsdmfilter[2].Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
    haudio_in_dfsdmfilter[2].Init.FilterParam.SincOrder        = SincOrder;
    /* Set the DFSDM Filters Oversampling to have correct sample rate */
    haudio_in_dfsdmfilter[2].Init.FilterParam.Oversampling     = OverSampling(AudioFreq);
    haudio_in_dfsdmfilter[2].Init.FilterParam.IntOversampling  = 1;
    haudio_in_dfsdmfilter[2].Instance                          = AUDIO_IN_DFSDM_3rd_FILTER;

    if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdmfilter[2]) == HAL_DFSDM_FILTER_STATE_RESET)
    {
      /* Init the DFSDM Filter */
      DFSDMx_FilterMspInit(&haudio_in_dfsdmfilter[2]);
    }

    if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdmfilter[2]))
    {
      return AUDIO_ERROR;
    }

    /* Configure injected channel */
    if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdmfilter[2], DFSDM_CHANNEL_7, DFSDM_CONTINUOUS_CONV_ON))
    {
      return AUDIO_ERROR;
    }
  }

  if (ChnlNbr > 3)
  {
    /*####AUDIO CH 4: CHANNEL 6####*/
    haudio_in_dfsdmchannel[3].Init.OutputClock.Activation   = ENABLE;
    haudio_in_dfsdmchannel[3].Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
    /* Set the DFSDM clock OUT audio frequency configuration */
    haudio_in_dfsdmchannel[3].Init.OutputClock.Divider      = ClockDivider(AudioFreq);
    haudio_in_dfsdmchannel[3].Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    haudio_in_dfsdmchannel[3].Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
    haudio_in_dfsdmchannel[3].Init.Input.Pins               = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
    haudio_in_dfsdmchannel[3].Init.SerialInterface.Type     = DFSDM_CHANNEL_SPI_FALLING;
    haudio_in_dfsdmchannel[3].Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    haudio_in_dfsdmchannel[3].Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
    haudio_in_dfsdmchannel[3].Init.Awd.Oversampling         = 10;
    haudio_in_dfsdmchannel[3].Init.Offset                   = 0;
    haudio_in_dfsdmchannel[3].Init.RightBitShift            = shift_amount;
    haudio_in_dfsdmchannel[3].Instance                      = dfsdm_next_channel(DATIN, 3);

    if(HAL_DFSDM_ChannelGetState(&haudio_in_dfsdmchannel[3]) != HAL_DFSDM_CHANNEL_STATE_RESET)
    {
      return AUDIO_ERROR;
    }

    if(HAL_OK != HAL_DFSDM_ChannelInit(&haudio_in_dfsdmchannel[3]))
    {
      return AUDIO_ERROR;
    }
    /*####AUDIO CH 4: FILTER 3####*/
    haudio_in_dfsdmfilter[3].Init.RegularParam.Trigger         = DFSDM_FILTER_SYNC_TRIGGER;
    haudio_in_dfsdmfilter[3].Init.RegularParam.FastMode        = ENABLE;
    haudio_in_dfsdmfilter[3].Init.RegularParam.DmaMode         = ENABLE;
    haudio_in_dfsdmfilter[3].Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
    haudio_in_dfsdmfilter[3].Init.InjectedParam.ScanMode       = DISABLE;
    haudio_in_dfsdmfilter[3].Init.InjectedParam.DmaMode        = DISABLE;
    haudio_in_dfsdmfilter[3].Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
    haudio_in_dfsdmfilter[3].Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
    haudio_in_dfsdmfilter[3].Init.FilterParam.SincOrder        = SincOrder;
    /* Set the DFSDM Filters Oversampling to have correct sample rate */
    haudio_in_dfsdmfilter[3].Init.FilterParam.Oversampling     = OverSampling(AudioFreq);
    haudio_in_dfsdmfilter[3].Init.FilterParam.IntOversampling  = 1;
    haudio_in_dfsdmfilter[3].Instance                          = AUDIO_IN_DFSDM_4th_FILTER;

    if(HAL_DFSDM_FilterGetState(&haudio_in_dfsdmfilter[3]) == HAL_DFSDM_FILTER_STATE_RESET)
    {
      /* Init the DFSDM Filter */
      DFSDMx_FilterMspInit(&haudio_in_dfsdmfilter[3]);
    }

    if(HAL_OK != HAL_DFSDM_FilterInit(&haudio_in_dfsdmfilter[3]))
    {
      return AUDIO_ERROR;
    }

    /* Configure injected channel */
    if(HAL_OK != HAL_DFSDM_FilterConfigRegChannel(&haudio_in_dfsdmfilter[3], DFSDM_CHANNEL_6, DFSDM_CONTINUOUS_CONV_ON))
    {
      return AUDIO_ERROR;
    }
  }
  return AUDIO_OK;
}

/**
* @brief  Initializes the DFSDM filter MSP.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
static void DFSDMx_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)

{
  DMA_HandleTypeDef *hdma_dfsdmReg = NULL;

  /* Enable the DMA clock */
  AUDIO_IN_DFSDM_DMA_CLK_ENABLE() ;

  if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_1st_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[0];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_2st_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[1];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_3rd_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[2];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_4th_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[3];
  }

  /* Configure the hdma_dfsdmReg handle parameters */
  hdma_dfsdmReg->Init.Request = DMA_REQUEST_0;
  hdma_dfsdmReg->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dfsdmReg->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dfsdmReg->Init.MemInc = DMA_MINC_ENABLE;
  hdma_dfsdmReg->Init.PeriphDataAlignment =
    AUDIO_IN_DFSDM_DMA_PERIPH_DATA_SIZE;
  hdma_dfsdmReg->Init.MemDataAlignment = AUDIO_IN_DFSDM_DMA_MEM_DATA_SIZE;
  hdma_dfsdmReg->Init.Mode = DMA_CIRCULAR;
  hdma_dfsdmReg->Init.Priority = DMA_PRIORITY_HIGH;

  if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_1st_FILTER)
  {
    hdma_dfsdmReg->Instance = AUDIO_IN_DFSDM_DMA_1st_CHANNEL;
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_2st_FILTER)
  {
    hdma_dfsdmReg->Instance = AUDIO_IN_DFSDM_DMA_2nd_CHANNEL;
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_3rd_FILTER)
  {
    hdma_dfsdmReg->Instance = AUDIO_IN_DFSDM_DMA_3rd_CHANNEL;
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_4th_FILTER)
  {
    hdma_dfsdmReg->Instance = AUDIO_IN_DFSDM_DMA_4th_CHANNEL;
  }

  /* Associate the DMA handle */
  __HAL_LINKDMA(hdfsdm_filter, hdmaReg, *hdma_dfsdmReg);

  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(hdma_dfsdmReg);

  /* Configure the DMA Channel */
  HAL_DMA_Init(hdma_dfsdmReg);

  /* DMA IRQ Channel configuration */
  if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_1st_FILTER)
  {
    HAL_NVIC_SetPriority(AUDIO_IN_DFSDM_DMA_1st_CH_IRQn,
                         1, AUDIO_IN_IRQ_PREPRIO);
    HAL_NVIC_EnableIRQ(AUDIO_IN_DFSDM_DMA_1st_CH_IRQn);
  }
}

/**
* @brief  De-initializes the Digital Filter for Sigma-Delta Modulators interface (DFSDM).
* @retval AUDIO_OK if correct communication, else wrong communication
*/
static uint8_t DFSDMx_DeInit(PinName CKOUT, PinName DATIN)
{
  /* De-initializes the DFSDM filters to allow access to DFSDM internal registers */
  uint16_t index = 0;
  GPIO_TypeDef *port;

  for (index = 0; index < MP34DT01_Handler.MicChannels; index ++)
  {
    if(HAL_OK != HAL_DFSDM_FilterDeInit(&haudio_in_dfsdmfilter[index]))
    {
      return AUDIO_ERROR;
    }
    if(HAL_OK != HAL_DFSDM_ChannelDeInit(&haudio_in_dfsdmchannel[index]))
    {
      return AUDIO_ERROR;
    }
    DFSDMx_FilterMspDeInit(&haudio_in_dfsdmfilter[index]);
  }

  port = get_GPIO_Port(STM_PORT(CKOUT));
  HAL_GPIO_DeInit(port, STM_GPIO_PIN(CKOUT));

  port = get_GPIO_Port(STM_PORT(DATIN));
  HAL_GPIO_DeInit(port, STM_GPIO_PIN(CKOUT));

  return AUDIO_OK;
}

/**
* @brief  DeInitializes the DFSDM filter MSP.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
static void DFSDMx_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DMA_HandleTypeDef *hdma_dfsdmReg = NULL;

  if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_1st_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[0];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_2st_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[1];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_3rd_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[2];
  }
  else if (hdfsdm_filter->Instance == AUDIO_IN_DFSDM_4th_FILTER)
  {
    hdma_dfsdmReg = &hdma_dfsdmReg_FLT[3];
  }

  /* Configure the DMA Channel */
  HAL_DMA_DeInit(hdma_dfsdmReg);
}



/**
* @brief  Regular conversion complete callback.
* @note   This function performs an HP filter in order to remove DC offset and arranges PCM data following the standard PCM format.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;

  if (hdfsdm_filter == &haudio_in_dfsdmfilter[0])
  {
    for(j=0; j < MP34DT01_Handler.MicChannels; j ++)
    {
      for (i = 0; i < (MP34DT01_Handler.Sampling_Freq / 1000) * N_MS_PER_INTERRUPT; i++)
      {
        MP34DT01_Handler.HP_Filters[j].Z = ((RecBuff[j][i + (MP34DT01_Handler.Sampling_Freq / 1000) * N_MS_PER_INTERRUPT] >> 8) * AudioInVolume) >> 7;
        MP34DT01_Handler.HP_Filters[j].oldOut = (0xFC * (MP34DT01_Handler.HP_Filters[j].oldOut + MP34DT01_Handler.HP_Filters[j].Z - MP34DT01_Handler.HP_Filters[j].oldIn)) / 256;
        MP34DT01_Handler.HP_Filters[j].oldIn = MP34DT01_Handler.HP_Filters[j].Z;
        MP34DT01_Handler.PCM_Data[i * MP34DT01_Handler.MicChannels + j] = SaturaLH(MP34DT01_Handler.HP_Filters[j].oldOut, -32760, 32760);
      }
    }
    MP34DT01_TransferComplete_CallBack();
  }


}

/**
* @brief  Half regular conversion complete callback.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;

  if (hdfsdm_filter == &haudio_in_dfsdmfilter[0])
  {
    for(j=0; j < MP34DT01_Handler.MicChannels; j ++)
    {
      for (i = 0; i < (MP34DT01_Handler.Sampling_Freq / 1000) * N_MS_PER_INTERRUPT; i++)
      {
        MP34DT01_Handler.HP_Filters[j].Z = ((RecBuff[j][i] >> 8) * AudioInVolume) >> 7;
        MP34DT01_Handler.HP_Filters[j].oldOut = (0xFC * (MP34DT01_Handler.HP_Filters[j].oldOut + MP34DT01_Handler.HP_Filters[j].Z - MP34DT01_Handler.HP_Filters[j].oldIn)) / 256;
        MP34DT01_Handler.HP_Filters[j].oldIn = MP34DT01_Handler.HP_Filters[j].Z;
        MP34DT01_Handler.PCM_Data[i * MP34DT01_Handler.MicChannels + j] = SaturaLH(MP34DT01_Handler.HP_Filters[j].oldOut, -32760, 32760);
      }
    }
    MP34DT01_HalfTransfer_CallBack();
  }
}

/**
* @brief  User callback when record buffer is filled.
* @param  None
* @retval None
*/
__weak void MP34DT01_TransferComplete_CallBack(void) {
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  User callback when record buffer is half filled.
* @param  None
* @retval None
*/
__weak void MP34DT01_HalfTransfer_CallBack(void) {
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
  * @brief  This function handles DFSDM DMAinterrupt request.
  * @param  None
  * @retval None
  */
//void AUDIO_IN_DFSDM_DMA_1st_CH_IRQHandler(void)
void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_dfsdmReg_FLT[0]);
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
