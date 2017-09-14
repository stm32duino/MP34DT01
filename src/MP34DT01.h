/**
******************************************************************************
* @file    MP32DT01.h
* @author  WI6LABS
* @version V0.0.1
* @date    11-July-2017
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
#ifndef __MP34DT01_H
#define __MP34DT01_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "mp34dt01_Driver.h"

#define MONO    1
#define STEREO  2

class MP34DT01 {

  public:

  /* Builder */
  MP34DT01(uint8_t ckout, uint8_t datin);

  /**
  * @brief  Initializes audio acquisition with default values
  *         AudioFreq = 32000, ChnlNbr = 2, 16bits resolution
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
  uint8_t begin(void) {
    return begin(AUDIO_FREQUENCY_32K, 2);
  };

  /**
  * @brief  Initializes audio acquisition in 16bits resolution
  * @param  AudioFreq: Audio frequency to be configured for the peripherals.
  * 		  Possible values are 8000, 16000, 32000, 48000 OR 96000 Hz
  * @param  ChnlNbr: Number of channel to be configured.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
  uint8_t begin(uint32_t AudioFreq, uint32_t ChnlNbr);

  /**
  * @brief  DeInitializes the audio peripheral.
  * @retval  AUDIO_OK if correct communication, else wrong communication
  */
  uint8_t end(void);


  /**
  * @brief  Starts audio recording.
  * @param  * pbuf: Buffer that will contain 1 ms of PCM for each microphone.
  Its dimension must be equal to (in uint16_t words):
  ((PCM sampling frequency)/1000 * Channels). It is used in circular mode.
  * @param audio_complete : callback called as soon as the transfer has
  * been done. Even if the DMA work with the half and complete callback,
  * the entire pbuf is fill each time.
  * The user should perform some operations on the half buffer that has just been
  * filled because the DMA is making some operation on the other part of the buffer.
  * @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  uint8_t record(uint16_t* pbuf, void (*audio_complete)(void));

  /**
  * @brief  Stops audio recording.
  * @param  None
  * @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  uint8_t stop_recording(void);


  /**
  * @brief  Pauses the audio file stream.
  * @param  None
  * @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  uint8_t pause_recording(void);

  /**
  * @brief  Resumes the audio file stream.
  * @param  None
  * @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  uint8_t resume_recording(void);


  /**
  * @brief  Controls the audio in volume level.
  * @param  Volume: Volume level to be set. This value has the same behaviour of the
  Volume parameter of the PDM to PCM software decimation library.
  Values must be in the range from 0 to 64
  * @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  uint8_t set_volume(uint8_t volume) ;

  private:

  uint32_t AudioFreq;
  uint32_t ChnlNbr;

  PinName _ckout = NC;
  PinName _datin = NC;
};

extern MP34DT01 mp34dt01;

#endif /* __MP34DT01_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
