/**
 ******************************************************************************
 * @file    MP32DT01.cpp
 * @author  WI6LABS
 * @version V0.0.1
 * @date    11-July-2017
 * @brief   mp34dt01 microphone arduino interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Includes */
#include <stdlib.h>
#include <MP34DT01.h>

void (*g_audio_complete_cb)(void);


/** Builder
*/
MP34DT01::MP34DT01(uint8_t ckout, uint8_t datin)
{
    AudioFreq = 32000;
    ChnlNbr = 2;
    _ckout   = digitalPinToPinName(ckout);
    _datin  = digitalPinToPinName(datin);
}

/**
* @brief  Initializes audio acquisition in 16bits resolution
* @param  AudioFreq: Audio frequency to be configured for the peripherals.
* 		  Possible values are 8000, 16000, 32000, 48000 OR 96000 Hz
* @param  ChnlNbr: Number of channel to be configured.
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t MP34DT01::begin(uint32_t AudioFreq, uint32_t ChnlNbr)
{
  return MP34DT01_Init(AudioFreq, 0, ChnlNbr, _ckout, _datin);

}

/**
* @brief  DeInitializes the audio peripheral.
* @retval  AUDIO_OK if correct communication, else wrong communication
*/
uint8_t MP34DT01::end(void)
{
  return MP34DT01_DeInit(_ckout, _datin);
}


/**
* @brief  Starts audio recording.
* @param  * pbuf: Buffer that will contain 1 ms of PCM for each microphone.
Its dimension must be equal to (in uint16_t words):
((PCM sampling frequency)/1000 * Channels). It is used in circular mode.
* @param audio_complete : callback called as soon as half of the transfer has
* been done
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01::record(uint16_t* pbuf, void (*audio_complete)(void))
{
  g_audio_complete_cb = audio_complete;
  return MP34DT01_Record(pbuf, 0);
}

/**
* @brief  Stops audio recording.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01::stop_recording(void)
{
  return MP34DT01_Stop();
}


/**
* @brief  Pauses the audio file stream.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01::pause_recording(void)
{
  return MP34DT01_Pause();
}

/**
* @brief  Resumes the audio file stream.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01::resume_recording(void)
{
  return MP34DT01_Resume();
}


/**
* @brief  Controls the audio in volume level.
* @param  Volume: Volume level to be set. This value has the same behaviour of the
Volume parameter of the PDM to PCM software decimation library.
Values must be in the range from 0 to 64
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
uint8_t MP34DT01::set_volume(uint8_t volume)
{
  return MP34DT01_SetVolume(volume);
}



#ifdef __cplusplus
extern "C" {
#endif
/**
* @brief  User callback when record buffer is filled.
* @param  None
* @retval None
*/
void MP34DT01_TransferComplete_CallBack(void) {
  if(g_audio_complete_cb) {
    g_audio_complete_cb();
  }
}

/**
* @brief  User callback when record buffer is half filled.
* @param  None
* @retval None
*/
void MP34DT01_HalfTransfer_CallBack(void) {
  if(g_audio_complete_cb) {
    g_audio_complete_cb();
  }
}

#ifdef __cplusplus
}
#endif


/****************** define for  configuration *******************************/
