/**
 ******************************************************************************
 * @file    DISCO_IOT_SimpleRecord.ino
 * @author  WI6LABS
 * @version V1.0.0
 * @date    11 July 2017
 * @brief   Arduino test application for the STM32 discory IOT kit
 *          The example takes audio samples from the mp34dt01 two i2s mics and
 *          display it so it can be replay on an audio software
 ******************************************************************************
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
#include <MP34DT01.h>


/* -------------------- Parameters to configure by user ----------------------*/
/* Sampling at 16kHz. Possible values are: 8, 16, 32, 48 or 96 kHz */
#define AUDIO_SAMPLING_FREQUENCY  AUDIO_FREQUENCY_16K

/* two channels available (left and right)
  MONO to set only one channel
  STEREO to set both channel
*/
#define AUDIO_CHANNELS STEREO
/* ---------------------------------------------------------------------------*/

/* ----------- Internal parameters configuration, do not change---------------*/
/* The N_MS value defines the number of millisecond to be processed at each
AudioProcess call,that must be consistent with the N_MS_PER_INTERRUPT defined in
the audio driver.
The default value of the N_MS_PER_INTERRUPT directive in the driver is set to 1,
for backward compatibility: leaving this values as it is allows to avoid any
modification in the application layer developed with the older versions of the
driver
*/
#define N_MS N_MS_PER_INTERRUPT

// Each PCM sample is 16 bits
#define SIZE_SAMPLE 2

// Size of buffer to fill in 1 ms
#define BUFFER_SIZE (((AUDIO_CHANNELS * AUDIO_SAMPLING_FREQUENCY) / 1000)* N_MS)

//Default time of sampling in ms
#define DEFAULT_TIME_SAMPLING 1000

//audio reception buffer
uint16_t audio_buffer[BUFFER_SIZE];
uint8_t audio_buffer_byte[BUFFER_SIZE*SIZE_SAMPLE];

// Timing of sampling
uint32_t time_sampling = DEFAULT_TIME_SAMPLING;

// Use PE9 as CKOUT = 60
// Use PE7 as DATIN = 58
MP34DT01 mp34dt01(60, 58);

void audio_complete(void);

void setup() {
  // Initialize serial for output.
  Serial.begin(9600);

  if(mp34dt01.begin(AUDIO_SAMPLING_FREQUENCY, AUDIO_CHANNELS) != AUDIO_OK) {
    Serial.println("Cannot initialize the MIC interface");
  }

  // User can manage the gain, value must be between 1 and 255
  mp34dt01.set_volume(DEFAULT_AUDIO_IN_VOLUME);

  if(mp34dt01.record(audio_buffer, audio_complete) != AUDIO_OK) {
    Serial.println("Cannot start recording");
  }
}

void loop() {
}


//function called when the audio half tranfer ends
void audio_complete(void)
{
  // Copy data
  for (uint8_t i=0; i<BUFFER_SIZE; i++)
  {
    audio_buffer_byte[i*SIZE_SAMPLE] = (uint8_t)((audio_buffer[i] & 0xFF00) >> 8);
    audio_buffer_byte[i*SIZE_SAMPLE+1] = (uint8_t)(audio_buffer[i] & 0x00FF);
    Serial.print(audio_buffer_byte[i*SIZE_SAMPLE], HEX);
    Serial.print(audio_buffer_byte[i*SIZE_SAMPLE+1], HEX);
  }

  Serial.println();

  time_sampling--;
  if (time_sampling == 0)
  {
    mp34dt01.stop_recording(); // Stop the sampling
  }
}
