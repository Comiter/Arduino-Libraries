/*
 
	PlainADC: Data acquisition library for single shot and scanned data
	Tested with ATmega328 powered Arduino
	Copyright (C) 2012-2013 Didier Longueville

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
*/

#ifndef PlainADC_h /* Prevent loading library twice */
#define PlainADC_h

/* Include std libraries */
#include <inttypes.h> /* integer types */
#include <avr/interrupt.h>/* interrupts */
#include <math.h> /* sqrt */
#include <string.h> /* memcopy */
#include <stdlib.h> /* malloc */

class PlainADC
{
public:
	/* Options */
	#define ADC_OPT_NONE 0x00
	#define ADC_OPT_DIS_TIM_0 0x01
	#define ADC_OPT_DIS_TIM_2 0x02
	#define ADC_OPT_NOI_CANCELLER 0x04
	/* Voltage references */
	#define ADC_REF_VOL_EXTERNAL 0x00 /* As per AREF pin voltage */
	#define ADC_REF_VOL_DEFAULT 0x01 /* VCC */
	#define ADC_REF_VOL_INTERNAL 0x03 /* 1V1 on ATMEGA 168/328 */
	/* Data format */
	#define ADC_DAT_FMT_UNDEFINED 0x00 
	#define ADC_DAT_FMT_INT 0x01 
	#define ADC_DAT_FMT_DBL 0x02 
	/* Specifications */
	#define ADC_MIN_SMP_FREQUENCY 0.125
	#define ADC_MAX_SMP_FREQUENCY 130000.0
	/* Constructor */
	PlainADC(void);
	/* Destructor */
	~PlainADC(void);
	/* Functions */
	float Average(uint8_t *vData, uint16_t samples);
	uint8_t *GetDataAddress(void);
	void GetScanData(void);
	float GetSingleDbl32Data(void);
	uint16_t GetSingleUInt16Data(void);
	float ReadDbl32Data(uint16_t sample);
	uint16_t ReadUInt16Data(uint16_t sample);
	void ReleaseAcquisitionEngine(void);
	float RMS(uint8_t *vData, uint16_t samples);
	uint8_t *SetAcquisitionEngine(uint8_t adcChannel, uint8_t refVoltage, float samplingFrequency, uint16_t samples, uint8_t dataFormat, uint8_t options);

private:
	/* Data acquisition status */
	#define ADC_DISABLED 0x00 
	#define ADC_IDLE 0x01 /* Idle, no acquisition */
	#define ADC_WAITING 0x02 /* Ready to acquire, waiting for an ADC sample to be taken */
	#define ADC_TRIGGERED 0x03 /* An ADC sample has been taken, ready to be recorded */
	/* Data acquisition parameters */
 	uint8_t _refVoltage;
 	uint16_t _samples;
 	uint16_t _lastSamples;
 	float _samplingFrequency;
 	uint8_t _dataFormat;
	uint8_t _lastDataFormat;
 	uint8_t _dataSize;
	uint8_t *_vData;
	uint8_t _options;


};

#endif