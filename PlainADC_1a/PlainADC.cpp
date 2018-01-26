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

/* Include libraries */
#include <PlainADC.h>

/* Volatile variables: <!> Cannot be declared in the header file */
volatile uint8_t _dataAcqStatus = ADC_DISABLED; 
volatile uint8_t _adcLSB;
volatile uint8_t _adcMSB;
uint8_t _originalTIMSK0; 
uint8_t _originalTIMSK2; 
uint8_t _originalDIDR0; 

PlainADC::PlainADC(void) 
/* Constructor */
{
	/* Set default acquisition status */
	_dataAcqStatus = ADC_DISABLED; 
}


PlainADC::~PlainADC(void) 
/* Destructor */
{
	ReleaseAcquisitionEngine();
}


/* Public functions */


void PlainADC::ReleaseAcquisitionEngine(void)
/* Release acquisition engine and relative functions */
{
	if (_dataAcqStatus == ADC_IDLE) {
		cli(); /* Disable all interrupts */
		TIMSK1 &= ~(1 << OCIE1B); /* Disable timer1 interrupt */
		/* Free memory space */
		free(_vData);
		_vData = NULL;
		_dataAcqStatus = ADC_DISABLED; /* Set acquisition status */
		DIDR0 = _originalDIDR0;
		sei(); /* Enable all interrupts */
	}
}


uint8_t* PlainADC::SetAcquisitionEngine(uint8_t adcChannel, uint8_t refVoltage, float samplingFrequency, uint16_t samples, uint8_t dataFormat, uint8_t options) 
/* Set acquisition parameters, full options */
{
	if (_dataAcqStatus == ADC_DISABLED) {
		/* Set options*/
		_options = options;
		/* Set global variables */
		_refVoltage = refVoltage;
		_lastSamples = _samples;
		_samples = samples;
		if (samplingFrequency < ADC_MIN_SMP_FREQUENCY)  {
			samplingFrequency = ADC_MIN_SMP_FREQUENCY;
		} 
		if (samplingFrequency > ADC_MAX_SMP_FREQUENCY) {
			samplingFrequency = ADC_MAX_SMP_FREQUENCY;
		}
		_samplingFrequency = samplingFrequency;
		_lastDataFormat = _dataFormat;
		_dataFormat = dataFormat;
		/* Data vector */
		if ((_lastSamples != _samples) || (_lastDataFormat != _dataFormat)) {
			/* Free memory space and avoid memory leakage */
			free(_vData); /* Delete vector if existing */
			_vData = NULL;
		}
		if (_vData == NULL) {
			/* Set vector size */
			_dataSize = (1 << _dataFormat);
			_vData = (uint8_t*)malloc(_samples * _dataSize); /* Create destination vector */
			while(_vData == NULL);
		}
		/* Disable all interrupts */
		cli();
		/* Digital Input Disable Register */
		_originalDIDR0 = DIDR0;
		DIDR0 |= adcChannel;
		/* Clear ADC control registers */
		ADCSRA = 0x00;
		ADCSRB = 0x00;
		ADMUX  = 0x00;
		ADMUX |= (_refVoltage << 6); /* Set reference voltage */
		ADMUX |= adcChannel; /* Set ADC channel */
		/* Compute ADC prescaler */
		float divisionFactor = ((13.0 * _samplingFrequency * 1.0E+6) / 16.0);
		uint8_t adcPrescaler = 7; /* Default division factor exponent */
		while ((divisionFactor > float(1 << adcPrescaler)) && (adcPrescaler > 2)) {
			adcPrescaler -= 1;
		}
		ADCSRA |= adcPrescaler; /* Set ADC prescaler */
		ADCSRA |= (1 << ADEN); /* Enable ADC */
		ADCSRA |= (1 << ADATE); /* ADC Auto Trigger Enable */
		ADCSRA |= (1 << ADIE); /* ADC interrupt Enable */
		ADCSRB |= ((1 << ADTS0) | (1 << ADTS2)); /* ADC Auto Trigger Source Selection */
		/* Noise reduction mode */
		SMCR = (1 << SM0); /* Set ADC noise reduction mode */
		/* Set Timer1 */
		/* Reset Timer/Counter1 control registers and Interrupt Mask Register */
		TCCR1A = 0x00; 
		TCCR1B = 0x00; 
		TIMSK1 = 0x00; 
		/* Compute clock timer prescaler */
		uint8_t preScalers[] = {0, 3, 6, 8, 10}; /* Log2 of prescalers */	
		uint8_t timerPrescaler = 0;
		uint32_t upperCount;
		do {
			upperCount = uint32_t(F_CPU / (_samplingFrequency * (1 << preScalers[timerPrescaler]))) - 1 ;
			timerPrescaler += 1;
		} while ((upperCount > 0xFFFF) && (timerPrescaler < 5));
		OCR1A = uint16_t(upperCount);
		TCCR1B |=  (1 << WGM12); /* Set Clear Timer on Compare Match (CTC) Mode (Mode 4) */
		TCCR1B |= timerPrescaler; /* Set timer prescaler */
		TIMSK1 |= (1 << OCIE1B); /* Enable timer1 interrupt  */
		_dataAcqStatus = ADC_IDLE; /* Set acquisition status */
		sei(); /* Enable all interrupts */
	}
	return(_vData);
}


void PlainADC::GetScanData(void) 
/* Acquire data from one specific detector (base 0) */
{
	if (_dataAcqStatus == ADC_IDLE) {
		/* Set options*/
		if ((_options & ADC_OPT_DIS_TIM_0) == ADC_OPT_DIS_TIM_0) {
			_originalTIMSK0 = TIMSK0; /* Record timer/counter0 mask */
			TIMSK0 = 0x00; /* Disable timer/counter0 */
		} else if ((_options & ADC_OPT_DIS_TIM_2) == ADC_OPT_DIS_TIM_2) {
			_originalTIMSK2 = TIMSK2; /* Record timer/counter2 mask */
			TIMSK2 = 0x00; /* Disable timer/counter2 */
		} else if ((_options & ADC_OPT_NOI_CANCELLER) == ADC_OPT_NOI_CANCELLER) {
			SMCR |= (1 << SE); /* Activate ADC noise reduction mode */	
		}
		/* This loop is made as compact as possible in order to prevent any latency and allow fast data acquisition rates */	
		for (uint8_t *dataPointer = _vData; dataPointer < (_vData + (_samples * _dataSize)); dataPointer += _dataSize) {
			/* Set waiting state */
			_dataAcqStatus = ADC_WAITING;
			/* Wait for triggered state */
			while (_dataAcqStatus != ADC_TRIGGERED);
			/* Record data in data buffer LSB->MSB */
			// *dataPointer = ADCL;
			// *(dataPointer + 1) = ADCH; 
			*dataPointer = _adcLSB;
			*(dataPointer + 1) = _adcMSB; 
		}
		/* Stop sequential recording mode */
		_dataAcqStatus = ADC_IDLE; /* Reset status */
		/* Convert data as appropriate */
		if (_dataFormat == ADC_DAT_FMT_DBL) {
			/* Reset data location ptr */
			uint16_t UInt16Data;
			for (uint8_t *dataPointer = _vData; dataPointer < (_vData + (_samples * _dataSize)); dataPointer += _dataSize) {
				/* At this stage, the vector length is equal _samples * 4 and integer data are equally spaced very 4 bytes */
				/* Read 16 bits int */
				memcpy(&UInt16Data, dataPointer, sizeof(uint16_t));
				/* Cast data */
				float Dbl32Data = float(UInt16Data);
				/* Write 32 bits float data */
				memcpy(dataPointer, &Dbl32Data, sizeof(float));
			}
		}
		/* Reset options*/
		if ((_options & ADC_OPT_DIS_TIM_0) == ADC_OPT_DIS_TIM_0) {
			TIMSK0 = _originalTIMSK0; /* Restore timer/counter0 operation */
		} else if ((_options & ADC_OPT_DIS_TIM_2) == ADC_OPT_DIS_TIM_2) {
			TIMSK2 = _originalTIMSK2; /* Restore timer/counter2 operation */
		} else if ((_options & ADC_OPT_NOI_CANCELLER) == ADC_OPT_NOI_CANCELLER) {
			SMCR &= ~(1 << SE); /* Deactivate ADC noise reduction mode */
		}		
	}
}


uint16_t PlainADC::GetSingleUInt16Data(void)
/* Acquire snap shot data */
{
	uint16_t result = 0;
	if (_dataAcqStatus == ADC_IDLE) {
		/* Set waiting state */
		_dataAcqStatus = ADC_WAITING;
		/* Wait for triggered state */
		while (_dataAcqStatus != ADC_TRIGGERED);
		result = (_adcLSB | (_adcMSB << 8));
		/* Reset status */
		_dataAcqStatus = ADC_IDLE; 
	}
	/* Return result */
	return(result);
}


float PlainADC::GetSingleDbl32Data(void)
/* Acquire snap shot data */
{
	return(float(GetSingleUInt16Data()));
}


uint8_t* PlainADC::GetDataAddress(void) 
{
	return(_vData);
}


/* Interrupts management */


ISR(TIMER1_COMPB_vect)
/* Invoked on completion of counting (Compare mode) */
{
	/* Do not ask me why, the presence of this function is mandatory!!! */
}


ISR(ADC_vect)
/* Invoked on completion of conversions */
{
	if (_dataAcqStatus == ADC_WAITING) {
		/* When both LSBs and MSBs are to be read, LSBs MUST be read first */
		_adcLSB = ADCL;
		_adcMSB = ADCH;
		_dataAcqStatus = ADC_TRIGGERED; /* Set new acquisition status */
	}
}  


float PlainADC::ReadDbl32Data(uint16_t sample) 
/* Reads float data from vector in random access mode */
{
	float Dbl32Data = 0.0;	
	if ((_vData != NULL) && (_dataFormat == ADC_DAT_FMT_DBL)) {
		memcpy(&Dbl32Data, (_vData + (sample << 2)), 4);
	}
	return(Dbl32Data);
}


uint16_t PlainADC::ReadUInt16Data(uint16_t sample) 
/* Reads integer data from vector in random access mode */
{
	uint16_t UInt16Data = 0;	
	if ((_vData != NULL) && (_dataFormat == ADC_DAT_FMT_INT)) {
		memcpy(&UInt16Data, (_vData + (sample << 1)), 2);
	}
	return(UInt16Data);	
}


/* Complementary functions */


float PlainADC::Average(uint8_t *ptrData, uint16_t samples) 
{
	/* Compute average */
	float sum = 0.0;
	uint16_t UInt16Data = 0;	
	float Dbl32Data = 0.0;	
	for (uint16_t i = 0; i < samples; i += 1) {
		if (_dataFormat == ADC_DAT_FMT_INT) {
			memcpy(&UInt16Data, (_vData + (i << 1)), 2);
			sum += (float)UInt16Data;
		} else {
			memcpy(&Dbl32Data, (_vData + (i << 2)), 4);
			sum += Dbl32Data;
		}	
	}	
	return(sum / samples);	
}


float PlainADC::RMS(uint8_t *ptrData, uint16_t samples) 
{
	/* Compute true RMS */
	float sumOfSquares = 0.0;
	uint16_t UInt16Data = 0;	
	float Dbl32Data = 0.0;	
	for (uint16_t i = 0; i < samples; i += 1) {
		if (_dataFormat == ADC_DAT_FMT_INT) {
			memcpy(&UInt16Data, (_vData + (i << 1)), 2);
			Dbl32Data = (float)UInt16Data;
		} else if (_dataFormat == ADC_DAT_FMT_DBL) {
			memcpy(&Dbl32Data, (_vData + (i << 2)), 4);
		}	
		sumOfSquares += square(Dbl32Data);
	}	
	return(sqrt(sumOfSquares / samples));	
}
