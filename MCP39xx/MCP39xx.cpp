/* MCP39xx library version 1.0 by Vidar Østevik


*/

#include "MCP39xx.h"

#ifdef UseArduinoSPI
#include <SPI.h>
#else
#include <spi4teensy3.h>
#endif

MCP3911::MCP3911(uint8_t CsPin, uint8_t DRpin, uint8_t CLKIpin, float CLKIspeed){
	_CSpin = CsPin;
	_DRpin = DRpin;
	_CLKIpin = CLKIpin;

	pinMode(_CSpin, OUTPUT);
	pinMode(_CLKIpin, OUTPUT);
	pinMode(_DRpin, INPUT_PULLUP);
	
	digitalWriteFast(_CSpin, HIGH);
	analogWriteFrequency(_CLKIpin, CLKIspeed);
	analogWrite(_CLKIpin, 128); //50% duty cycle

}

MCP3911::~MCP3911(){

}

void MCP3911::Configuration(byte DitherMode,byte PreScale, byte OSR, byte boost, byte PGA_CH0, byte PGA_CH1){
	
	digitalWriteFast(_CSpin, LOW);
	delay(10);
	#ifdef UseArduinoSPI
		SPI.begin();
	#else
		spi4teensy3::init(3); //12MHz on 96/48MHz CPU
	#endif	
	Settings.Boost = boost;
	Settings.DitherMode = DitherMode;
	Settings.OSR = OSR;
	Settings.PGA_CH0 = PGA_CH0;
	Settings.PGA_CH1 = PGA_CH1;
	Settings.PreScale = PreScale;
	Settings.res_24 = true;
	Settings.Bitmask = 0xFF; //default all bits active in LSByte
	byte _cfgbyte[3] = {0,0,0};
	_cfgbyte[0] = (boost<<6) | (PGA_CH1<<3) | (PGA_CH0);
	_cfgbyte[1] = (PreScale<<6) | (OSR<<3) | (DitherMode<<1);
	_cfgbyte[2] = 2; // Only Clock Ext true
	
	Serial.print("CFG[0]: ");
	printBinaryByte(&_cfgbyte[0],1);
	Serial.println();
	Serial.print("CFG[1]: ");
	printBinaryByte(&_cfgbyte[1],1);
	Serial.println();
	Serial.print("CFG[2]: ");
	printBinaryByte(&_cfgbyte[2],1);
	Serial.println();
	Serial.print("Command: ");
	byte command = (MCP3911_WRITE_MASK | (Addresses.GAIN <<1));
	printBinaryByte(&command,1);
	Serial.println();//DEBUG PrintCode
	
	
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		SPI.transfer(MCP3911_WRITE_MASK | (Addresses.GAIN<<1)); //GAIN Register = 0x09
		SPI.transfer(_cfgbyte[0]);	// GAIN Register
		SPI.transfer(0x13);			// STATUSCOM Register: The DR pin state is a logic high when data is NOT ready, Data Ready pulses from the lagging ADC between the two are output on DR pin. The lagging ADC depends on the PHASE register and on the OSR (DEFAULT).
		SPI.transfer(0xB8);			//                     Address counter loops on register types (DEFAULT), Address counter loops on entire register map (DEFAULT),  Both channels are in 24-bit mode(DEFAULT), EN_OFFCAL = EN_GAINCAL = 0 (Disabled)
		SPI.transfer(_cfgbyte[1]);	// CONFIG Register
		SPI.transfer(_cfgbyte[2]);
		SPI.endTransaction();
	#else
		spi4teensy3::send(MCP3911_WRITE_MASK | (Addresses.GAIN<<1));	//GAIN Register = 0x09
		spi4teensy3::send(_cfgbyte[0]);	// GAIN Register
		spi4teensy3::send(0x13);		//STATUSCOM Register:	The DR pin state is a logic high when data is NOT ready, Data Ready pulses from the lagging ADC between the two are output on DR pin. The lagging ADC depends on the PHASE register and on the OSR (DEFAULT).
		spi4teensy3::send(0xB8);		//						Address counter loops on register types (DEFAULT), Address counter loops on entire register map (DEFAULT),  Both channels are in 24-bit mode(DEFAULT), EN_OFFCAL = EN_GAINCAL = 0 (Disabled)
		spi4teensy3::send(_cfgbyte[1]);	// CONFIG Register
		spi4teensy3::send(_cfgbyte[2]);
	#endif
	digitalWriteFast(_CSpin, HIGH);
}

void MCP3911::SetPGA(int PGA_Ch0, int PGA_Ch1) {
	if ((Settings.PGA_CH0 == PGA_Ch0) && (Settings.PGA_CH1 == PGA_Ch1))
		return; // If no changes done, exit
	Settings.PGA_CH0 = PGA_Ch0;
	Settings.PGA_CH1 = PGA_Ch1;
	digitalWriteFast(_CSpin, LOW);
	byte _cfgbyte = 0;
	_cfgbyte = (Settings.Boost << 6) | (PGA_Ch1 << 3) | (PGA_Ch0);
#ifdef UseArduinoSPI
	SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST, SPI_MODE0));
	SPI.transfer(MCP3911_WRITE_MASK | (Addresses.GAIN << 1)); //GAIN Register = 0x09
	SPI.transfer(_cfgbyte);
	SPI.endTransaction();
#else
	spi4teensy3::send(MCP3911_WRITE_MASK | (Addresses.GAIN << 1));	//GAIN Register = 0x09
	spi4teensy3::send(_cfgbyte);	// GAIN Register
#endif
	digitalWriteFast(_CSpin, HIGH);
}

uint8_t MCP3911::GetPGA(int Channel) {
	uint8_t PGA_Setting;
	if (Channel == 0)
		PGA_Setting = Settings.PGA_CH0;
	else
		PGA_Setting = Settings.PGA_CH1;
	// Convert to actual PGA setting
	switch (PGA_Setting) {
	case 1:
		return  2;
		break;
	case 2:
		return 4;
		break;
	case 3:
		return 8;
		break;
	case 4:
		return 16;
		break;
	case 5:
		return 32;
		break;
	default:
		return -1;
		break;
	}
}

void MCP3911::SetBitMask(byte Bitmask) {
	Settings.Bitmask = Bitmask;
}

byte MCP3911::GetBitMask(void) {
	return Settings.Bitmask;
}

/*void MCP3911::SetDataWidth(boolean ADC_24bit) {
	digitalWriteFast(_CSpin, LOW);

	#ifdef UseArduinoSPI
	SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST, SPI_MODE0));
	SPI.transfer(MCP3911_WRITE_MASK | (Addresses.STATUSCOM << 1)); 
	SPI.transfer(0x13);			// STATUSCOM Register: The DR pin state is a logic high when data is NOT ready, Data Ready pulses from the lagging ADC between the two are output on DR pin. The lagging ADC depends on the PHASE register and on the OSR (DEFAULT).
	if (ADC_24bit)
		SPI.transfer(0xB8);			//                     Address counter loops on register types (DEFAULT), Address counter loops on entire register map (DEFAULT),  Both channels are in 24-bit mode(DEFAULT), EN_OFFCAL = EN_GAINCAL = 0 (Disabled)
	else
		SPI.transfer(0xA0);
	#else
	spi4teensy3::send(MCP3911_WRITE_MASK | (Addresses.STATUSCOM << 1));
	spi4teensy3::send(0x13);		//STATUSCOM Register:	The DR pin state is a logic high when data is NOT ready, Data Ready pulses from the lagging ADC between the two are output on DR pin. The lagging ADC depends on the PHASE register and on the OSR (DEFAULT).
	if (ADC_24bit)
		spi4teensy3::send(0xB8);
	else
		spi4teensy3::send(0xA0);

	Settings.res_24 = ADC_24bit;
	#endif
	digitalWriteFast(_CSpin, HIGH);
}*/

int32_t MCP3911::ReadValue(int32_t Data[], uint8_t chToRead){
	if(chToRead == 0)
	return NULL;
	
	byte Buf[4] = {0,0,0,0};
	int32_t temp = 0;
	
	digitalWriteFast(_CSpin, LOW);
	unsigned char readAddress = Addresses.CHANNEL0 << 1; // Channel 0 address = 0x00;
	readAddress |= MCP3911_READ_MASK;
	
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		Buf[0] = SPI.transfer(readAddress);
		Buf[1] = SPI.transfer(0); // Byte 23-16
		Buf[2] = SPI.transfer(0); // Byte 15-8
		Buf[3] = SPI.transfer(0); // Byte 7-0
	#else
		spi4teensy3::send(readAddress);
		spi4teensy3::receive(Buf,3);
	#endif
	
	temp = (
	((uint32_t)Buf[1]<<16) |
	((uint32_t)Buf[2]<<8) |
	((uint32_t)(Buf[3] & Settings.Bitmask))
	);
	
	//Convert to 32bit 2's complement
	if((temp & 0x00800000) > 0){
		temp |= 0xFF000000;
	}
	else{
		temp &= 0x00FFFFFF;
	}
	Data[0] = temp;
	
	if (chToRead == 2){
		temp = 0;
		#ifdef UseArduinoSPI
			Buf[1] = SPI.transfer(0); // Byte 23-16
			Buf[2] = SPI.transfer(0); // Byte 15-8
			Buf[3] = SPI.transfer(0); // Byte 7-0
		#else
			spi4teensy3::receive(Buf,3);
		#endif
		
		temp = (
		((uint32_t)Buf[1]<<16) |
		((uint32_t)Buf[2]<<8) |
		((uint32_t)(Buf[3] & Settings.Bitmask))
		);
		
		//Convert to 32bit 2's complement
		if((temp & 0x00800000) > 0){
			temp |= 0xFF000000;
		}
		else{
			temp &= 0x00FFFFFF;
		}

		Data[1] = temp;
	}
	digitalWriteFast(_CSpin, HIGH);
	#ifdef UseArduinoSPI
		SPI.endTransaction();
	#endif

	return (int32_t)chToRead;	
}

int32_t MCP3911::ReadSingleValue(uint8_t chAddr){
	byte channel;
	if (chAddr == 0){
		channel = Addresses.CHANNEL0;
	}
	else if(chAddr == 1){
		channel = Addresses.CHANNEL1;
	}
	else{
		channel = Addresses.CHANNEL0; //default
	}
	
	byte Buf[4] = {0,0,0,0};
	
	digitalWriteFast(_CSpin, LOW);
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		Buf[3] = SPI.transfer(MCP3911_READ_MASK | (channel<<1));
		Buf[0] = SPI.transfer(0); // Byte 23-16
		Buf[1] = SPI.transfer(0); // Byte 15-8
		Buf[2] = SPI.transfer(0); // Byte 7-0
	#else
		spi4teensy3::send(MCP3911_READ_MASK | (channel<<1));
		spi4teensy3::receive(Buf,3);
	#endif
	
	digitalWriteFast(_CSpin, HIGH);

	//Test to reduce jitter by removing certain amounts of LSB's
	byte bitmask = 0xFC; // remove the two last bits
	int32_t temp = 0;
	temp = (
	((uint32_t)Buf[0]<<16) |
	((uint32_t)Buf[1]<<8) |
	((uint32_t)(Buf[2]& Settings.Bitmask))
	);

	
	
	
	//Convert to 32bit 2's complement
	if((temp & 0x00800000) > 0){
		temp |= 0xFF000000;
	}
	else{
		temp &= 0x00FFFFFF;
	}
	return temp;		
}

void MCP3911::ReadConfiguration(void){
	digitalWriteFast(_CSpin, LOW);
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		SPI.transfer(MCP3911_READ_MASK | (Addresses.GAIN<<1)); // Gain Conf register = 0x09
		Registers._GainSettings = SPI.transfer(0xFF);
		byte MSB = SPI.transfer(0xFF);
		byte LSB = SPI.transfer(0xFF);
		Registers._StatusComSettings = ((uint16_t)MSB<<8) | LSB;
		MSB = SPI.transfer(0xFF);
		LSB = SPI.transfer(0xFF);
		Registers._ConfigRegister = ((uint16_t)MSB<<8) | LSB;
		SPI.endTransaction();
	#else
		byte buf[5] {0,0,0,0,0};
		spi4teensy3::send(MCP3911_READ_MASK | (Addresses.GAIN<<1));// Gain Conf register = 0x09
		spi4teensy3::receive(buf,5);
		Registers._GainSettings = buf[0];
		Registers._StatusComSettings = ((uint16_t)buf[1]<<8) | buf[2];
		Registers._ConfigRegister = ((uint16_t)buf[3]<<8) | buf[4];
	#endif
	digitalWriteFast(_CSpin, HIGH);
	Serial.print("Gain Settings: ");	
	printBinaryByte(&Registers._GainSettings,1);
	Serial.println();
	Serial.print("Status COM Settings: ");
	printBinaryInt16(&Registers._StatusComSettings,1);
	Serial.println();
	Serial.print("Config Register: ");
	printBinaryInt16(&Registers._ConfigRegister,1);
	Serial.println();
}

void MCP3911::printBinaryByte(byte *data, uint8_t length){
	for(int i=0; i<length;i++){
		for (unsigned int mask = 0x80; mask; mask >>= 1) {
			Serial.print(mask&data[i]?'1':'0');
		}
	}
}

void MCP3911::printBinaryInt16(uint16_t *data, uint8_t length){
	for(int i=0; i<length;i++){
		for (unsigned int mask = 0x8000; mask; mask >>= 1) {
			Serial.print(mask&data[i]?'1':'0');
		}
	}
}

float MCP3911::getSingleReading_mV(uint8_t channel) {
	int32_t raw = 0;

	uint8_t PGA_Setting;
	if (channel == 0)
		PGA_Setting =Settings.PGA_CH0;
	else
		PGA_Setting = Settings.PGA_CH1;
	// Convert to actual PGA setting
	switch (PGA_Setting) {
	case 1:
		PGA_Setting = 2;
		break;
	case 2:
		PGA_Setting = 4;
		break;
	case 3:
		PGA_Setting = 8;
		break;
	case 4:
		PGA_Setting = 16;
		break;
	case 5:
		PGA_Setting = 32;
		break;
	default:
		PGA_Setting = 1;
		break;
	}

	raw = MCP3911::ReadSingleValue(channel);

	/*Serial.println("Data from core:");
	Serial.print("Channel: ");
	Serial.println(channel);
	Serial.print("Raw Value = ");
	Serial.println(raw);
	Serial.print("PGA Setting: ");
	Serial.println(PGA_Setting);*/

	if (Settings.res_24) { //24 bit mode (width = 1)
		return ((float)raw * 1200.0) / (8388608.0 * (float)PGA_Setting * 1.5);
	}
	else {
		return ((float)raw * 1200) / (32768 * PGA_Setting*1.5);
	}

}

uint8_t MCP3911::getReading_mV(float Data[], uint8_t chToRead) {
	if (chToRead == 0)
		return NULL;
	int32_t RawData[2] = { 0,0 };
	uint8_t PGA_Ch0_Setting;
	uint8_t PGA_Ch1_Setting;
	// Convert to actual PGA setting. We allways need CH0, so convert it here.
	switch (Settings.PGA_CH0) {
	case 1:
		PGA_Ch0_Setting = 2;
		break;
	case 2:
		PGA_Ch0_Setting = 4;
		break;
	case 3:
		PGA_Ch0_Setting = 8;
		break;
	case 4:
		PGA_Ch0_Setting = 16;
		break;
	case 5:
		PGA_Ch0_Setting = 32;
		break;
	default:
		PGA_Ch0_Setting = 1;
		break;
	}
	if (chToRead == 1) {
		ReadValue(RawData, 1);																		// Get Raw value
		if (Settings.res_24) { 
			Data[0] = ((float)RawData[0] * 1200.0) / (8388608.0 * (float)PGA_Ch0_Setting * 1.5);		// Calculate mV value based on settings
		}
		else {
			Data[0] = ((float)RawData[0] * 1200) / (32768 * (float)PGA_Ch0_Setting*1.5);
		}
	}
	else {
		ReadValue(RawData, 2);
		switch (Settings.PGA_CH1) {																	// We need PGA_CH1 as well
		case 1:
			PGA_Ch1_Setting = 2;
			break;
		case 2:
			PGA_Ch1_Setting = 4;
			break;
		case 3:
			PGA_Ch1_Setting = 8;
			break;
		case 4:
			PGA_Ch1_Setting = 16;
			break;
		case 5:
			PGA_Ch1_Setting = 32;
			break;
		default:
			PGA_Ch1_Setting = 1;
			break;
		}

		if (Settings.res_24) { //24 bit mode (width = 1)
			Data[0] = ((float)RawData[0] * 1200.0) / (8388608.0 * (float)PGA_Ch0_Setting * 1.5);
			Data[1] = ((float)RawData[1] * 1200.0) / (8388608.0 * (float)PGA_Ch1_Setting * 1.5);
		}
		else {
			Data[0] = ((float)RawData[0] * 1200) / (32768 * (float)PGA_Ch0_Setting*1.5);
			Data[1] = ((float)RawData[1] * 1200) / (32768 * (float)PGA_Ch1_Setting*1.5);
		}
	}

	return chToRead;

}

MCP3912::MCP3912(uint8_t CsPin, uint8_t DRpin, uint8_t CLKIpin, float CLKIspeed){
	_CSpin = CsPin;
	_DRpin = DRpin;
	_CLKIpin = CLKIpin;

	pinMode(_CSpin, OUTPUT);
	pinMode(_CLKIpin, OUTPUT);
	pinMode(_DRpin, INPUT_PULLUP);
	
	digitalWriteFast(_CSpin, HIGH);
	analogWriteFrequency(_CLKIpin, CLKIspeed);
	analogWrite(_CLKIpin, 128); //50% duty cycle	
}

MCP3912::~MCP3912(){
	
}

void MCP3912::Configuration(byte DitherMode,byte PreScale, byte OSR, byte boost, byte PGA_CH0, byte PGA_CH1, byte PGA_CH2, byte PGA_CH3){
	#ifdef UseArduinoSPI
		SPI.begin();
	#else
		spi4teensy3::init(4); //12MHz on 96/48MHz CPU
	#endif
	Settings.Boost = boost;
	Settings.DitherMode = DitherMode;
	Settings.OSR = OSR;
	Settings.PGA_CH0 = PGA_CH0;
	Settings.PGA_CH1 = PGA_CH1;
	Settings.PGA_CH2 = PGA_CH2;
	Settings.PGA_CH3 = PGA_CH3;
	Settings.PreScale = PreScale;
	
	byte _Gain[3] = {0,0,0};
	byte _Config0[3], _Config1[3], _StatusCom[3] = {0,0,0};
	
	_Gain[1] = (PGA_CH3<<1)|(PGA_CH2>>2);
	_Gain[2] = (PGA_CH2<<6)|(PGA_CH1<<3)|PGA_CH0;
	
	//Address counter auto-increments, and loops on register TYPES, 
	//Address counter auto-increments and loops on writable part of the register map (DEFAULT), 
	//The DR pin state is a logic high when data is NOT ready
	// CRC = 16-bit
	// ADC Data = 24-bit
	_StatusCom[0] = 0xB9;
	// CRC is disabled
	// The interrupt flag for the CRCREG checksum verification is disabled. The CRCREG<15:0> bits are
	// still calculated properly and can still be read in this mode. No interrupt is generated even when a
	// CRCREG checksum error happens. (Default)
	_StatusCom[1] = 0x00;
	// DRSTATUS<3:0>: Data ready status bit for each individual ADC channel
	// DRSTATUS<n> = 1 - Channel CHn data is not ready (DEFAULT)
	// DRSTATUS<n> = 0 - Channel CHn data is ready. The status bit is set back to '1' after reading the
	// STATUSCOM register. The status bit is not set back to '1' by the read of the corresponding channel
	//ADC data.
	_StatusCom[2] = 0x0F;
	
	_Config0[0] = (DitherMode<<4)|(boost<<2)|PreScale;
	_Config0[1] = OSR<<5;
	_Config0[2] = 0x50; //See page 36 in datasheet for vrefcal description
		
	_Config1[0] = 0x00; //No re-set mode active
	_Config1[1] = 0x00; //No shut-down active
	_Config1[2] = 0x40;//Internal voltage reference, external clock
	
// 	Serial.println("Configuration sent to ADC");
// 	Serial.print("Gain Register: ");
// 	printBinaryByte(_Gain,3);
// 	Serial.println();
// 	Serial.print("StatusCom: ");
// 	printBinaryByte(_StatusCom,3);
// 	Serial.println();
// 	Serial.print("Config0: ");
// 	printBinaryByte(_Config0,3);	
// 	Serial.println();
// 	Serial.print("Config1: ");
// 	printBinaryByte(_Config1,3);
// 	Serial.println();
	
	digitalWriteFast(_CSpin, LOW);
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		SPI.transfer(MCP3912_WRITE_MASK | (Addresses.GAIN<<1)); 
		SPI.transfer(_Gain[0]);	
		SPI.transfer(_Gain[1]);
		SPI.transfer(_Gain[2]);
		SPI.transfer(_StatusCom[0]);
		SPI.transfer(_StatusCom[1]);
		SPI.transfer(_StatusCom[2]);
		SPI.transfer(_Config0[0]);
		SPI.transfer(_Config0[1]);
		SPI.transfer(_Config0[2]);
		SPI.transfer(_Config1[0]);
		SPI.transfer(_Config1[1]);
		SPI.transfer(_Config1[2]);
		SPI.endTransaction();
	#else
		spi4teensy3::send(MCP3912_WRITE_MASK | (Addresses.GAIN<<1));	
		spi4teensy3::send(_Gain,3);
		spi4teensy3::send(_StatusCom,3);
		spi4teensy3::send(_Config0,3);
		spi4teensy3::send(_Config1,3);
	#endif
	digitalWriteFast(_CSpin, HIGH);
}

int32_t MCP3912::ReadValue(int32_t Data[], uint8_t chToRead){
	if(chToRead == 0)
	return NULL;
	
	byte Buf[4] = {0,0,0,0};
	int32_t temp = 0;
	
	digitalWriteFast(_CSpin, LOW);
	unsigned char readAddress = Addresses.CHANNEL0 << 1; // Channel 0 address = 0x00;
	readAddress |= MCP3912_READ_MASK;
	
	#ifdef UseArduinoSPI
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		Buf[0] = SPI.transfer(readAddress);
		Buf[1] = SPI.transfer(0); // Byte 23-16
		Buf[2] = SPI.transfer(0); // Byte 15-8
		Buf[3] = SPI.transfer(0); // Byte 7-0
	#else
		spi4teensy3::send(readAddress);
		spi4teensy3::receive(Buf,3);
	#endif
	
	temp = (
	((uint32_t)Buf[1]<<16) |
	((uint32_t)Buf[2]<<8) |
	((uint32_t)Buf[3])
	);
	
	//Convert to 32bit 2's complement
	if((temp & 0x00800000) > 0){
		temp |= 0xFF000000;
	}
	else{
		temp &= 0x00FFFFFF;
	}
	Data[0] = temp;
	
	if (chToRead >= 2){
		temp = 0;
		#ifdef UseArduinoSPI
		Buf[1] = SPI.transfer(0); // Byte 23-16
		Buf[2] = SPI.transfer(0); // Byte 15-8
		Buf[3] = SPI.transfer(0); // Byte 7-0
		#else
		spi4teensy3::receive(Buf,3);
		#endif
		
		temp = (
		((uint32_t)Buf[1]<<16) |
		((uint32_t)Buf[2]<<8) |
		((uint32_t)Buf[3])
		);
		
		//Convert to 32bit 2's complement
		if((temp & 0x00800000) > 0){
			temp |= 0xFF000000;
		}
		else{
			temp &= 0x00FFFFFF;
		}

		Data[1] = temp;
	}
	if (chToRead >= 3){
		temp = 0;
		#ifdef UseArduinoSPI
		Buf[1] = SPI.transfer(0); // Byte 23-16
		Buf[2] = SPI.transfer(0); // Byte 15-8
		Buf[3] = SPI.transfer(0); // Byte 7-0
		#else
		spi4teensy3::receive(Buf,3);
		#endif
		
		temp = (
		((uint32_t)Buf[1]<<16) |
		((uint32_t)Buf[2]<<8) |
		((uint32_t)Buf[3])
		);
		
		//Convert to 32bit 2's complement
		if((temp & 0x00800000) > 0){
			temp |= 0xFF000000;
		}
		else{
			temp &= 0x00FFFFFF;
		}
		Data[2] = temp;
	}

	if (chToRead >= 4){
		temp = 0;
		#ifdef UseArduinoSPI
		Buf[1] = SPI.transfer(0); // Byte 23-16
		Buf[2] = SPI.transfer(0); // Byte 15-8
		Buf[3] = SPI.transfer(0); // Byte 7-0
		#else
		spi4teensy3::receive(Buf,3);
		#endif
	
		temp = (
		((uint32_t)Buf[1]<<16) |
		((uint32_t)Buf[2]<<8) |
		((uint32_t)Buf[3])
		);
	
		//Convert to 32bit 2's complement
		if((temp & 0x00800000) > 0){
			temp |= 0xFF000000;
		}
		else{
			temp &= 0x00FFFFFF;
		}
		Data[3] = temp;
	}
	
	digitalWriteFast(_CSpin, HIGH);
	#ifdef UseArduinoSPI
	SPI.endTransaction();
	#endif

	return (int32_t)chToRead;
}

int32_t MCP3912::ReadSingleValue(uint8_t chAddr){
	byte channel;
	if (chAddr == 0){
		channel = Addresses.CHANNEL0;
	}
	else if(chAddr == 1){
		channel = Addresses.CHANNEL1;
	}
	else if(chAddr == 2){
		channel = Addresses.CHANNEL2;
	}
	else if(chAddr == 3){
		channel = Addresses.CHANNEL3;
	}
	else{
		channel = Addresses.CHANNEL0; //default
	}
	
	byte Buf[4] = {0,0,0,0};
	
	digitalWriteFast(_CSpin, LOW);
	#ifdef UseArduinoSPI
	SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
	Buf[3] = SPI.transfer(MCP3912_READ_MASK | (channel<<1));
	Buf[0] = SPI.transfer(0); // Byte 23-16
	Buf[1] = SPI.transfer(0); // Byte 15-8
	Buf[2] = SPI.transfer(0); // Byte 7-0
	#else
	spi4teensy3::send(MCP3912_READ_MASK | (channel<<1));
	spi4teensy3::receive(Buf,3);
	#endif
	
	digitalWriteFast(_CSpin, HIGH);
	int32_t temp = 0;
	temp = (
	((uint32_t)Buf[0]<<16) |
	((uint32_t)Buf[1]<<8) |
	((uint32_t)Buf[2])
	);
	
	//Convert to 32bit 2's complement
	if((temp & 0x00800000) > 0){
		temp |= 0xFF000000;
	}
	else{
		temp &= 0x00FFFFFF;
	}
	return temp;
}

void MCP3912::ReadConfiguration(void){
	digitalWriteFast(_CSpin, LOW);
	
	#ifdef UseArduinoSPI
		byte RegisterInfo[3] = {0,0,0};
		SPI.beginTransaction(SPISettings(ArduinoSPISpeed, MSBFIRST,SPI_MODE0));
		SPI.transfer(MCP3912_READ_MASK | (Addresses.GAIN<<1));
		Registers._GainSettings[0] = SPI.transfer(0xFF);
		Registers._GainSettings[1] = SPI.transfer(0xFF); 
		Registers._GainSettings[2] = SPI.transfer(0xFF);
		Registers._StatusComSettings [0] = SPI.transfer(0xFF);
		Registers._StatusComSettings [1] = SPI.transfer(0xFF);
		Registers._StatusComSettings [2] = SPI.transfer(0xFF);
		Registers._Config0Register[0] = SPI.transfer(0xFF);
		Registers._Config0Register[1] = SPI.transfer(0xFF);
		Registers._Config0Register[2] = SPI.transfer(0xFF);
		Registers._Config1Register[0] = SPI.transfer(0xFF);
		Registers._Config1Register[1] = SPI.transfer(0xFF);
		Registers._Config1Register[2] = SPI.transfer(0xFF);
		SPI.endTransaction();
	#else
		byte buf[12] {0,0,0,0,0,0,0,0,0,0,0,0};
		spi4teensy3::send(MCP3912_READ_MASK | (Addresses.GAIN<<1));
		spi4teensy3::receive(buf,12);
		Registers._GainSettings[0] = buf[0];
		Registers._GainSettings[1] = buf[1];
		Registers._GainSettings[2] = buf[2];
		Registers._StatusComSettings [0] = buf[3];
		Registers._StatusComSettings [1] = buf[4];
		Registers._StatusComSettings [2] = buf[5];
		Registers._Config0Register[0] = buf[6];
		Registers._Config0Register[1] = buf[7];
		Registers._Config0Register[2] = buf[8];
		Registers._Config1Register[0] = buf[9];
		Registers._Config1Register[1] = buf[10];
		Registers._Config1Register[2] = buf[11];
	#endif
// 	Serial.print("Gain Settings: ");	
// 	printBinaryByte(Registers._GainSettings,3);
// 	Serial.println();
// 	Serial.print("Status COM Settings: ");
// 	printBinaryByte(Registers._StatusComSettings,3);
// 	Serial.println();
// 	Serial.print("Config0 Register: ");
// 	printBinaryByte(Registers._Config0Register,3);
// 	Serial.println();
// 	Serial.print("Config1 Register: ");
// 	printBinaryByte(Registers._Config1Register,3);
// 	Serial.println();
}

void MCP3912::printBinaryByte(byte *data, uint8_t length){
	for(int i=0; i<length;i++){
		for (unsigned int mask = 0x80; mask; mask >>= 1) {
			Serial.print(mask&data[i]?'1':'0');
		}
	}
}

void MCP3912::printBinaryInt16(uint16_t *data, uint8_t length){
	for(int i=0; i<length;i++){
		for (unsigned int mask = 0x8000; mask; mask >>= 1) {
			Serial.print(mask&data[i]?'1':'0');
		}
	}
}

void MCP3912::printBinaryInt32(uint32_t *data, uint8_t length){
	for(int i=0; i<length;i++){
		for (unsigned int mask = 0x80000000; mask; mask >>= 1) {
			Serial.print(mask&data[i]?'1':'0');
		}
	}
}

