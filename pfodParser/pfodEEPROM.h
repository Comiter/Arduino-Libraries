/*
 * (c)2014-2017 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code is not warranted to be fit for any purpose. You may only use it at your own risk.
 * This code may be freely used for both private and commercial use
 * Provide this copyright is maintained.
 */
#ifndef pfodEEPROM_h
#define pfodEEPROM_h

#ifdef __RFduino__
#define __no_pfodEEPROM__
#elif ARDUINO_ARCH_AVR
#include <EEPROM.h>
#elif ARDUINO_ARCH_ESP8266
#include <EEPROM.h>
#elif ARDUINO_ARCH_ARC32
#include <EEPROM.h>
#else
#define __no_pfodEEPROM__
#endif

#endif //pfodEEPROM_h

