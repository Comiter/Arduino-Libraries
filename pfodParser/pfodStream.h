#ifndef pfodStream_h
#define pfodStream_h

// This include handles the rename of Stream for MBED compiles
// it is included for pfodParser, pfodBLEBufferedStream, pfod_Base, pfodSMS_SIM900 and pfodWaitForUtils 
#if defined ( ARDUINO_NRF52_FEATHER )
  #include <Stream.h>
#elif defined( ARDUINO_RBL_nRF51822 ) || defined( NRF52 ) || defined( __MBED__ )
  #include <WStream.h>
  #define Stream WStream
#else
  #include <Stream.h>
#endif

#endif