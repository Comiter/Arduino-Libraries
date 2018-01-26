#ifndef pfodBLEBufferedSerial_h
#define pfodBLEBufferedSerial_h
/**
  (c)2015 Forward Computing and Control Pty. Ltd.
  This code may be freely used for both private and commerical use.
  Provide this copyright is maintained.
*/

#include "pfodStream.h"

class pfodBLEBufferedSerial : public Stream {

  public:
    pfodBLEBufferedSerial(); // default 1024 byte buffer and default send delay 200mS
    pfodBLEBufferedSerial(size_t _bufferSize); // set buffer size (min size is 32) and use default send delay 200mS
    pfodBLEBufferedSerial* connect(Stream* _stream);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();
    void setDebugStream(Print* out);
    size_t bytesToBeSent(); // bytes in buffer to be sent
  private:
    Stream* stream;
    void sendAfterDelay();
    size_t _write(uint8_t c);
    static const unsigned long BLE_SEND_DELAY_TIME = 200; // 200mS delay between 20byte msgs
    static const size_t PFOD_DEFAULT_SEND_BUFFER_SIZE = 1024; // Max data size pfodApp msg
    size_t bufferSize;
    static const size_t BLE_SEND_BLOCK_SIZE = 20; // BLE msg size
    uint8_t* sendBuffer; // allow for terminating null
    static const size_t defaultBufferSize = 32; // BLE msg size    
    uint8_t defaultBuffer[defaultBufferSize]; // if malloc fails
    size_t sendBufferIdxHead = 0;
    size_t sendBufferIdxTail = 0;
    unsigned long sendTimerStart = 0;
    bool timerRunning = false;
    void setBuffer(size_t _bufferSize);
    Print* debugOut;
};

#endif // pfodBLEBufferedSerial_h
