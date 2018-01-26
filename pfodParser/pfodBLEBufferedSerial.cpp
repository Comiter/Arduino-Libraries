#include <Arduino.h>
#include "pfodBLEBufferedSerial.h"
/**
  (c)2015 Forward Computing and Control Pty. Ltd.
  This code may be freely used for both private and commerical use.
  Provide this copyright is maintained.
*/

// uncomment this next line and call setDebugStream(&Serial); to enable debug out
//#define DEBUG

void pfodBLEBufferedSerial::setDebugStream(Print* out) {
  debugOut = out;
}

pfodBLEBufferedSerial::pfodBLEBufferedSerial() {
  stream = NULL;
  debugOut = NULL;
  size_t _bufferSize = PFOD_DEFAULT_SEND_BUFFER_SIZE;
  setBuffer(_bufferSize);
}

pfodBLEBufferedSerial::pfodBLEBufferedSerial(size_t _bufferSize) {
  stream = NULL;
  debugOut = NULL;
  setBuffer(_bufferSize);
}

void pfodBLEBufferedSerial::setBuffer(size_t _bufferSize) {
  sendBufferIdxHead = 0;
  sendBufferIdxTail = 0;
  if ((sendBuffer == NULL) && (_bufferSize > defaultBufferSize)) {
    bufferSize = _bufferSize;
    sendBuffer = (uint8_t*)malloc(bufferSize);
  }
  if (sendBuffer == NULL) {
    bufferSize = defaultBufferSize;
    sendBuffer = defaultBuffer;
  }
}

pfodBLEBufferedSerial* pfodBLEBufferedSerial::connect(Stream* _stream) {
#ifdef DEBUG
  if (debugOut) {
    debugOut->println("pfodBLEBufferedSerial connected.");
  }
#endif // DEBUG		
  stream = _stream;
  sendBufferIdxHead = 0;
  sendBufferIdxTail = 0;
  timerRunning = false;
  return this;
}


size_t pfodBLEBufferedSerial::write(const uint8_t *buf, size_t size) {
  if (!stream) {
    return 0;
  }
  for (size_t i = 0; i < size; i++) {
    _write(buf[i]); // may block if 1460 buffer fills and stream starts sending packet
  }
  return size;
}

size_t pfodBLEBufferedSerial::write(uint8_t c) {
  if (!stream) {
    return 0;
  }
  return _write(c);
}

size_t pfodBLEBufferedSerial::_write(uint8_t c) {
  if (!stream) {
    return 0;
  }

  sendAfterDelay(); // try sending first to free some buffer space

  size_t rtn = 1;
  if (!timerRunning) {
    timerRunning = true;
    sendTimerStart = millis();
  }

  size_t i = (sendBufferIdxHead + 1) % bufferSize;

  if (i == sendBufferIdxTail) {
    // If the output buffer is full, just drop the char
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("buffer full dropping "); debugOut->print((char)c);
    }
#endif // DEBUG    
  } else {
    sendBuffer[sendBufferIdxHead] = c;
    sendBufferIdxHead = i;
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print((char)c);
    }
#endif // DEBUG    
  }
  sendAfterDelay(); // try sending last if this was first write

  return rtn;
}

size_t pfodBLEBufferedSerial::bytesToBeSent() {
  if (sendBufferIdxTail <= sendBufferIdxHead) {
    return (sendBufferIdxHead - sendBufferIdxTail);
  } // else
  return (sendBufferIdxHead + bufferSize - sendBufferIdxTail);
}

void pfodBLEBufferedSerial::sendAfterDelay() {
  if ((!stream) || (sendBufferIdxHead == sendBufferIdxTail)) {  // common cases
    return; // nothing to do
  }
  if (timerRunning && ((millis() - sendTimerStart) > BLE_SEND_DELAY_TIME) ) {
    sendTimerStart = millis(); // update for next send
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->println();
      debugOut->print("sendAfterDelay() "); debugOut->print(bytesToBeSent()); debugOut->println(" bytes waiting to be sent");
      debugOut->println(millis());
    }
#endif // DEBUG    
    // send next 20 bytes
    size_t i = 0; // max to send is
    while ((i < BLE_SEND_BLOCK_SIZE) && (sendBufferIdxHead != sendBufferIdxTail)) {
      stream->write((const uint8_t)(sendBuffer[sendBufferIdxTail]));
#ifdef DEBUG
      if (debugOut != NULL) {
        debugOut->print((char)sendBuffer[sendBufferIdxTail]);
      }
#endif // DEBUG    
      i++;
      sendBufferIdxTail = (sendBufferIdxTail + 1) % bufferSize;
    }
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->println();
    }
#endif // DEBUG    
    if (sendBufferIdxHead == sendBufferIdxTail) {
      // empty
      timerRunning = false;
    }
  }
}


// expect available to ALWAYS called before read() so update timer here
int pfodBLEBufferedSerial::available() {
  sendAfterDelay();
  if (!stream) {
    return 0;
  }
  return stream->available();
}

int pfodBLEBufferedSerial::read() {
  sendAfterDelay();
  if (!stream) {
    return -1;
  }
  int c = stream->read();
  return c;
}

int pfodBLEBufferedSerial::peek() {
  sendAfterDelay();
  if (!stream) {
    return -1;
  }
  return stream->peek();
}

void pfodBLEBufferedSerial::flush() {
  sendAfterDelay();
}
