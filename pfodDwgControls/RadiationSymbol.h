#ifndef RadiationSymbol_h
#define RadiationSymbol_h
/*
 * (c)2014-2017 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code is not warranted to be fit for any purpose. You may only use it at your own risk.
 * This code may be freely used for both private and commercial use
 * Provide this copyright is maintained.
 */

#include <pfodDwgControls.h>

class RadiationSymbol : public pfodControl {
  public:
    RadiationSymbol(pfodDwgs *_dwgsPtr);
    void draw();
    void update();
    void setReading(float newReading);
    float getReading();

  private:
    float reading;
    int z_idx; // z-order start index
};
#endif // RadiationSymbol_h
