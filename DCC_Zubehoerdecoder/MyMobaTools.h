#include <MobaTools.h>
#include "Interface.h"


#ifdef USE_I2C
#define RISE_STEPS 25
  typedef struct myLedData_t {        // stores all Data for I2cSoftLed
    uint8_t   aStep;                  // actual step between on/off or off/on ( always counts up )
    uint8_t   pwmOn;                  // maximum pwm value in percent in on state
    LedStats_t state;                 // actual state: steady or incementing/decrementing
    volatile uint8_t invFlg;
    uint8_t pin;                      // Outputpin as I2C Port number
    uint8_t ledType;                  // Type of lamp (linear or bulb)
    int16_t ledRiseTime;              // length of startphase in ms (min 20ms)
  };
#endif

class I2cSoftLed {
  public:
    I2cSoftLed();
#ifdef USE_I2C
    static const unsigned long MAX_PWM = 4096;
#define LINEAR     0
#define BULB       1
    uint8_t attach(uint8_t i2cPin, uint8_t invArg = false);  // Led-pin with soft on
    void riseTime( uint16_t );                               // in millisec - falltime is the same
    void on();                                               // switch On
    void off();                                              // switch Off
    void on(uint8_t);                                        // pwm value for ON ( in % )
    void off(uint8_t);                                       // pwmValue for OFF ( in % ); pwmValue not used at the moment
    void write( uint8_t state);                              // state == 0 => OFF ; state > 0 => ON
    void write( uint8_t state, uint8_t type );               // whether it is a linear or bulb type
    void toggle( void );
    void process();

  private:
    MoToTimer blendTimer;
    myLedData_t _ledData;

    void _setState(LedStats_t state);
    void _set(uint8_t pwm);
#endif
};

union {
  MoToSoftLed *motoSoftLed;
  I2cSoftLed *i2cSoftLed;
}PtrSoftLed;

class MySoftLed {
  public:
    MySoftLed();
    uint8_t attach(uint8_t pinArg, uint8_t invArg = false ); // Led-pin with soft on
    void riseTime( uint16_t );                               // in millisec - falltime is the same
    void on();
    void off();
    void on(uint8_t);                                        // pwm value for ON ( in % )
    void off(uint8_t);                                       // pwmValue for OFF ( in % )
    void write( uint8_t );                                   // is ON or OFF
    void write( uint8_t state, uint8_t type  );               //whether it is a linear or bulb type
    void toggle( void );
    void process();

  private:
    bool isI2C;
    union {
      MoToSoftLed *motoSoftLed;
      I2cSoftLed *i2cSoftLed;
    }pSoftLed;
};
