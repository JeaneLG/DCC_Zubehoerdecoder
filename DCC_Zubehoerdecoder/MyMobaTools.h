#include <MobaTools.h>
#include "Interface.h"


#ifdef USE_I2C
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
#define RISE_STEPS 25
#define MAX_PWM 4096
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

#ifdef USE_I2C

typedef struct myServoData_t {      // stores all Data for I2cServo
  byte pin;                         // Outputpin as I2C Port number
  byte startPos;                    // start Position of servo in degrees
  byte sollPos;                     // target Position of servo in degrees
  byte flags;                       // Servo states and options
  uint8_t speed;                    // Speed of the Servo
};

#define MIN_SERVO_PWM 100
#define MAX_SERVO_PWM 510
#define PWM_RANGE (MAX_SERVO_PWM - MIN_SERVO_PWM)
#define SERVO_UPDATE 40
#define SERVO_PAUSE 1000
#define SERVO_SPEED_MUL 8
#define SERVO_POS_UNKOWN 0xFFu
#define FLAG_AUTO_OFF     B1        // Pause PWM signal if servo is not moving
#define FLAG_IN_AUTO_OFF B10        // True if PWM signal is paused
#define FLAG_IS_MOVING  B100        // true if servo is moved with speed controll
class I2cServo {
  public:
    I2cServo();
    uint8_t attach(byte pin, bool autoOff);          // automatic switch off pulses with constant length
    void detach();
    void write(uint16_t degree);
    void setSpeed(int speed);                       // Set movement speed, the higher the faster
    uint8_t moving();                               // returns the remaining Way to the angle last set with write() in
                                                    // in percentage. '0' means, that the angle is reached
    uint8_t read();                                 // returns the actual servo position angle
    void process();

  private:
    MoToTimer servoTimer;
    myServoData_t _servoData;
};

class MyServo
{
  public:
    // don't allow copying and moving of Servo objects
    MyServo &operator= (const MyServo & )   =delete;
    MyServo &operator= (MyServo && )        =delete;
    MyServo (const MyServo & )              =delete;
    MyServo (MyServo && )                   =delete;
    
    MyServo();
    uint8_t attach(int pin); // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
    uint8_t attach( int pin, bool autoOff );        // automatic switch off pulses with constant length
    uint8_t attach(int pin, uint16_t pos0, uint16_t pos180 ); // also sets position values (in us) for angele 0 and 180
    uint8_t attach(int pin, uint16_t pos0, uint16_t pos180, bool autoOff );
    void detach();
    void write(uint16_t);    // specify the angle in degrees, 0 to 180. Values obove 180 are interpreted
                             // as microseconds, limited to MaximumPulse and MinimumPulse
    void setSpeed(int);      // Set movement speed, the higher the faster
                             // Zero means no speed control (default)
    void setSpeed(int,bool); // Set compatibility-Flag (true= compatibility with version V08 and earlier)
    
    uint8_t moving();        // returns the remaining Way to the angle last set with write() in
                             // in percentage. '0' means, that the angle is reached
    uint8_t read();          // current position in degrees (0...180)
    uint16_t  readMicroseconds();// current pulsewidth in microseconds
    uint8_t attached();
    void setMinimumPulse(uint16_t);  // pulse length for 0 degrees in microseconds, 700uS default
    void setMaximumPulse(uint16_t);  // pulse length for 180 degrees in microseconds, 2300uS default
    void process();
    
  private:
    bool isI2C;
    union {
      MoToServo *motoServo;
      I2cServo  *i2cServo;
    }pServo;
};
#endif
