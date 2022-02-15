#include "MyMobaTools.h"

MySoftLed::MySoftLed() {
  isI2C = false;
#ifdef USE_I2C
  pSoftLed.motoSoftLed = NULL;
#else
  pSoftLed.motoSoftLed = new MoToSoftLed();
#endif
}

#ifdef USE_I2C
uint8_t MySoftLed::attach(uint8_t pinArg, uint8_t invArg) {
  // Led-Ausgang mit Softstart.
  if ((pinArg & I0) == I0) {
    byte i2cPin = pinArg - I0;
    if (pSoftLed.i2cSoftLed == NULL) pSoftLed.i2cSoftLed = new I2cSoftLed();
    isI2C = true;
    return pSoftLed.i2cSoftLed->attach(i2cPin, !invArg);
  }
  if (pSoftLed.motoSoftLed == NULL) pSoftLed.motoSoftLed = new MoToSoftLed();
  return pSoftLed.motoSoftLed->attach(pinArg, invArg);
}
void MySoftLed::riseTime( uint16_t time) {
  // in millisec - falltime is the same
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->riseTime(time);
  else pSoftLed.motoSoftLed->riseTime(time);
}
void MySoftLed::on() {
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->on();
  else pSoftLed.motoSoftLed->on();
}
void MySoftLed::off() {
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->off();
  else pSoftLed.motoSoftLed->off();
}
void MySoftLed::on(uint8_t pwm) {
  // pwm value for ON ( in % )
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->on(pwm);
  else pSoftLed.motoSoftLed->on(pwm);
}
void MySoftLed::off(uint8_t pwm) {
  //  pwmValue for OFF ( in % )
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->off(pwm);
  else pSoftLed.motoSoftLed->off(pwm);
}
void MySoftLed::write( uint8_t state) {
  // is ON or OFF
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->write(state);
  else pSoftLed.motoSoftLed->write(state);
}
void MySoftLed::write( uint8_t time, uint8_t type  ) {
  //whether it is a linear or bulb type
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->write(time, type);
  else pSoftLed.motoSoftLed->write(time, type);
}
void MySoftLed::toggle( void ) {
  if (pSoftLed.i2cSoftLed == NULL) return;
  if (isI2C) pSoftLed.i2cSoftLed->toggle();
  else pSoftLed.motoSoftLed->toggle();
}
void MySoftLed::process() {
  if (isI2C) pSoftLed.i2cSoftLed->process();
}
#else
uint8_t MySoftLed::attach(uint8_t pinArg, uint8_t invArg) {
  pSoftLed.motoSoftLed->attach(pinArg, invArg);
}
void MySoftLed::riseTime( uint16_t time) {
  pSoftLed.motoSoftLed->riseTime(time);
}
void MySoftLed::on() {
  pSoftLed.motoSoftLed->on();
}
void MySoftLed::off() {
  pSoftLed.motoSoftLed->off();
}
void MySoftLed::on(uint8_t pwm) {
  pSoftLed.motoSoftLed->on(pwm);
}
void MySoftLed::off(uint8_t pwm) {
  pSoftLed.motoSoftLed->off(pwm);
}
void MySoftLed::write( uint8_t state) {
  pSoftLed.motoSoftLed->write(state);
}
void MySoftLed::write( uint8_t time, uint8_t type  ) {
  pSoftLed.motoSoftLed->write(time, type);
}
void MySoftLed::toggle( void ) {
  pSoftLed.motoSoftLed->toggle();
}
void MySoftLed::process() {
  
}
#endif

//------------ I2cSoftLed methods --------------------
I2cSoftLed::I2cSoftLed() {
  _ledData.invFlg = true;
  _ledData.state = NOTATTACHED;
}
#ifdef USE_I2C
uint8_t I2cSoftLed::attach(uint8_t i2cPin, uint8_t invArg) {
  // Led-Ausgang mit Softstart.
  _ledData.pin = i2cPin;    // i2c Pin-Nbr
  _ledData.invFlg = invArg;
  _ledData.state  = STATE_OFF;   // initialize
  riseTime( LED_DEFAULT_RISETIME );
  if ( _ledData.invFlg ) {
    pwmController.setChannelOn(i2cPin);
  } else {
    pwmController.setChannelOff(i2cPin);
  }
  return true;
}
void I2cSoftLed::riseTime( uint16_t riseTime ) {
    if ( _ledData.state == NOTATTACHED ) return;
    // length of startphase in ms (min 25ms)
    if (riseTime == 0) {
      _ledData.ledRiseTime = 0;
      return;
    }
    if (riseTime < 25) riseTime = 25;
    _ledData.ledRiseTime = riseTime;
}
void I2cSoftLed::on() {
  on(100);
}
void I2cSoftLed::off() {
  off(0);
}
void I2cSoftLed::on(uint8_t pwm) {
  // hier wird nur der pwm Wert in Prozent empfangen.
  _ledData.pwmOn = pwm;
  _setState(STATE_ON);
  if (_ledData.ledRiseTime == 0) _set(pwm);
}
void I2cSoftLed::off(uint8_t pwm) {
  // hier wird nur der pwm Wert in Prozent empfangen.
  _setState(STATE_OFF);
  if (_ledData.ledRiseTime == 0) _set(pwm);
}
void I2cSoftLed::write( uint8_t state) {
  if ( state > 0 ) on(); else off();
}
void I2cSoftLed::write( uint8_t state, uint8_t type  ) {
  _ledData.ledType = type;
  write( state ) ;
}
void I2cSoftLed::toggle( void ) {
  if ((_ledData.state == STATE_ON) || (_ledData.state == INCBULB) || (_ledData.state == INCLIN)) off();
  else on();
}
void I2cSoftLed::process() {
  if (_ledData.state < ACTIVE) return; // LED wird hart ein- und ausgeschaltet oder die Auf- / Abblendphase ist abgeschlossen
  // Auf- oder Abblenden steuern
  if (blendTimer.running()) return; // nächster Schritt noch nicht erreicht

  _ledData.aStep++;
  uint16_t pwm = _ledData.pwmOn;
  bool increase = true;
  switch (_ledData.state) {
    // erste implementierung: wir unterscheiden nicht zwischen Bulb und Lin
    case INCBULB:
    case INCLIN:
      pwm = (pwm * _ledData.aStep) / RISE_STEPS;
      increase = true;
      break;
    case DECBULB:
    case DECLIN:
      pwm = (pwm * (RISE_STEPS - _ledData.aStep)) / RISE_STEPS;
      increase = false;
      break;
  }
  _set(pwm);
  if (_ledData.aStep < RISE_STEPS) {
    blendTimer.restart(); // timer neu starten
  }
  else {                  // Auf- Abblenden ist beendet
    if (increase) {
      _set(_ledData.pwmOn);
      _ledData.state = STATE_ON;
    }
    else {
      _set(0);
      _ledData.state = STATE_OFF;
    }
    blendTimer.stop();
    #ifdef DEBUG
      DB_PRINT ("blendTimer stopped. PWM is '%d'", pwm);
    #endif
  }
}
/* 
 *  set the LED state depending on soft/hard LED
 *  should only be called with 'STATE_ON' or 'STATE_OFF' (not checked).
 */
void I2cSoftLed::_setState(LedStats_t state){
  if (_ledData.state == state) return;  // wenn der status nicht geändert wird nichts tun
  if (_ledData.ledRiseTime == 0) {      // wenn die LED hart ein- und ausschaltet kann der Status direkt übernommen werden
    _ledData.state = state;
    return;
  }
  switch (state) {
    case STATE_ON:
      if ((_ledData.state == INCBULB) || (_ledData.state == INCLIN)) return; // in der Aufblendphase nicht nochmal einschalten
      if (_ledData.ledType) _ledData.state = INCBULB;
      else _ledData.state = INCLIN;
      break;
    case STATE_OFF:
      if ((_ledData.state == DECBULB) || (_ledData.state == DECLIN)) return; // in der Abblendphase nicht nochmal abschalten
      if (_ledData.ledType) _ledData.state = DECBULB;
      else _ledData.state = DECLIN;
      break;
  }
  _ledData.aStep = 0;
  // setze Timer auf die Zeit eines Schrittes beim Auf- oder Abblenden
  blendTimer.setTime(_ledData.ledRiseTime / RISE_STEPS);
}
/*
 * Set pwm value for LED in percent.
 */
void I2cSoftLed::_set(uint8_t pwm) {
  if (_ledData.state == NOTATTACHED) return;
  unsigned long absPwm = MAX_PWM;
  // hier wird nur der pwm Wert in Prozent empfangen.
  if ( 100 < pwm ) pwm = 100; // Wert darf max 100 sein.
  if (_ledData.invFlg) pwm = 100 - pwm; // Wenn Ausgang invertiert, dann Prozentzahl invertieren
  absPwm = (absPwm * pwm) / 100;
  #ifdef DEBUG
    DB_PRINT ("_set. pwm: '%d' absPWM is '%d'", pwm, absPwm);
  #endif
  // set absolute pwm value
  if (absPwm < 1) {
    pwmController.setChannelOff(_ledData.pin);
    return;
  }
  if (absPwm > (MAX_PWM-1)) {
    pwmController.setChannelOn(_ledData.pin);
    return;
  }
  pwmController.setChannelPWM(_ledData.pin, absPwm);
}

// ---------------------------------------------------------------------------------------------
I2cServo::I2cServo(){
  
}
uint8_t I2cServo::attach(int pin, bool autoOff){
  return 0;
}
void I2cServo::detach(){
  
}


MyServo::MyServo(){
  isI2C = false;
}
uint8_t MyServo::attach(int pin){
  // attach to a pin, sets pinMode, returns 0 on failure, won't
  // position the servo until a subsequent write() happens
  return attach(pin, false);
}
//Es wird nur diese Version der attach Methode aufgerufen
uint8_t MyServo::attach(int pin, bool autoOff){
  if ((pin & I0) == I0) {
    byte i2cPin = pin - I0;
    if (pServo.i2cServo == NULL) pServo.i2cServo = new I2cServo();
    isI2C = true;
    return pServo.i2cServo->attach(pin, autoOff);
  }
  if (pServo.motoServo == NULL) pServo.motoServo = new MoToServo();
  pServo.motoServo->attach(pin, autoOff);
}
// diese Methode wird von Fservo nicht aufgerufen
uint8_t MyServo::attach(int pin, uint16_t pos0, uint16_t pos180){
  // also sets position values (in us) for angele 0 and 180
  return attach(pin, pos0, pos180, false);
}
// diese Methode wird von Fservo nicht aufgerufen
uint8_t MyServo::attach(int pin, uint16_t pos0, uint16_t pos180, bool autoOff){
  return 0;
}
// diese Methode wird von Fservo nicht aufgerufen
void MyServo::detach(){
  if (isI2C) {
    pServo.i2cServo->detach();
    delete (pServo.i2cServo);
    pServo.i2cServo = NULL;
  }
  else {
    pServo.motoServo->detach();
    delete (pServo.motoServo);
    pServo.motoServo = NULL;
  }
}
void MyServo::write(uint16_t degree){
  // specify the angle in degrees, 0 to 180. Values obove 180 are interpreted
  // as microseconds, limited to MaximumPulse and MinimumPulse
  if (isI2C) {
    
  }
  else {
    pServo.motoServo->write(degree);
  }
}
void MyServo::setSpeed(int speed){
  // Set movement speed, the higher the faster
  // Zero means no speed control (default)
  if (isI2C) {
    
  }
  else {
    pServo.motoServo->setSpeed(speed);
  }
}
// diese Methode wird von Fservo nicht aufgerufen
void MyServo::setSpeed(int speed, bool comp){
  setSpeed(speed); 
}

uint8_t MyServo::moving(){
  // returns the remaining Way to the angle last set with write() in
  // in percentage. '0' means, that the angle is reached
  if (isI2C) {
    return 0;
  }
  else {
    return pServo.motoServo->moving();
  }
}
uint8_t MyServo::read(){
  // current position in degrees (0...180)
  if (isI2C) {
    return 0;
  }
  else {
    return pServo.motoServo->read();
  }
}
// diese Methode wird von Fservo nicht aufgerufen
uint16_t  MyServo::readMicroseconds(){
  // current pulsewidth in microseconds
  if (isI2C) {
    return 0;
  }
  else {
    return pServo.motoServo->readMicroseconds();
  }  
}
// diese Methode wird von Fservo nicht aufgerufen
uint8_t MyServo::attached(){
  if (isI2C) {
    return 0;
  }
  else {
    return pServo.motoServo->attached();
  }
}
// diese Methode wird von Fservo nicht aufgerufen
void MyServo::setMinimumPulse(uint16_t){
  // pulse length for 0 degrees in microseconds, 700uS default
}
// diese Methode wird von Fservo nicht aufgerufen
void MyServo::setMaximumPulse(uint16_t){
  // pulse length for 180 degrees in microseconds, 2300uS default
}
#endif
