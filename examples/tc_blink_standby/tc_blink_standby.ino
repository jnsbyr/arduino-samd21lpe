/**
 * reduce SAMD21 power consumption for blink by passive waiting in sleep mode STANDBY
 * to approximately 2 µA at 48 MHz / 3.00 V (excluding LEDs)
 */

#include <System.h>
#include <TimerCounter.h>
using namespace SAMD21LPE;


TimerCounter timer;

void setupTimer() {
  // configure low power clock generator
  const byte GCLKGEN_ID_1K = 6;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // configure timer counter to run at 1 kHz and to continue in standby
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16, 1000U, true);

  // select sleep mode STANDBY for timer wait
  System::setSleepMode(System::STANDBY);
}

void sleep(uint32_t ms) {
  timer.wait(ms);
  //delay(ms);
}

void setup() {
  // disable non essential MCU modules (incl. USB)
  System::reducePowerConsumption();
  System::enablePORT();

  // setup timer counter
  setupTimer();
}

void loop() {
  digitalWrite(PIN_LED3, LOW);
  sleep(50);

  digitalWrite(PIN_LED3, HIGH);
  sleep(1000);
}