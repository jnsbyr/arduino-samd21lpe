/**
 * reduce SAMD21 power consumption for blink by passive waiting in sleep mode IDLE2
 * to approximately 2 mA at 48 MHz / 3.00 V (excluding LEDs)
 */

#include <System.h>
#include <TimerCounter.h>
using namespace SAMD21LPE;


TimerCounter timer;

void setupTimer() {
  // configure low power clock generator to run at 1 kHz
  const byte GCLKGEN_ID_1K = 6;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // configure timer counter to run at 1 kHz
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16);

  // select MCU sleep mode IDLE2 for timer wait
  System::setSleepMode(System::IDLE2);
}

void sleep(uint32_t ms) {
  timer.wait(ms);
  //delay(ms);
}

void setup() {
  // disable non essential MCU modules (including USB)
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