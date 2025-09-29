/**
 * reduce SAMD21 power consumption for blink by passive waiting in sleep mode STANDBY
 * to approximately 2 µA at 48 MHz / 3.00 V (excluding LEDs)
 */

#include <System.h>
#include <TimerCounter.h>
using namespace SAMD21LPE;


TimerCounter timer;

void timerInterruptHandler() {
  // reenable SysTick after wakeup from STANDBY
  System::enableSysTick();
  
  // toggle LED
  if (digitalRead(PIN_LED3)) {
    // LED on
    digitalWrite(PIN_LED3, LOW);
    timer.restart(50);
  } else {
    // LED off
    digitalWrite(PIN_LED3, HIGH);    
    timer.restart(1000);
  }

  // disable SysTick before entering STANDBY
  System::disableSysTick();
}

void setupTimer() {
  // configure low power clock generator
  const byte GCLKGEN_ID_1K = 6;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // configure and start timer counter
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16, 1000U, true);
  timer.start(1, false, timerInterruptHandler);
}

void setup() {
  // disable non essential MCU modules (including USB)
  System::reducePowerConsumption();
  System::enablePORT();

  // setup timer counter
  setupTimer();

  // select sleep mode STANDBY between ISR calls and enable sleep-on-exit mode
  System::setSleepOnExitISR(true);
  System::setSleepMode(System::STANDBY);  
}

void loop() {
  // sleep-on-exit mode, do nothing here
  __WFI();  
}
