/**
 * wait for external interrupt of SAMD21 in standby mode
 *
 * - 2 µA standby current at 3.0 V excl. USB/LED
 */

#include <System.h>
using namespace SAMD21LPE;


void externalInterruptHandler() {
  // mirror state of A0/D0 to LED3
  digitalWrite(PIN_LED3, digitalRead(PIN_A0));
}

void setupEIC() {
  noInterrupts();

  // enable PORT and EIC modules
  System::enablePORT();
  System::enableEIC();

  // enable external interrupt on pin A0/D0
  pinMode(PIN_A0, INPUT_PULLUP);
  attachInterrupt(PIN_A0, externalInterruptHandler, CHANGE);

  // configure low power clock generator to run at 1 kHz
  const byte GCLKGEN_ID_1K = 6;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // change clock generator for EIC from 0 (DFLL48M, assigned by attachInterrupt) to 6 (OSCULP32K), which stays enabled in standby (required for input edge detection)
  System::enableClock(GCM_EIC, GCLKGEN_ID_1K);

  interrupts();
}

void setup() {
  // disable non essential MCU modules (incl. USB, PORT and EIC)
  System::reducePowerConsumption();

  // setup external interrupt controller
  setupEIC();

  // select sleep mode STANDBY between ISR calls and enable sleep-on-exit mode
  System::setSleepOnExitISR(true);
  System::setSleepMode(System::STANDBY);
}

void loop() {
  // sleep-on-exit mode, do nothing here
  System::sleep();
}
