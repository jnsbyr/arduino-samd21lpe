/**
 * blocking read of internal temperature of SAMD21
 * sample using passive waiting with timer wakeup from standby
 *
 * - 1 Hz sampling rate
 * - 750 kHz ADC clock, 12 bits resolution, 15 µs sampling time, 
 *   hardware averaging of 8 samples -> 205 µs conversion time
 * - 48 MHz, 3 V, IDLE0: 8 mA
 * - 2 µA standby current at 3.0 V excl. USB/LED
 */

#include <cfloat>

#include <Analog2DigitalConverter.h>
#include <System.h>
#include <TimerCounter.h>
using namespace SAMD21LPE;

// comment in to enable serial debug output
//#define USB_DEBUG

Analog2DigitalConverter& adc = Analog2DigitalConverter::instance();
TimerCounter timer;

volatile float averageTemp = FLT_MAX;

void timerInterruptHandler() {
  float t = adc.read(ADC_INPUTCTRL_MUXPOS_TEMP_Val);

  // calculate exponential moving average (1:3)
  if (averageTemp == FLT_MAX) {
    // init temp
    averageTemp = t;
  } else {
    // update temp
    averageTemp = (3 * averageTemp + t) / 4;
  }

#ifdef USB_DEBUG
  // print temperature
  SerialUSB.print(averageTemp);
  SerialUSB.println(" °C");
#else
  // flash LED
  digitalWrite(PIN_LED3, !digitalRead(PIN_LED3));
#endif
}

void setupADC() {
  // enable ADC (48 MHz / 64 -> 750 kHz)
  adc.enable(GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, Analog2DigitalConverter::DIV64);

  // configure continuous hardware averaging (2^3 = 8)
  adc.setSampling(0, 3);

  // disable ADC to save power until start of conversion
  adc.disable();
}

void setupTimer() {
  // configure low power clock generator to run at 1 kHz
  const byte GCLKGEN_ID_1K = 6;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // configure timer counter to run at 1 kHz and to continue in standby
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16, true);

  // start timer with 1000 ms period and callback
  timer.start(1000, true, timerInterruptHandler);
}

void setup() {
#ifdef USB_DEBUG
  // enable serial over USB to print temperature and wait for serial monitor
  SerialUSB.begin(9600);
  while(!SerialUSB);
#else
  // disable non essential MCU modules (incl. USB)
  System::reducePowerConsumption();
  System::enablePORT();
#endif

  // setup ADC and sampling timer
  setupADC();
  setupTimer();

#ifdef USB_DEBUG
  // select sleep mode IDLE0 to keep USB alive
  System::setSleepMode(System::IDLE0);
#else
  // select sleep mode STANDBY between ISR calls and enable sleep-on-exit mode
  System::setSleepOnExitISR(true);
  System::setSleepMode(System::STANDBY);
#endif
}

void loop() {
  // sleep-on-exit mode, do nothing here
  System::sleep();
}