/**
 * non-blocking read of internal temperature of SAMD21
 * using ADC result ready callback
 *
 * - 5 kHz sampling rate:
 * - 93.75 kHz ADC clock, 12 bits resolution, 15 µs sampling time,
 *   hardware averaging of 2 samples -> 200 µs conversion time
 * - 48 MHz, 3 V, IDLE0: 12 mA
 * - 48 MHz, 3 V, IDLE2: 6 mA excl. USB/LED
 */

#include <cfloat>

#include <Analog2DigitalConverter.h>
#include <System.h>
using namespace SAMD21LPE;

// comment in to enable serial debug output
//#define USB_DEBUG

Analog2DigitalConverter& adc = Analog2DigitalConverter::instance();

volatile float averageTemp = FLT_MAX;

void readADC() {
  // read internal temperature
  float t = adc.read();

  // calculate exponential moving average (1:3)
  if (averageTemp == FLT_MAX) {
    // init temp
    averageTemp = t;
  } else {
    // update temp
    averageTemp = (3 * averageTemp + t) / 4;
  }

#ifndef USB_DEBUG
  // toggle LED
  digitalWrite(PIN_LED3, millis()%1000 < 50? LOW : HIGH);
#endif
}

void setupADC() {
  // enable ADC (48 MHz / 512 -> 93.75 kHz)
  adc.enable(GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, Analog2DigitalConverter::DIV512);

  // configure continuous sampling with hardware averaging (2^1 = 2)
  adc.setSampling(0, 1, true);

  // start internal temperature measurement and register callback
  adc.start(ADC_INPUTCTRL_MUXPOS_TEMP_Val, readADC);
}

void setup() {
#ifdef USB_DEBUG
  // enable serial over USB to print temperature and wait for serial monitor
  SerialUSB.begin(9600);
  while(!SerialUSB);
#else
  // disable non essential MCU modules (including USB)
  System::reducePowerConsumption();
  System::enablePORT();
#endif

  // setup ADC
  setupADC();

#ifdef USB_DEBUG
  // select sleep mode IDLE0 to keep USB alive
  System::setSleepMode(System::IDLE0);
#else
  // select sleep mode IDLE2 between ISR calls and enable sleep-on-exit mode
  System::setSleepOnExitISR(true);
  System::setSleepMode(System::IDLE2);
#endif
}

uint16_t c = 0;

void loop() {
  System::sleep();

#ifdef USB_DEBUG
  // print temperature
  if (++c >= 5000)
  {
    SerialUSB.print(averageTemp);
    SerialUSB.println(" °C");
    c = 0;
  }
#endif
}