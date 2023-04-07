/**
 * blocking read of internal temperature of SAMD21
 * sample using active waiting (delay)
 *
 * - 1 Hz sampling rate
 * - 93.75 kHz ADC clock, 12 bits resolution, 15 µs sampling time,
 *   hardware averaging of 8 samples -> 803 µs conversion time
 * - 48 MHz, 3.0 V: 13 mA, 7 mA excl. USB/LED
 */

#include <Analog2DigitalConverter.h>
#include <System.h>
using namespace SAMD21LPE;

// comment in to enable serial debug output
//#define USB_DEBUG

Analog2DigitalConverter& adc = Analog2DigitalConverter::instance();

void setupADC() {
  // enable ADC (48 MHz / 512 -> 93.75 kHz)
  adc.enable(GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, Analog2DigitalConverter::DIV512);

  // configure hardware averaging (2^3 = 8)
  adc.setSampling(0, 3);

  // disable ADC to save power until start of conversion
  adc.disable();
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
}

void loop() {
  // read internal temperature
  float t = adc.read(ADC_INPUTCTRL_MUXPOS_TEMP_Val);

#ifdef USB_DEBUG
  // print temperature
  SerialUSB.print(t);
  SerialUSB.println(" °C");
#else
  // flash LED
  digitalWrite(PIN_LED3, LOW);
  delay(50);
  digitalWrite(PIN_LED3, HIGH);
#endif

  // wait
  delay(1000);
}
