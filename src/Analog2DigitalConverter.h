/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     Analog2DigitalConverter.h
 * encoding: UTF-8
 * created:  03.02.2023
 *
 *****************************************************************************
 *
 * Copyright (C) 2023 Jens B.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *****************************************************************************/

#pragma once

#include "System.h"

namespace SAMD21LPE
{
/**
 * SAMD21 ADC access (CMSIS based implementation)
 *
 * Compatiblity:
 *
 * - can be used in parallel to Arduino core analogRead()
 * - call analogReference() before calling analogRead() and setReference() before calling read() when mixing
 *
 * Background:
 *
 * Arduino core does not support:
 * - reading internal voltages
 * - reading internal temperature
 * - variable input gain
 * - variable sampling length
 * - hardware averaging
 * - interrupt based sampling
 * - idle/standby operation
 */
class Analog2DigitalConverter
{
public:
  enum Prescaler
  {
    DIV4    = 0,
    DIV8    = 1,
    DIV16   = 2,
    DIV32   = 3,
    DIV64   = 4,
    DIV128  = 5,
    DIV256  = 6,
    DIV512  = 7
  };

  enum Reference
  {
    REF_INT1V           = 0, // 1.0V
    REF_INTVCC_DIV_1_48 = 1, // 1/1.48 VDDANA
    REF_INTVCC_DIV_2    = 2, // 1/2 VDDANA (only for VDDANA > 2.0V)
    REF_EXTA            = 3, // external A
    REF_EXTB            = 4  // external B
  };

  enum Gain
  {
    GAIN_1    = 0,
    GAIN_2    = 1,
    GAIN_4    = 2,
    GAIN_8    = 3,
    GAIN_16   = 4,
    GAIN_DIV2 = 5
  };

public:
  struct Pin
  {
    uint8_t group; // group number (32 pins per group)
    uint8_t pin;   // pin number within group
    uint32_t port; // bit within group
  };

  static const uint8_t PIN_COUNT = 20; // MCU I/O pins with ADC/AIN capability

  static const Pin PIN_MAPPING[PIN_COUNT];

public:
  Analog2DigitalConverter(const Analog2DigitalConverter&) = delete;
  void operator=(const Analog2DigitalConverter&) = delete;

  static Analog2DigitalConverter& instance();

private:
  Analog2DigitalConverter() = default;
  ~Analog2DigitalConverter() = default;

public:
  /**
   * enable ADC module and ADC generic clock:
   * - 1.0 V Vref
   * - gain 1.0
   * - 12 bit resolution
   * - 1 sample, no averaging
   * - 2 µs sample duration
   *
   * note:
   * - ADC will not be started, use read()
   *
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param clkDiv GCLK prescaler 0..7, 2^(clkDiv + 2), ADC clock must be between 30 kHz and 2.1 MHz
   * @param runStandby keep ADC active in standby
   */
  void enable(uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv, bool runStandby = false);

  /**
   * reenable ADC module and ADC generic clock
   */
  void reenable();

  /**
   * disable ADC module and ADC generic clock
   */
  void disable();

  /**
   * configure sampling parameters
   *
   * note: ADC must be enabled
   *
   * @param sampleTime 1..64 multiple of ADC clock half periods, max. 32 ADC clock periods, use durationToHalfPeriods() to convert from [µs], ignored when reading internal temperature
   * @param averageCount 0..10 2^averageCount
   * @param freeRun if false (default) start() will initiate single conversion, otherwise continuous conversion
   */
  void setSampling(uint8_t sampleTime, uint8_t averageCount = 0, bool freeRun = false);

  /**
   * set reference voltage
   *
   * note:
   * - will be applied by start() or read()
   * - ignored when reading internal sources
   *
   * @param type reference voltage source, default REF_INT1V
   * @param voltage reference voltage [V], default 1.0 V, can also be used to scale result of read()
   * @param forceUpdate force reference update, e.g. when changed by calling analogReference()
  */
  void setReference(Reference type, float voltage, bool forceUpdate = false);

  /**
   * set input gain
   *
   * note:
   * - will be applied by start() or read()
   * - ignored when reading internal sources
   *
   * @param gain input gain, default 1.0
   */
  void setGain(Gain gain);

  /**
   * enable pin as analog input
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX_Val, use g_APinDescription[<Arduino board pin>].ulADCChannelNumber to convert from Arduino pin
   */
  void enablePin(uint8_t muxPosVal);

  /**
   * configure reference, gain and input, enable ADC and start conversion if callback is defined
   *
   * notes:
   * - single ended measurement from pin or internal source
   * - singleshot conversion
   * - use enable() to setup ADC
   * - the following internal sources are supported:
   *     ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val: internal MCU voltage (~1.2 V)
   *     ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val:   external MCU input voltage
   *     ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:       internal bandgap voltage (~1.1 V)
   *     ADC_INPUTCTRL_MUXPOS_TEMP_Val:          internal temperature [°C]
   * - use read() to retrieve conversion result from callback
   * - use stop() to disable ADC
   * - use disable() to disable ADC module
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX_Val, use g_APinDescription[<Arduino pin>].ulADCChannelNumber to convert from Arduino pin
   * @param callback function to call at ADC interrupt, optional
   */
  void start(uint8_t muxPosVal, void (*callback)() = nullptr);

  /**
   * analog read, non blocking
   *
   * notes:
   * - use to retrieve scaled conversion result when conversion is completed (e.g. ready interrupt)
   * - use start() to initiate conversion
   *
   * @return ADC scaled result [V] or [°C], also see setReference()
   */
  float read();

  /**
   * disable ADC
   */
  void stop();

  /**
   * analog read, blocking
   *
   * notes:
   * - single ended measurement from pin or internal source
   * - singleshot conversion
   * - use enable() to setup ADC
   * - the following internal sources are supported:
   *     ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val: internal MCU voltage (~1.2 V)
   *     ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val:   external MCU input voltage
   *     ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:       internal bandgap voltage (~1.1 V)
   *     ADC_INPUTCTRL_MUXPOS_TEMP_Val:          internal temperature [°C]
   * - call setReference() and setGain() after using analogReference()
   * - combines reenable(), start(), read(), stop() and disable()
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX_Val, use g_APinDescription[<Arduino pin>].ulADCChannelNumber to convert from Arduino pin
   * @return ADC scaled result [V] or [°C], also see setReference()
   */
  float read(uint8_t muxPosVal);

  /**
   * load production calibration values for temperature calculation
   *
   * see Atmel AT14164: User Calibration of Internal Temperature Sensor - SAM R21 - chapter 2.2
   */
  static void loadTemperatureCalibrationValues();

  /**
   * convert ADC voltage of internal temperature sensor [°C]
   *
   * see Atmel AT11481: ADC Configurations with Examples, chapter 12
   */
  float toTemperature(float coarseVoltage);

  /**
   * estimate Vref based on current temperature and calibration data
  */
  float toReferenceVoltage(float actualTemperature);

  /**
   * convert duration to number of ADC clock half periods
   * @param micros [µs], max. 32 periods of ADC clock
   * @return 0..64 ADC clock half periods
   */
  uint8_t durationToHalfPeriods(uint16_t micros);

public:
  /**
   * determine the position of highest set bit, equivalent to (int)log2(x)
   * @param x calling for x == 0 is not defined and will return 0
   */
  static uint8_t highestBitSet(uint8_t x);

  /**
   * ADC interrupt handler
   */
  static void isrHandler();

protected:
  static float roomTemp; // [°C]
  static float hotTemp;  // [°C]
  static float roomInt1vRef; // [V]
  static float hotInt1vRef;  // [V]
  static uint32_t roomADC;
  static uint32_t hotADC;
  static bool temperaturCalibrationValuesLoaded;

protected:
  bool runStandby = false;
  bool freeRun = true;
  uint8_t clkGenId = UCHAR_MAX;
  uint8_t clkDiv = 1;
  uint8_t sampleLength = 0;
  uint8_t muxPosVal = 0;
  uint16_t adcFullScale = 4095;
  uint32_t refSel = ADC_REFCTRL_REFSEL_INT1V;
  uint32_t lastRefSel = ULONG_MAX;
  uint32_t gain = ADC_INPUTCTRL_GAIN_1X;
  uint32_t adcFrequency;
  float refVal = 1.0; // [V]
  float refVal1V = 1.0; // [V]
  float inputScale = 1.0;
  float resultScale = 1.0;
  float effectiveResultScale = 1.0;
  void (*adcHandler)() = nullptr;
};

}
