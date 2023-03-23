/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     Analog2DigitalConverter.hpp
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
 * Arduino core does not provide a solution to read:
 * - internal voltages
 * - core temperature
 * and does not support:
 * - variable input gain
 * - variable sampling length
 * - averaging
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

  static const uint8_t PIN_COUNT = 20;

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
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param clkDiv GCLK prescaler 0..7, 2^(clkDiv + 2), ADC clock must be at or below 2.1 MHz
   */
  void enable(uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv);

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
   * note: ADC must be enabled
   *
   * @param sampleTime 1..64 multiple of ADC clock half periods, max. 32 ADC clock periods, use durationToHalfPeriods() to convert from [µs]
   * @param averageCount 0..10 2^averageCount
   */
  void setSampling(uint8_t sampleTime, uint8_t averageCount);

  /**
   * configure reference voltage
   *
   * @param type reference voltage source, default REF_INT1V
   * @param voltage reference voltage [V], default 1.0 V, can also be used to scale result of read()
   * @param forecUpdate force reference update, e.g. when changed by calling analogReference()
  */
  void setReference(Reference type, float voltage, bool forecUpdate = false);

  /**
   * configure input gain
   *
   * @param gain input gain, default 1.0
   */
  void setGain(Gain gain);

  /**
   * enable pin as analog input
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX_Val, use g_APinDescription[<Arduino pin>].ulADCChannelNumber to convert from Arduino pin
   */
  void enablePin(uint8_t muxPosVal);

  /**
   * analog read:
   * - single ended measurement or internal source, blocking
   *
   * notes:
   * - ADC must be enabled
   * - the following internal sources are supported:
   *     ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val: internal MCU voltage (~1.2 V)
   *     ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val:   external MCU input voltage
   *     ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:       internal bandgap voltage (~1.1 V)
   *     ADC_INPUTCTRL_MUXPOS_TEMP_Val:          core temperature [°C]
   * - call setReference() and setGain() after using analogReference()
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX_Val, use g_APinDescription[<Arduino pin>].ulADCChannelNumber to convert from Arduino pin
   * @return ADC scaled result [V], see setReference()
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
   * determine the position of highest set bit, equivalent to (int)log2(x)
   * @param x calling for x == 0 is not defined and will return 0
   */
  static uint8_t highestBitSet(uint8_t x);

  /**
   * covert duration to number of ADC clock half periods
   * @param micros [µs], max. 32 periods of ADC clock
   */
  uint8_t durationToHalfPeriods(uint16_t micros);

private:
  static float roomTemp; // [°C]
  static float hotTemp;  // [°C]
  static float roomInt1vRef; // [V]
  static float hotInt1vRef;  // [V]
  static uint32_t roomADC;
  static uint32_t hotADC;
  static bool temperaturCalibrationValuesLoaded;

private:
  uint8_t clkGenId = UCHAR_MAX;
  uint8_t clkDiv = 1;
  uint8_t sampleLength = 0;
  uint16_t adcFullScale = 4095;
  uint32_t refSel = ADC_REFCTRL_REFSEL_INT1V;
  uint32_t lastRefSel = ULONG_MAX;
  uint32_t gain = ADC_INPUTCTRL_GAIN_1X;
  uint32_t adcFrequency;
  float refVal = 1.0; // [V]
  float refVal1V = 1.0; // [V]
  float inputScale = 1.0;
  float resultScale = 1.0;
};

}
