/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     RealTimeClock.h
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
 * SAMD21 RTC in 32 bit counter mode for periodic interrupt callback (CMSIS based implementation)
 *
 * Compatibility:
 *
 * - alternative to Arduino library RTCZero
 *
 * Background:
 *
 * Arduino library RTCZero does not support:
 * - RTC counter mode
 * - selectable generic clock
 */
class RealTimeClock
{
public:
  RealTimeClock(const RealTimeClock&) = delete;
  void operator=(const RealTimeClock&) = delete;

  static RealTimeClock& instance();

private:
  RealTimeClock() = default;
  ~RealTimeClock() = default;

public:

  /**
   * setup RTC as 32 bit counter (mode 0)
   *
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   *
   * @see RTCZero.cpp RTCZero::begin(bool)
   * @see RTCZero.cpp RTCZero::configureClock()
   */
  void enable(uint8_t clkGenId, uint32_t clkGenFrequency);

  /**
   * disable RTC module and RTC generic clock
   */
  void disable();

  /**
   * set timer period
   * @param millis [ms]
   *
   * note: only 1 second resolution available due to clock divider setup
   */
  void setTimer(uint32_t millis, void (*callback)() = nullptr);

public:
  static void (*rtcHandler)();

private:
  uint64_t clkGenFrequency = 1024; // [Hz]
  uint16_t clkDiv = 1024; // 1..1024
};

}
