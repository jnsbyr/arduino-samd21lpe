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
   * @param clkDiv GCLK prescaler 2^clkDiv 0..10
   * @param durationScale unit/scale of duration 0: clock ticks, 1: 1 s, 1000: 1 ms (default), 1000000: 1 µs
   *
   * notes:
   * - timer resolution depends on clock frequency and divider setup: 2^clkDiv/clkGenFrequency
   * - counter will be zeroed and started
   * - counter is periodic
   * - adjust clkGenFrequency and clkDiv to provide the required timing resolution and jitter
   * - for durationScale > 0 the counter value is calculated using unsigned 64 bits
   *   integer arithmetics: counterValue = clkGenFrequency/clkDiv*duration/durationScale
   */
  void enable(uint8_t clkGenId, uint32_t clkGenFrequency, uint8_t clkDiv = 10, uint32_t durationScale = 1000U);

  /**
   * disable RTC module and RTC generic clock
   */
  void disable();

  /**
   * convert duration to clock ticks
   * @param duration timer duration
   */
  uint32_t toClockTicks(uint32_t duration);

  /**
   * set timer period
   * @param duration timer period
   */
  void setTimer(uint32_t duration, void (*callback)() = nullptr);

  /**
   * set counter value
   * @param duration counter value
   */
  void setElapsed(uint32_t duration);

  /**
   * get counter value
   * @return elapsed duration
   */
  uint32_t getElapsed();

public:
  static void (*rtcHandler)();

private:
  uint16_t clkDiv = 0;          // 1..1024
  uint32_t clkGenFrequency = 0; // [Hz]
  uint32_t durationScale;       // 0, 1, 1000, 1000000
};

}
