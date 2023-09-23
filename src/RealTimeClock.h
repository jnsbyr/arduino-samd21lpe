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
   * setup RTC as 32 bit counter (mode 0) and start counter
   * execution time: 2t
   *
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param clkDiv GCLK prescaler 2^clkDiv 0..10
   * @param durationScale unit/scale of duration 0: RTC clock ticks, 1: 1 s, 1000: 1 ms (default), 1000000: 1 µs
   * @param clearOnTimer zero counter when timer expires, default disabled
   * @param irqPriority 0 (highest) .. 3 (lowest, default), must be 3 to use SysTick dependent operations in ISR
   *
   * notes:
   * - timer resolution depends on clock frequency and divider setup: 2^clkDiv/clkGenFrequency
   * - counter will be zeroed and started
   * - counter will overflow with clearOnTimer = false or will be zeroed periodically with clearOnTimer = true
   * - with a clock frequency of 1 kHz the counter will overflow after 49.7 days
   * - adjust clkGenFrequency and clkDiv to provide the required timing resolution and jitter
   * - for durationScale > 0 the raw counter value is calculated using unsigned 64 bits
   *   integer arithmetics: counterValue = clkGenFrequency/clkDiv*duration/durationScale
   * - max. execution time: t = 6/clkGenFrequency + 3/fAPB -> ~6 ms @ 1kHz
   */
  void enable(uint8_t clkGenId, uint32_t clkGenFrequency, uint8_t clkDiv = 10, uint32_t durationScale = 1000U, bool clearOnTimer = false, uint8_t irqPriority = 3);

  /**
   * disable RTC module and RTC generic clock (will stop counter)
   * execution time: 1t
   */
  void disable();

  /**
   * convert duration to clock ticks
   * @param duration timer duration
   */
  uint32_t toClockTicks(uint32_t duration) const;

  /**
   * start timer
   * execution time: 1t in clearOnTimer mode, otherwise 2t
   * @param duration timer duration
   * @param periodic single if false, periodic if true
   * @param callback function to call at timer interrupt, optional
   */
  void start(uint32_t duration, bool periodic = false, void (*callback)() = nullptr);

  /**
   * cancel timer (counter will continue)
   */
  void cancel();

  /**
   * set scaled counter value
   * execution time: 1t
   * @param duration counter value
   */
  void setElapsed(uint32_t duration);

  /**
   * get scaled counter value
   * execution time: 0t in requestCounter() callback, otherwise 1t
   * @return elapsed duration
   */
  uint32_t getElapsed() const;

  /**
   * get raw counter value
   * execution time: 0t in requestCounter() callback, otherwise 1t
   * @return elapsed duration
   */
  uint32_t getCounter() const;

  /**
   * request counter value
   *
   * note: use getCounter() or getElapsed() to retrieve counter value in callback
   */
  void requestCounter(void (*callback)());

  /**
   * wait until RTC read or write sync is completed
   * execution time: 1t
   *
   * note: sync is an integrated part of all RTC operations except the timer ISR handler
   */
  void sync() const;

  /**
   * RTC interrupt handler
   *
   * note: call sync() with configuration "periodic & !clearOnTimer" before accessing RTC to prevent peripheral bus stall
   */
  void isrHandler();

protected:
  uint16_t clkDiv = 0;          // 1..1024
  uint32_t clkGenFrequency = 0; // [Hz]
  uint32_t durationScale = 0;   // 0, 1, 1000, 1000000
  uint32_t durationTicks = 0;
  uint32_t lastCounter = 0;
  bool clearOnTimer = false;
  bool periodic = false;
  bool counterAvailable = false;
  void (*rtcHandler)() = nullptr;
  void (*counterCallback)() = nullptr;
};

}
