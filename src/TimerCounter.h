/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     TimerCounter.h
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
 * SAMD21 timer counter for single or periodic interrupt callback (CMSIS based implementation)
 *
 * Compatiblity:
 *
 * - can be used in parallel to Arduino core analogWrite()
 * - Arduino core uses a fixed mapping of timer counter to PWM pin, see g_APinDescription in variant.h
 *   to avoid conflicts
 *
 * Background:
 *
 * Arduino core uses timer counters to provide PWM output but does not support:
 * - timer interrupts
 * - configurable generic clock
 * - idle/standby operation
 */
class TimerCounter
{
public:
  enum Prescaler
  {
    DIV1    = 0,
    DIV2    = 1,
    DIV4    = 2,
    DIV8    = 3,
    DIV16   = 4,
    DIV64   = 5,
    DIV256  = 6,
    DIV1024 = 7
  };

  enum Resolution
  {
    RES8  =  8,
    RES16 = 16,
    RES32 = 32
  };

public:
  TimerCounter() = default;
  ~TimerCounter() = default;
  TimerCounter(const TimerCounter&) = delete;
  void operator=(const TimerCounter&) = delete;

public:
  /**
   * enable TC module(s), configure and enable TC generic clock, configure TC and enable IRQ
   *
   * notes:
   * - timer will not be started automatically, use start()
   * - when using sleep mode, timer interrupt will wakeup MCU
   * - adjust clkGenFrequency and clkDiv to provide the required timing resolution and jitter
   * - for durationScale > 0 the counter value is calculated using unsigned 64 bits
   *   integer arithmetics: counterValue = clkGenFrequency/clkDiv*duration/durationScale
   * - max. execution time: t = 6/clkGenFrequency + 3/fAPB -> ~6 ms @ 1kHz
   *   disable, reenable, toClockTicks, isrHandler: 0t
   *   cancel, getElapsed: 1t
   *   restart: 1-2t
   *   enable: 2t
   *   start: 4t
   *
   * @param id timer counter ID 3..5
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param clkDiv GCLK prescaler 0..7
   * @param resolution counter resolution [bits] 8, 16, TC4: 32 by using TC5
   * @param durationScale unit/scale of duration 0: clock ticks, 1: 1 s, 1000: 1 ms (default), 1000000: 1 µs
   * @param runStandby keep timer active in standby (default: disabled)
   * @param irqPriority 0 (highest, default) .. 3 (lowest), must be 3 to use SysTick dependent operations in ISR
   */
  bool enable(uint8_t id, uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv, Resolution resolution, uint32_t durationScale = 1000U, bool runStandby = false, uint8_t irqPriority = 0);

  /**
   * reenable TC module(s) and TC generic clock
   */
  void reenable();

  /**
   * disable TC module(s) and TC generic clock
   */
  void disable();

  /**
   * convert duration to clock ticks
   * @param duration timer duration
   */
  uint32_t toClockTicks(uint32_t duration);

  /**
   * start timer
   * @param duration timer duration
   * @param periodic single if false, periodic if true
   * @param callback function to call at timer interrupt, optional
   */
  void start(uint32_t duration, bool periodic = false, void (*callback)() = nullptr);

  /**
   * cancel timer
   */
  void cancel();

  /**
   * get elapsed duration
   * @return elapsed timer duration since calling start()
   */
  uint32_t getElapsed();

  /**
   * start oneshot timer and wait for completion, blocking, using configured sleep mode, see System::setSleepMode()
   * note: will clear callback
   *
   * @param duration timer/wait duration
   */
  void wait(uint32_t duration);

  /**
   * check if timer is stopped, primarily for use with oneshot timer
   */
  bool isStopped();

  /**
   * restart oneshot timer with previous setting for duration and callback
   * @param duration timer duration, optional, otherwise use duration from last call of start() or restart()
   */
  void restart(uint32_t duration = 0);

  /**
   * timer counter interrupt handler
   */
  static void isrHandler(uint8_t id);

private:
  void sync() const;
  void setCounterRegister(uint32_t duration);

private:
  static TimerCounter* timerCounter[3];

private:
  bool periodic;
  bool interrupted = false;
  uint8_t id;
  uint8_t clkGenId = UCHAR_MAX;
  uint16_t clkDiv;          // 1..1024
  uint32_t durationScale;   // 0, 1, 1000, 1000000
  uint32_t clkGenFrequency; // [Hz]
  uint32_t duration;
  Resolution resolution;    // 8, 16, 32
  IRQn_Type irq;
  Tc* tc = nullptr;
  void (*tcHandler)() = nullptr;
};

}
