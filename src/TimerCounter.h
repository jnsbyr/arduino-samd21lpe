/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     TimerCounter.hpp
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
   * @param id timer counter ID 3..5
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param clkDiv GCLK prescaler 0..7
   * @param resolution counter resolution [bits] 8, 16, TC4: 32
   */
  bool enable(uint8_t id, uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv, Resolution resolution);

  /**
   * reenable TC module(s) and TC generic clock
   */
  void reenable();

  /**
   * disable TC module(s) and TC generic clock
   */
  void disable();

  /**
   * @param duration [ms]
   */
  void start(uint32_t duration, uint8_t periodic = false, void (*callback)() = nullptr);

  /**
   * restart oneshot timer with previous setting for duration and callback
   */
  void restart();

  static void timerHandler(uint8_t id);

private:
  static TimerCounter* timerCounter[3];

private:
  uint8_t id;
  uint8_t clkGenId = UCHAR_MAX;
  uint64_t clkGenFrequency; // [Hz]
  uint16_t clkDiv;          // 1..1024
  Resolution resolution;    // 8, 16, 32
  IRQn_Type irq;
  Tc* tc = nullptr;
  void (*tcHandler)() = nullptr;
};

}
