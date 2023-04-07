/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     System.h
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

#include <Arduino.h>  // Arduino LLC., LGPL 2.1
#include <sam.h>      // Atmel Corporation, BSD 3-Clause

namespace SAMD21LPE
{

/**
 * SAMD21 generic clock and sleep mode utilities (CMSIS based implementation)
 *
 * Compatiblity:
 *
 * - can be used in parallel to Arduino core
 * - tune() will disable several modules that are enabled by the Arduino core
 *   incl. USB and ADC - they have to be reenabled explicitly when needed
 *
 * Background:
 *
 * Arduino core does not provide public functions to handle:
 * - generic clock generators and generic clocks
 * - SysTick
 * and the ArduinoLowPowerClass requires an USBDevice.
 */
class System
{
public:
  enum SleepMode
  {
    IDLE0 = 0,
    IDLE1 = 1,
    IDLE2 = 2,
    STANDBY = 3
  };

public:
  /**
   * setup a generic clock generator with given divider
   * with OSCULP32K as source and RUNSTDBY option

   * @param div divide OSC frequency by 2^(div + 1), -1 disables divider
   */
  static void setupClockGenOSCULP32K(uint8_t genId, int8_t div = -1);

  /**
   * setup a generic clock generator with given divider
   * with OSC8M as source with ONDEMAND option
   *
   * @param div divide OSC frequency by 2^(div + 1), -1 disables divider
   * @param runStandby keep generator active in standby
   */
  static void setupClockGenOSC8M(uint8_t genId, int8_t div = -1, bool runStandby = false);

  /**
   * disable a generic clock generator
   */
  static void disableClockGen(uint8_t genId);

  /**
   * enable generic clock by connecting it to a generic clock generator
   *
   * notes:
   * - also enables module via PM for some generic clocks (ADC, EIC, RTC, USB)
   *   including APB B or APB C bus
   *
   */
  static void enableClock(uint8_t clkId, uint8_t clkGenId);

  /**
   * disable generic clock by connecting it to disabled generic clock generator 8
   *
   * notes:
   * - also disables module via PM for some generic clocks (ADC, EIC, RTC, USB)
   * - also disables APB B and APB C bus via PM if no module remains enabled on the bus
   *
   */
  static void disableClock(uint8_t clkId);

  /**
   * @return true if clock is enabled
   */
  static bool isClockEnabled(uint8_t clkId);

  /**
   * startup power optimization:
   * - disable bus clocks of non essential modules
   * - use OSC8M instead of XOSC32K/DFLL48M depending on F_CPU setting
   * - disable unused generic clocks generators
   * - disable unused generic clocks
   * - adjust NVM read wait states
   */
  static void reducePowerConsumption();

  /**
   * enable SysTick, call in ISR after wakeup from standby
   */
  static void enableSysTick()
  {
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  }

  /**
   * disable SysTick, call in ISR before returning to standby
   */
  static void disableSysTick()
  {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  }

  /**
   * enable bus clock for PORT module
   */
  static void enablePORT();

  /**
   * enable bus clock for EIC module
   *
   * note: must be called before call to attachInterrupt() after calling reducePowerConsumption()
   */
  static void enableEIC();

  /**
   * configure sleep mode
   *
   * @param mode 0..2 idle, 3 deep sleep
   */
  static void setSleepMode(SleepMode mode);

  /**
   * put MCU to sleep until wakeup interrupt, use setSleepMode() to configure sleep mode
   */
  static void sleep();

  /**
   * put MCU to sleep until wakeup interrupt
   *
   * @param mode 0..2 idle, 3 deep sleep
   *
   * @see ArduinoLowPower.cpp ArduinoLowPowerClass::idle()
   */
  static void sleep(SleepMode mode);

  /**
   * put MCU to sleep when returning from ISR
   */
  static void setSleepOnExitISR(bool on);

};

}
