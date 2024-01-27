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

// comment in to use ISR handlers statically linked instead of dynamically assigned (saves 200..500 bytes SRAM)
//#define SAMD21LPE_USE_STATIC_ISR_HANDLER

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
    IDLE0   = 0, //  4.1 µs wakeup, stopped: CPU
    IDLE1   = 1, // 16.4 µs wakeup, stopped: CPU, AHB (USB, DMAC, NVMCTRL, DSU, HPB0-2)
    IDLE2   = 2, // 17.2 µs wakeup, stopped: CPU, AHB (USB, DMAC, NVMCTRL, DSU, HPB0-2), APB (all other peripheral)
    STANDBY = 3  // 22.1 µs wakeup, stopped: CPU, AHB, APB, oscillators (max. startup times: OSC8M 3.3 µs, DFLL48M 9 µs, OSC32K 61 µs, XOSC32K 916 ms)
  };

public:
  /**
   * read SAMD21 128 bit serial number
   *
   * @param index word index, 0..3
   * @return 32 bit of serial number
  */
  static uint32_t getSerialNumber(uint8_t index);

  /**
   * setup a generic clock generator with given divider
   * with OSCULP32K as source and RUNSTDBY option
   *
   * @param genId 0 .. 8 (GCLK_CLKCTRL_GEN_GCLKn_Val or GENERIC_CLOCK_GENERATOR_XXXX)
   * @param div divide OSC frequency by 2^(div + 1), -1 disables divider
   */
  static void setupClockGenOSCULP32K(uint8_t genId, int8_t div = -1);

  /**
   * setup a generic clock generator with given divider
   * with OSC8M as source with ONDEMAND option
   *
   * @param genId 0 .. 8 (GCLK_CLKCTRL_GEN_GCLKn_Val or GENERIC_CLOCK_GENERATOR_XXXX)
   * @param div divide OSC frequency by 2^(div + 1), -1 disables divider
   * @param runStandby keep generator active in standby
   */
  static void setupClockGenOSC8M(uint8_t genId, int8_t div = -1, bool runStandby = false);

  /**
   * disable a generic clock generator
   *
   * @param genId 0 .. 8 (GCLK_CLKCTRL_GEN_GCLKn_Val or GENERIC_CLOCK_GENERATOR_XXXX)
   */
  static void disableClockGen(uint8_t genId);

  /**
   * enable generic clock by connecting it to a generic clock generator
   *
   * notes:
   * - also enables module via PM for some generic clocks (ADC, EIC, RTC, SERCOM, TCC2, TC3-5, USB)
   *   including APB B or APB C bus
   * - for TCC2 and TC3-5 PM->APBBMASK must be set additionally because timer clock is not timer specific
   * - when using SPI enableDMAc() must be called additionally
   * - SDK claims the following generic clock generator (see startup.c):
   *   - 0 GENERIC_CLOCK_GENERATOR_MAIN/GENERIC_CLOCK_MULTIPLEXER_DFLL48M
   *   - 1 GENERIC_CLOCK_GENERATOR_XOSC32K/GENERIC_CLOCK_GENERATOR_OSC32K
   *   - 2 GENERIC_CLOCK_GENERATOR_OSCULP32K
   *   - 3 GENERIC_CLOCK_GENERATOR_OSC8M
   * - generic clock generator 8 reserved as target for unused generic clocks
   *
   * @param genId 0 .. 8 (GCLK_CLKCTRL_GEN_GCLKn_Val or GENERIC_CLOCK_GENERATOR_XXXX)
   * @param clkId 0 .. 25 (GCLK_CLKCTRL_ID_XXXX or GCM_XXXX)
   */
  static void enableClock(uint8_t clkId, uint8_t clkGenId);

  /**
   * disable generic clock by connecting it to disabled generic clock generator 8
   *
   * notes:
   * - also disables module via PM for some generic clocks (ADC, EIC, RTC, SERCOM, TCC2, TC3-5, USB)
   * - also disables APB B and APB C bus via PM if no module remains enabled on the bus
   *
   * @param clkId 0 .. 25 (GCLK_CLKCTRL_ID_XXXX or GCM_XXXX)
   */
  static void disableClock(uint8_t clkId);

  /**
   * @param clkId 0 .. 25 (GCLK_CLKCTRL_ID_XXXX or GCM_XXXX)
   * @return true if clock is enabled
   */
  static bool isClockEnabled(uint8_t clkId);

  /**
   * startup power optimization:
   * - disable bus clocks of non essential modules
   * - use OSC8M instead of XOSC32K/DFLL48M depending on F_CPU setting
   * - disable unused generic clocks generators 2 .. 8
   * - disable unused generic clocks 1 .. 36 except RTC
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
   * enable bus clock for EIC module
   *
   * note: must be called before call to attachInterrupt() after calling reducePowerConsumption()
   */
  static void enableEIC();

  /**
   * enable bus clock for DMAC module
   */
  static void enableDMAC();

  /**
   * enable bus clock for PORT module
   */
  static void enablePORT();

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

#ifndef SAMD21LPE_USE_STATIC_ISR_HANDLER
  /**
   * copy interrupt vector table from flash to RAM
   * notes:
   * - allows changed ISR handlers at runtime
   * - slightly reduces ISR latency
   */
  static void cacheVectorTable();
#endif

  /**
   * allows read and write access to interrupt vector table
   * note:
   * - will cache interrupt vector table in RAM if SAMD21LPE_USE_STATIC_ISR_HANDLER is not enabled
   * - write access not availabe if SAMD21LPE_USE_STATIC_ISR_HANDLER is enabled
   */
  static DeviceVectors& getVectorTable();

};

}
