/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     RealTimeClock.cpp
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

#include "RealTimeClock.h"
using namespace SAMD21LPE;


RealTimeClock& RealTimeClock::instance()
{
  static RealTimeClock rtc_;
  return rtc_;
}

void RealTimeClock::enable(uint8_t clkGenId, uint32_t clkGenFrequency)
{
  // save GCLK frequency for calculating time duration
  this->clkGenFrequency = clkGenFrequency;

  // set GCLK as source for RTC (GCLK Mux 4)
  System::enableClock(GCM_RTC, clkGenId);

  // RTC software reset
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
  while (RTC->MODE0.CTRL.bit.SWRST);

  // setup RTC for mode 0, 1 Hz, clear on match (periodic timer), no continuous read
  RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;
  RTC->MODE0.CTRL.reg = (uint16_t)(RTC_MODE0_CTRL_MODE_COUNT32 |
                                    RTC_MODE0_CTRL_PRESCALER_DIV1024 |
                                    RTC_MODE0_CTRL_MATCHCLR);
  this->clkDiv = 1024;

  // enable RTC interrupt
  NVIC_DisableIRQ(RTC_IRQn);
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_SetPriority(RTC_IRQn, 0x00);
  NVIC_EnableIRQ(RTC_IRQn);

  // enable RTC
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void RealTimeClock::disable()
{
  // RTC software reset
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
  while (RTC->MODE0.CTRL.bit.SWRST);

  // disable RTC GCM
  System::disableClock(GCM_RTC);

  // clear RTC interrupts
  NVIC_DisableIRQ(RTC_IRQn);
  NVIC_ClearPendingIRQ(RTC_IRQn);
}

void RealTimeClock::setTimer(uint32_t millis, void (*callback)())
{
  rtcHandler = callback;

  RTC->MODE0.COMP[0].reg = (uint32_t)clkGenFrequency/clkDiv*millis/1000;

  RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void (*RealTimeClock::rtcHandler)() = nullptr;


/**
 * SAMD21 RTC interrupt handler
 */
void RTC_Handler()
{
  // clear all interrupt flags
  RTC->MODE0.INTFLAG.reg = RTC->MODE0.INTENSET.reg;

  // handle interrupt
  if (SAMD21LPE::RealTimeClock::rtcHandler)
  {
    SAMD21LPE::RealTimeClock::rtcHandler();
  }
}
