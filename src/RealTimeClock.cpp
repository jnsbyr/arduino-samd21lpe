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

void RealTimeClock::enable(uint8_t clkGenId, uint32_t clkGenFrequency, uint8_t clkDiv, uint32_t durationScale, bool clearOnTimer, uint8_t irqPriority)
{
  if (clkDiv <= 10)
  {
    // save GCLK frequency for calculating time duration
    this->clkGenFrequency = clkGenFrequency;
    this->clkDiv = 1<<clkDiv;
    this->durationScale = durationScale;
    this->clearOnTimer = clearOnTimer;

    // set GCLK as source for RTC
    System::enableClock(GCM_RTC, clkGenId);

    // RTC software reset
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.CTRL.bit.SWRST);

    // setup RTC for mode 0, no continuous read, optionally clear on match (periodic timer)
    RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;
    RTC->MODE0.CTRL.reg = (uint16_t)(RTC_MODE0_CTRL_MODE_COUNT32 |
                                     RTC_MODE0_CTRL_PRESCALER(clkDiv) |
                                     (clearOnTimer? RTC_MODE0_CTRL_MATCHCLR : 0));

    // enable RTC interrupt
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, min(irqPriority, (1 << __NVIC_PRIO_BITS) - 1));
    NVIC_EnableIRQ(RTC_IRQn);

    // enable RTC
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }
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

uint32_t RealTimeClock::toClockTicks(uint32_t duration) const
{
  uint64_t count = durationScale? max((uint64_t)clkGenFrequency/clkDiv*duration/durationScale, 1U) : duration;
  if (count >= ULONG_MAX)
  {
    return ULONG_MAX;
  }
  else
  {
    return count;
  }
}

void RealTimeClock::start(uint32_t duration, bool periodic, void (*callback)())
{
  this->periodic = periodic;
  rtcHandler = callback;

  // set counter compare value
  durationTicks = toClockTicks(duration);
  lastCounter = durationTicks + (clearOnTimer? 0 : getCounter());
  RTC->MODE0.COMP[0].reg = lastCounter;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);

  // clear and enable counter compare match interrupt
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;
  RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
}

void RealTimeClock::cancel()
{
  RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
  rtcHandler = nullptr;
}

void RealTimeClock::setElapsed(uint32_t duration)
{
  // set counter value
  RTC->MODE0.COUNT.reg = toClockTicks(duration);
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

uint32_t RealTimeClock::getElapsed() const
{
  // request read of counter register
  uint64_t result = getCounter();

  // convert counter value to duration
  result = durationScale? result*durationScale*clkDiv/clkGenFrequency : result;

  return result < ULONG_MAX? result : ULONG_MAX;
}

uint32_t RealTimeClock::getCounter() const
{
  // request read of counter register
  RTC->MODE0.READREQ.reg = RTC_READREQ_RREQ | RTC_READREQ_ADDR(RTC_MODE0_COUNT_OFFSET);
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);

  return RTC->MODE0.COUNT.reg;
}

void RealTimeClock::isrHandler()
{
  // clear all interrupt flags
  RTC->MODE0.INTFLAG.reg = RTC->MODE0.INTENSET.reg;

  if (!periodic)
  {
    // not periodic, disable counter compare match interrupt
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
  }
  else if (!clearOnTimer)
  {
    // periodic and no clear on timer, update counter compare register
    lastCounter += durationTicks;
    RTC->MODE0.COMP[0].reg = lastCounter;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }

  // handle interrupt
  if (rtcHandler)
  {
    rtcHandler();
  }
}

/**
 * SAMD21 RTC interrupt handler
 */
void RTC_Handler()
{
  RealTimeClock::instance().isrHandler();
}
