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

    // enable RTC interrupt
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, min(irqPriority, (1 << __NVIC_PRIO_BITS) - 1));
  #ifndef SAMD21LPE_USE_STATIC_ISR_HANDLER
    System::getVectorTable().pfnRTC_Handler = (void*)isrHandler;
  #endif
    NVIC_EnableIRQ(RTC_IRQn);

    // setup RTC for mode 0, no continuous read, optionally clear on match (periodic timer) and enable
    RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;
    RTC->MODE0.CTRL.reg = (uint16_t)(RTC_MODE0_CTRL_MODE_COUNT32 |
                                     RTC_MODE0_CTRL_PRESCALER(clkDiv) |
                                     (clearOnTimer? RTC_MODE0_CTRL_MATCHCLR : 0) |
                                     RTC_MODE0_CTRL_ENABLE);
    sync();
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
#ifndef SAMD21LPE_USE_STATIC_ISR_HANDLER
  System::getVectorTable().pfnRTC_Handler = nullptr;
#endif
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
  sync();

  // clear and enable counter compare match interrupt
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;
  RTC->MODE0.INTENSET.reg |= RTC_MODE0_INTENSET_CMP0;
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
  sync();
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
  if (!counterAvailable)
  {
    RTC->MODE0.READREQ.reg = RTC_READREQ_RREQ | RTC_READREQ_ADDR(RTC_MODE0_COUNT_OFFSET);
    sync();
  }

  return RTC->MODE0.COUNT.reg;
}

void RealTimeClock::requestCounter(void (*callback)())
{
  counterCallback = callback;
  counterAvailable = false;
  if (callback)
  {
    RTC->MODE0.INTENSET.reg |= RTC_MODE0_INTENSET_SYNCRDY;
    RTC->MODE0.READREQ.reg = RTC_READREQ_RREQ | RTC_READREQ_ADDR(RTC_MODE0_COUNT_OFFSET);
  }
}

void RealTimeClock::sync() const
{
  while (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void RealTimeClock::isrHandler()
{
  RealTimeClock& rtc = RealTimeClock::instance();

  // clear all interrupt flags
  uint8_t intFlag = RTC->MODE0.INTFLAG.reg;
  RTC->MODE0.INTFLAG.reg = RTC->MODE0.INTENSET.reg;

  if ((intFlag & RTC_MODE0_INTENSET_SYNCRDY) && rtc.counterCallback)
  {
    // handle counter read interrupt
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_SYNCRDY;
    rtc.counterAvailable = true;
    rtc.counterCallback();
    rtc.counterCallback = nullptr;
    rtc.counterAvailable = false;
  }

  if (intFlag & RTC_MODE0_INTENSET_CMP0)
  {
    // handle timer interrupt
    if (!rtc.periodic)
    {
      // not periodic, disable counter compare match interrupt
      RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
    }
    else if (!rtc.clearOnTimer)
    {
      // periodic and no clear on timer, update counter compare register
      rtc.lastCounter += rtc.durationTicks;
      RTC->MODE0.COMP[0].reg = rtc.lastCounter;
      // skipping sync reduces ISR latency
    }

    // handle interrupt
    if (rtc.rtcHandler)
    {
      rtc.rtcHandler();
    }
  }
}

#ifdef SAMD21LPE_USE_STATIC_ISR_HANDLER
/**
 * SAMD21 RTC interrupt handler
 */
void RTC_Handler()
{
  RealTimeClock::instance().isrHandler();
}
#endif
