/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     TimerCounter.cpp
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

#include "TimerCounter.h"
using namespace SAMD21LPE;


bool TimerCounter::enable(uint8_t id, uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv, Resolution resolution, bool runStandby)
{
  if (id >= 3 && id <= 5 && (resolution == RES8 || resolution == RES16 || (id == 4 && resolution == RES32)))
  {
    this->id = id;
    this->clkGenId = clkGenId;
    this->clkGenFrequency = clkGenFrequency;
    this->resolution = resolution;

    switch (clkDiv)
    {
      case DIV2:
        this->clkDiv = 2;
        break;
      case DIV4:
        this->clkDiv = 4;
        break;
      case DIV8:
        this->clkDiv = 8;
        break;
      case DIV16:
        this->clkDiv = 16;
        break;
      case DIV64:
        this->clkDiv = 64;
        break;
      case DIV256:
        this->clkDiv = 256;
        break;
      case DIV1024:
        this->clkDiv = 1024;
        break;
      default:
        this->clkDiv = 1;
    }

    uint8_t modeCount;
    switch (resolution)
    {
      case RES8:
        modeCount = TC_CTRLA_MODE_COUNT8;
        break;
      case RES32:
        modeCount = TC_CTRLA_MODE_COUNT32;
        break;
      default:
        modeCount = TC_CTRLA_MODE_COUNT16;
    }

    // enable TC module(s) and TC generic clock
    reenable();

    // get TC CMSIS instance and TC interrupt number
    switch (id)
    {
      case 3:
        tc = TC3;
        irq = TC3_IRQn;
        break;

      case 4:
        tc = TC4;
        irq = TC4_IRQn;
        break;

      case 5:
        tc = TC5;
        irq = TC5_IRQn;
        break;
    }

    // TC software reset
    tc->COUNT8.CTRLA.reg |= TC_CTRLA_SWRST;
    while (tc->COUNT8.CTRLA.bit.SWRST);

    // configure TC
    tc->COUNT8.CTRLA.reg = modeCount |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER(clkDiv) |
                           (runStandby? TC_CTRLA_RUNSTDBY : 0);
    while (tc->COUNT8.STATUS.bit.SYNCBUSY);

    // setup IRQ
    NVIC_DisableIRQ(irq);
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, 0x00);
    NVIC_EnableIRQ(irq);

    // enable overflow interrupt (needed for oneshot mode)
    tc->COUNT8.INTENSET.reg = TC_INTENSET_OVF;
    //tc->COUNT8.INTENSET.reg = TC_INTENSET_MC0;

    // register instance
    timerCounter[id - 3] = this;

    return true;
  }
  else
  {
    // invalid parameters
    return false;
  }
}

void TimerCounter::reenable()
{
  if (clkGenId != UCHAR_MAX)
  {
    switch (id)
    {
      case 3:
        System::enableClock(GCM_TCC2_TC3, clkGenId);
        PM->APBCMASK.reg |= PM_APBCMASK_TC3;
        break;

      case 4:
        System::enableClock(GCM_TC4_TC5, clkGenId);
        PM->APBCMASK.reg |= PM_APBCMASK_TC4;
        if (resolution == 32)
        {
          // also enable TC5
          PM->APBCMASK.reg |= PM_APBCMASK_TC5;
        }
        break;

      case 5:
        System::enableClock(GCM_TC4_TC5, clkGenId);
        PM->APBCMASK.reg |= PM_APBCMASK_TC5;
        break;
    }
  }
}

void TimerCounter::disable()
{
  switch (id)
  {
    case 3:
      PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
      System::disableClock(GCM_TCC2_TC3);
      break;

    case 4:
      PM->APBCMASK.reg &= ~PM_APBCMASK_TC4;
      if (resolution == 32)
      {
        // also disable TC5
        PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;
      }
      System::disableClock(GCM_TC4_TC5);
      break;

    case 5:
      PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;
      System::disableClock(GCM_TC4_TC5);
      break;
  }
}

void TimerCounter::start(uint32_t duration, bool periodic, void (*callback)())
{
  if (tc)
  {
    tcHandler = callback;

    // disable counter
    tc->COUNT8.CTRLA.bit.ENABLE = 0;
    while (tc->COUNT8.STATUS.bit.SYNCBUSY);

    // set counter period value
    uint64_t count = clkGenFrequency/clkDiv*duration/1000;
    switch (resolution)
    {
      case RES8:
        if (count > UCHAR_MAX) count = UCHAR_MAX;
        tc->COUNT8.CC[0].reg = (uint8_t)count;
        tc->COUNT8.COUNT.reg = 0;
        break;

      case RES32:
        if (count > ULONG_MAX) count = ULONG_MAX;
        tc->COUNT32.CC[0].reg = (uint32_t)count;
        tc->COUNT32.COUNT.reg = 0;
        break;

      default:
        if (count > USHRT_MAX) count = USHRT_MAX;
        tc->COUNT16.CC[0].reg = (uint16_t)count;
        tc->COUNT16.COUNT.reg = 0;
    }

    // set one-shot mode
    tc->COUNT8.CTRLBSET.bit.ONESHOT = periodic? 0 : 1;
    while (tc->COUNT8.STATUS.bit.SYNCBUSY);

    // enable counter
    tc->COUNT8.CTRLA.bit.ENABLE = 1;
    while (tc->COUNT8.STATUS.bit.SYNCBUSY);
  }
}

void TimerCounter::wait(uint32_t duration)
{
  interrupted = false;
  start(duration);
  while (!interrupted) System::sleep();
}

bool TimerCounter::isStopped()
{
  if (tc)
  {
    return tc->COUNT8.STATUS.bit.STOP;
  }
  else
  {
    return true;
  }
}

void TimerCounter::restart()
{
  if (tc && tc->COUNT8.CTRLBSET.bit.ONESHOT)
  {
    tc->COUNT8.CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;
    while (tc->COUNT8.STATUS.bit.SYNCBUSY);
  }
}

void TimerCounter::isrHandler(uint8_t id)
{
  if (id < 3)
  {
    TimerCounter* timterCounter = timerCounter[id];
    if (timterCounter)
    {
      // clear all interrupt flags
      timterCounter->tc->COUNT8.INTFLAG.reg = timterCounter->tc->COUNT8.INTENSET.reg;
      timterCounter->interrupted = true;

      // handle interrupt
      if (timterCounter->tcHandler)
      {
        timterCounter->tcHandler();
      }
    }
  }
}

TimerCounter* TimerCounter::timerCounter[3] = { nullptr };

/**
 * SAMD21 TC3 interrupt handler
 */
void TC3_Handler()
{
  SAMD21LPE::TimerCounter::isrHandler(0);
}

/**
 * SAMD21 TC4 interrupt handler
 */
void TC4_Handler()
{
  SAMD21LPE::TimerCounter::isrHandler(1);
}

/**
 * SAMD21 TC5 interrupt handler
 */
void TC5_Handler()
{
  SAMD21LPE::TimerCounter::isrHandler(2);
}
