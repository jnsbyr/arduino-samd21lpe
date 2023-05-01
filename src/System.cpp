/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     System.cpp
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

#include "System.h"
using namespace SAMD21LPE;

uint32_t System::getSerialNumber(uint8_t index)
{
  switch (index)
  {
    case 0:  return *(uint32_t*)0x0080A00C;
    case 1:  return *(uint32_t*)0x0080A040;
    case 2:  return *(uint32_t*)0x0080A044;
    case 3:  return *(uint32_t*)0x0080A048;
    default: return 0;
  }
}

void System::setupClockGenOSCULP32K(uint8_t genId, int8_t div)
{
  // setup GCLK divider
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(genId)|
                      (div>=0? GCLK_GENDIV_DIV(div) : 0);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // set OSCULP32K as source for GCLK
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      (div>=0? GCLK_GENCTRL_DIVSEL : 0) |
                      (div>=0? GCLK_GENCTRL_IDC : 0) |
                      GCLK_GENCTRL_RUNSTDBY;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void System::setupClockGenOSC8M(uint8_t genId, int8_t div, bool runStandby)
{
  // ensure OSC8M is enabled and running at 8 MHz
  SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_PRESC_Msk;
  if (runStandby)
  {
    SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_RUNSTDBY;
  }
  else
  {
    SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_RUNSTDBY;
  }
  SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE | SYSCTRL_OSC8M_ONDEMAND;

  // setup GCLK divider
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(genId) |
                      (div>=0? GCLK_GENDIV_DIV(div) : 0);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // set OSC8M as source for GCLK
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSC8M |
                      (div>=0? GCLK_GENCTRL_DIVSEL : 0) |
                      (div>=0? GCLK_GENCTRL_IDC : 0) |
                      (runStandby? GCLK_GENCTRL_RUNSTDBY : 0);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void System::disableClockGen(uint8_t genId)
{
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void System::enableClock(uint8_t clkId, uint8_t clkGenId)
{
  // set GCLK as source for TC
  GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(clkId);
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(clkId) |
                                  GCLK_CLKCTRL_CLKEN |
                                  GCLK_CLKCTRL_GEN(clkGenId));
  while (GCLK->STATUS.bit.SYNCBUSY);

  // enable module and B/C bus
  switch (clkId)
  {
    // APB A
    case GCM_RTC:
      PM->APBAMASK.reg |= PM_APBAMASK_RTC;
      break;

    case GCM_EIC:
      enableEIC();
      break;

    // APB B
    case GCM_USB:
      PM->AHBMASK.reg |= PM_AHBMASK_USB | PM_AHBMASK_HPB1;
      PM->APBBMASK.reg |= PM_APBBMASK_USB;
      break;

    // APB C
    case GCM_TCC2_TC3:
    case GCM_TC4_TC5:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      break;

    case GCM_ADC:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_ADC;
      break;

    case GCM_SERCOM0_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
      break;
    case GCM_SERCOM1_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;
      break;
    case GCM_SERCOM2_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;
      break;
    case GCM_SERCOM3_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;
      break;
    case GCM_SERCOM4_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;
      break;
    case GCM_SERCOM5_CORE:
      PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
      break;
  }
}

void System::disableClock(uint8_t clkId)
{
  // disable module and B/C bus
  switch (clkId)
  {
    // APB A
    case GCM_RTC:
      PM->APBAMASK.reg &= ~PM_APBAMASK_RTC;
      break;

    case GCM_EIC:
      PM->APBAMASK.reg &= ~PM_APBAMASK_EIC;
      break;

    // AHB + APB B
    case GCM_USB:
      PM->APBBMASK.reg &= ~PM_APBBMASK_USB;
      PM->AHBMASK.reg &= ~PM_AHBMASK_USB;
      if (!PM->APBBMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB1;
      }
      break;

    // APB C
    case GCM_TCC2_TC3:
    case GCM_TC4_TC5:
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;

    // APB C
    case GCM_ADC:
      PM->APBCMASK.reg |= PM_APBCMASK_ADC;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;

    case GCM_SERCOM0_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
    case GCM_SERCOM1_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
    case GCM_SERCOM2_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
    case GCM_SERCOM3_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
    case GCM_SERCOM4_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
    case GCM_SERCOM5_CORE:
      PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
      if (!PM->APBCMASK.reg)
      {
        PM->AHBMASK.reg &= ~PM_AHBMASK_HPB2;
      }
      break;
  }

  // connect to disabled generic clock generator 8
  GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(clkId);
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(clkId) |
                                  GCLK_CLKCTRL_GEN(GCLK_GEN_NUM_MSB));
  while (GCLK->STATUS.bit.SYNCBUSY);
}

bool System::isClockEnabled(uint8_t clkId)
{
  uint8_t* id = (uint8_t*)&GCLK->CLKCTRL;
  *id = GCLK_CLKCTRL_ID(clkId);
  return GCLK->CLKCTRL.bit.CLKEN;
}


void System::reducePowerConsumption()
{
  // read core voltage
  //float vCore = readADC(ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC);

  // essential clocks: AHB: NVMCTRL, HPB0, APBA: SYSCTRL, PM, GCLK
  // additional clocks: APBA: RTC
  // disable all other clocks that are enabled at reset
  // @see product datasheet table 16-1 - Peripheral Clock Default State
  // @see product datasheet chapter 37.7 - Power Consumption
  PM->AHBMASK.reg = PM_AHBMASK_NVMCTRL | PM_AHBMASK_HPB0;
  PM->APBAMASK.reg = PM_APBAMASK_SYSCTRL | PM_APBAMASK_PM | PM_APBAMASK_GCLK | PM_APBAMASK_RTC;
  PM->APBBMASK.reg = 0;
  PM->APBCMASK.reg = 0;

  // set bus clocks
  if (SystemCoreClock <= 12000000L)
  {
    // lower CPU clock frequency, bus clocks undivided (to reduce sync overhead)
    PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1;
    PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1;
    PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1;
  }
  else
  {
    // higher CPU clock frequency, reduce bus clocks (to reduce consumption)
    PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV4;
    PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV128;
    PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV128;
  }

  // optimize use of oscillators
  if (SystemCoreClock == 8000000L || SystemCoreClock == 4000000L || SystemCoreClock == 2000000L || SystemCoreClock == 1000000L)
  {
    // CPU clock frequency less equal 8 MHz or divided by the power of 2, switch GCLKGEN0 from XOSC32K/DFLL48M to OSC8M oscillator
    setupClockGenOSC8M(0, 8000000L/SystemCoreClock - 2);
    // disable GCLKGEN1 and DFLL48M
    disableClockGen(1);
    disableClock(GCLK_CLKCTRL_ID_DFLL48_Val);
    SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
    // disable XOSC32K
    SYSCTRL->XOSC32K.bit.ENABLE = 0;
  }
  else
  {
    // although unused, do not disable OSC8M (would increase power consumption)
  }

  // disable GCLKGEN2 and higher
  for (size_t genId = 2; genId < GCLK_GEN_NUM; genId++)
  {
    disableClockGen(genId);
  }

  // disable most GCLKs (will reduce supply current by ~50 %)
  for (size_t clkId = GCLK_CLKCTRL_ID_FDPLL_Val; clkId < GCLK_NUM; clkId++)
  {
    if (clkId != GCLK_CLKCTRL_ID_RTC_Val)
    {
      disableClock(clkId);
    }
  }

  // reduce NVM wait states (or Vdd >= 2.7 V, F_CPU <= 24000000L)
  // @see product datasheet chapter 37.12 - NVM Characteristics
  if (SystemCoreClock <= 14000000L)
  {
    NVMCTRL->CTRLB.bit.RWS = 0;
  }
}

void System::enableEIC()
{
  PM->APBAMASK.reg |= PM_APBAMASK_EIC;
}

void System::enableDMAC()
{
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC | PM_AHBMASK_HPB1;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
}

void System::enablePORT()
{
  PM->AHBMASK.reg |= PM_AHBMASK_HPB1;
  PM->APBBMASK.reg |= PM_APBBMASK_PORT;
}

void System::setSleepMode(SleepMode mode)
{
  if (mode < STANDBY)
  {
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    PM->SLEEP.reg = mode;
  }
  else
  {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  }

  // DSB required when called as last operation in ISR
  __DSB();
}

void System::sleep()
{
  bool standby = SCB->SCR & SCB_SCR_SLEEPDEEP_Msk;

  // disable systick interrupt
  if (standby)
  {
    disableSysTick();
  }

  // wait for interrupt
  __DSB();
  __WFI();

  // reenable systick interrupt
  if (standby)
  {
    enableSysTick();
  }
}

void System::sleep(SleepMode mode)
{
  // disable systick interrupt
  if (mode == STANDBY)
  {
    disableSysTick();
  }

  // wait for interrupt
  setSleepMode(mode);
  __WFI();

  // reenable systick interrupt
  if (mode == STANDBY)
  {
    enableSysTick();
  }
}

void System::setSleepOnExitISR(bool on)
{
  if (on)
  {
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
  }
  else
  {
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
  }
}
