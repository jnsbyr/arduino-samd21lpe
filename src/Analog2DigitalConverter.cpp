/*****************************************************************************
 *
 * SAMD21 Low Power Extensions
 *
 * file:     Analog2DigitalConverter.cpp
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

#include "Analog2DigitalConverter.h"
using namespace SAMD21LPE;

#include <climits>


Analog2DigitalConverter& Analog2DigitalConverter::instance()
{
  static Analog2DigitalConverter a2dc_;
  return a2dc_;
}

void Analog2DigitalConverter::enable(uint8_t clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv)
{
  this->clkGenId = clkGenId;
  System::enableClock(GCM_ADC, clkGenId);

  /* setup ADC clock to 0.75 .. 1 MHz
  // determine clock divider to make ADC clock run around 1 MHz ()
  uint8_t optClkDiv = clkGenFrequency/1000000UL;
  uint8_t intLog2 = highestBitSet(optClkDiv);
  if (optClkDiv != 1 << intLog2)
  {
    // round up to get higher divider value / lower ADC frequency
    intLog2++;
  }
  this->clkDiv = intLog2 >= 9 ? 7 : intLog2 > 2? intLog2 - 2 : 0; // min. div is 4
  */

  this->clkDiv = clkDiv;
  this->adcFrequency = clkGenFrequency/(1<<(clkDiv + 2));

  setReference(REF_INT1V, 1.0f);
  setGain(GAIN_1);
  setSampling(durationToHalfPeriods(2), 0);
}

void Analog2DigitalConverter::reenable()
{
  if (clkGenId != UCHAR_MAX)
  {
    System::enableClock(GCM_ADC, clkGenId);
  }
}

void Analog2DigitalConverter::disable()
{
  System::disableClock(GCM_ADC);
}

void Analog2DigitalConverter::setSampling(uint8_t sampleTime, uint8_t averageCount)
{
  this->sampleLength = ADC_SAMPCTRL_SAMPLEN(sampleTime <= 1? 0 : sampleTime > 64? 63 : sampleTime - 1);

  uint8_t adjustResult = 0;
  if (averageCount < 4)
  {
    adjustResult = averageCount;
  }
  else
  {
    adjustResult = 4;
  }

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER(clkDiv) | (averageCount > 0? ADC_CTRLB_RESSEL_16BIT : ADC_CTRLB_RESSEL_12BIT);
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM(averageCount) | ADC_AVGCTRL_ADJRES(adjustResult);

  // result resolution is 12 bit, even when averaging is used
  this->adcFullScale = 4095;
}

void Analog2DigitalConverter::setReference(Reference type, float voltage, bool forecUpdate)
{
  switch (type)
  {
    case REF_INT1V:
      refSel = ADC_REFCTRL_REFSEL_INT1V;
      refVal1V = voltage;
      break;
    case REF_INTVCC_DIV_1_48:
      refSel = ADC_REFCTRL_REFSEL_INTVCC0;
      break;
    case REF_INTVCC_DIV_2:
      refSel = ADC_REFCTRL_REFSEL_INTVCC1;
      break;
    case REF_EXTA:
      refSel = ADC_REFCTRL_REFSEL_AREFA;
      break;
    case REF_EXTB:
      refSel = ADC_REFCTRL_REFSEL_AREFB;
      break;
  }
  refVal = voltage;
  resultScale = refVal*inputScale/adcFullScale;

  if (forecUpdate)
  {
    lastRefSel = ULONG_MAX;
  }
}

void Analog2DigitalConverter::setGain(Gain gain)
{
  switch (gain)
  {
    case GAIN_1:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_1X;
      this->inputScale = 1.0f;
      break;
    case GAIN_2:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_2X;
      this->inputScale = 0.5f;
      break;
    case GAIN_4:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_4X;
      this->inputScale = 0.25f;
      break;
    case GAIN_8:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_8X;
      this->inputScale = 0.125f;
      break;
    case GAIN_16:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_16X;
      this->inputScale = 0.0625f;
      break;
    case GAIN_DIV2:
      this->gain = (uint32_t)ADC_INPUTCTRL_GAIN_DIV2;
      this->inputScale = 2.0f;
      break;
  }
  resultScale = refVal*inputScale/adcFullScale;
}

void Analog2DigitalConverter::enablePin(uint8_t muxPosVal)
{
  if (muxPosVal < PIN_COUNT)
  {
    Pin p = PIN_MAPPING[muxPosVal];
    if (p.pin > 0)
    {
      PORT->Group[p.group].PINCFG[p.pin].reg = PORT_PINCFG_PMUXEN; // enable PMUX
      if (p.pin & 1)
      {
        // odd pin
        PORT->Group[p.group].PMUX[p.pin >> 1].reg = PORT_PMUX_PMUXO_B; // select PMUX function B = ADC
      }
      else
      {
        // even pin
        PORT->Group[p.group].PMUX[p.pin >> 1].reg = PORT_PMUX_PMUXE_B; // select PMUX function B = ADC
      }
    }
  }
}

float Analog2DigitalConverter::read(uint8_t muxPosVal)
{
  // enable temperature sensor or bandgap output and select sampleLength and input gain
  uint8_t sampleLength = this->sampleLength;
  uint32_t refSel = this->refSel;
  uint32_t gain = this->gain;
  float resultScale = this->resultScale;
  switch (muxPosVal)
  {
    case ADC_INPUTCTRL_MUXPOS_TEMP_Val:
      SYSCTRL->VREF.bit.TSEN = 1;
      sampleLength = ADC_SAMPCTRL_SAMPLEN(63); // min. 48 µs required, set to max. value
      refSel = ADC_REFCTRL_REFSEL_INT1V;
      gain = ADC_INPUTCTRL_GAIN_1X;
      resultScale = 1.0f/adcFullScale; // use Vref = 1.0 V for compatiblity with temperature conversion algorithm
      break;

    case ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:
      SYSCTRL->VREF.bit.BGOUTEN = 1;
      refSel = ADC_REFCTRL_REFSEL_INT1V;
      gain = ADC_INPUTCTRL_GAIN_DIV2;
      resultScale = refVal1V*2/adcFullScale;
      break;

    case ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val:
    case ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val:
      refSel = ADC_REFCTRL_REFSEL_INT1V;
      gain = ADC_INPUTCTRL_GAIN_1X;
      resultScale = refVal1V/adcFullScale;
      break;

    default:
      // configure pin for analog input
      enablePin(muxPosVal);
  }

  // set minimum sample length
  ADC->SAMPCTRL.reg = sampleLength;

  // select ADC single ended input and set gain
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS(muxPosVal) | ADC_INPUTCTRL_MUXNEG_GND | gain;

  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);

  // start 1st conversion and discard on reference change
  if (refSel != lastRefSel)
  {
    // changing reference is only effective if ADC module is enabled
    ADC->REFCTRL.reg = refSel;
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);
    ADC->INTFLAG.bit.RESRDY = 1;

    lastRefSel = refSel;
  }

  // start 2nd conversion and read
  ADC->SWTRIG.bit.START = 1;
  while (!ADC->INTFLAG.bit.RESRDY);
  uint16_t result = ADC->RESULT.reg;

  // disable ADC
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);

  // disable temperature sensor or bandgap output and scale result
  switch (muxPosVal)
  {
    case ADC_INPUTCTRL_MUXPOS_TEMP_Val:
      SYSCTRL->VREF.bit.TSEN = 0;
      break;

    case ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:
      SYSCTRL->VREF.bit.BGOUTEN = 0;
      break;

    case ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val:
    case ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val:
      result = result << 2; // copensate divider 1:4
      break;
  }

  // return voltage or temperature
  float voltage = resultScale*result;
  if (muxPosVal == ADC_INPUTCTRL_MUXPOS_TEMP_Val)
  {
    return toTemperature(voltage);
  }
  else
  {
    return voltage;
  }
}

void Analog2DigitalConverter::loadTemperatureCalibrationValues()
{
  if (!temperaturCalibrationValuesLoaded)
  {
    uint8_t roomTempInt = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
    uint8_t roomTempDec = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
    roomTemp = roomTempInt + (float)roomTempDec/10;

    roomADC = ((*(uint32_t*)FUSES_ROOM_ADC_VAL_ADDR & FUSES_ROOM_ADC_VAL_Msk) >> FUSES_ROOM_ADC_VAL_Pos);

    uint8_t hotTempInt = (*(uint32_t*)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
    uint8_t hotTempDec = (*(uint32_t*)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
    hotTemp = hotTempInt + (float)hotTempDec/10;

    hotADC = ((*(uint32_t*)FUSES_HOT_ADC_VAL_ADDR & FUSES_HOT_ADC_VAL_Msk) >> FUSES_HOT_ADC_VAL_Pos);

    int8_t roomInt1vDrift = (int8_t)((*(uint32_t*)FUSES_ROOM_INT1V_VAL_ADDR & FUSES_ROOM_INT1V_VAL_Msk) >> FUSES_ROOM_INT1V_VAL_Pos);
    roomInt1vRef = 1 - (float)roomInt1vDrift/1000;

    int8_t hotInt1vDrift  = (int8_t)((*(uint32_t*)FUSES_HOT_INT1V_VAL_ADDR & FUSES_HOT_INT1V_VAL_Msk) >> FUSES_HOT_INT1V_VAL_Pos);
    hotInt1vRef = 1 - (float)hotInt1vDrift/1000;

    temperaturCalibrationValuesLoaded = true;
  }
}

float Analog2DigitalConverter::toTemperature(float coarseVoltage)
{
  loadTemperatureCalibrationValues();

  float roomVoltage = roomInt1vRef*roomADC/adcFullScale;
  float hotVoltage = hotInt1vRef*hotADC/adcFullScale;
  float coarseTemp =  roomTemp + (hotTemp - roomTemp)*(coarseVoltage - roomVoltage)/(hotVoltage - roomVoltage);
  float refinedVoltage = coarseVoltage*toReferenceVoltage(coarseTemp);

  return roomTemp + (hotTemp - roomTemp)*(refinedVoltage - roomVoltage)/(hotVoltage - roomVoltage);
}

float Analog2DigitalConverter::toReferenceVoltage(float actualTemperature)
{
  return roomInt1vRef + (hotInt1vRef - roomInt1vRef)*(actualTemperature - roomTemp)/(hotTemp - roomTemp);
}

uint8_t Analog2DigitalConverter::highestBitSet(uint8_t x)
{
  uint8_t shifts = 0;
  while (x >>= 1) shifts++;
  return shifts;
}

uint8_t Analog2DigitalConverter::durationToHalfPeriods(uint16_t micros)
{
  uint32_t halfPeriods = adcFrequency*2*micros/1000000U;
  return halfPeriods > 63? 63 : halfPeriods;
}

float Analog2DigitalConverter::roomTemp = 0;
float Analog2DigitalConverter::hotTemp = 0;
float Analog2DigitalConverter::roomInt1vRef = 0;
float Analog2DigitalConverter::hotInt1vRef = 0;
uint32_t Analog2DigitalConverter::roomADC = 0;
uint32_t Analog2DigitalConverter::hotADC = 0;
bool Analog2DigitalConverter::temperaturCalibrationValuesLoaded = false;

// mapping for SAMD2xG variant A-D, see datasheet table 7-1 "PORT Function Multiplexing"
const Analog2DigitalConverter::Pin Analog2DigitalConverter::PIN_MAPPING[Analog2DigitalConverter::PIN_COUNT] =
{
  { 0,  2, PORT_PA02 },
  { 0,  3, PORT_PA03 },
  { 1,  8, PORT_PB08 },
  { 1,  9, PORT_PB09 },
  { 0,  4, PORT_PA04 },
  { 0,  5, PORT_PA05 },
  { 0,  6, PORT_PA06 },
  { 0,  7, PORT_PA07 },
  { 0,  0,         0 },
  { 0,  0,         0 },
  { 1, 10, PORT_PB10 },
  { 1, 11, PORT_PB11 },
  { 0,  0,         0 },
  { 0,  0,         0 },
  { 0,  0,         0 },
  { 0,  0,         0 },
  { 0, 16, PORT_PA16 },
  { 0, 17, PORT_PA17 },
  { 0, 18, PORT_PA18 },
  { 0, 19, PORT_PA19 }
};
