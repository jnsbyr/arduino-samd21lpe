# Arduino Core for SAMD21 Low Power Extensions

Take Arduino low power projects for the SAMD21 to the next level.

### Table of contents

[1. Motivation](#motivation)  
[2. MCU Power Management](#mcu-power-management)  
[3. Licenses and Credits](#licenses-and-credits)  

## Motivation

There are already several libraries available that support low power features of the SAMD21 MCU:

- [Arduino Low Power library](https://github.com/arduino-libraries/ArduinoLowPower)
- [RTCZero Library for Arduino](https://github.com/arduino-libraries/RTCZero)

They provide solutions for specific use cases but fall short of making full use of the potential of the SAMD21 MCU. E.g. none of them tries to disable the USB module at runtime, the most power hungry functions of the MCU, although typically not required for low power usage.

Another part of the problem is the rigid way the Arduino Core for SAMD21 bootstraps the MCU for 48 MHz with most MCU features enabled and configured for maximum performance. In contrast to classic Arduino MCUs like the Atmel ATmega 328 the SAMD21 MCU supports up to 9 clock sources at the same time, that may run at different frequencies. Using the generic clock controller (GCLK) the clocks can be selectively assigned to different MCU modules and can be stopped in standby mode. Combined with the power manager (PM) this allows disabling unused MCU modules and running the required MCU modules with clock frequencies that are optimized for the use case.

There are also individual hardware aspects of each board to consider that adversely affect low power usage, like permanently powered LEDs or a reverse current into the voltage regulator.

## MCU Power Management

Well known from other CPUs and MCUs effective power management can be achieved by the combination of several aspects:
- **reducing clock frequencies**, as power consumption often has a linear relation to the clock frequency
- dynamically enabling and **disabling MCU features**
- making use of MCU specific **power management** features (e.g. sleep mode)
- **reducing supply voltage**

This library provides several functions for that purpose with the *SAMD21LPE::System* class. Using the the SAMD21 datasheet or looking into other documentation that is available online will add the necessary background. For a quick start it will be enough to look into the [examples](examples) that have been included.

The [System](src/System.h) class supports the following operations:
- configure the internal clock sources *OSCULP32K* and *OSC8M*
- disable any clock source
- enable/disable the clock of any MCU module including the power management of the internal MCU buses for the MCU modules *ADC*, *EIC*, *RTC*, *TC2-TC5* and *USB*
- enable/disable SysTick
- configure/enter sleep mode (idle 0-2 or standby)
- enable/disable sleep-on-exit-ISR mode
- disable all non essential modules incl. *USB*

In this list the **reduction of the main clock** is not included. Changing this clock at any time is possible but in combination with the Arduino framework this is typically not a good choice as many Arduino library assume that the main clock frequency *F_CPU* is fixed at compile time. Changing it at runtime to a different value will result in the loss of various functions, especially everything that is timing related (e.g. millis() will no longer be real milliseconds). As the Arduino Core for SAMD21 only supports F_CPU fixed to 48 MHz a patch if provided for [startup.c](patches/Seeduino-hardware-samd-1.8.3/startup.c) and [boards.txt](patches/Seeduino-hardware-samd-1.8.3/boards.txt) to support alternative values of 32, 24, 16, 12, 8, 6, 3 and 1.5 MHz. The provided patch is for the Seeed Studio XIAO SAMD21, but this patch can also applied to any other SAMD21 MCU with no or little changes.

The SAMD21 *USB* module requires a clock of 48 MHz, although this does not have to be the main clock. Changing the main clock without setting up an alternative clock for the USB module will cause the USB operations to fail, so **disabling the *USB* module** is recommended, especially if low power consumption is required, as the active USB module uses 10.3 mA by itself at 3.3 V.

Using these functions, especially combined, can reduce the average runtime power significantly and allows to put the MCU into standby mode with a supply current of around 2 µA at 3 V.

## Licenses and Credits

### Documentation and Photos

Copyright (c) 2023 [Jens B.](https://github.com/jnsbyr)

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

### Source Code

Copyright (c) 2023 [Jens B.](https://github.com/jnsbyr)

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

The source code was edited and build using the [Arduino IDE](https://www.arduino.cc/en/software/), [Arduino CLI](https://github.com/arduino/arduino-cli) and [Microsoft Visual Studio Code](https://code.visualstudio.com).

The source code depends on:

#### Arduino SDK & Seed Studio SDK for SAMD21 and SAMD51

Copyright (C) 2014 [Arduino LLC](https://github.com/arduino/Arduino)

[![License: LGPL v2.1](https://img.shields.io/badge/License-LGPL%202.1%20only-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)

#### CMSIS Atmel

Copyright (C) 2015 [Atmel Corporation](https://github.com/arduino/ArduinoModule-CMSIS-Atmel)

[![License](https://img.shields.io/badge/License-BSD_2--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

Also many thanks to the Open Source community, the bloggers and the forum users that have shared their experience with the SAM D21 MCU.