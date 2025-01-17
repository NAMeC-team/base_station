#!/bin/bash
set -e # stop script on any $$ != 0
mbed compile
sixtron_flash stm32l4a6rg BUILD/ZEST_CORE_STM32L4A6RG/GCC_ARM/base_station.bin

