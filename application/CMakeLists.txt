# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ecg-temp-ble-project)

# make sure to add the source code for new libraries
target_sources(app PRIVATE src/main.c src/read_temperature_sensor.c src/ble-lib.c src/heartrate.c)
