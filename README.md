# ECG & Temperature Monitor with BLE Project Overview

**Project Description.**

*The ECG & Temperature Monitor with BLE is a Bluetooth-enabled device designed to measure ECG signals, body temperature, and simulate battery voltage monitoring. The device uses various sensors, microcontrollers, and LEDs to display real-time data and transmit results via Bluetooth Low Energy (BLE) to a mobile application.

## Project Goals

* Implement an embedded system that measures ECG signals and temperature.
* Use a BLE-enabled microcontroller for data transmission.
* Display results visually using LEDs.
  
## Project Goals

* Use best coding practices throughout the development of your firmware.
* Functions should be short and do one thing.  They should return an exit code that is checked in the calling function, indicating success or failure.
* MACROS!  Avoid hard-coded values in your code.
* Use structs to organize related data.
* Use libraries for code that could be re-used in other projects.
* Use the `LOGGING` module to log errors, warnings, information and debug messages.
* You should not have any compiler/build warnings.  The CI script will build against `v2.6.2` of the Zephyr SDK; using `v2.7.0` will cause build errors related to the SMF module.

## Firmware Functional Specifications

* Device Initialization:
  * Initializes sensors, LEDs, and BLE services during the INIT state.
* Measurement & Notifications:

  * ECG Measurement: Reads ECG signals from a function generator and calculates average heart rate (40-200 BPM).
  * Temperature Measurement: Reads temperature using the MPR9808 sensor.
  * Battery Monitoring: Measures voltage (0-3.7V) using AIN0, simulating a battery level.

  * Implement states of your choosing for the following measurements, calculations and BLE communications.
* Have a heartbeat `LED0` that blinks every 1 second with a 50% duty cycle (`ON:OFF` time) in all states.
* Implement functionality to measure a battery voltage (0-3.7 V) using `AIN0`:
  1. When the device first powers on, and then
  1. Every 1 minute thereafter, but only when in the `IDLE` state.
  1. You won't actually be connecting a battery to your device; you can use a power support or another voltage source to input a voltage to `AIN0` to simulate a battery level.
* Have the brightness of `LED1` linearly modulated by the percentage of the battery level.
* Implement functionality to make two measurements after pressing `BUTTON1`:
  1. Read temperature with your MPR9808 sensor (in degrees Celsius).
  1. Calculate the average heart rate (40-200 BPM) using 25-30 seconds of an ECG signal (ranging from -500 - 500 mV, note this is bipolar) from the function generator (see video on how to setup the function generator to output an ECG signal).
* Pressing `BUTTON1` during the measurements should post an error and go to the `ERROR` state.
* Blink `LED2` with a 25% duty cycle (`ON:OFF` time) at the average heart rate after the measurements are complete.
* Have Bluetooth notifications after the measurements are complete and data have been processed, using the BLE services and characteristics described below.
  * Configure the **DIS (Device Information Service)** to report the device model as your Team Name (come up with something fun).
  * Set the **BAS (Battery Service)** to report the battery level of your device.  (This isn't actually a battery level, but we're using the `AIN0` measurement as a surrogate for a battery level.)
  * Set the **Heart Rate Service** to report the average heart rate.  (See Resources section below.)
  * Setup a custom service with the following custom characterisitics:
    * `Temperature` for the I2C temperature sensor data in degrees Celcius.
    * `Error Code` for the error code that caused the device to enter the `ERROR` state.
* `BUTTON2` should clear (turn off) a blinking `LED2`, and if `LED2` is not blinking because a measurement hasn't been taken, then it should log a warning (`LOG_WRN()`) as to why it appears nothing happened.
* `BUTTON3` should be used to reset the device from the `ERROR` state and return to the `IDLE` state.
* Use timers, kernel events, work queues, threads and any other Zephyr RTOS features as needed to implement the above functionality.

## BLE Server (Mobile App)

Your device can connect via BLE to a mobile app called [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-mobile).  This app can be used to read the services and characteristics that your device is advertising.

## State Diagram

![State Diagram](state_diagram.png)

## Testing & Verification

Complete testing analysis described in [testing/final_project.ipynb](testing/final_project.ipynb) to verify the accuracy of your firmware.

## Resources

### Heart Rate Service (GATT)
* https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/bluetooth/peripheral_hr
* https://docs.zephyrproject.org/latest/doxygen/html/group__bt__hrs.html
