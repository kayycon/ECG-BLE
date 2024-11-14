# Final Project: ECG & Temperature Monitor with BLE

**You are allowed to work in groups of up to 3 students for this final project.**

* Have one person (referred to as the Team Leader below) in your group fork this repository into their userspace.
* All other team members should fork the Team Leader's repository.
* Each team member should add Dr. Palmeri and all of the class teaching assistants as Maintainers to their repository.
* Questions should be asked exclusively through GitLab Issues.
* All team members should contribute to the final project, and this equal contribution should be reflected through the git history.
* All team members should create Merge Requests to the Team Leader's repository for their contributions throughout the project.
* If you work as a team, the only way for everyone to get credit is through git commits and merge requests that reflect individual user contributions.  **You will not be able to say "we worked on this together" without git commits and merge requests to back it up to receive credit for working on the project.**
* Inequalities in contributions will be reflected in the **individual** final project grades.
* :scream: If collaborating with other students in the class stresses you out, or you worry about effort inequity, then I would recommend working alone.

## Git Version Control Management

* Use best version control practices throughout the development of your firmware.
* Use branches for development and merge into your `main` branch when the code is stable.
* Commit often with meaningful commit messages.
* Merge Requests to the Team Leader's repository should be made when focused, but functional, changes are complete.
* Using libraries will help avoid conflicts in `main.c` that will occur if everyone writes code in the same file.
* Be careful collaborating with Jupyter notebooks; they don't play well with merges.  You may want each team member to perform testing in separate notebooks that you manually combine at the end of the project, or you may divide up the labor of testing and analysis between team members.

## Best Coding Practices

* Use best coding practices throughout the development of your firmware.
* Functions should be short and do one thing.  They should return an exit code that is checked in the calling function, indicating success or failure.
* MACROS!  Avoid hard-coded values in your code.
* Use structs to organize related data.
* Use libraries for code that could be re-used in other projects.
* Use the `LOGGING` module to log errors, warnings, information and debug messages.
* You should not have any compiler/build warnings.  The CI script will build against `v2.6.2` of the Zephyr SDK; using `v2.7.0` will cause build errors related to the SMF module.

## Firmware Functional Specifications

* Write all firmware using the state machine framework.
  * Do all device initialization in an `INIT` state.
  * Have an `IDLE` state when the device isn't making any measurements.
  * Have an `ERROR` state if any error exit codes are returned from any functions.
    * All 4 LEDs should blink at a 50% duty cycle (`ON:OFF` time), in-phase with each other, in the `ERROR` state.
    * An error condition should post an error-related event that causes the device to enter the `ERROR` state.
    * The error code should specify the error condition that caused the device to enter the `ERROR` state.  For example, you may choose to have a bit array that can capture multiple error conditions.
    * A BLE notification should be sent with the error code (see BLE custom service/characteristic below).
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

Generate a detailed state diagram that all states, events and actions for your firmware.  A starter diagram is provided in the `state_diagram.puml` file, along with its rendering below.  You should add states, events and actions as needed to fully describe the functionality of your firmware.

![State Diagram](state_diagram.png)

## Testing & Verification

Complete the testing analysis described in [testing/final_project.ipynb](testing/final_project.ipynb) to verify the accuracy of your firmware.

## Extra Credit

* :star: Implement signal processing to get the ECG measurement to work on a noisy signal with low-frequency "DC drift".  This will be graded based on the quality of the implementation and the ability to accurately measure the average heart rate.

## Grading

* This final project is worth 75% of your grade.  Absolutely no late submissions will be accepted.
* Git version control will be graded based on best practices.
* Firmware will be graded based on all best practices taught throughout the semester.
* Code organization and coding best practices will be graded.
* State diagram will be graded based on completeness, accuracy and ease of interpretation.
* Testing and analysis technical report will be graded based on presentation, completeness, and accuracy.
* Your demo will be graded based on the functionality of your device and your ability to answer questions about your firmware.
* Extra credit will only be considered after all required functionality is implemented and working correctly.  
  * Completing the extra credit will forgive some/all late penalties for labs this semester.  (Applies to all team members.)
  * If you do not have any late penalties, then the extra credit will be added to your final project grade, including going over 100%.

## What to Submit

* Make sure that all of your development branches have been merged into `main` in your Team Leader's repository.
* Create an annotated tag called `v1.0.0` to mark the commit that you want to be graded.  If you fix any bugs after creating this tag, you can create another tag called `v1.0.1`, etc.  Your latest tag will be the one that is graded.
* Create an Issue in your repository with the title "Final Project Submission", and assign it to Dr. Palmeri.
* **All repositories will be cloned at the due date/time for grading.  Absolutely no changes will be accepted after this time.**
* **Your team must schedule a time to do a live demo of your device with Dr. Palmeri before the due date of this assignment.  Your team will be asked questions during this demo.**

## Resources

### Heart Rate Service (GATT)
* https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/bluetooth/peripheral_hr
* https://docs.zephyrproject.org/latest/doxygen/html/group__bt__hrs.html