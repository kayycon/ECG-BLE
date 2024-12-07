#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h> 
#include <zephyr/smf.h> 
#include <math.h>

#include "read_temperature_sensor.h"
#include "ble-lib.h"
#include "heartrate.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// Constants
#define MEASUREMENT_DELAY_MS 1000
#define DEBOUNCE_DELAY_MS 20 // Debounce delay in milliseconds

// Thread stack sizes and priorities
#define LOG_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define LOG_THREAD_PRIORITY 5          // Define the thread priority
#define ERROR_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define ERROR_THREAD_PRIORITY 4          // Define the thread priority

// Battery and PWM configurations
#define MAX_BATTERY_VOLTAGE_MV 3700   // Maximum battery voltage in millivolts
#define PWM_PERIOD_USEC 1000         // PWM period in microseconds (1 kHz frequency)

// ECG configurations
#define SAMPLING_INTERVAL_MS 5
#define BLINK_TIMER_INTERVAL_MS 500
#define TIMER_INTERVAL_MS (1000 / ECG_SAMPLE_RATE) 
#define MIN_RR_INTERVAL_MS 250 
#define MAX_RR_INTERVAL_MS 2000 
#define ECG_BUFFER_SIZE (ECG_SAMPLE_RATE * 30) // 30 seconds of data

#define R_PEAK_THRESHOLD 250 
#define MIN_PEAK_INTERVAL_MS 240
#define ECG_THREAD_STACK_SIZE 4096
#define ECG_THREAD_PRIORITY 2  
#define ECG_WORKQUEUE_STACK_SIZE 4096
#define ECG_WORKQUEUE_PRIORITY 3
#define MAX_HEARTRATE 220

// Define a separate event group for ECG events
K_EVENT_DEFINE(ecg_events);
#define ECG_SAMPLING_COMPLETE BIT(0) // Define a new bit for ECG completion

// ADC Struct Macro
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

int16_t buf1, buf2;

struct adc_sequence sequence0 = {
    .buffer = &buf1,
    .buffer_size = sizeof(buf1), // bytes
};

struct adc_sequence_options options = {
        .interval_us = 100,
        .extra_samplings = ECG_BUFFER_SIZE - 1,  // -1 b/c first sample is already in the buffer
        //.extra_samplings = 0,  // Read one sample per adc_read
        .callback = NULL,  // called after all extra samples are collected
};

struct adc_sequence sequence2 = {
        .options = &options,  // add the options to the sequence
        .buffer = &buf2,  // buf is now a pointer to the first index of an array
        .buffer_size = ECG_BUFFER_SIZE * sizeof(buf2),  // need to specify the size of the buffer array in bytes
        /*  If the buffer is not global or local in scope, the buffer (array) name will just be a pointer to the first element of the array!  The buffer_size will need to be calculated as the product of the size (in bytes) of this first index and the length of the array (number of indices in the array)

        .buffer_size = BUFFER_ARRAY_LEN * sizeof(buf),  // non-global/local array scope
        */
};

// Event definitions
K_EVENT_DEFINE(button_events);
#define GET_BTN_PRESS BIT(0)
#define CLEAR_BTN_PRESS BIT(1)
#define RESET_BTN_PRESS BIT(2)
#define ANY_BTN_PRESS (GET_BTN_PRESS | CLEAR_BTN_PRESS | RESET_BTN_PRESS)

// error event
K_EVENT_DEFINE(errors);
#define ERROR_ADC_INIT      BIT(0)
#define ERROR_PWM_INIT      BIT(1)
#define ERROR_BLE_INIT      BIT(2)
#define ERROR_GPIO_INIT     BIT(3)
#define ERROR_SENSOR_INIT   BIT(4)
#define ERROR_IN_MEASURE    BIT(5)
#define BTN_PRESS_ERROR     BIT(6)
#define ERROR_EVENT         BIT(7)

// Declare the thread stack
K_THREAD_STACK_DEFINE(ecg_thread_stack, ECG_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ecg_workqueue_stack, ECG_WORKQUEUE_STACK_SIZE); 

// function declarations
const struct device *const temp_sensor = DEVICE_DT_GET_ONE(microchip_mcp9808);
void adjust_led_brightness(const struct pwm_dt_spec *pwm1, int32_t battery_voltage_mv);

// button gpio structs
const struct gpio_dt_spec measure_button = GPIO_DT_SPEC_GET(DT_ALIAS(measurebutton), gpios);
const struct gpio_dt_spec clear_button = GPIO_DT_SPEC_GET(DT_ALIAS(clearbutton), gpios);
const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);

// led gpio structs
const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(battery), gpios);
const struct gpio_dt_spec hrate_led = GPIO_DT_SPEC_GET(DT_ALIAS(measure), gpios);
const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);

// adc structs storing all the DT parameters
static const struct adc_dt_spec vadcbatt = ADC_DT_SPEC_GET_BY_ALIAS(vadc0);
static const struct adc_dt_spec vadchr = ADC_DT_SPEC_GET_BY_ALIAS(vadc1);

// pwm structs defined based on DT aliases
static const struct pwm_dt_spec pwm1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));
//static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(temp);

// global variables
int32_t temperature_degC;
int32_t error_code = 0;  // Global variable to store error codes
int32_t battery_voltage = 0; // Global variable to store the battery voltage
int ecg_index;
float blink_frequency, on_time_seconds, pwm_duty, pwm_duty_max;
//static int16_t ecg_buffer[ECG_BUFFER_SIZE];

int32_t average_hr = 0; // Global variable to store the average heart rate
static uint32_t last_press_time = 0; // Variable to store the last press time
struct k_thread ecg_thread_data;
struct k_work_q ecg_workqueue;
int32_t val_mv0, val_mv1;
int bpm;

// function declarations

int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv);
int calculate_battery_percentage(int32_t voltage_mv);
int calculate_average_heart_rate(const int16_t *ecg_data, size_t data_length);
void apply_moving_average_filter(const int16_t *input, int16_t *output, size_t length, size_t window_size);

static const struct smf_state states[];
const struct smf_state *smf_get_current_state(const struct smf_ctx *ctx);
void smf_set_state(struct smf_ctx *ctx, const struct smf_state *state);

// Callback Function Declarations
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// Handler function declarations
void heartbeat_blink_handler(struct k_timer *heartbeat_timer);
void battery_blink_handler(struct k_timer *battery_blink_timer);
void out_duration_handler(struct k_timer *out_duration_timer);
void pwm_led_handler(struct k_timer *pwm_led_timer);
void temp_read_callback(struct k_timer *temp_read_timer);
void battery_measure_callback(struct k_timer *battery_measure_timer); 
void hrate_blink_handler(struct k_timer *hrate_blink_timer);
void error_blink_handler(struct k_timer *error_blink_timer);

//void ecg_sampling_stop_callback(struct k_timer *ecg_sampling_stop_timer); // Function prototype
void setup_ble(int *ret);
extern struct bt_gatt_service remote_srv;

void battery_measure_work_handler(struct k_work *work);

// Forward declaration of the work handler function
void BLE_notify_handler(struct k_work *work);
void setup_ble(int *ret);

// initialize GPIO Callback Structs
static struct gpio_callback measure_button_cb; // need one per CB
static struct gpio_callback clear_button_cb;
static struct gpio_callback reset_button_cb;
struct heart_rate_config {
    int r_peak_threshold; // Threshold for R-peak detection
    int min_bpm;          // Minimum BPM for valid heart rate
    int max_bpm;          // Maximum BPM for valid heart rate
};

// Create a default configuration
struct heart_rate_config hr_config = {
    .r_peak_threshold = 200,
    .min_bpm = 40,
    .max_bpm = 200,
};

// Timer declarations
K_TIMER_DEFINE(heartbeat_timer, heartbeat_blink_handler, NULL);
K_TIMER_DEFINE(temp_read_timer, temp_read_callback, NULL);
K_TIMER_DEFINE(battery_blink_timer, battery_blink_handler, NULL);
K_TIMER_DEFINE(battery_measure_timer, battery_measure_callback, NULL);
K_TIMER_DEFINE(hrate_blink_timer, hrate_blink_handler, NULL);
K_TIMER_DEFINE(error_blink_timer, error_blink_handler, NULL);
K_WORK_DEFINE(battery_measure_work, battery_measure_work_handler);

// define states for state machine
enum smf_states{ Init, Idle, Measure, Error };

// event struct
struct s_object {
        // This must be first
        struct smf_ctx ctx;
        int32_t events;

} s_obj;


// led struct
struct led {
    int64_t toggle_time;  // int64_t b/c time functions in Zephyr use this type
    bool illuminated; // state of the LED (on/off)
    bool toggled;  // flag to indicate that the LED has been toggled
    bool saved_state;  // flag to indicate that the LED state has been saved
};

// define LED structs for each LED as instances of the led struct
struct led hbeat_led_status = {
    .toggle_time = 0,
    .illuminated = true,
    .toggled = false,
    .saved_state = false
};
struct led battery_led_status = {
    .toggle_time = 0,
    .illuminated = true,
    .toggled = false,
    .saved_state = false
};
struct led hrate_led_status = {
    .toggle_time = 0,
    .illuminated = true,
    .toggled = false,
    .saved_state = false
};
struct led error_led_status = {
    .toggle_time = 0,
    .illuminated = true,
    .toggled = false,
    .saved_state = false
};

// Init state
static void init_entry(void *o)
{
    LOG_INF("Init Entry State");

    // Initialize Bluetooth
    int ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    if (ret) {
        LOG_ERR("Bluetooth initialization failed (ret = %d)", ret);
        k_event_post(&errors, ERROR_BLE_INIT);
    }

    // configure GPIO buttons
    int err = gpio_pin_configure_dt(&measure_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure acquire button.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    err = gpio_pin_configure_dt(&clear_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure clear button.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure reset button.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    // configure GPIO LEDs
    err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure heartbeat LED.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    err = gpio_pin_configure_dt(&battery_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure out LED.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    err = gpio_pin_configure_dt(&hrate_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure heart rate LED.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }
    err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT | GPIO_ACTIVE_HIGH); // didn't specify active low b/c it is not a requirement of the assignment
    if (err < 0) {
        LOG_ERR("Cannot configure error LED.");
        k_event_post(&errors, ERROR_GPIO_INIT);
    }

    k_work_queue_start(
        &ecg_workqueue,
        ecg_workqueue_stack,
        K_THREAD_STACK_SIZEOF(ecg_workqueue_stack),
        ECG_WORKQUEUE_PRIORITY,
        NULL
    );
    LOG_INF("ECG workqueue started with priority %d", ECG_WORKQUEUE_PRIORITY);

    if (!device_is_ready(vadcbatt.dev)) {
    LOG_ERR("ADC controller device for battery not ready");
    k_event_post(&errors, ERROR_ADC_INIT);
    return;
    }
    err = adc_channel_setup_dt(&vadcbatt);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel for battery (%d)", err);
        k_event_post(&errors, ERROR_ADC_INIT);
        return;
    }
    if (!device_is_ready(vadchr.dev)) {
    LOG_ERR("ADC controller device for heart rate not ready");
    k_event_post(&errors, ERROR_ADC_INIT);
    return;
    }
    err = adc_channel_setup_dt(&vadchr);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel for heart rate (%d)", err);
        k_event_post(&errors, ERROR_ADC_INIT);
        return;
    }
    if (!device_is_ready(pwm1.dev))  {
    LOG_ERR("PWM device %s is not ready.", pwm1.dev->name);
    k_event_post(&errors, ERROR_PWM_INIT);
    }
    if (!device_is_ready(temp_sensor)) {
        LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
        k_event_post(&errors, ERROR_SENSOR_INIT);
    }
    else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }
    
    err = gpio_pin_interrupt_configure_dt(&measure_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot attach callback to sw1.");
    }
    err = gpio_pin_interrupt_configure_dt(&clear_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot attach callback to sw2.");
    }
    err = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (err < 0) {
        LOG_ERR("Cannot attach callback to sw3.");
    }
    // populate CB struct with information about the CB function and pin
    gpio_init_callback(&measure_button_cb, measure_button_callback, BIT(measure_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&measure_button, &measure_button_cb);

    gpio_init_callback(&clear_button_cb, clear_button_callback, BIT(clear_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&clear_button, &clear_button_cb);

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&reset_button, &reset_button_cb);

        // Start the heartbeat timer (1-second period, 50% duty cycle)

    k_timer_start(&heartbeat_timer, K_MSEC(BLINK_TIMER_INTERVAL_MS), K_MSEC(BLINK_TIMER_INTERVAL_MS));

    k_timer_start(&battery_blink_timer, K_NO_WAIT, K_MSEC(500));  // 500ms interval
    int32_t battery_voltage = 0;
    if (measure_battery_voltage(&vadcbatt, &battery_voltage) != 0) {
        LOG_ERR("Failed to measure battery voltage at startup");
        error_code |= ERROR_ADC_INIT;
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    LOG_INF("Startup Battery Voltage (mV): %d", battery_voltage);
    smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
}

static void init_run(void *o)
{
    LOG_INF("Init Run State");
}

static void init_exit(void *o)
{
    LOG_INF("Init Exit State");
}

static void idle_entry(void *o)
{
    LOG_INF("Idle Entry State");

    // Stop all non-heartbeat LED timers
    k_timer_stop(&battery_blink_timer);
    k_timer_stop(&hrate_blink_timer);
    
    // Turn off all non-heartbeat LEDs
    gpio_pin_set_dt(&battery_led, 0);
    gpio_pin_set_dt(&hrate_led, 0);
    gpio_pin_set_dt(&error_led, 0);

    // Start the battery timer for 1-minute periodic measurements
    k_timer_init(&battery_measure_timer, battery_measure_callback, NULL);
    k_timer_start(&battery_measure_timer, K_NO_WAIT, K_MINUTES(1));
    LOG_DBG("Battery measurement timer started.");
}

static void idle_run(void *o)
{   
    //LOG_INF("Idle Run State");

    if (s_obj.events & RESET_BTN_PRESS) {
        LOG_INF("Reset button pressed, returning to initial state.");
        smf_set_state(SMF_CTX(&s_obj), &states[Init]);
        s_obj.events &= ~RESET_BTN_PRESS; // Clear the event flag
    }  

    if (s_obj.events & GET_BTN_PRESS) {
        LOG_INF("Measure button pressed, transitioning to MEASURE state.");
        smf_set_state(SMF_CTX(&s_obj), &states[Measure]);
        s_obj.events &= ~GET_BTN_PRESS; // Clear the event flag
    }

    if (s_obj.events & CLEAR_BTN_PRESS) {
        // Check if LED2 (Heart Rate LED) is blinking
        if (k_timer_status_get(&hrate_blink_timer) > 0) {
            // Stop the timer and turn off LED2
            k_timer_stop(&hrate_blink_timer);
            gpio_pin_set_dt(&hrate_led, 0);  // Turn off LED2
            LOG_INF("Heart Rate LED turned off.");
        } else {
            // Log a warning if LED2 is not blinking
            LOG_WRN("Cannot clear LED2: No measurement has been taken.");
        }
        s_obj.events &= ~CLEAR_BTN_PRESS; // Clear the event flag
    }  
}

static void idle_exit(void *o) {
    LOG_INF("Exiting Idle State");

    // Stop the battery timer
    //k_timer_stop(&battery_timer);

    // Stop the battery measurement timer
    k_timer_stop(&battery_measure_timer);
    LOG_DBG("Battery measurement timer stopped.");
}

static void measure_entry(void *o) {
    LOG_INF("Measure Entry State");

    // Start ECG sampling timer (sampling every 10 ms for 30 seconds)
    //k_timer_start(&ecg_sampling_timer, K_MSEC(100), K_MSEC(TIMER_INTERVAL_MS));

    LOG_INF("Measurement process started.");
}

static void measure_run(void *o) {
    // Wait for the ECG_SAMPLING_COMPLETE event
    /* if (k_event_wait(&ecg_events, ECG_SAMPLING_COMPLETE, false, K_NO_WAIT) & ECG_SAMPLING_COMPLETE) {
        LOG_INF("ECG Sampling Complete Event Received."); */

        // Step 1: Read Temperature
        int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("Failed to read temperature sensor");
            k_event_post(&errors, ERROR_EVENT);
            return;
        }
        LOG_INF("Temperature: %d °C", temperature_degC);

        const int samples_per_period = 500;
        const int periods = 10;
        int16_t ecg_buffer[samples_per_period];
        int peak_position[100];  // Accumulate peaks across periods
        int total_peaks = 0;

        for (int i = 0; i < periods; i++) {
        LOG_INF("Measuring period %d of %d", i + 1, periods);

        // Perform ADC sampling
        (void)adc_sequence_init_dt(&vadchr, &sequence2);
        
        int ret = adc_read(vadchr.dev, &sequence2);
        if (ret < 0) {
        LOG_ERR("Could not read ADC: %d", ret);
        } else {
        LOG_DBG("Raw ADC Buffer: %d", buf2);
        }

        signal(ecg_buffer, samples_per_period, 5);
        // Detect peaks and accumulate peak indices
        int current_peaks[10]; // Temporary buffer for peaks in this period
        int current_num_peaks = detect_peaks(ecg_buffer, samples_per_period, current_peaks, 10);
        for (int j = 0; j < current_num_peaks && total_peaks < 100; j++) {
           peak_position[total_peaks++] = (i * samples_per_period) + current_peaks[j];
    }
}
    LOG_INF("ECG Measurement complete.");

    //Calculate BPM from accumulated peaks
    LOG_INF("Number of peaks collected: %d", total_peaks);
    bpm = calc_BPM(peak_position, total_peaks);
    if (bpm > 0) {
        LOG_INF("Calculated BPM: %d", bpm);
    } else {
        LOG_ERR("Failed to calculate BPM");
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    // heartrate = 200;
    bt_hrs_notify(bpm);


        // Step 4: Send BLE Notifications
        if (current_conn) {
            ret = send_BT_notification(current_conn, (uint8_t *)&temperature_degC, sizeof(temperature_degC));
            if (ret) {
                LOG_ERR("Failed to send temperature notification");
            }

            ret = send_BT_notification(current_conn, (uint8_t *)&average_hr, sizeof(average_hr));
            if (ret) {
                LOG_ERR("Failed to send heart rate notification");
            } else {
                LOG_INF("Heart rate notification sent: %d BPM", average_hr);
            }
        } else {
            LOG_WRN("No active BLE connection. Notifications skipped.");
        }

        bt_hrs_notify(bpm);

        // Transition back to IDLE
        smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
    }

    // If the event isn't set yet, just wait until ECG sampling completes

static void measure_exit(void *o) {
    LOG_INF("Exiting Measure State");

    k_timer_stop(&hrate_blink_timer);  // Stop the Heart Rate LED Blink Timer
}

static void error_entry(void *o) {
    LOG_INF("Error Entry State");

    k_timer_stop(&heartbeat_timer);
    k_timer_stop(&battery_blink_timer);
    k_timer_stop(&hrate_blink_timer);
    k_timer_stop(&error_blink_timer);

    pwm_set_pulse_dt(&pwm1, pwm1.period);

    // Blink all LEDs at 50% duty cycle
    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&battery_blink_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&hrate_blink_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&error_blink_timer, K_NO_WAIT, K_MSEC(500)); 

    LOG_INF("Started all LED timers in Error State.");

    int ret = send_BT_notification(current_conn, (uint8_t *)&error_code, sizeof(error_code));
    if (ret) {
        LOG_ERR("Failed to send error notification");
    }
}

static void error_run(void *o)
{   
    LOG_INF("Error Run State");

    if (s_obj.events & RESET_BTN_PRESS) {
        LOG_INF("Reset button pressed, resetting device.");

        // Clear error events and reset the error code
        k_event_set(&errors, 0);
        error_code = 0;

        // Transition back to the Init state to reinitialize
        smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
    }
}

static void error_exit(void *o) {

    k_timer_stop(&error_blink_timer);
    LOG_INF("Exiting Error State");

    //Initialize all the LEDs
    gpio_pin_set_dt(&heartbeat_led, 0);
    gpio_pin_set_dt(&battery_led, 0);
    gpio_pin_set_dt(&hrate_led, 0);
    gpio_pin_set_dt(&error_led, 0);
    k_timer_start(&heartbeat_timer, K_MSEC(BLINK_TIMER_INTERVAL_MS), K_MSEC(BLINK_TIMER_INTERVAL_MS));

}

// Populate state table 
static const struct smf_state states[] = {
        [Init] = SMF_CREATE_STATE(init_entry, init_run, init_exit),
        [Idle] = SMF_CREATE_STATE(idle_entry, idle_run, idle_exit),
        [Measure] = SMF_CREATE_STATE(measure_entry, measure_run, measure_exit),
        [Error] = SMF_CREATE_STATE(error_entry, error_run, error_exit),
};


int main(void) {

    // Initialize state machine context
    smf_set_initial(SMF_CTX(&s_obj), &states[Init]); 
    
    while (1) {
        // Run the current state
        int32_t ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) {
            LOG_ERR("State machine error: %d", ret);
            break;
        }

        // Check for button events without blocking
        s_obj.events |= k_event_wait(&button_events, ANY_BTN_PRESS, false, K_NO_WAIT);

        k_msleep(10);
    }

    return 0;
}

void log_thread(void) {
    while (1) {
        if (hbeat_led_status.toggled) {
            LOG_INF("Heartbeat LED toggled at %lld ms", hbeat_led_status.toggle_time);
            hbeat_led_status.toggled = false; // Reset the toggled flag
        }

        if (battery_led_status.toggled) {
            LOG_INF("Battery LED toggled at %lld ms", battery_led_status.toggle_time);
            battery_led_status.toggled = false; // Reset the toggled flag
        }

        if (hrate_led_status.toggled) {
            LOG_INF("HR LED toggled at %lld ms", hrate_led_status.toggle_time);
            hrate_led_status.toggled = false; // Reset the toggled flag
        }

        if (error_led_status.toggled) {
            LOG_INF("Error LED toggled at %lld ms", error_led_status.toggle_time);
            error_led_status.toggled = false; // Reset the toggled flag
        }
        k_sleep(K_MSEC(10)); // Sleep for 100 ms
    }
}

void error_thread(void) {
    while (1) {
        s_obj.events = k_event_wait(&errors, ERROR_EVENT, true, K_FOREVER);

        if (s_obj.events & ERROR_EVENT) {
            LOG_ERR("Error detected, entering error state.");
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }
    }
}

// Create a thread for the log and error handlers
K_THREAD_DEFINE(log_thread_id, LOG_THREAD_STACK_SIZE, log_thread, NULL, NULL, NULL, LOG_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(error_thread_id, ERROR_THREAD_STACK_SIZE, error_thread, NULL, NULL, NULL, ERROR_THREAD_PRIORITY, 0, 0);

// Define functions
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Add debounce check if needed
    uint32_t current_time = k_uptime_get_32();
    if (current_time - last_press_time < DEBOUNCE_DELAY_MS) {
        return;
    }
    last_press_time = current_time;

    // Check if LED2 (Heart Rate LED) is blinking
    if (k_timer_status_get(&hrate_blink_timer) > 0) {   
        k_timer_stop(&hrate_blink_timer); // Stop the timer and turn off LED2
        gpio_pin_set_dt(&hrate_led, 0);  // Turn off LED2
        LOG_INF("Heart Rate LED turned off.");
    } else {
        LOG_WRN("Cannot clear LED2: No measurement has been taken.");
    }   
    k_event_post(&button_events, CLEAR_BTN_PRESS);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    uint32_t current_time = k_uptime_get_32();

    if (current_time - last_press_time < DEBOUNCE_DELAY_MS) { // Debounce check
        return;
    }
    last_press_time = current_time;

    // Reset if in ERROR state
    if (SMF_CTX(&s_obj)->current == &states[Error]) {
        LOG_INF("Resetting device from ERROR state to IDLE state.");

        k_event_set(&errors, 0);
        error_code = 0;

        // Transition back to the IDLE state
        smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
    } else {
        LOG_WRN("Reset button pressed but device is not in ERROR state.");
    }

    k_event_post(&button_events, RESET_BTN_PRESS);
}

void temp_read_callback(struct k_timer *timer_id) {
    extern int32_t temperature_degC;

    int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
    if (ret != 0) {
        LOG_ERR("Failed to read temperature: %d", ret);
    } else {
        LOG_INF("Temperature: %d °C", temperature_degC);
    }
}

// define handler functions for blinking LEDs
void heartbeat_blink_handler(struct k_timer *heartbeat_timer) {
    static bool led_on = false;

    gpio_pin_set_dt(&heartbeat_led, led_on);
    led_on = !led_on;  // Toggle state
}

void battery_blink_handler(struct k_timer *battery_blink_timer) {
    int32_t voltage_mv;
    int battery_percentage;
    
    if (measure_battery_voltage(&vadcbatt, &voltage_mv) == 0) {
        battery_percentage = calculate_battery_percentage(voltage_mv);
        
        if (battery_percentage > 75) {
            gpio_pin_set_dt(&battery_led, 1);  // Solid on
        } else if (battery_percentage > 50) {
            gpio_pin_toggle_dt(&battery_led);  // Blink at 1 Hz (assuming timer interval is 500ms)
        } else if (battery_percentage > 25) {
            static int count = 0;
            if (count++ % 2 == 0) gpio_pin_toggle_dt(&battery_led);  // Blink at 0.5 Hz
        } else {
            static int count = 0;
            if (count++ % 4 == 0) gpio_pin_toggle_dt(&battery_led);  // Blink at 0.25 Hz
        }     
        LOG_INF("Battery: %d%% (%d mV)", battery_percentage, voltage_mv);
    }
}

void battery_measure_handler(struct k_timer *timer_id)
{
    LOG_INF("Battery Measure Handler called");
}
void hrate_blink_handler(struct k_timer *hrate_blink_timer) {
    static bool led_on = false;
    const struct gpio_dt_spec *led = &hrate_led;

    gpio_pin_set_dt(led, led_on);
    led_on = !led_on;  // Toggle state

    LOG_DBG("Heart Rate LED toggled to %s at %d BPM", led_on ? "ON" : "OFF", average_hr);
}

void error_blink_handler(struct k_timer *error_blink_timer) {
    static bool led_on = false;

    gpio_pin_set_dt(&heartbeat_led, led_on);
    gpio_pin_set_dt(&battery_led, led_on);
    gpio_pin_set_dt(&hrate_led, led_on);
    gpio_pin_set_dt(&error_led, led_on);

    led_on = !led_on;  // Toggle state for next call

    LOG_DBG("All LEDs toggled to %s in Error State", led_on ? "ON" : "OFF");
}

void battery_measure_callback(struct k_timer *timer_id) {
    if (SMF_CTX(&s_obj)->current == &states[Idle]) {
        battery_voltage = 0;

        k_work_submit(&battery_measure_work);
    }
}

void battery_measure_work_handler(struct k_work *work) {

    if (measure_battery_voltage(&vadcbatt, &battery_voltage) == 0) {
        LOG_INF("Battery Voltage: %d mV", battery_voltage);
        bluetooth_set_battery_level(battery_voltage); // BLE notification

        // Adjust LED brightness based on battery level
        adjust_led_brightness(&pwm1, battery_voltage); 
    } else {
        LOG_ERR("Failed to measure battery voltage");
        error_code |= ERROR_ADC_INIT;
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
    }
}

int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv) {
    if (!device_is_ready(adc->dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV; // Device not ready error
    }

    k_sleep(K_MSEC(10));

    (void)adc_sequence_init_dt(&vadcbatt, &sequence0);

    int ret = adc_read(vadcbatt.dev, &sequence0);
    if (ret < 0) {
        LOG_ERR("Could not read ADC 1: %d", ret);
    } else {
        LOG_DBG("Raw ADC Buffer: %d", buf1);
    }

    int32_t raw_adc = buf1;
    ret = adc_raw_to_millivolts_dt(&vadcbatt, &raw_adc);
    if (ret < 0) {
        LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
        *voltage_mv = buf1;
    } else {
    LOG_INF("Battery voltage: %d", raw_adc);
    *voltage_mv = raw_adc;
}
    int battery_percentage = calculate_battery_percentage(*voltage_mv);

    adjust_led_brightness(&pwm1, *voltage_mv);

    if (current_conn) {
        uint8_t battery_level = (uint8_t)battery_percentage;
        ret = send_BT_notification(current_conn, &battery_level, sizeof(battery_level));
        if (ret) {
            LOG_ERR("Failed to send battery level notification: %d", ret);
        } else {
            LOG_INF("Battery level notification sent: %d%%", battery_level);
        }
    } else {
        LOG_WRN("No active Bluetooth connection, skipping battery level notification");
    }

    bluetooth_set_battery_level(*voltage_mv);

    return 0;
}

int calculate_battery_percentage(int32_t voltage_mv) {
    const int32_t MIN_BATTERY_MV = 0;
    const int32_t MAX_BATTERY_MV = 3700;

    if (voltage_mv >= MAX_BATTERY_MV) {
        return 100;
    } else if (voltage_mv <= MIN_BATTERY_MV) {
        return 0;
    } else {
        return (voltage_mv * 100) / MAX_BATTERY_MV;
    }
}

void adjust_led_brightness(const struct pwm_dt_spec *pwm1, int32_t voltage_mv) {
    float voltage_v = voltage_mv / 1000.0f; // Convert mV to V

    // Calculate the duty cycle as a percentage of the maximum battery voltage
    int32_t duty_cycle_percentage = (voltage_v / 3.7f) * 100; // Scale 0–100%

    // Ensure the duty cycle percentage is within bounds
    if (duty_cycle_percentage > 100) {
        duty_cycle_percentage = 100; // Cap at 100%
    } else if (duty_cycle_percentage < 0) {
        duty_cycle_percentage = 0; // Ensure it doesn't go below 0%
    }

    // Convert to PWM period
    uint32_t duty_cycle = (duty_cycle_percentage * PWM_PERIOD_USEC) / 100; // Convert to PWM period

    // Set the PWM with the calculated duty cycle
    int ret = pwm_set_dt(pwm1, PWM_PERIOD_USEC, duty_cycle);
    if (ret != 0) {
        LOG_ERR("Failed to set PWM for battery LED: %d", ret);
    } else {
        LOG_DBG("Battery LED brightness set to %d%%", duty_cycle_percentage);
    }
}

void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t current_time = k_uptime_get_32(); // Get current time in milliseconds

    // Check if enough time has passed since the last valid press  
    if ((current_time - last_press_time) < DEBOUNCE_DELAY_MS) {
        LOG_DBG("Button press ignored due to debounce.");
        return; // Ignore the button press
    }

    // Process the button press
    if (SMF_CTX(&s_obj)->current == &states[Measure]) {
        LOG_ERR("BUTTON1 pressed during measurements. Posting error event.");
        k_event_post(&errors, BTN_PRESS_ERROR);  // Trigger error event
    } else {
        LOG_INF("BUTTON1 pressed. Starting measurement.");
        k_event_post(&button_events, GET_BTN_PRESS);  // start/Trigger measurement event
    }

    // Update the last press time
    last_press_time = current_time;
}