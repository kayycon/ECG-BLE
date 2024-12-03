#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h> 
#include <zephyr/smf.h> 
#include <math.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/services/bas.h> // Battery Service

#include "read_temperature_sensor.h"
#include "ble-lib.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// Constants
#define MEASUREMENT_DELAY_MS 1000
//#define DEBOUNCE_DELAY_MS 20 // Debounce delay in milliseconds

// Thread stack sizes and priorities
#define LOG_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define LOG_THREAD_PRIORITY 5          // Define the thread priority
#define ERROR_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define ERROR_THREAD_PRIORITY 4          // Define the thread priority

// Battery and PWM configurations
#define MAX_BATTERY_VOLTAGE_MV 3700   // Maximum battery voltage in millivolts
#define PWM_PERIOD_USEC 1000         // PWM period in microseconds (1 kHz frequency)

// ECG configurations
#define ECG_SAMPLE_RATE 100 // Sampling rate in Hz
#define ECG_BUFFER_SIZE (ECG_SAMPLE_RATE * 30) // 30 seconds of data (need to change)
#define R_PEAK_THRESHOLD 2000 // Threshold for R peak detection
#define ECG_THREAD_STACK_SIZE 2048
#define ECG_THREAD_PRIORITY 2  // Adjust priority as needed
#define ECG_WORKQUEUE_STACK_SIZE 1024
#define ECG_WORKQUEUE_PRIORITY 3

// ADC Struct Macro
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

// Event definitions
K_EVENT_DEFINE(button_events);
#define GET_BTN_PRESS BIT(0)
#define CLEAR_BTN_PRESS BIT(1)
#define RESET_BTN_PRESS BIT(2)
#define ANY_BTN_PRESS (GET_BTN_PRESS | CLEAR_BTN_PRESS | RESET_BTN_PRESS)

// error event
K_EVENT_DEFINE(error_event);
#define ERROR_EVENT BIT(0)
atomic_t error_events = ATOMIC_INIT(0);

// Error codes
K_EVENT_DEFINE(errors);
#define ERROR_ADC_INIT      BIT(0)
#define ERROR_PWM_INIT      BIT(1)
#define ERROR_BLE_INIT      BIT(2)
#define ERROR_GPIO_INIT     BIT(3)
#define ERROR_SENSOR_READ   BIT(4)

// Declare the thread stack
K_THREAD_STACK_DEFINE(log_thread_stack, LOG_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(error_thread_stack, ERROR_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ecg_thread_stack, ECG_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ecg_workqueue_stack, ECG_WORKQUEUE_STACK_SIZE);

// function declarations
const struct device *const temp_sensor = DEVICE_DT_GET_ONE(microchip_mcp9808);
void adjust_led_brightness(const struct pwm_dt_spec *pwm_led, int32_t battery_voltage_mv);

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
static const struct adc_dt_spec vadc_battery = ADC_DT_SPEC_GET_BY_ALIAS(vadcbatt);
static const struct adc_dt_spec vadc_hrate = ADC_DT_SPEC_GET_BY_ALIAS(vadchr);

// pwm structs defined based on DT aliases
static const struct pwm_dt_spec pwm_hbeat = PWM_DT_SPEC_GET(DT_ALIAS(pwmled0));
static const struct pwm_dt_spec pwm_battery = PWM_DT_SPEC_GET(DT_ALIAS(pwmled1));
static const struct pwm_dt_spec pwm_hrate = PWM_DT_SPEC_GET(DT_ALIAS(pwmled2));
static const struct pwm_dt_spec pwm_err = PWM_DT_SPEC_GET(DT_ALIAS(pwmled3));

// global variables
int32_t temperature_degC;
int32_t error_code = 0;  // Global variable to store error codes
int32_t battery_voltage = 0; // Global variable to store the battery voltage
static int16_t ecg_buffer[ECG_BUFFER_SIZE];
static size_t ecg_index = 0;
int32_t average_hr = 0; // Global variable to store the average heart rate
static uint32_t last_press_time = 0; // Variable to store the last press time
struct k_thread ecg_thread_data;
struct k_work_q ecg_workqueue;

// function declarations
int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv);
int calculate_average_heart_rate(const struct adc_dt_spec *adc);

//volatile atomic_t error_events = 0;
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
void ecg_sampling_callback(struct k_timer *ecg_sampling_timer);
void hrate_blink_handler(struct k_timer *hrate_blink_timer);
void error_blink_handler(struct k_timer *error_blink_timer);

void battery_measure_work_handler(struct k_work *work);

// Forward declaration of the work handler function
void ecg_sampling_work_handler(struct k_work *work);
void ecg_sampling_thread(void *arg1, void *arg2, void *arg3);

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
K_TIMER_DEFINE(ecg_sampling_timer, ecg_sampling_callback, NULL); 
K_TIMER_DEFINE(hrate_blink_timer, hrate_blink_handler, NULL);
K_TIMER_DEFINE(error_blink_timer, error_blink_handler, NULL);
K_TIMER_DEFINE(ecg_sampling_stop_timer, NULL, NULL);
K_WORK_DEFINE(battery_measure_work, battery_measure_work_handler);
// Declare the work item
K_WORK_DEFINE(ecg_sampling_work, ecg_sampling_work_handler);
// define states for state machine (THESE ARE ONLY PLACEHOLDERS)
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

    //error_code = 0; // Reset any previous error codes

     // Initialize Bluetooth
     /*
    int ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    if (ret) {
        LOG_ERR("Bluetooth initialization failed (ret = %d)", ret);
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }
    LOG_INF("Bluetooth initialized");
*/
    // Start the heartbeat timer (1-second period, 50% duty cycle)
    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_SECONDS(1));

    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_SECONDS(1));
    LOG_INF("Heartbeat timer started.");
    

    // Measure battery voltage at startup
    //CONFIGURE//INITILAISE//READ
    
    // Check GPIO port readiness (common for all buttons and LEDs)
    if (!device_is_ready(reset_button.port)) {
        LOG_ERR("GPIO interface not ready.");
        error_code |= ERROR_GPIO_INIT;
        //smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }
    // configure GPIO buttons
    int err = gpio_pin_configure_dt(&measure_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure acquire button.");
    }
    err = gpio_pin_configure_dt(&clear_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure clear button.");
    }
    err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure reset button.");
    }
    // configure GPIO LEDs
    err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure heartbeat LED.");
    }
    err = gpio_pin_configure_dt(&battery_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure out LED.");
    }
    err = gpio_pin_configure_dt(&hrate_led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (err < 0) {
        LOG_ERR("Cannot configure heart rate LED.");
    }
    err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT | GPIO_ACTIVE_HIGH); // didn't specify active low b/c it is not a requirement of the assignment
    if (err < 0) {
        LOG_ERR("Cannot configure error LED.");
    }

    k_work_queue_start(
        &ecg_workqueue,
        ecg_workqueue_stack,
        K_THREAD_STACK_SIZEOF(ecg_workqueue_stack),
        ECG_WORKQUEUE_PRIORITY,
        NULL
    );

    // Check that the ADC interface is ready
    if (!device_is_ready(vadc_hrate.dev)) {
    LOG_ERR("ADC controller device for heart rate not ready");
    error_code |= ERROR_ADC_INIT;
    smf_set_state(SMF_CTX(&s_obj), &states[Error]);
    return;
    }

    // Configure the ADC channels
    err = adc_channel_setup_dt(&vadc_battery);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", err);
    }
    err = adc_channel_setup_dt(&vadc_hrate);
    if (err < 0) {
    LOG_ERR("Could not setup ADC channel (%d)", err);
    error_code |= ERROR_ADC_INIT;
    smf_set_state(SMF_CTX(&s_obj), &states[Error]);
    return;
}

    /*
    void test_adc_read(void) {
    int32_t voltage_mv = 0;
    int ret = measure_battery_voltage(&vadc_battery, &voltage_mv);
    if (ret == 0) {
        LOG_INF("Test ADC read successful, voltage: %d mV", voltage_mv);
    } else {
        LOG_ERR("Test ADC read failed: %d", ret);
    }
}
*/
    // check that the PWM controller is ready
    if (!device_is_ready(pwm_hbeat.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm_hbeat.dev->name);
        // return -1;
    }
    if (!device_is_ready(pwm_battery.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm_battery.dev->name);
        // return -1;
    }
    if (!device_is_ready(pwm_hrate.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm_hrate.dev->name);
        // return -1;
    }
    if (!device_is_ready(pwm_err.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm_err.dev->name);
        // return -1;
    }

    // check that the temp sensor is ready
    if (!device_is_ready(temp_sensor)) {
            LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
            //return -1;
    }
    else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }
    // associate callback with GPIO pin
    // trigger on transition from INACTIVE -> ACTIVE; ACTIVE could be HIGH or LOW
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


    int32_t battery_voltage = 0;
    if (measure_battery_voltage(&vadc_battery, &battery_voltage) != 0) {
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

    // Start the battery timer for 1-minute periodic measurements
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

    // Reset ECG buffer index
    ecg_index = 0;

    k_thread_create(
        &ecg_thread_data,
        ecg_thread_stack,
        K_THREAD_STACK_SIZEOF(ecg_thread_stack),
        ecg_sampling_thread,
        NULL, NULL, NULL,
        ECG_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );

    // Start ECG sampling timer (sampling every 10 ms for 30 seconds)
    //k_timer_start(&ecg_sampling_timer, K_NO_WAIT, K_MSEC(10)); // 100 Hz sampling rate


    // Optionally, set a timer to stop sampling after 30 seconds
    //k_timer_start(&ecg_sampling_stop_timer, K_SECONDS(30), K_NO_WAIT);
    LOG_INF("Measurement process started.");

    // Start ECG sampling timer (30 seconds)
    //k_timer_start(&ecg_sampling_timer, K_NO_WAIT, K_SECONDS(30));
}

static void measure_run(void *o) {
    LOG_INF("Measure Run State");

    // Check for error event
    //if (s_obj.events & ERROR_EVENT) {
    if (atomic_test_and_clear_bit(&error_events, 0)) {
        LOG_ERR("Error detected during measurement. Transitioning to ERROR state.");
       //s_obj.events &= ~ERROR_EVENT; // Clear the error event flag
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    // Step 1: Read Temperature
    int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
    if (ret != 0) {
        LOG_ERR("Failed to read temperature sensor");
        error_code |= ERROR_SENSOR_READ;
        //smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }
    LOG_INF("Temperature: %d °C", temperature_degC);

    // Step 2: Ensure ECG Sampling is Complete
    if (ecg_index < ECG_BUFFER_SIZE) {
        LOG_INF("ECG sampling not yet complete. Waiting...");
        return;  // Wait for ECG sampling to complete
    }

    // Step 3: Calculate Average Heart Rate
    ret = calculate_average_heart_rate(&vadc_hrate);
    if (ret == 0) {
        LOG_INF("Average Heart Rate calculated successfully: %d BPM", average_hr);
    } else {
        LOG_ERR("Failed to calculate heart rate");
        error_code |= ERROR_ADC_INIT;
        //smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    // Step 4: Set Timer for Heart Rate LED Blink
    if (average_hr > 0) {
        uint32_t period_ms = 60000 / average_hr;      // Timer period in milliseconds
        uint32_t on_time_ms = period_ms / 4;         // 25% duty cycle

        k_timer_start(&hrate_blink_timer, K_MSEC(on_time_ms), K_MSEC(period_ms));
        LOG_INF("Heart Rate LED blink started with %d ms ON time and %d ms period", on_time_ms, period_ms);
    }

/*
    // Step 5: Send BLE Notifications
    // Send Temperature Notification
    ret = send_BT_notification(current_conn, (uint8_t *)&temperature_degC, sizeof(temperature_degC));
    if (ret) {
        LOG_ERR("Failed to send temperature notification");
    }

    if (current_conn) {
    int ret = send_BT_notification(current_conn, (uint8_t *)&average_hr, sizeof(average_hr));
    if (ret) {
        LOG_ERR("Failed to send heart rate notification (ret = %d)", ret);
    } else {
        LOG_INF("Heart rate notification sent: %d BPM", average_hr);
    }
    } else {
    LOG_ERR("No active BLE connection. Heart rate notification skipped.");
    }
    */

    // Transition back to IDLE
    smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
}

static void measure_exit(void *o) {
    LOG_INF("Exiting Measure State");
    k_timer_stop(&hrate_blink_timer);  // Stop the Heart Rate LED Blink Timer

    k_timer_stop(&hrate_blink_timer);  // Stop blinking LED2
}

static void error_entry(void *o) {
    LOG_INF("Error Entry State");

    // Blink all LEDs at 50% duty cycle
    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&battery_blink_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&hrate_blink_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&error_blink_timer, K_NO_WAIT, K_MSEC(500));

    LOG_INF("Started all LED timers in Error State.");

    // Notify Error Code via BLE
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
    k_timer_stop(&heartbeat_timer);
    k_timer_stop(&battery_blink_timer);
    k_timer_stop(&temp_read_timer);
    LOG_INF("Exiting Error State");
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

        // Sleep briefly to allow other threads to execute
        k_msleep(1);
    }

    return 0;
}

/*
int main(void) {
    
    int ret;
 
    ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
  
    }

    // Initialize state machine context
    smf_set_initial(SMF_CTX(&s_obj), &states[Init]); 

    while (1) {
        // State machine started; terminates if a non-zero value is returned
        int32_t ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) {
            LOG_ERR("State machine error: %d", ret);
            break;
        }
        s_obj.events = k_event_wait(&button_events, ANY_BTN_PRESS, true, K_FOREVER);  

        k_msleep(1);  // sleep for 1 ms for logging purposes
        
        // read the temperature every MEASUREMENT_DELAY_MS
        ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("Problem reading the temperature sensor (%d)", ret);
            continue;
            //return ret;
        }

        LOG_INF("Temperature: %d", temperature_degC);

        k_msleep(MEASUREMENT_DELAY_MS);

    }

    return 0;
}
*/

void log_thread(void) {
    while (1) {
        // Log toggling status of the heartbeat LED
        if (hbeat_led_status.toggled) {
            LOG_INF("Heartbeat LED toggled at %lld ms", hbeat_led_status.toggle_time);
            hbeat_led_status.toggled = false; // Reset the toggled flag
        }

        // Log toggling status of the battery LED
        if (battery_led_status.toggled) {
            LOG_INF("Battery LED toggled at %lld ms", battery_led_status.toggle_time);
            battery_led_status.toggled = false; // Reset the toggled flag
        }

        // Log toggling status of the HR LED
        if (hrate_led_status.toggled) {
            LOG_INF("HR LED toggled at %lld ms", hrate_led_status.toggle_time);
            hrate_led_status.toggled = false; // Reset the toggled flag
        }

        // Log toggling status of the error LED
        if (error_led_status.toggled) {
            LOG_INF("Error LED toggled at %lld ms", error_led_status.toggle_time);
            error_led_status.toggled = false; // Reset the toggled flag
        }

        /*  Log battery voltage periodically if needed
        static int64_t last_battery_log_time = 0;
        int64_t current_time = k_uptime_get();
        if (current_time - last_battery_log_time >= 60000) { // Log every 60 seconds
            LOG_INF("Battery voltage: %d mV", battery_voltage);
            last_battery_log_time = current_time;
        } */

        // Sleep to prevent busy-waiting
        k_sleep(K_MSEC(10)); // Sleep for 100 ms
    }
}

void error_thread(void) {
    
    while (1) {
        s_obj.events = k_event_wait(&error_event, ERROR_EVENT, true, K_FOREVER);

        if (s_obj.events & ERROR_EVENT) {
            LOG_ERR("Error detected, entering error state.");
            atomic_set_bit(&error_events, 0);  // Set the error flag
            //s_obj.events &= ~ERROR_EVENT;
            //smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }

    }
}

void ecg_sampling_thread(void *arg1, void *arg2, void *arg3) {
    while (1) {
        if (ecg_index >= ECG_BUFFER_SIZE) {
            LOG_INF("Finished ECG sampling");
            // Optionally signal completion to the main thread
            break;
        }

        int16_t ecg_sample = 0;

        // Read ECG signal using ADC
        struct adc_sequence sequence = {
            .channels = BIT(vadc_hrate.channel_id),
            .buffer = &ecg_sample,
            .buffer_size = sizeof(ecg_sample),
            .resolution = vadc_hrate.resolution,
        };

        int ret = adc_read(vadc_hrate.dev, &sequence);
        if (ret == 0) {
            ecg_buffer[ecg_index++] = ecg_sample;
            LOG_DBG("ecg_sample=%d, ecg_index=%d", ecg_sample, ecg_index);
        } else {
            LOG_ERR("Failed to sample ECG, adc_read returned: %d", ret);
            error_code |= ERROR_ADC_INIT;
            atomic_set_bit(&error_events, 0);  // Post error event
            break;
        }

        // Sleep for the sampling interval
        k_sleep(K_MSEC(10));  // 100 Hz sampling rate
    }
}


// Create a thread for the log and error handlers
K_THREAD_DEFINE(log_thread_id, LOG_THREAD_STACK_SIZE, log_thread, NULL, NULL, NULL, LOG_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(error_thread_id, ERROR_THREAD_STACK_SIZE, error_thread, NULL, NULL, NULL, ERROR_THREAD_PRIORITY, 0, 0);

// Define functions
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, CLEAR_BTN_PRESS);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
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
    LOG_DBG("Heartbeat LED toggled to %s", led_on ? "ON" : "OFF");
}

void battery_blink_handler(struct k_timer *timer_id) {
    static bool battery_led_on = false;
    const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(battery), gpios);

    if (!device_is_ready(battery_led.port)) {
        LOG_ERR("Battery LED port not ready");
        return;
    }

    gpio_pin_set_dt(&battery_led, battery_led_on);
    battery_led_on = !battery_led_on; // Toggle the LED state
    LOG_DBG("Battery LED toggled to %s", battery_led_on ? "ON" : "OFF");
}

void error_blink_handler(struct k_timer *timer_id) {
    static bool led_on = false;

    gpio_pin_set_dt(&error_led, led_on);
    led_on = !led_on;  // Toggle state
    LOG_DBG("Error LED toggled to %s", led_on ? "ON" : "OFF");
}


void battery_measure_callback(struct k_timer *timer_id) {
    if (SMF_CTX(&s_obj)->current == &states[Idle]) {
        battery_voltage = 0;

        k_work_submit(&battery_measure_work);
    }
}

void battery_measure_work_handler(struct k_work *work) {
    battery_voltage = 0;

    if (measure_battery_voltage(&vadc_battery, &battery_voltage) == 0) {
        LOG_INF("Battery Voltage: %d mV", battery_voltage);
        // bluetooth_set_battery_level(battery_voltage); // BLE notification

        // Adjust LED brightness based on battery level
        adjust_led_brightness(&pwm_battery, battery_voltage);
    } else {
        LOG_ERR("Failed to measure battery voltage");
        error_code |= ERROR_ADC_INIT;
        // smf_set_state(SMF_CTX(&s_obj), &states[Error]);
    }
}


int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv) {
    if (!device_is_ready(adc->dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV; // Device not ready error
    }

    int16_t adc_sample = 0;  // Use int16_t for proper alignment

    // Clear the buffer (though this is usually unnecessary)
    memset(&adc_sample, 0, sizeof(adc_sample));

    struct adc_sequence sequence = {
        .channels    = BIT(adc->channel_id),
        .buffer      = &adc_sample, // Use the address of adc_sample
        .buffer_size = sizeof(adc_sample),
        .resolution  = adc->resolution,
    };

    // Add a delay before adc_read
    k_sleep(K_MSEC(10));

    int ret = adc_read(adc->dev, &sequence);
    if (ret != 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret; // Return the error code from ADC read
    }

    // Convert raw ADC value to millivolts
    int32_t raw_value = adc_sample;
    *voltage_mv = raw_value * adc->channel_cfg.reference / (1 << adc->resolution);

    LOG_INF("Battery voltage: %d mV", *voltage_mv);
    return 0;
}

/*
int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv) {
    //configure
    
    if (!device_is_ready(adc->dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV; // Device not ready error
    }

    struct adc_sequence sequence = {
        .channels    = BIT(adc->channel_id),
        .buffer      = voltage_mv, // Store result directly in voltage_mv
        .buffer_size = sizeof(*voltage_mv),
        .resolution  = adc->resolution,
    };//define out

    int ret = adc_read(adc->dev, &sequence);
    if (ret != 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret; // Return the error code from ADC read
    }

    // Convert raw ADC value to millivolts
    *voltage_mv = (*voltage_mv * adc->channel_cfg.reference) / (1 << adc->resolution);

    LOG_INF("Battery voltage: %d mV", *voltage_mv);
    return 0;
}
*/


void adjust_led_brightness(const struct pwm_dt_spec *pwm_led, int32_t voltage_mv) {
    
    // Calculate the duty cycle as a percentage of the maximum battery voltage
    uint32_t duty_cycle_percentage = (voltage_mv * 100) / MAX_BATTERY_VOLTAGE_MV; // Scale 0–100%

    // Ensure the duty cycle percentage is within bounds
    if (duty_cycle_percentage > 100) {
        duty_cycle_percentage = 100; // Cap at 100%
    } else if (duty_cycle_percentage < 0) {
        duty_cycle_percentage = 0; // Ensure it doesn't go below 0%
    }

    // Convert to PWM period
    uint32_t duty_cycle = (duty_cycle_percentage * PWM_PERIOD_USEC) / 100; // Convert to PWM period

    // Set the PWM with the calculated duty cycle
    if (pwm_set_dt(pwm_led, PWM_PERIOD_USEC, duty_cycle) != 0) {
        LOG_ERR("Failed to set PWM for battery LED");
    } else {
        LOG_DBG("Battery LED brightness set to %d%%", duty_cycle_percentage);
    }
}

void ecg_sampling_callback(struct k_timer *timer_id) {
    LOG_DBG("ecg_sampling_callback called, ecg_index: %d", ecg_index);

    if (ecg_index >= ECG_BUFFER_SIZE) {
        k_timer_stop(&ecg_sampling_timer);
        LOG_INF("Finished ECG sampling");
        return;
    }

    // Submit work to perform ADC read in thread context
    //k_work_submit(&ecg_sampling_work);
    k_work_submit_to_queue(&ecg_workqueue, &ecg_sampling_work);
}

void ecg_sampling_work_handler(struct k_work *work) {
    LOG_DBG("ecg_sampling_work_handler called");

    int16_t ecg_sample = 0;

    // Read ECG signal using ADC
    struct adc_sequence sequence = {
        .channels = BIT(vadc_hrate.channel_id),
        .buffer = &ecg_sample,
        .buffer_size = sizeof(ecg_sample),
        .resolution = vadc_hrate.resolution,
    };

    int ret = adc_read(vadc_hrate.dev, &sequence);
    if (ret == 0) {
    ecg_buffer[ecg_index++] = ecg_sample;
    LOG_DBG("ADC read success, ecg_sample=%d, ecg_index=%d", ecg_sample, ecg_index);
    } else {
    LOG_ERR("Failed to sample ECG, adc_read returned: %d", ret);
    k_timer_stop(&ecg_sampling_timer);
    error_code |= ERROR_ADC_INIT;
    atomic_set_bit(&error_events, 0);  // Post error event
    //k_event_post(&error_event, ERROR_EVENT);
}

}


int calculate_average_heart_rate(const struct adc_dt_spec *adc) {
    // Ensure ECG sampling is complete
    if (ecg_index < ECG_BUFFER_SIZE) {
        LOG_ERR("ECG sampling incomplete");
        return -1;
    }

    size_t r_peak_count = 0;
    //int16_t threshold = 200; // threshold for R-peak detection

    //  R-peak detection
    for (size_t i = 1; i < ecg_index - 1; i++) {
    if (ecg_buffer[i] > hr_config.r_peak_threshold &&
        ecg_buffer[i] > ecg_buffer[i - 1] &&
        ecg_buffer[i] > ecg_buffer[i + 1]) {
        r_peak_count++;
        LOG_DBG("R-peak detected at index %d: %d", i, ecg_buffer[i]);
        }
    }

    // Calculate average BPM
    int bpm = (r_peak_count * 60) / 30; // BPM = Peaks per minute
    LOG_INF("Calculated BPM: %d", bpm);

    // Assign to global variable
    average_hr = bpm;

    return 0; // Success
}

void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t current_time = k_uptime_get_32(); // Get current time in milliseconds

    // Check if enough time has passed since the last valid press
    /*
    if ((current_time - last_press_time) < DEBOUNCE_DELAY_MS) {
        LOG_DBG("Button press ignored due to debounce.");
        return; // Ignore the button press
    }
    */

    // Process the button press
    if (SMF_CTX(&s_obj)->current == &states[Measure]) {
        LOG_ERR("BUTTON1 pressed during measurements. Posting error event.");
        k_event_post(&error_event, ERROR_EVENT);  // Trigger error event
    } else {
        LOG_INF("BUTTON1 pressed. Starting measurement.");
        k_event_post(&button_events, GET_BTN_PRESS);  // start/Trigger measurement event
    }

    // Update the last press time
    last_press_time = current_time;
}

void hrate_blink_handler(struct k_timer *timer_id) {
    static bool led_on = false;
    const struct gpio_dt_spec *led = &hrate_led;

    // Toggle LED state
    gpio_pin_set_dt(led, led_on);
    led_on = !led_on;  // Toggle state

    // Log the toggle state with heart rate info
    LOG_DBG("Heart Rate LED toggled to %s at %d BPM", led_on ? "ON" : "OFF", average_hr);
}




