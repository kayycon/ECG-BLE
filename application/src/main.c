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

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MEASUREMENT_DELAY_MS 1000
#define LOG_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define LOG_THREAD_PRIORITY 5          // Define the thread priority
#define ERROR_THREAD_STACK_SIZE 1024     // Define the size of the thread stack
#define ERROR_THREAD_PRIORITY 5          // Define the thread priority
#define MAX_BATTERY_VOLTAGE_MV 3700   // Maximum battery voltage in millivolts
#define PWM_PERIOD_USEC 1000         // PWM period in microseconds (1 kHz frequency)

// ADC Struct Macro
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

// Declare the thread stack
K_THREAD_STACK_DEFINE(log_thread_stack, LOG_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(error_thread_stack, ERROR_THREAD_STACK_SIZE);

// list of events that can be triggered by the buttons
K_EVENT_DEFINE(button_events);
#define GET_BTN_PRESS BIT(0)
#define CLEAR_BTN_PRESS BIT(1)
#define RESET_BTN_PRESS BIT(2)
#define ANY_BTN_PRESS (GET_BTN_PRESS | CLEAR_BTN_PRESS | RESET_BTN_PRESS)

// error event
K_EVENT_DEFINE(error_event);
#define ERROR_EVENT BIT(0)

// define the error event
K_EVENT_DEFINE(errors);
#define ERROR_ADC_INIT      BIT(0)
#define ERROR_PWM_INIT      BIT(1)
#define ERROR_BLE_INIT      BIT(2)
#define ERROR_GPIO_INIT     BIT(3)
#define ERROR_SENSOR_READ   BIT(4)

// function declarations

const struct device *const temp_sensor = DEVICE_DT_GET_ONE(microchip_mcp9808);
void adjust_led_brightness(const struct pwm_dt_spec *pwm_led, int32_t battery_voltage_mv);


// global variables
int32_t temperature_degC;
int32_t error_code = 0;  // Global variable to store error codes
// Global variable to store the battery voltage
int32_t battery_voltage = 0;

int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv);

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

// function declarations
// define callback functions
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// define handler functions for blinking LEDs
void heartbeat_blink_handler(struct k_timer *heartbeat_timer);
void battery_blink_handler(struct k_timer *battery_blink_timer);
void out_duration_handler(struct k_timer *out_duration_timer);
void pwm_led_handler(struct k_timer *pwm_led_timer);
void temp_read_callback(struct k_timer *temp_read_timer);
void battery_measure_callback(struct k_timer *battery_measure_timer); 

// define timers
K_TIMER_DEFINE(heartbeat_timer, heartbeat_blink_handler, NULL);
K_TIMER_DEFINE(temp_read_timer, temp_read_callback, NULL);
K_TIMER_DEFINE(battery_blink_timer, battery_blink_handler, NULL);
K_TIMER_DEFINE(battery_measure_timer, battery_measure_callback, NULL);


// initialize GPIO Callback Structs
static struct gpio_callback measure_button_cb; // need one per CB
static struct gpio_callback clear_button_cb;
static struct gpio_callback reset_button_cb;


//Forward declaration of state table
static const struct smf_state states[];
const struct smf_state *smf_get_current_state(const struct smf_ctx *ctx);
void smf_set_state(struct smf_ctx *ctx, const struct smf_state *state);

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

    error_code = 0; // Reset any previous error codes

    // Start the heartbeat timer (1-second period, 50% duty cycle)
    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_SECONDS(1));

    // Measure battery voltage at startup
    int32_t battery_voltage = 0;
    if (measure_battery_voltage(&vadc_battery, &battery_voltage) != 0) {
        LOG_ERR("Failed to measure battery voltage at startup");
        error_code |= ERROR_ADC_INIT;
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    LOG_INF("Startup Battery Voltage (mV): %d", battery_voltage);

    // Check GPIO port readiness (common for all buttons and LEDs)
    if (!device_is_ready(reset_button.port)) {
        LOG_ERR("GPIO interface not ready.");
        error_code |= ERROR_GPIO_INIT;
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
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

    // Check that the ADC interface is ready
    if (!device_is_ready(vadc_battery.dev)) {
        LOG_ERR("ADC controller device(s) not ready");
    }
    // Configure the ADC channels
    err = adc_channel_setup_dt(&vadc_battery);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", err);
    }
    err = adc_channel_setup_dt(&vadc_hrate);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", err);
    }

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

    // check that the temperature sensor is ready
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

}


static void idle_run(void *o)
{   
    LOG_INF("Idle Run State");

    if (s_obj.events & RESET_BTN_PRESS) {
        LOG_INF("Reset button pressed, returning to initial state.");
        smf_set_state(SMF_CTX(&s_obj), &states[Init]);
    }  

    if (s_obj.events & GET_BTN_PRESS) {
        LOG_INF("Measure button pressed, transitioning to MEASURE state.");
        smf_set_state(SMF_CTX(&s_obj), &states[Measure]);
    }

    
}

static void idle_exit(void *o) {
    LOG_INF("Exiting Idle State");

    // Stop the battery timer
    //k_timer_stop(&battery_timer);

    // Stop the battery measurement timer
    k_timer_stop(&battery_measure_timer);
}

static void measure_entry(void *o) {
    LOG_INF("Measure Entry State");
}

static void measure_run(void *o) {
    LOG_INF("Measure Run State");

    int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
    if (ret != 0) {
        LOG_ERR("Failed to read temperature sensor");
        error_code |= ERROR_SENSOR_READ;
        smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        return;
    }

    LOG_INF("Temperature: %d °C", temperature_degC);

    // Send BLE notification with temperature
    if (send_BT_notification(current_conn, (uint8_t *)&temperature_degC, sizeof(temperature_degC)) != 0) {
        LOG_ERR("Failed to send BLE notification");
    }

    // Transition back to IDLE
    smf_set_state(SMF_CTX(&s_obj), &states[Idle]);
}


static void error_entry(void *o) {
    LOG_INF("Error Entry State");

    // Blink all LEDs at 50% duty cycle
    k_timer_start(&heartbeat_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&battery_blink_timer, K_NO_WAIT, K_MSEC(500));
    k_timer_start(&temp_read_timer, K_NO_WAIT, K_MSEC(500));

    // Send BLE notification with the error code
    extern int32_t error_code; // Make sure error_code is globally accessible
    if (send_BT_notification(current_conn, (uint8_t *)&error_code, sizeof(error_code)) != 0) {
        LOG_ERR("Failed to send BLE notification");
    }
}

static void error_run(void *o)
{   
    LOG_INF("Error Run State");
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
        [Measure] = SMF_CREATE_STATE(measure_entry, measure_run, NULL),
        [Error] = SMF_CREATE_STATE(error_entry, error_run, error_exit),
};

int main(void) {
    /*
    int ret;
 
    ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);

   
    }*/

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
        k_sleep(K_MSEC(100)); // Sleep for 100 ms
    }
}

void error_thread(void) {
    
    while (1) {
        s_obj.events = k_event_wait(&error_event, ERROR_EVENT, true, K_FOREVER);

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
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, GET_BTN_PRESS);
}
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

// unable to get current state
bool is_measuring_battery = false;

void battery_measure_callback(struct k_timer *timer_id) {
    if (is_measuring_battery) {
        LOG_DBG("Battery measurement in progress. Skipping this callback.");
        return;
    }

    is_measuring_battery = true;

    if (smf_get_current_state(SMF_CTX(&s_obj)) == &states[Idle]) {
        int32_t voltage_mv = 0;

        int ret = measure_battery_voltage(&vadc_battery, &voltage_mv);
        if (ret == 0) {
            LOG_INF("Battery Voltage: %d mV", voltage_mv);

            // Optional: Send BLE notification with battery voltage
            bluetooth_set_battery_level(voltage_mv);

            // Adjust LED brightness based on battery voltage
            adjust_led_brightness(&pwm_battery, voltage_mv);
        } else {
            LOG_ERR("Failed to measure battery voltage");
            error_code |= ERROR_ADC_INIT;
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }
    }

    is_measuring_battery = false;
}


/*
bool is_measuring_battery = false;

void battery_measure_callback(struct k_timer *timer_id) {
    if (is_measuring_battery) {
        LOG_DBG("Battery measurement in progress. Skipping this callback.");
        return;
    }

    is_measuring_battery = true;

    if (smf_get_current_state(SMF_CTX(&s_obj)) == &states[Idle]) {
        int32_t voltage_mv = 0;

        int ret = measure_battery_voltage(&vadc_battery, &voltage_mv);
        if (ret == 0) {
            LOG_INF("Battery Voltage: %d mV", voltage_mv);

            // Optional: Send BLE notification with battery voltage
            bluetooth_set_battery_level(voltage_mv);

            // Adjust LED brightness based on battery voltage
            adjust_led_brightness(&pwm_battery, voltage_mv);
        } else {
            LOG_ERR("Failed to measure battery voltage");
            error_code |= ERROR_ADC_INIT;
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }
    }

    is_measuring_battery = false;
}
*/

/*
void battery_measure_callback(struct k_timer *timer_id) {
    if (smf_get_current_state(SMF_CTX(&s_obj)) == &states[Idle]) { 
        int32_t battery_voltage = 0;

        if (measure_battery_voltage(&vadc_battery, &battery_voltage) != 0) {
            LOG_ERR("Failed to measure battery voltage");
            error_code |= ERROR_ADC_INIT;
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        } else {
            LOG_INF("Battery Voltage (mV): %d", battery_voltage);
            // Optionally, send the battery voltage over BLE

            // Update the BLE Battery Service with the measured value
            bluetooth_set_battery_level(battery_voltage);
        }
    }
} */

/*
void battery_measure_callback(struct k_timer *timer_id) {
    if (smf_get_current_state(SMF_CTX(&s_obj)) == &states[Idle]) {
        int32_t battery_voltage = 0;

        if (measure_battery_voltage(&vadc_battery, &battery_voltage) == 0) {
            LOG_INF("Battery Voltage: %d mV", battery_voltage);
            bluetooth_set_battery_level(battery_voltage); // Optional BLE notification

            // Adjust LED brightness based on battery level
            adjust_led_brightness(&pwm_battery, battery_voltage);
        } else {
            LOG_ERR("Failed to measure battery voltage");
            error_code |= ERROR_ADC_INIT;
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }
    }
}
*/


int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv) {
    struct adc_sequence sequence = {
        .channels    = BIT(adc->channel_id),
        .buffer      = voltage_mv, // Store result directly in voltage_mv
        .buffer_size = sizeof(*voltage_mv),
        .resolution  = adc->resolution,
    };

    if (!device_is_ready(adc->dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    int ret = adc_read(adc->dev, &sequence);
    if (ret != 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret;
    }

    // Convert raw ADC value to millivolts
    *voltage_mv = (*voltage_mv * adc->channel_cfg.reference) / (1 << adc->resolution);

    LOG_INF("Battery voltage: %d mV", *voltage_mv);
    return 0;
}

/*
void battery_measure_callback(struct k_timer *timer_id) {
    if (smf_get_current_state(SMF_CTX(&s_obj)) == &states[Idle]) {
        int32_t voltage_mv = 0;

        int ret = measure_battery_voltage(&vadc_battery, &voltage_mv);
        if (ret == 0) {
            LOG_INF("Battery Voltage: %d mV", voltage_mv);

            // Optional: Send BLE notification with battery voltage
            bluetooth_set_battery_level(voltage_mv); 

            // Adjust LED brightness based on battery voltage
            adjust_led_brightness(&pwm_battery, voltage_mv);
        } else {
            LOG_ERR("Failed to measure battery voltage");
            error_code |= ERROR_ADC_INIT;
            smf_set_state(SMF_CTX(&s_obj), &states[Error]);
        }
    }
}
*/

void adjust_led_brightness(const struct pwm_dt_spec *pwm_led, int32_t voltage_mv) {
    uint32_t duty_cycle = (voltage_mv * 100) / MAX_BATTERY_VOLTAGE_MV; // Scale 0–100%
    duty_cycle = (duty_cycle * PWM_PERIOD_USEC) / 100; // Convert to PWM period

    if (pwm_set_dt(pwm_led, PWM_PERIOD_USEC, duty_cycle) != 0) {
        LOG_ERR("Failed to set PWM for battery LED");
    } else {
        LOG_DBG("Battery LED brightness set to %d%%", (voltage_mv * 100) / MAX_BATTERY_VOLTAGE_MV);
    }
}



/*
int measure_battery_voltage(const struct adc_dt_spec *adc, int32_t *voltage_mv) {
    struct adc_sequence sequence = {
        .channels    = BIT(adc->channel_id),
        .buffer      = voltage_mv, // Store result directly in voltage_mv
        .buffer_size = sizeof(*voltage_mv),
        .resolution  = adc->resolution,
    };

    if (!device_is_ready(adc->dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    int ret = adc_read(adc->dev, &sequence);
    if (ret != 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret;
    }

    // Convert raw ADC value to millivolts
    *voltage_mv = (*voltage_mv * adc->channel_cfg.reference) / (1 << adc->resolution);

    LOG_INF("Measured Battery Voltage: %d mV", *voltage_mv);
    return 0;
}
*/



