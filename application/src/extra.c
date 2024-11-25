#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define ECG_BUFFER_SIZE 256
#define SAMPLE_RATE 250 // Sample rate in Hz
#define HEART_RATE_BUFFER_SIZE 10 // Number of heart rate samples for averaging
#define THRESHOLD 0.5 // Threshold for peak detection
#define DC_DRIFT_REMOVAL_FACTOR 0.01 // Factor for DC removal

static float ecg_buffer[ECG_BUFFER_SIZE]; // Buffer for raw ECG data
static float filtered_ecg[ECG_BUFFER_SIZE]; // Buffer for filtered ECG data
static uint32_t heart_rate_samples[HEART_RATE_BUFFER_SIZE]; // Buffer for heart rate samples
static uint32_t sample_index = 0;
static uint32_t last_peak_time = 0;
static uint32_t heart_rate_index = 0;
static float dc_offset = 0.0;

// Function to apply a simple high-pass filter to remove DC drift
void high_pass_filter(float *input, float *output, size_t size) {
    for (size_t i = 0; i < size; i++) {
        dc_offset += DC_DRIFT_REMOVAL_FACTOR * (input[i] - dc_offset);
        output[i] = input[i] - dc_offset; // Subtract the DC offset
    }
}

// Function to detect R-peaks in the filtered ECG signal
bool detect_r_peak(float *signal, size_t size, uint32_t *peak_time) {
    for (size_t i = 1; i < size - 1; i++) {
        // Simple peak detection
        if (signal[i] > THRESHOLD && signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
            *peak_time = i; // Store the peak time
            return true; // Peak detected
        }
    }
    return false; // No peak detected
}

// Function to calculate heart rate from detected peaks
void calculate_heart_rate(uint32_t current_time) {
    if (last_peak_time > 0) {
        uint32_t interval = current_time - last_peak_time; // Time between peaks in ms
        uint32_t heart_rate = (60 * 1000) / interval; // Convert to beats per minute

        // Store the heart rate sample
        heart_rate_samples[heart_rate_index] = heart_rate;
        heart_rate_index = (heart_rate_index + 1) % HEART_RATE_BUFFER_SIZE;

        // Calculate average heart rate
        uint32_t sum = 0;
        for (size_t i = 0; i < HEART_RATE_BUFFER_SIZE; i++) {
            sum += heart_rate_samples[i];
        }
        uint32_t average_heart_rate = sum / HEART_RATE_BUFFER_SIZE;

        // Log the average heart rate
        LOG_INF("Average Heart Rate: %d bpm", average_heart_rate);
    }
}


#define ECG_BUFFER_SIZE 256
#define SAMPLE_RATE 250 // Sample rate in Hz
#define HEART_RATE_BUFFER_SIZE 10 // Number of heart rate samples for averaging
#define THRESHOLD 0.5 // Threshold for peak detection
#define DC_DRIFT_REMOVAL_FACTOR 0.01 // Factor for DC removal

static float ecg_buffer[ECG_BUFFER_SIZE]; // Buffer for raw ECG data
static float filtered_ecg[ECG_BUFFER_SIZE]; // Buffer for filtered ECG data
static uint32_t heart_rate_samples[HEART_RATE_BUFFER_SIZE]; // Buffer for heart rate samples
static uint32_t sample_index = 0;
static uint32_t last_peak_time = 0;
static uint32_t heart_rate_index = 0;
static float dc_offset = 0.0;

// Function to apply a simple high-pass filter to remove DC drift
void high_pass_filter(float *input, float *output, size_t size) {
    for (size_t i = 0; i < size; i++) {
        dc_offset += DC_DRIFT_REMOVAL_FACTOR * (input[i] - dc_offset);
        output[i] = input[i] - dc_offset; // Subtract the DC offset
    }
}

// Function to detect R-peaks in the filtered ECG signal
bool detect_r_peak(float *signal, size_t size, uint32_t *peak_time) {
    for (size_t i = 1; i < size - 1; i++) {
        // Simple peak detection
        if (signal[i] > THRESHOLD && signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
            *peak_time = i; // Store the peak time
            return true; // Peak detected
        }
    }
    return false; // No peak detected
}

// Function to calculate heart rate from detected peaks
void calculate_heart_rate(uint32_t current_time) {
    if (last_peak_time > 0) {
        uint32_t interval = current_time - last_peak_time; // Time between peaks in ms
        uint32_t heart_rate = (60 * 1000) / interval; // Convert to beats per minute

        // Store the heart rate sample
        heart_rate_samples[heart_rate_index] = heart_rate;
        heart_rate_index = (heart_rate_index + 1) % HEART_RATE_BUFFER_SIZE;

        // Calculate average heart rate
        uint32_t sum = 0;
        for (size_t i = 0; i < HEART_RATE_BUFFER_SIZE; i++) {
            sum += heart_rate_samples[i];
        }
        uint32_t average_heart_rate = sum / HEART_RATE_BUFFER_SIZE;

        // Log the average heart rate
        LOG_INF("Average Heart Rate: %d bpm", average_heart_rate);
    }
}

// Main function to process ECG data
void process_ecg_data(float *raw_ecg_data, size_t size) {
    
    // Step 1: High-pass filter to remove DC drift
    high_pass_filter(raw_ecg_data, filtered_ecg, size);

    // Step 2: Detect R-peaks
    uint32_t peak_time;
    if (detect_r_peak(filtered_ecg, size, &peak_time)) {
        // Step 3: Calculate heart rate
        calculate_heart_rate(peak_time);
        last_peak_time = peak_time; // Update last peak time
    }
}