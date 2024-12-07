#include "heartrate.h"
#include <string.h> // For memcpy

void signal(int16_t *data, int size, int duration) {
    for (int i = 0; i < size; i++) {
        int sum = 0;
        int count = 0;
        for (int j = -duration / 2; j <= duration / 2; j++) {
            if (i + j >= 0 && i + j < size) {
                sum += data[i + j];
                count++;
            }
        }
        data[i] = sum / count;
    }
}

int detect_peaks(int16_t *data, int size, int *peak_position, int max_peaks) {
    int num_peaks = 0;
    for (int i = 1; i < size - 1; i++) {
        if (data[i] > -4 && data[i] > data[i - 1] && data[i] > data[i + 1]) {//Need to tune the 200 threshold
            if (num_peaks < max_peaks) {
                peak_position[num_peaks++] = i;
            }
        }
    }
    return num_peaks;
}

int calc_BPM(int *peak_position, int num_peaks) {
    if (num_peaks < 2) {
        return 0; // Not enough peaks detected
    }

    // Calculate time differences between successive peaks
    int total_time_diff = 0;
    for (int i = 1; i < num_peaks; i++) {
        total_time_diff += (peak_position[i] - peak_position[i - 1]);
    }

    // Average time difference in samples
    int avg_time_diff = total_time_diff / (num_peaks - 1);

    // Convert to BPM
    return (60 * ECG_SAMPLE_RATE) / avg_time_diff;
}