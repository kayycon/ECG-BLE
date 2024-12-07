#ifndef HEARTRATE_H
#define HEARTRATE_H

#include <stdint.h>

// Define the sample rate 
#define ECG_SAMPLE_RATE 100

// Function prototypes
void signal(int16_t *data, int size, int window_size);
int detect_peaks(int16_t *data, int size, int *peak_position, int max_peaks);
int calc_BPM(int *peak_position, int num_peaks);

#endif // HEARTRATE_H