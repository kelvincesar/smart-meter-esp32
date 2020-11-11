// Inclusion guard, to prevent multiple includes of the same header
#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>                     // C standard library
#define BUFFER_SIZE         2048        // Define buffer size for current and voltage
#define SM_TIMESERIE_SIZE   32          // Define smart meter main measures timeseires size

// # Buffer struct
typedef struct {
    int16_t data[BUFFER_SIZE];      // Array to store values
    uint16_t size;                  // Store array size
    uint32_t sum;                   // Store sum of values in array
} Buffer;

// # Smart Meter main data block struct
typedef struct {
	float v_rms;
	float i_rms;
	float aparent_power;
	float active_power;
    float reactive_power;
    float frequency;
    float voltage_amp;
    float voltage_freq;
    float voltage_phase;
    float current_amp;
    float current_phase;
    float current_freq;
    float fp;
    float THD_V;
    float THD_I;
    float THD_P;
    uint32_t timestamp;
} SmartMeter;

// # Smart Meter timeseries block
typedef struct {
    SmartMeter data[SM_TIMESERIE_SIZE];
    uint16_t pointer;
} TimeSerieSM;

// # Buffer functions
int buffer_push (Buffer *buf, int16_t value);
int is_buffer_full(Buffer *buf);
void buffer_clean (Buffer *buf);

// # Timeseries functions
int sm_push (SmartMeter *buf);
#endif
