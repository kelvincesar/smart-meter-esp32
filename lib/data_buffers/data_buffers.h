// Inclusion guard, to prevent multiple includes of the same header
#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>                     // C standard library
#define BUFFER_SIZE         2048        // Define buffer size for current and voltage
#define SM_TIMESERIE_SIZE   50          // Define smart meter main measures timeseires size
#define WINDOW_LENGTH       (0.2)       // Define smart meter window length (delta T)
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
    float E_P;
    float E_Q;
} SmartMeter;



// # Smart Meter payload block
typedef struct {
    float vRms;
    float iRms;
    float sPower;
    float aPower;
    float rPower;
    float freq;
    float fp;
    float THDV;
    float THDI;
    float THDP;
    float H_vRms;
    float H_iRms;
    float H_sPower;
    float H_aPower;
    float H_rPower;
    float H_freq;
    float H_fp;
    float H_THDV;
    float H_THDI;
    float H_THDP;
    float L_vRms;
    float L_iRms;
    float L_sPower;
    float L_aPower;
    float L_rPower;
    float L_freq;
    float L_fp;
    float L_THDV;
    float L_THDI;
    float L_THDP;
    float vAmp;
    float vFreq;
    float vPhase;
    float cAmp;
    float cPhase;
    float cFreq;
    float E_P;
    float E_Q;
} SM_Payload;

// # Smart Meter timeseries block
typedef struct {
    SM_Payload data;
    uint16_t counter;
} TimeSerieSM;
// # Buffer functions
int buffer_push (Buffer *buf, int16_t value);
int is_buffer_full(Buffer *buf);
void buffer_clean (Buffer *buf);

// # Timeseries functions
int sm_push (SmartMeter *data);
void sm_compute_payload(SM_Payload *payload);

void sm_init_data ();
uint16_t sm_get_counter();
#endif
