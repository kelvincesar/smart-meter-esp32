// Inclusion guard, to prevent multiple includes of the same header
#ifndef SMART_METER_H
#define SMART_METER_H

// ## Include used libraries ##
#include <time.h>							// C standard library with date and time functions
#include "freertos/FreeRTOS.h"				// FreeRTOS library with RTOS functions and constants
#include "freertos/task.h"					// FreeRTOS library with task functions


//#include <freertos/semphr.h>
#include "esp_err.h"						// ESP-IDF library for error codes and error handling
#include "esp_log.h"						// ESP-IDF logging library

#include <string.h>                         // C standard library
#include <stdio.h>                          // C standard library
#include <stddef.h>                         // C standard library
#include <stdint.h>                         // C standard library
#include <stdlib.h>
#include <math.h>

#include <driver/gpio.h>                    // ESP-IDF library for gpio control
#include <driver/i2s.h>                     // ESP-IDF library for i2s drive
#include <driver/adc.h>                     // ESP-IDF library for adc drive
#include <esp_adc_cal.h>                    // ESP-IDF library for ADC calibration

// ESP-IDF library for i2s multiple adc channels 
#include "soc/dport_reg.h"                      
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "soc/efuse_reg.h"
#include "esp32/rom/lldesc.h"
#include "soc/syscon_struct.h"

// Application lib files
#include "wireless.h"                           // Wi-fi functions
#include "data_buffers.h"                     // Data buffers library
#include "goertzel.h"                       // Goertzel library
#include "moving_avg.h"                 // Moving average algo

// Signal generator library
#include <driver/dac.h>
// - Watchdog
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"



// ## DEFINES ##
// # Current sensor configuration:
#define SENS_CURR_RATIO         (1)                           // Current sensor ratio;
#define ADC_CURRENT_CHANNEL     (0)                           // Define channel used to measure current
// # Voltage sensor configuration:
#define SENS_VOLT_RATIO         (0.1884)                      // Voltage sensor ratio; 3.3/4095 * (2/3.3)*311
#define SENS_DC_OFFSET          ((2450 - 180) / 2)          // Voltage sensor DC offset value in [mV]
#define ADC_VOLTAGE_CHANNEL     (3)                           // Define channel used to measure voltage


// # ADC configuration:
#define MAIN_FREQ               (60)                          // Main signal frequency in Hz;
#define I2S_NUM                 (0)                     // I2S drive number;
#define ADC_NUM_OF_CH           (2)                                 // Number of channels that are read;
#define ADC_SAMPLE_RATE         (10240)                       // Sampling rate in Hz
#define ADC_BUFFER_SIZE         (1024)                        // I2S Buffer size (limit: 1024)

#define ADC_DMA_COUNT           (24)                          // Number of DMA buffers


#define ADC_V_REF               (3300)                        // ADC Voltage reference in mV;
#define ADC_RESOLUTION          (4096)                        // ADC resolution
#define ADC_SIGNAL_IS_AC        true                        // Define that signal read is AC;

#define ADC_GET_MEASURE(s)      (s & 0xFFF)                 // Macro used to get 12 bit part from adc read;



#define SAMPLING_WINDOW_TIME    (0.2)                         // Define the sampling window time from signal
// # Signal generator:
#define SIN_WAVE_NUM        (255)                             // Number of samples per period  

// # Goertzel
#define G_MAIN_FREQ_NUM     (5)                               // Number of frequencies to use on interpolation

// # Harmonics matrix
#define H_AMP               (0)
#define H_FREQ              (1)
#define H_PHASE             (2)
#define H_RMS               (3)




#define DEBUG_EN            (true)                                        // Enable debug mode

#define ARRAY_LEN(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))   // Macro to compute array size

// Define uint to adc sample
typedef uint16_t adc_sample_t;

// #-------------------------------------
// # Global variables 
// #-------------------------------------

// Buffer variables
Buffer buf_voltage = {.size = 0, .sum = 0};
Buffer buf_current = {.size = 0, .sum = 0};
int16_t buffer_handle = 0;

// Goertzel variables
GoertzelState g_state;


uint8_t goertzel_handle = 0;
uint8_t ipdft_handle = 0;
// # Struct to hold ipdft returns
typedef struct {
	float frequency;
    float amplitude;
    float phase_ang;
} IpDFT;



//
static const char *TAG_SM = "SM";	        // Define general log tag



// Task handles
TaskHandle_t sample_read_task_handle = NULL;	    // sample_read_task handle
TaskHandle_t signal_generator_task_handle = NULL;	// signal_generator_task handle



// Measurements sample task
void sample_read_task(void *parameters);

// Task to simulate signal
void signal_generator_task(void *parameters);

// Function to compute Ip DFT 
uint8_t do_ipdft(Buffer *buf, IpDFT *ipdft, u16_t harmonic_index);

// Function to convert adc read binary data to mV
uint16_t convert_to_voltage(uint16_t value);
void show_sm_values();

// End of the inclusion guard
#endif