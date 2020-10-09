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
#include "wifi.h"                           // Wi-fi functions

// Signal generator library
#include <driver/dac.h>
// - Watchdog
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
// #-------------------------------------
// # Global variables 
// #-------------------------------------

static const char *TAG_SM = "SM";	        // Define log tag

// Task handles
TaskHandle_t sample_read_task_handle = NULL;	    // sample_read_task handle
TaskHandle_t signal_generator_task_handle = NULL;	// signal_generator_task handle

// Define uint to adc sample
typedef uint16_t adc_sample_t;

// Measurements sample task
void sample_read_task(void *parameters);

// Task to simulate signal
void signal_generator_task(void *parameters);



// End of the inclusion guard
#endif