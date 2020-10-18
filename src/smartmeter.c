// Include used libraries
#include "smartmeter.h"					// Smartmeter header


// # Current sensor configuration:
#define S_AMP_RATIO         1                               // Current sensor ratio;
#define ADC_CURRENT_CHANNEL 6                               // Define channel used to measure current
// # Voltage sensor configuration:
#define S_VOL_RATIO         1                               // Voltage sensor ratio;
#define ADC_VOLTAGE_CHANNEL 7                               // Define channel used to measure voltage


// # ADC configuration:
#define MAIN_FREQ           60                              // Main signal frequency in Hz;
#define I2S_NUM 0                                           // I2S drive number;
#define ADC_NUM_OF_CH 2                                     // Number of channels that are read;
#define ADC_SAMPLING_RATE   61440                           // Sampling rate in Hz
#define ADC_BUFFER_SIZE     (ADC_SAMPLING_RATE/MAIN_FREQ)   // Calculate buffer size (limit: 1024)

#define ADC_DMA_COUNT       8                               // Number of DMA buffers

#define ADC_GET_MEASURE(s)  (s & 0xFFF)                     // Macro used to get 12 bit part from adc read;
#define ADC_V_REF           3300                            // ADC Voltage reference in mV;
#define ADC_RESOLUTION      4096                            // ADC resolution
#define ADC_SIGNAL_IS_AC    true                            // Define that signal read is AC;
#define ADC_SIGNAL_OFFSET   2048                            // Define offset for AC signal;

// # Signal generator:
#define SIN_WAVE_NUM        255                             // Number of samples per period  

// Redefine array para zero
#define ZERO_ANY(T, a, n) do{\
   T *a_ = (a);\
   size_t n_ = (n);\
   for (; n_ > 0; --n_, ++a_)\
     *a_ = (T) { 0 };\
} while (0)

#define ENABLE_DEBUG                                        // Enable debug mode


static void adc_i2s_init(void)
{
    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = ADC_SAMPLING_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL3,
        .dma_buf_count = ADC_DMA_COUNT,
        .dma_buf_len = ADC_BUFFER_SIZE,
        .use_apll = false,
    };
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

    //init ADC pad
    static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_6,
        },
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_7
        }
    };

    i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));


    // Delay necessary or reliable start;
    vTaskDelay(1000/portTICK_RATE_MS);
}
// # ----------------------------------------------
// # SIGNAL GENERATOR 
// # ----------------------------------------------


void signal_generator_task(void *parameters)
{
    static uint8_t table_sin_wave[SIN_WAVE_NUM] = {
    // Sin wave
        128,131,134,137,141,144,147,150,153,156,159,162,165,168,171,174,
        177,180,183,186,188,191,194,196,199,202,204,207,209,212,214,216,218,221,223,225,227,229,231,233,234,236,
        238,239,241,242,243,245,246,247,248,249,250,251,252,253,253,254,254,255,255,255,255,255,255,
        255,255,255,255,254,254,253,253,252,251,251,250,249,248,247,245,244,243,241,240,238,
        237,235,233,232,230,228,226,224,222,220,217,215,213,210,208,205,203,200,198,195,192,190,187,184,181,178,176,
        173,170,167,164,161,158,155,151,148,145,142,139,136,133,130,126,123,120,117,114,111,108,105,101,
        98,95,92,89,86,83,80,78,75,72,69,66,64,61,58,56,53,51,48,46,43,41,39,36,34,32,30,28,26,24,23,21,
        19,18,16,15,13,12,11,9,8,7,6,5,5,4,3,3,2,2,1,1,1,1,1,1,1,1,1,1,2,2,3,3,4,5,6,7,8,9,10,
        11,13,14,15,17,18,20,22,23,25,27,29,31,33,35,38,40,42,44,47,49,52,54,
        57,60,62,65,68,70,73,76,79,82,85,88,91,94,97,100,103,106,109,112,115,119,122,125
    };

    uint16_t i = 0;
    // Enable DAC CHANNEL 1
    dac_output_enable(DAC_CHANNEL_1);
    int64_t start_time = 0;
    while (true){

        // To not hog the CPU
        if( (esp_timer_get_time() - start_time >= 60) || start_time == 0){   // Check delta in uS
            dac_output_voltage(DAC_CHANNEL_1, table_sin_wave[i]);
            i++;
            if (i >= SIN_WAVE_NUM) i = 0;

            start_time = esp_timer_get_time();
        }
        // Feed watchdog
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;

    }
}



void sample_read_task(void *parameters)
{
    	// Local variables
	int64_t start_time = 0;        			// Start time tick [us]
	int64_t end_time = 0;          			// End time tick [us]
	float time_spent = 0;					// Time spent in the execution [s]
    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_V_REF, &cal);

    adc_i2s_init();

    adc_sample_t buf[ADC_BUFFER_SIZE];



    //read data from I2S bus, in this case, from ADC.
    size_t measures_read;
    // Obtain current time
    //i2s_adc_enable(I2S_NUM);
    //while (true) {
    for (int k = 0; k < 1; ++k) {
        
        esp_err_t err = i2s_read(I2S_NUM, buf, sizeof(buf), &measures_read, portMAX_DELAY);

        if (err != ESP_OK)
        {
            printf("i2s_read: %d\n", err);
        }

        //DBG_ASSERT(measures_read % sizeof(adc_sample_t) == 0);
        measures_read /= sizeof(adc_sample_t);

        int32_t voltage_converted = 0;  // Voltage read;
        int32_t current_converted = 0;  // Current read;


        uint32_t sum_voltage = 0;   // Voltage sum;
        uint32_t sum_current = 0;   // Current sum;
        uint32_t sum_active_power  = 0;     // Power sum;

        float rms_voltage = 0;
        float rms_current = 0;
        float active_power = 0;

        u16_t current_ch_off = 0;   // Offset applied to get current measure from buffer
        u16_t voltage_ch_off = 0;   // Offset applied to get voltage measure from buffer
        u16_t num_samples = measures_read/2;
        start_time = esp_timer_get_time();
        // Get channels offset on buffer
        switch (buf[0] >> 12){
            case ADC_CURRENT_CHANNEL:
                current_ch_off = 0;
                voltage_ch_off = 1;
            break;
            case ADC_VOLTAGE_CHANNEL:
                current_ch_off = 1;
                voltage_ch_off = 0;                
            break; 

            default:
                ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[0]>>12);

        }

        // Loop on measures to calculate sum square for RMS value
        for (u16_t i = 0; i < measures_read; i += ADC_NUM_OF_CH)
        {
            voltage_converted = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i+voltage_ch_off]), &cal);  // - ADC_V_REF / 2       
            current_converted = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i+current_ch_off]), &cal);  // - ADC_V_REF / 2
            

            sum_voltage += (uint32_t) (voltage_converted*voltage_converted);
            sum_current += (uint32_t) (current_converted*current_converted);
            sum_active_power += (uint32_t) (voltage_converted*current_converted);
        }

        rms_voltage = (float) sqrt(sum_voltage / num_samples);
        rms_current = (float) sqrt(sum_current / num_samples);
        active_power = (float) sum_active_power / num_samples;
        // Obtain current time
        end_time = esp_timer_get_time();
        //printf("[measures_read VARIABLE] = %d\n", measures_read);
        // Calculating time spent
        time_spent = (float)(end_time - start_time) / 1000000;
        // Printing after execution
        printf("execution time = %f s\n", time_spent);

        printf("Voltage RMS: %f mV\n", rms_voltage);
        printf("Current RMS: %f mV\n", rms_current);
        printf("Active Power: %f mV\n", active_power);

    }
    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    vTaskDelete(NULL);
    
}

void app_main(void)
{
    // start the main task
    //xTaskCreate(&connect_to_wifi, "con2WIFI", 2048, NULL, 5, NULL);
    //set RGB PINs
    //gpio_pad_select_gpio(LED_R);
    //gpio_pad_select_gpio(LED_G);
    //gpio_pad_select_gpio(LED_B);
    
    //gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    //gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    //gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    //wifi_init_softap();


    // Local variables
	BaseType_t task_create_ret;				// Return of task create
	// Print info to show main task is running
	ESP_LOGI(TAG_SM, "# Running app_main in core %d", xPortGetCoreID());

    /*
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating signal_generator task");

	// Create task to generate sine wave
	task_create_ret = xTaskCreatePinnedToCore(
		signal_generator_task,				// Function executed in the task
		"SGT",					            // Task name (for debug)
		4096,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&signal_generator_task_handle,		// Used to pass back a handle by which the created task can be referenced
		1);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating signal_generator task"); }
	// Delay 
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    */
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating sample_read_task task");

    
	// Create task to write audio to wav file in SD card
	task_create_ret = xTaskCreatePinnedToCore(
		sample_read_task,					// Function executed in the task
		"SRT",					            // Task name (for debug)
		4096,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&sample_read_task_handle,			// Used to pass back a handle by which the created task can be referenced
		0);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating sample_read task"); }


    
    
}
