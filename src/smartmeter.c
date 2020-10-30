// Include used libraries
#include "smartmeter.h"					// Smartmeter header

// Global variable

SmartMeter sm_data;                     // Store smart meter main variables data;    


static void adc_i2s_init(void)
{

    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = ADC_SAMPLE_RATE*2,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = ADC_DMA_COUNT,
        .dma_buf_len = ADC_BUFFER_SIZE,
        .use_apll = false,
    };
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    // ADC-I2S Scanning does not normal start (more badly) after the reset of ESP32. 
    // It is necessary to add a delay of 5 seconds before switching on the ADC for a reliable start, this is issue.
    vTaskDelay(5000/portTICK_RATE_MS);
    //init ADC pad
    static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_0,
        },
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_3
        }
    };

    i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));

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
    // Enable DAC CHANNEL 1 - GPIO25
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

void compute_goertzel(Buffer *buf){
    // Debug
	int64_t start_time = 0;             // Start time tick [us]
	int64_t end_time = 0;               // End time tick [us]
	float time_spent = 0;	            // Time spent in the execution [s]
    start_time = esp_timer_get_time();      // Debug



    u16_t i = 0;
    //u8_t freq_multiple = 1;             // Frequency multiple used to compute harmonics   
    u8_t epsilon;   
    float delta, delta_pi, main_freq, main_amp;

    // Frequecies and amplitudes around 60 Hz where Goertzel algorithm will be applied.
	float dft[5][2] = {      
		{0, 50},                        // {amp, freq}           
		{0, 55},                        // {amp, freq}
		{0, 60},                        // {amp, freq}
		{0, 65},                        // {amp, freq}
    	{0, 70},                        // {amp, freq}
	};
	/*float harmonics[2][2] = {                 
		{0, 0},                         // {amp, freq} - 3rd
		{0, 0}                          // {amp, freq} - 5th
	};*/

    u8_t gtz_config = 0x1;           // Config to compute applying Hanning Window and return just amplitude;

    // Compute goertzel for frequencies arround main frequency
    for (i = 0; i < G_MAIN_FREQ_NUM; i++){
        goertzel_handle = goertzel(buf, &g_state, dft[i][1], ADC_SAMPLE_RATE, gtz_config);
        //printf("g_state.DFT_m %f\n", g_state.DFT_m);
        dft[i][0] = 1 * g_state.DFT_m;
        printf("%f, %f\n", dft[i][0], dft[i][1]);
    }
    

    // ## Interpolation ##
    u8_t k = 0;         // Index from the highest amplitude found using Goertzel algo
    u16_t k_m = 0;       // Index of DFT
    // # Find highest amplitude in DFT vector:
    for (i = 1; i < G_MAIN_FREQ_NUM-1; i++){
        if(dft[i][0] > dft[i-1][0] && dft[i][0] > dft[i+1][0]){
            k = i;
        }
    }
    
    // # Compute IpDFT algorithm for Hanning Window
    if(k > 0){
        epsilon = (dft[k+1][0] >= dft[k-1][0]) ? 1 : -1;
        delta = epsilon * (2 * dft[k+epsilon][0] - dft[k][0]) / (dft[k][0] + dft[k+epsilon][0]);
        k_m = (u16_t) floor((0.5 + (buf->size * dft[k][1] / ADC_SAMPLE_RATE)));

        main_freq = (k_m + delta)* (1 / SAMPLING_WINDOW_TIME);
        delta_pi = M_PI * delta;
        main_amp = fabs (dft[k][0]);
       
        main_amp *= fabs ( (float) (delta_pi) / (float) sinf(delta_pi));
        
        main_amp *= fabs ( (float) (delta*delta - 1));
    
        // Debug
        printf("\nMaior amplitude = %f, k = %d\n", dft[k][0], k);
        printf("- Epsilon: %d \n", epsilon);
        printf("- delta: %f \n", delta);
        printf("- k_m: %d \n", k_m);
        printf("- delta_pi: %f \n", delta_pi);
        printf("- Main frequency: %f\n", main_freq);
        printf("- Main amplitude: %f\n", main_amp);
    }
    
    // Obtain current time
    end_time = esp_timer_get_time();

    // Calculating time spent
    time_spent = (float)(end_time - start_time) / 1000000;
    printf("compute_goertzel execution time = %f us\n", time_spent);

    /*
    // Calculate goertzel for 3rd and 5th hamornics
    for (i = 0; i < G_NUM_OF_HARM; i++){
        // Compute harmonic frequency based on main frequency multiple
        harmonics[i][1] = (float) main_freq * freq_multiple;

        // Apply goertzel to the frequency found
        goertzel_handle = goertzel(&buf, &g_state, harmonics[i][1], ADC_SAMPLE_RATE);
        printf("g_state.DFT_m %f\n", g_state.DFT_m);
        harmonics[i][0] = SENS_VOLT_RATIO * g_state.DFT_m;

        // Calculate frequency mutiple to next iteration
        freq_multiple = freq_multiple+2;

    }
    */

}
// Compute rms and remove dc offset
void do_rms(Buffer *voltage, Buffer *current){
    u32_t sum_voltage = 0;
    u32_t sum_current = 0;

    // # Calcula o valor de offset do sinal
    u16_t voltage_offset = (voltage->max + voltage->min) / 2;
    u16_t current_offset = (current->max + current->min) / 2;
    
    // Remove o offset do sinal e computa o somatório da sua potência
    for (u16_t i = 0; i < BUFFER_SIZE; i++){
        // Atualiza o valor do buffer removendo o offset
        voltage->data[i] -= voltage_offset;
        current->data[i] -= current_offset;
        // Remove o offset do valor de tensão e atualiza no buffer
        sum_voltage += (uint32_t) (voltage->data[i]*voltage->data[i]);
        sum_current += (uint32_t) (current->data[i]*current->data[i]);
    }
    // Cálculo do RMS
    sm_data.v_rms = fast_sqrt(sum_voltage / BUFFER_SIZE);
    sm_data.i_rms = fast_sqrt(sum_current / BUFFER_SIZE);

    // # Debug
    printf("==================\n");
    printf("v_rms = %f\n", sm_data.v_rms );
    printf("i_rms = %f\n", sm_data.i_rms );
    printf("voltage_offset = %d\n", voltage_offset );
    printf("current_offset = %d\n", current_offset );
    printf("==================\n");
}


// # Verify buffers state
bool buffers_verify(){
    return (buf_voltage.size >= BUFFER_SIZE || buf_current.size >= BUFFER_SIZE || buffer_handle == -1) ? true : false;
}


/*
    Apply linear equation to convert binary to mV
    y = 0.8165x + 143.21
*/ 
uint16_t convert_to_voltage(uint16_t value){
    return (float) (value * 0.8165 + 143.21);
}

void sample_read_task(void *parameters)
{
    // # Define ADC Characteristics for convertion
    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_V_REF, &cal);



    // # Declare I2S variables
    adc_sample_t buf[ADC_BUFFER_SIZE];  // Raw buffer with measures from ADC;  
    size_t measures_read;               // Receive number of measures read from I2S - ADC
    
    // # Declare local variables
    u16_t sample = 0;                   // Generic variable to momently store sample value converted to mV
    u16_t loop_ctrl = 0;                // Used to control while loop

    // # Start ADC I2S Setup
    ESP_LOGI(TAG_SM, "# Starting ADC configuration");
    adc_i2s_init();
    ESP_LOGI(TAG_SM, "# ADC setup successfuly");
    buffer_clean(&buf_current);
    buffer_clean(&buf_voltage);
    ESP_LOGI(TAG_SM, "# Buffers cleaned");

    while (loop_ctrl == 0) {
        // # Read data from i2s.      
        esp_err_t err = i2s_read(I2S_NUM, buf, sizeof(buf), &measures_read, portMAX_DELAY);

        if (err != ESP_OK)
        {
            printf("i2s_read: %d\n", err);

        }
        // # Compute vector size
        measures_read /= sizeof(adc_sample_t);  

        // # Loop on measures to convert and save on buffer
        for (u16_t i = 0; i < measures_read; i ++)
        {
            // Get channels offset on buffer
            switch (buf[i] >> 12){
                case ADC_CURRENT_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));

                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_current, sample);
                    break;
                case ADC_VOLTAGE_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));

                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_voltage, sample);
                    break; 
                default:
                    ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[0]>>12);

            }
        }
        if(buf_voltage.size != buf_current.size){
            ESP_LOGE(TAG_SM, "# Voltage buffer and current doesn't have the same size [%d/%d]", buf_voltage.size, buf_current.size);
        } else {

            // Verifica se os buffers chegaram ao limite
            if(buffers_verify()){
                // # Compute RMS and remove DC offset from signals;
                do_rms(&buf_voltage, &buf_current);
                // # Apply goertzel to voltage signal
                compute_goertzel(&buf_voltage);


                for(uint16_t x = 0; x < buf_voltage.size ; x++){
                    printf("%u,%d;", x, buf_voltage.data[x]);
                }
                printf("\n\n");
                buffer_clean(&buf_voltage);
                // # Apply goertzel to current signal
                compute_goertzel(&buf_current);
                buffer_clean(&buf_current);

                // # Debug 
                loop_ctrl = 1;
            }
        }
            

    }
    // Debug
    /*
    int s = 0;
    s = socket_tcp_connect();
    if(s >= 0){
        uint8_t* tcp_buffer;
        tcp_buffer = (uint8_t*) calloc(8192, sizeof(uint8_t)); 

        for(uint16_t x = 0; x < buf_voltage.size ; x++){
            if(x % 2 == 0){
                tcp_buffer[x] = buf_voltage.data[x] & 0xFF;
            } else { 
                tcp_buffer[x] = buf_voltage.data[x] >> 8; 
            }

            //printf("Buffer [%d] - %d (%d)\n", x, tcp_buffer[x], buf_voltage.data[x]);
        }
            //printf("Buffer [%d] = %d\n", x, buf_voltage.data[x]);
        socket_tcp_send_data(s, tcp_buffer, 8192);
        free(tcp_buffer); 
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
    }
    socket_tcp_close_connection(s);
    */

    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    vTaskDelete(NULL);
    
}

void app_main(void)
{


    //setup_wifi();



    // Local variables
	BaseType_t task_create_ret;				// Return of task create
	// Print info to show main task is running
	ESP_LOGI(TAG_SM, "# Running app_main in core %d", xPortGetCoreID());

    /**/
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
	vTaskDelay(10000 / portTICK_PERIOD_MS);
    
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating sample_read_task");

    
	// Create task to write audio to wav file in SD card
	task_create_ret = xTaskCreatePinnedToCore(
		sample_read_task,					// Function executed in the task
		"SRT",					            // Task name (for debug)
		32000,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&sample_read_task_handle,			// Used to pass back a handle by which the created task can be referenced
		0);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating sample_read task"); }


    
    
}
