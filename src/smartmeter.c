// Include used libraries
#include "smartmeter.h"					// Smartmeter header

// Global variable

SmartMeter sm_data;                     // Store smart meter main variables data;    
IpDFT ip_dft_data;                      // Store IpDFT algorithm return values

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

uint8_t do_thd(){
    // Compute the following parameters from harmonic = 2 ... 7;
    // - Frequency
    // - Amplitude
    // - Phase

    u16_t i = 0;
    // Config to compute applying complete Goertzel calculus;
    u8_t gtz_config = 0x2;        
    
    
    float delta_phi = 0;
    float main_active_power = 0;
    float main_cosphi = 0;

    float harm_active_power = 0;
    float aux_thd_i_v  =   0;           // Store Sqrt(1 - THD_v^2)*Sqrt(1 - THD_i^2)
    // {amp, freq, phase, rms}
    float v_harm_data[7][4] = {
		{sm_data.voltage_amp, sm_data.voltage_freq, sm_data.voltage_phase}, // Main 
		{0, 0, 0, 0},          // 2nd order                 
		{0, 0, 0, 0},          // 3rd order
		{0, 0, 0, 0},          // 4th order
    	{0, 0, 0, 0},          // 5th order
    	{0, 0, 0, 0},          // 6th order
    	{0, 0, 0, 0},          // 7th order
    };
    float i_harm_data[7][4] = {
		{sm_data.current_amp, sm_data.current_freq, sm_data.current_phase}, // Main 
		{0, 0, 0, 0},          // 2nd order
		{0, 0, 0, 0},          // 3rd order
		{0, 0, 0, 0},          // 4th order
    	{0, 0, 0, 0},          // 5th order
    	{0, 0, 0, 0},          // 6th order
    	{0, 0, 0, 0},          // 7th order
    };
    // # Compute voltage and current RMS considering a pure sine wave
    float v_main_rms = (float) (v_harm_data[0][H_AMP] / M_SQRT2);
    float i_main_rms = (float) (i_harm_data[0][H_AMP] / M_SQRT2);

    // # Compute values for the main frequency
    delta_phi = (float) (v_harm_data[0][H_PHASE] - i_harm_data[0][H_PHASE]);
    main_cosphi = cosf(delta_phi);

    sm_data.active_power   = (float) (v_main_rms * i_main_rms * main_cosphi);
    main_active_power = sm_data.active_power;  // used to compute Pn/P1

    sm_data.reactive_power = (float) (v_main_rms * i_main_rms * sinf(delta_phi));

    sm_data.THD_V = 0;
    sm_data.THD_I = 0;

    //printf("\n Debug delta_phi=%f, main_cosphi=%f, v_main_rms=%f, i_main_rms=%f, main_active_power=%f, sm_data.reactive_power=%f\n", delta_phi, main_cosphi, v_main_rms, i_main_rms,main_active_power, sm_data.reactive_power);

    // # Compute harmonics amplitude and phase using Goertzel.
    // Start from i=1 because first position stores main signal values.
    for (i = 1; i < ARRAY_LEN(v_harm_data) - 1; i++){
        printf("- Harmonic number [%d] \n", i+1);
        // # Compute harmonic frequency
        v_harm_data[i][H_FREQ] = (float) (v_harm_data[0][H_FREQ] * (i+1));
        i_harm_data[i][H_FREQ] = (float) (i_harm_data[0][H_FREQ] * (i+1));

        // # Apply goertzel to current and voltage harmonic frequency
        goertzel_handle = goertzel(&buf_voltage, &g_state, v_harm_data[i][H_FREQ], ADC_SAMPLE_RATE, gtz_config);
        v_harm_data[i][H_AMP] = 1 * g_state.DFT_m;      // # Store Goertzel magnitude
        v_harm_data[i][H_PHASE] = g_state.DFT_arg;      // # Store Goertzel angle

        goertzel_handle = goertzel(&buf_current, &g_state, i_harm_data[i][H_FREQ], ADC_SAMPLE_RATE, gtz_config);
        i_harm_data[i][H_AMP] = 1 * g_state.DFT_m;      // # Store Goertzel magnitude
        i_harm_data[i][H_PHASE] = g_state.DFT_arg;      // # Store Goertzel angle

        // Compute harmonic RMS value
        v_harm_data[i][H_RMS] = (float) (v_harm_data[i][H_AMP] / M_SQRT2);
        i_harm_data[i][H_RMS] = (float) (i_harm_data[i][H_AMP] / M_SQRT2);

        // # Compute active and reactive power for the current harmonic
        delta_phi = (float) (v_harm_data[0][H_PHASE] - i_harm_data[0][H_PHASE]);    // Use aux to compute delta phi
        harm_active_power = (float) (v_harm_data[i][H_RMS] * i_harm_data[i][H_RMS]);// Used as aux to compute V*I
        sm_data.reactive_power += (float) (harm_active_power * sinf(delta_phi));    // Sum harmonic piece into total active power 

        harm_active_power *= cosf(delta_phi);                                       // Multiply by cos(phi), now it is active power 
        sm_data.active_power += harm_active_power;                                  // Sum harmonic piece into total active power 
        

        // Compute sum of (Vrms_n / Vrms_1) and (Irms_n / Irms_1)
        sm_data.THD_V += (float) (v_harm_data[i][H_RMS] / v_main_rms);
        sm_data.THD_I += (float) (i_harm_data[i][H_RMS] / i_main_rms);
        sm_data.THD_P += (float) (harm_active_power / main_active_power);
         


        // Compute 
        // Compute active and reactive power (P and Q)
    }



    // # Compute THD_v and THD_i
    sm_data.THD_V = fast_sqrt(sm_data.THD_V);
    sm_data.THD_I = fast_sqrt(sm_data.THD_I);

    // # Compute Sqrt(1 - THD_v^2)*Sqrt(1 - THD_i^2) 
    aux_thd_i_v  = (float) (fast_sqrt(1+sm_data.THD_V * sm_data.THD_V));
    aux_thd_i_v *= (float) (fast_sqrt(1+sm_data.THD_I * sm_data.THD_I));
    
    // # Compute aparent power
    sm_data.aparent_power  = (float) (v_main_rms * i_main_rms * aux_thd_i_v);

    // # Compute FP
    sm_data.fp  = (float) (main_cosphi * (1 + sm_data.THD_P));
    sm_data.fp /= (float) (aux_thd_i_v);

    sm_data.frequency = (sm_data.voltage_freq + sm_data.current_freq) / 2;

    show_sm_values();
    return 1;
}
void show_sm_values(){
    printf("\n===========[ Smart Meter values ]===========\n ");
    printf(" - v_rms           = %f\n", sm_data.v_rms);
    printf(" - i_rms           = %f\n", sm_data.i_rms);         
    printf(" - aparent_power   = %f\n", sm_data.aparent_power);
    printf(" - active_power    = %f\n", sm_data.active_power);
    printf(" - reactive_power  = %f\n", sm_data.reactive_power);
    printf(" - frequency       = %f\n", sm_data.frequency);
    printf(" - voltage_amp     = %f\n", sm_data.voltage_amp);   
    printf(" - voltage_freq    = %f\n", sm_data.voltage_freq); 
    printf(" - voltage_phase   = %f\n", sm_data.voltage_phase);
    printf(" - current_amp     = %f\n", sm_data.current_amp);
    printf(" - current_phase   = %f\n", sm_data.current_phase); 
    printf(" - current_freq    = %f\n", sm_data.current_freq);
    printf(" - fp              = %f\n", sm_data.fp);
    printf(" - THD_V           = %f\n", sm_data.THD_V);         
    printf(" - THD_I           = %f\n", sm_data.THD_I);       
    printf(" - THD_P           = %f\n", sm_data.THD_P);
    printf("============================================\n");      
}
uint8_t do_ipdft(Buffer *buf, IpDFT *ipdft){


    u16_t i = 0;
    u8_t epsilon;   
    float delta, delta_pi;

    // Frequecies and amplitudes around 60 Hz where Goertzel algorithm will be applied.
	float dft[5][3] = {      
		{0, 50, 0},                        // {amp, freq, phase}           
		{0, 55, 0},                        // {amp, freq, phase}
		{0, 60, 0},                        // {amp, freq, phase}
		{0, 65, 0},                        // {amp, freq, phase}
    	{0, 70, 0},                        // {amp, freq, phase}
	};

    // Config to compute applying Hanning Window and complete Goertzel calculus;
    u8_t gtz_config = 0x3;           

    // Compute goertzel for frequencies arround main frequency
    for (i = 0; i < G_MAIN_FREQ_NUM; i++){
        // # Apply Goertzel to current frequency
        goertzel_handle = goertzel(buf, &g_state, dft[i][1], ADC_SAMPLE_RATE, gtz_config);
        
        dft[i][0] = 1 * g_state.DFT_m;  // # Store Goertzel magnitude
        dft[i][2] = g_state.DFT_arg;    // # Store Goertzel angle
        //printf("%f, %f\n", dft[i][0], dft[i][1]);
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
        //delta = epsilon * (2 * dft[k+epsilon][0] - dft[k][0]) / (dft[k][0] + dft[k+epsilon][0]);
        delta =  (float) (dft[k + epsilon][0] - dft[k - epsilon][0]);
        delta /= (float) (dft[k - epsilon][0] + 2* dft[k][0] + dft[k + epsilon][0]);
        delta *= (float) (2 * epsilon);
        // # Delta deve estar entre 0.5 <= δ < 0.5 
        if(delta >= 0.5) delta = 0.5;
        if(delta < -0.5) delta = -0.5;
        // # Compute k_m index
        k_m = (u16_t) floor((0.5 + (buf->size * dft[k][1] / ADC_SAMPLE_RATE)));
        // # Interpolate to get main frequency
        ipdft->frequency = (k_m + delta)* (1 / SAMPLING_WINDOW_TIME);

        // # Compute real signal amplitude
        delta_pi = M_PI * delta;
        ipdft->amplitude = fabs (dft[k][0]);
        ipdft->amplitude *= fabs ( (float) (delta_pi) / (float) sinf(delta_pi));
        ipdft->amplitude *= fabs ( (float) (delta*delta - 1));

        // # Compute signal phase
        ipdft->phase_ang = dft[k][2] - delta_pi;

        // Debug
        printf("\nMaior amplitude = %f, k = %d\n", dft[k][0], k);
        printf("- Epsilon: %d \n", epsilon);
        printf("- delta: %f \n", delta);
        printf("- k_m: %d \n", k_m);
        printf("- delta_pi: %f \n", delta_pi);
        printf("- Main frequency: %f\n", ipdft->frequency);
        printf("- Main amplitude: %f\n", ipdft->amplitude);
        printf("- Phase angle: %f\n", ipdft->phase_ang);
        return 1;
    } else {
        return 0;
    }
    

}
// Compute rms and remove dc offset
void do_rms(Buffer *voltage, Buffer *current){
    u32_t sum_voltage = 0;
    u32_t sum_current = 0;
    //printf("\n - Max value: %u", voltage->max);
    //printf("\n - Min value: %u\n", voltage->min);
    // # Calcula o valor de offset do sinal
    u16_t voltage_offset = voltage->sum / voltage->size;//(voltage->max + voltage->min) / 2;
    u16_t current_offset = current->sum / current->size;//(current->max + current->min) / 2;
    
    // Remove o offset do sinal e computa o somatório da sua potência
    for (u16_t i = 0; i < BUFFER_SIZE; i++){
        // Atualiza o valor do buffer removendo o offset
        voltage->data[i] -= voltage_offset;
        current->data[i] -= current_offset;
        // Função quadrada
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
    //return (float) (value * 0.8165 + 143.21);
    return (value * 3300 / 4095);  // To compare with generated values
}

void sample_read_task(void *parameters)
{
    // # Define ADC Characteristics for convertion
    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_V_REF, &cal);

    // # Struct to moving average
    FilterTypeDef filterStruct;
    Moving_Average_Init(&filterStruct);



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
                    sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_current, sample);
                    break;
                case ADC_VOLTAGE_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                    sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_voltage, sample);
                    break; 
                default:
                    ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[i]>>12);

            }
        }
        if(buf_voltage.size != buf_current.size){
            ESP_LOGE(TAG_SM, "# Voltage buffer and current doesn't have the same size [%d/%d]", buf_voltage.size, buf_current.size);
        } else {

            // Verifica se os buffers chegaram ao limite
            if(buffers_verify()){
                // Debug
                int64_t start_time = 0;             // Start time tick [us]
                int64_t end_time = 0;               // End time tick [us]
                float time_spent = 0;	            // Time spent in the execution [s]
                start_time = esp_timer_get_time();      // Debug
                // # Compute RMS and remove DC offset from signals;
                /*for(uint16_t x = 0; x < buf_voltage.size ; x++){
                    printf("%u,%d;", x, buf_voltage.data[x]);
                }*/
                do_rms(&buf_voltage, &buf_current);
                // # Apply goertzel to voltage signal
                //ESP_LOGI(TAG_SM, "- Executing IPDFT for voltage");
                ipdft_handle = do_ipdft(&buf_voltage, &ip_dft_data);
                if(ipdft_handle != 0){
                    // # Store result into main data block
                    sm_data.voltage_amp = ip_dft_data.amplitude;
                    sm_data.voltage_phase = ip_dft_data.phase_ang;
                    sm_data.voltage_freq = ip_dft_data.frequency;
                    //ESP_LOGI(TAG_SM, "- Executing IPDFT for current");
                    // # Apply goertzel to current signal
                    ipdft_handle = do_ipdft(&buf_current, &ip_dft_data);
                    if(ipdft_handle != 0){
                        // # Store result into main data block
                        sm_data.current_amp = ip_dft_data.amplitude;
                        sm_data.current_phase = ip_dft_data.phase_ang;
                        sm_data.current_freq = ip_dft_data.frequency;

                        //ESP_LOGI(TAG_SM, "- Executing THD and general calculations");
                        do_thd();

                    } else {
                        ESP_LOGE(TAG_SM, "# Error when computing IpDFT for current");
                    }
                } else {
                    ESP_LOGE(TAG_SM, "# Error when computing IpDFT for voltage");
                }

                buffer_clean(&buf_current);
                buffer_clean(&buf_voltage);
                // # Debug 
                // Obtain current time
                end_time = esp_timer_get_time();

                // Calculating time spent
                time_spent = (float)(end_time - start_time) / 1000;
                printf("smart meter execution time = %f ms\n", time_spent);
                //loop_ctrl = 1;
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
void test_functions(void *parameters) {
    FilterTypeDef filterStruct;
    Moving_Average_Init(&filterStruct);
    adc_sample_t buf[2048*2] = {2780, 15082, 2816, 15149, 2848, 15210, 2876, 15263, 2900, 15310, 2922, 15351, 2940, 15386, 2957, 15417, 2973, 15445, 2987, 15470, 3001, 15494, 3016, 15518, 3031, 15542, 3046, 15568, 3062, 15596, 3079, 15626, 3095, 15658, 3112, 15693, 3128, 15729, 3143, 15766, 3157, 15804, 3168, 15841, 3176, 15876, 3180, 15908, 3181, 15938, 3178, 15962, 3171, 15982, 3160, 15996, 3144, 16005, 3126, 16007, 3104, 16004, 3079, 15996, 3053, 15982, 3026, 15964, 2997, 15943, 2969, 15919, 2941, 15892, 2914, 15864, 2887, 15835, 2862, 15806, 2838, 15776, 2815, 15747, 2793, 15718, 2771, 15688, 2748, 15659, 2725, 15628, 2700, 15597, 2674, 15564, 2646, 15530, 2615, 15493, 2581, 15454, 2545, 15412, 2506, 15368, 2464, 15322, 2420, 15273, 2374, 15223, 2327, 15171, 2279, 15118, 2231, 15065, 2183, 15012, 2135, 14959, 2088, 14907, 2043, 14856, 1999, 14806, 1957, 14757, 1916, 14709, 1876, 14661, 1837, 14612, 1798, 14563, 1760, 14512, 1721, 14459, 1682, 14403, 1641, 14344, 1599, 14281, 1555, 14214, 1509, 14143, 1462, 14068, 1413, 13990, 1362, 13908, 1311, 13824, 1259, 13738, 1207, 13652, 1156, 13566, 1106, 13482, 1058, 13400, 1012, 13321, 969, 13247, 928, 13177, 890, 13113, 856, 13054, 825, 13001, 797, 12953, 772, 12910, 749, 12871, 729, 12836, 711, 12804, 694, 12775, 679, 12747, 665, 12720, 652, 12694, 640, 12668, 629, 12641, 618, 12614, 609, 12587, 602, 12559, 596, 12531, 591, 12504, 589, 12478, 588, 12454, 590, 12431, 595, 12411, 602, 12394, 611, 12380, 622, 12369, 636, 12362, 651, 12359, 667, 12359, 685, 12363, 704, 12369, 722, 12379, 742, 12390, 761, 12404, 780, 12420, 799, 12437, 818, 12456, 837, 12476, 856, 12497, 876, 12520, 897, 12545, 919, 12571, 943, 12599, 969, 12630, 996, 12662, 1027, 12697, 1059, 12735, 1095, 12775, 1132, 12817, 1172, 12861, 1214, 12907, 1257, 12955, 1302, 13003, 1348, 13053, 1394, 13102, 1441, 13152, 1488, 13202, 1534, 13252, 1580, 13301, 1625, 13351, 1670, 13401, 1715, 13451, 1760, 13502, 1805, 13556, 1850, 13611, 1897, 13670, 1945, 13732, 1994, 13798, 2045, 13869, 2097, 13944, 2151, 14023, 2207, 14108, 2263, 14196, 2321, 14287, 2379, 14381, 2437, 14476, 2493, 14572, 2549, 14666, 2602, 14759, 2652, 14848, 2699, 14933, 2743, 15012, 2782, 15086, 2818, 15153, 2849, 15213, 2877, 15266, 2901, 15312, 2923, 15353, 2941, 15388, 2958, 15419, 2973, 15446, 2988, 15471, 3002, 15495, 3017, 15519, 3031, 15544, 3047, 15570, 3063, 15597, 3079, 15628, 3096, 15660, 3113, 15695, 3129, 15731, 3144, 15768, 3157, 15806, 3168, 15842, 3176, 15878, 3180, 15910, 3181, 15939, 3178, 15963, 3170, 15983, 3159, 15997, 3144, 16005, 3125, 16007, 3103, 16004, 3078, 15995, 3052, 15981, 3024, 15963, 2996, 15942, 2967, 15917, 2939, 15891, 2912, 15863, 2886, 15834, 2861, 15804, 2837, 15775, 2814, 15746, 2792, 15716, 2769, 15687, 2747, 15657, 2724, 15627, 2699, 15595, 2673, 15563, 2644, 15528, 2613, 15491, 2580, 15452, 2543, 15410, 2504, 15366, 2462, 15319, 2418, 15271, 2372, 15220, 2325, 15168, 2277, 15115, 2228, 15062, 2180, 15009, 2133, 14957, 2086, 14905, 2041, 14854, 1997, 14804, 1955, 14755, 1914, 14706, 1874, 14658, 1835, 14610, 1796, 14560, 1758, 14510, 1719, 14456, 1680, 14400, 1639, 14341, 1596, 14278, 1552, 14211, 1507, 14140, 1459, 14064, 1410, 13986, 1360, 13904, 1308, 13820, 1257, 13734, 1205, 13648, 1154, 13562, 1104, 13478, 1056, 13396, 1010, 13317, 966, 13243, 926, 13174, 889, 13110, 855, 13051, 824, 12998, 796, 12951, 771, 12908, 748, 12869, 728, 12834, 710, 12803, 693, 12773, 678, 12746, 664, 12719, 651, 12693, 639, 12666, 628, 12640, 618, 12613, 609, 12585, 601, 12558, 595, 12530, 591, 12503, 589, 12477, 588, 12453, 591, 12430, 595, 12410, 602, 12393, 611, 12379, 623, 12369, 636, 12362, 652, 12359, 668, 12359, 686, 12363, 704, 12370, 723, 12379, 743, 12391, 762, 12405, 781, 12421, 800, 12438, 819, 12457, 838, 12477, 857, 12499, 877, 12522, 898, 12546, 920, 12573, 944, 12601, 970, 12631, 998, 12664, 1028, 12699, 1061, 12737, 1096, 12777, 1134, 12819, 1174, 12863, 1216, 12909, 1260, 12957, 1305, 13006, 1350, 13055, 1397, 13105, 1443, 13155, 1490, 13205, 1536, 13254, 1582, 13304, 1627, 13353, 1672, 13403, 1717, 13453, 1762, 13505, 1807, 13558, 1853, 13614, 1899, 13673, 1947, 13735, 1996, 13802, 2047, 13872, 2100, 13948, 2154, 14028, 2210, 14112, 2266, 14200, 2324, 14292, 2382, 14385, 2440, 14481, 2496, 14576, 2551, 14671, 2604, 14763, 2655, 14852, 2701, 14937, 2745, 15016, 2784, 15089, 2819, 15156, 2851, 15215, 2878, 15268, 2902, 15314, 2924, 15355, 2942, 15390, 2959, 15420, 2974, 15448, 2989, 15473, 3003, 15496, 3017, 15520, 3032, 15545, 3048, 15571, 3064, 15599, 3080, 15629, 3097, 15662, 3114, 15697, 3130, 15733, 3145, 15770, 3158, 15808, 3169, 15844, 3176, 15879, 3181, 15912, 3181, 15940, 3178, 15965, 3170, 15984, 3158, 15997, 3143, 16005, 3124, 16007, 3101, 16004, 3077, 15995, 3050, 15981, 3023, 15962, 2994, 15941, 2966, 15916, 2938, 15889, 2911, 15861, 2885, 15832, 2860, 15803, 2836, 15774, 2813, 15744, 2791, 15715, 2768, 15685, 2746, 15656, 2722, 15625, 2698, 15594, 2671, 15561, 2643, 15526, 2612, 15489, 2578, 15450, 2541, 15408, 2502, 15364, 2460, 15317, 2416, 15268, 2370, 15218, 2322, 15166, 2274, 15113, 2226, 15060, 2178, 15007, 2130, 14954, 2084, 14902, 2039, 14851, 1995, 14801, 1953, 14752, 1912, 14704, 1872, 14656, 1833, 14607, 1795, 14558, 1756, 14507, 1717, 14454, 1678, 14398, 1637, 14338, 1594, 14275, 1550, 14207, 1504, 14136, 1457, 14061, 1408, 13982, 1357, 13900, 1306, 13815, 1254, 13730, 1202, 13643, 1151, 13558, 1101, 13473, 1053, 13392, 1008, 13314, 964, 13240, 924, 13171, 887, 13107, 853, 13049, 822, 12996, 794, 12948, 769, 12906, 747, 12867, 727, 12833, 709, 12801, 692, 12772, 677, 12744, 663, 12718, 651, 12691, 639, 12665, 628, 12638, 618, 12611, 609, 12584, 601, 12556, 595, 12529, 591, 12502, 589, 12476, 589, 12451, 591, 12429, 595, 12409, 603, 12392, 612, 12379, 624, 12369, 637, 12362, 652, 12359, 669, 12359, 687, 12363, 705, 12370, 724, 12380, 743, 12392, 763, 12406, 782, 12421, 801, 12439, 820, 12458, 839, 12478, 858, 12500, 878, 12523, 899, 12548, 921, 12574, 945, 12602, 971, 12633, 999, 12666, 1030, 12701, 1063, 12739, 1098, 12779, 1136, 12821, 1176, 12866, 1218, 12912, 1262, 12959, 1307, 13008, 1353, 13057, 1399, 13107, 1446, 13157, 1492, 13207, 1538, 13257, 1584, 13306, 1630, 13356, 1675, 13406, 1719, 13456, 1764, 13508, 1809, 13561, 1855, 13617, 1902, 13676, 1949, 13738, 1999, 13805, 2050, 13876, 2102, 13952, 2157, 14032, 2212, 14116, 2269, 14205, 2327, 14296, 2385, 14390, 2442, 14486, 2499, 14581, 2554, 14676, 2607, 14768, 2657, 14857, 2704, 14941, 2747, 15020, 2786, 15093, 2821, 15159, 2852, 15218, 2880, 15271, 2904, 15317, 2925, 15357, 2943, 15391, 2960, 15422, 2975, 15449, 2989, 15474, 3004, 15498, 3018, 15521, 3033, 15546, 3048, 15572, 3064, 15600, 3081, 15631, 3098, 15664, 3115, 15698, 3131, 15735, 3146, 15772, 3158, 15809, 3169, 15846, 3177, 15881, 3181, 15913, 3181, 15942, 3177, 15966, 3169, 15985, 3158, 15998, 3142, 16006, 3123, 16007, 3100, 16003, 3076, 15994, 3049, 15980, 3021, 15961, 2993, 15939, 2965, 15915, 2937, 15888, 2910, 15860, 2884, 15831, 2859, 15802, 2835, 15772, 2812, 15743, 2789, 15713, 2767, 15684, 2745, 15654, 2721, 15624, 2697, 15592, 2670, 15559, 2641, 15524, 2610, 15487, 2576, 15448, 2539, 15406, 2500, 15361, 2458, 15315, 2413, 15266, 2367, 15215, 2320, 15163, 2272, 15110, 2224, 15057, 2175, 15004, 2128, 14951, 2081, 14900, 2036, 14849, 1993, 14799, 1950, 14750, 1910, 14702, 1870, 14653, 1831, 14605, 1793, 14555, 1754, 14504, 1715, 14451, 1676, 14395, 1635, 14335, 1592, 14271, 1548, 14204, 1502, 14132, 1454, 14057, 1405, 13978, 1355, 13896, 1303, 13811, 1251, 13725, 1200, 13639, 1149, 13553, 1099, 13469, 1051, 13388, 1005, 13310, 962, 13236, 922, 13167, 885, 13104, 851, 13046, 821, 12993, 793, 12946, 768, 12904, 746, 12866, 726, 12831, 708, 12800, 692, 12771, 677, 12743, 663, 12716, 650, 12690, 638, 12664, 627, 12637, 617, 12610, 608, 12582, 601, 12555, 595, 12527, 591, 12500, 588, 12474, 589, 12450, 591, 12428, 596, 12408, 603, 12392, 613, 12378, 624, 12368, 638, 12362, 653, 12359, 670, 12360, 688, 12364, 706, 12370, 725, 12380, 744, 12392, 764, 12406, 783, 12422, 802, 12440, 821, 12459, 840, 12479, 859, 12501, 879, 12524, 900, 12549, 923, 12575, 947, 12604, 973, 12635, 1001, 12668, 1031, 12703, 1064, 12741, 1100, 12781, 1138, 12823, 1178, 12868, 1220, 12914, 1264, 12962, 1309, 13010, 1355, 13060, 1401, 13110, 1448, 13160, 1495, 13210, 1541, 13259, 1587, 13309, 1632, 13358, 1677, 13408, 1722, 13459, 1766, 13510, 1812, 13564, 1857, 13620, 1904, 13679, 1952, 13742, 2001, 13808, 2052, 13880, 2105, 13955, 2159, 14036, 2215, 14121, 2272, 14209, 2330, 14301, 2388, 14395, 2445, 14490, 2502, 14586, 2557, 14680, 2609, 14772, 2659, 14861, 2706, 14945, 2749, 15024, 2788, 15096, 2823, 15162, 2854, 15221, 2881, 15273, 2905, 15319, 2926, 15358, 2944, 15393, 2960, 15423, 2976, 15450, 2990, 15475, 3004, 15499, 3019, 15523, 3034, 15547, 3049, 15574, 3065, 15602, 3082, 15632, 3099, 15665, 3116, 15700, 3131, 15737, 3146, 15774, 3159, 15811, 3169, 15848, 3177, 15883, 3181, 15915, 3181, 15943, 3177, 15967, 3169, 15985, 3157, 15998, 3141, 16006, 3122, 16007, 3099, 16003, 3074, 15993, 3048, 15979, 3020, 15960, 2992, 15938, 2963, 15913, 2935, 15887, 2908, 15858, 2882, 15829, 2857, 15800, 2834, 15771, 2811, 15741, 2788, 15712, 2766, 15682, 2743, 15653, 2720, 15622, 2695, 15591, 2669, 15557, 2640, 15523, 2608, 15485, 2574, 15446, 2537, 15404, 2498, 15359, 2456, 15312, 2411, 15263, 2365, 15212, 2318, 15160, 2270, 15107, 2221, 15054, 2173, 15001, 2126, 14949, 2079, 14897, 2034, 14846, 1991, 14796, 1948, 14748, 1908, 14699, 1868, 14651, 1829, 14602, 1791, 14553, 1752, 14502, 1713, 14448, 1674, 14392, 1633, 14332, 1590, 14268, 1546, 14200, 1500, 14129, 1452, 14053, 1403, 13974, 1352, 13891, 1301, 13807, 1249, 13721, 1197, 13635, 1146, 13549, 1097, 13465, 1049, 13384, 1003, 13306, 960, 13233, 920, 13164, 883, 13101, 850, 13043, 819, 12991, 792, 12944, 767, 12902, 745, 12864, 725, 12830, 707, 12798, 691, 12769, 676, 12742, 662, 12715, 649, 12689, 637, 12662, 627, 12636, 617, 12609, 608, 12581, 600, 12553, 595, 12526, 590, 12499, 588, 12473, 589, 12449, 591, 12427, 596, 12407, 603, 12391, 613, 12377, 625, 12368, 639, 12361, 654, 12359, 671, 12360, 689, 12364, 707, 12371, 726, 12381, 745, 12393, 765, 12407, 784, 12423, 803, 12441, 821, 12460, 841, 12480, 860, 12502, 880, 12525, 901, 12550, 924, 12577, 948, 12605, 974, 12636, 1002, 12669, 1033, 12705, 1066, 12743, 1102, 12783, 1140, 12825, 1180, 12870, 1222, 12916, 1266, 12964, 1311, 13013, 1357, 13062, 1404, 13112, 1450, 13162, 1497, 13212, 1543, 13262, 1589, 13311, 1634, 13361, 1679, 13411, 1724, 13461, 1769, 13513, 1814, 13567, 1860, 13623, 1906, 13682, 1954, 13745, 2004, 13812, 2055, 13883, 2108, 13959, 2162, 14040, 2218, 14125, 2275, 14214, 2333, 14306, 2391, 14400, 2448, 14495, 2505, 14591, 2559, 14685, 2612, 14777, 2662, 14865, 2708, 14949, 2751, 15028, 2790, 15100, 2824, 15165, 2855, 15224, 2882, 15276, 2906, 15321, 2927, 15360, 2945, 15394, 2961, 15424, 2976, 15451, 2991, 15476, 3005, 15500, 3019, 15524, 3034, 15549, 3050, 15575, 3066, 15603, 3083, 15634, 3100, 15667, 3116, 15702, 3132, 15738, 3147, 15776, 3160, 15813, 3170, 15850, 3177, 15884, 3181, 15916, 3181, 15944, 3177, 15968, 3168, 15986, 3156, 15999, 3140, 16006, 3120, 16007, 3098, 16003, 3073, 15993, 3046, 15978, 3018, 15959, 2990, 15937, 2962, 15912, 2934, 15885, 2907, 15857, 2881, 15828, 2856, 15799, 2832, 15769, 2810, 15740, 2787, 15710, 2765, 15681, 2742, 15651, 2719, 15621, 2694, 15589, 2667, 15556, 2638, 15521, 2607, 15483, 2572, 15444, 2535, 15402, 2496, 15357, 2453, 15310, 2409, 15261, 2363, 15210, 2315, 15158, 2267, 15105, 2219, 15052, 2171, 14999, 2123, 14946, 2077, 14894, 2032, 14844, 1988, 14794, 1946, 14745, 1906, 14697, 1866, 14649, 1827, 14600, 1789, 14550, 1750, 14499, 1711, 14445, 1672, 14389, 1630, 14329, 1588, 14265, 1543, 14197, 1497, 14125, 1449, 14049, 1400, 13970, 1349, 13887, 1298, 13803, 1246, 13717, 1195, 13630, 1144, 13545, 1094, 13461, 1046, 13380, 1001, 13302, 958, 13229, 918, 13161, 882, 13098, 848, 13040, 818, 12988, 790, 12942, 766, 12900, 744, 12862, 724, 12828, 706, 12797, 690, 12768, 675, 12740, 662, 12714, 649, 12687, 637, 12661, 626, 12634, 616, 12607, 607, 12580, 600, 12552, 594, 12525, 590, 12498, 588, 12472, 589, 12448, 591, 12426, 596, 12406, 604, 12390, 614, 12377, 626, 12367, 639, 12361, 655, 12359, 672, 12360, 690, 12364, 708, 12371, 727, 12381, 746, 12393, 765, 12408, 785, 12424, 803, 12442, 822, 12461, 842, 12481, 861, 12503, 881, 12526, 902, 12551, 925, 12578, 949, 12607, 975, 12638, 1004, 12671, 1035, 12707, 1068, 12745, 1104, 12785, 1142, 12828, 1182, 12872, 1225, 12919, 1269, 12967, 1314, 13015, 1360, 13065, 1406, 13115, 1453, 13165, 1499, 13215, 1545, 13264, 1591, 13314, 1636, 13363, 1681, 13413, 1726, 13464, 1771, 13516, 1816, 13569, 1862, 13626, 1909, 13685, 1957, 13748, 2006, 13815, 2057, 13887, 2110, 13963, 2165, 14044, 2221, 14129, 2278, 14218, 2336, 14310, 2393, 14404, 2451, 14500, 2507, 14595, 2562, 14690, 2615, 14781, 2664, 14870, 2710, 14953, 2753, 15031, 2791, 15103, 2826, 15168, 2857, 15227, 2883, 15278, 2907, 15323, 2927, 15362, 2946, 15396, 2962, 15426, 2977, 15453, 2991, 15477, 3006, 15501, 3020, 15525, 3035, 15550, 3051, 15576, 3067, 15605, 3084, 15636, 3101, 15669, 3117, 15704, 3133, 15740, 3148, 15778, 3160, 15815, 3170, 15851, 3177, 15886, 3181, 15918, 3181, 15945, 3176, 15969, 3168, 15987, 3155, 15999, 3139, 16006, 3119, 16007, 3097, 16002, 3072, 15992, 3045, 15977, 3017, 15958, 2989, 15936, 2960, 15911, 2933, 15884, 2906, 15856, 2880, 15826, 2855, 15797, 2831, 15768, 2808, 15738, 2786, 15709, 2764, 15679, 2741, 15650, 2718, 15619, 2693, 15587, 2666, 15554, 2637, 15519, 2605, 15482, 2571, 15442, 2533, 15399, 2494, 15355, 2451, 15307, 2407, 15258, 2360, 15207, 2313, 15155, 2265, 15102, 2216, 15049, 2168, 14996, 2121, 14943, 2075, 14892, 2030, 14841, 1986, 14792, 1944, 14743, 1904, 14694, 1864, 14646, 1825, 14598, 1787, 14548, 1748, 14497, 1709, 14443, 1670, 14386, 1628, 14326, 1586, 14262, 1541, 14193, 1495, 14121, 1447, 14045, 1398, 13966, 1347, 13883, 1295, 13798, 1244, 13712, 1192, 13626, 1141, 13541, 1092, 13457, 1044, 13376, 999, 13299, 956, 13226, 916, 13158, 880, 13095, 846, 13038, 816, 12986, 789, 12939, 765, 12898, 743, 12860, 723, 12826, 706, 12795, 689, 12766, 675, 12739, 661, 12712, 648, 12686, 636, 12660, 625, 12633, 616, 12606, 607, 12578, 600, 12551, 594, 12523, 590, 12496, 588, 12471, 589, 12447, 592, 12425, 597, 12406, 604, 12389, 614, 12376, 626, 12367, 640, 12361, 656, 12359, 673, 12360, 691, 12364, 709, 12372, 728, 12382, 747, 12394, 766, 12409, 785, 12425, 804, 12442, 823, 12462, 842, 12482, 862, 12504, 882, 12528, 903, 12553, 926, 12580, 950, 12608, 977, 12639, 1005, 12673, 1036, 12708, 1070, 12747, 1106, 12787, 1144, 12830, 1184, 12875, 1227, 12921, 1271, 12969, 1316, 13018, 1362, 13067, 1408, 13117, 1455, 13167, 1501, 13217, 1548, 13267, 1593, 13316, 1639, 13366, 1684, 13416, 1728, 13466, 1773, 13518, 1818, 13572, 1864, 13629, 1911, 13688, 1959, 13751, 2009, 13819, 2060, 13891, 2113, 13967, 2168, 14048, 2224, 14134, 2281, 14223, 2338, 14315, 2396, 14409, 2454, 14505, 2510, 14600, 2565, 14694, 2617, 14786, 2667, 14874, 2713, 14957, 2755, 15035, 2793, 15107, 2828, 15171, 2858, 15229, 2885, 15280, 2908, 15325, 2928, 15364, 2947, 15398, 2963, 15427, 2978, 15454, 2992, 15479, 3006, 15502, 3021, 15526, 3036, 15551, 3052, 15578, 3068, 15606, 3084, 15637, 3101, 15670, 3118, 15705, 3134, 15742, 3148, 15779, 3161, 15817, 3171, 15853, 3178, 15888, 3181, 15919, 3181, 15947, 3176, 15970, 3167, 15988, 3155, 16000, 3138, 16006, 3118, 16007, 3096, 16002, 3070, 15991, 3044, 15976, 3016, 15957, 2987, 15935, 2959, 15910, 2931, 15882, 2904, 15854, 2878, 15825, 2854, 15796, 2830, 15766, 2807, 15737, 2785, 15707, 2763, 15678, 2740, 15648, 2716, 15618, 2691, 15586, 2664, 15552, 2635, 15517, 2603, 15480, 2569, 15440, 2532, 15397, 2492, 15352, 2449, 15305, 2404, 15256, 2358, 15205, 2311, 15152, 2262, 15100, 2214, 15046, 2166, 14993, 2119, 14941, 2072, 14889, 2027, 14839, 1984, 14789, 1942, 14740, 1902, 14692, 1862, 14644, 1823, 14595, 1785, 14545, 1746, 14494, 1707, 14440, 1668, 14383, 1626, 14323, 1583, 14258, 1539, 14190, 1493, 14118, 1445, 14041, 1395, 13961, 1344, 13879, 1293, 13794, 1241, 13708, 1189, 13622, 1139, 13536, 1089, 13453, 1042, 13372, 996, 13295, 954, 13222, 914, 13154, 878, 13092, 845, 13035, 815, 12983, 788, 12937, 764, 12896, 742, 12858, 722, 12825, 705, 12794, 689, 12765, 674, 12738, 660, 12711, 648, 12685, 636, 12658, 625, 12632, 615, 12604, 607, 12577, 599, 12549, 594, 12522, 590, 12495, 588, 12469, 589, 12446, 592, 12424, 597, 12405, 605, 12389, 615, 12376, 627, 12367, 641, 12361, 657, 12359, 673, 12360, 691, 12365, 710, 12372, 729, 12382, 748, 12395, 767, 12409, 786, 12426, 805, 12443, 824, 12463, 843, 12483, 863, 12505, 883, 12529, 905, 12554, 927, 12581, 952, 12610, 978, 12641, 1007, 12674, 1038, 12710, 1071, 12749, 1107, 12789, 1146, 12832, 1186, 12877, 1229, 12923, 1273, 12971, 1318, 13020, 1364, 13070, 1411, 13120, 1457, 13170, 1504, 13220, 1550, 13269, 1596, 13319, 1641, 13368, 1686, 13418, 1731, 13469, 1775, 13521, 1821, 13575, 1867, 13631, 1913, 13691, 1962, 13755, 2011, 13822, 2063, 13894, 2116, 13971, 2170, 14052, 2226, 14138, 2284, 14227, 2341, 14319, 2399, 14414, 2457, 14509, 2513, 14605, 2568, 14699, 2620, 14790, 2669, 14878, 2715, 14961, 2757, 15039, 2795, 15110, 2829, 15174, 2859, 15232, 2886, 15283, 2909, 15327, 2929, 15366, 2947, 15399, 2964, 15429, 2979, 15455, 2993, 15480, 3007, 15504, 3022, 15528, 3037, 15552, 3052, 15579, 3069, 15608, 3085, 15639, 3102, 15672, 3119, 15707, 3135, 15744, 3149, 15781, 3161, 15819, 3171, 15855, 3178, 15889, 3181, 15921, 3180, 15948, 3176, 15971, 3167, 15988, 3154, 16000, 3137, 16007, 3117, 16007, 3094, 16001, 3069, 15991, 3042, 15976, 3014, 15956, 2986, 15934, 2958, 15908, 2930, 15881, 2903, 15853, 2877, 15824, 2853, 15794, 2829, 15765, 2806, 15735, 2784, 15706, 2762, 15677, 2739, 15647, 2715, 15616, 2690, 15584, 2663, 15551, 2634, 15515, 2602, 15478, 2567, 15438, 2530, 15395, 2489, 15350, 2447, 15303, 2402, 15253, 2356, 15202, 2308, 15150, 2260, 15097, 2211, 15044, 2163, 14991, 2116, 14938, 2070, 14887, 2025, 14836, 1982, 14787, 1940, 14738, 1900, 14690, 1860, 14641, 1821, 14593, 1783, 14543, 1745, 14491, 1705, 14437, 1665, 14380, 1624, 14319, 1581, 14255, 1537, 14186, 1490, 14114, 1442, 14037, 1393, 13957, 1342, 13875, 1290, 13790, 1238, 13704, 1187, 13617, 1136, 13532, 1087, 13449, 1039, 13368, 994, 13291, 952, 13219, 913, 13151, 876, 13089, 843, 13032, 813, 12981, 787, 12935, 762, 12894, 741, 12857, 721, 12823, 704, 12792, 688, 12764, 673, 12736, 660, 12710, 647, 12683, 635, 12657, 624, 12630, 615, 12603, 606, 12576, 599, 12548, 594, 12521, 590, 12494, 588, 12468, 589, 12444, 592, 12423, 597, 12404, 605, 12388, 615, 12375, 627, 12366, 642, 12361, 657, 12359, 674, 12360, 692, 12365, 711, 12373, 730, 12383, 749, 12395, 768, 12410, 787, 12426, 806, 12444, 825, 12464, 844, 12484, 864, 12506, 884, 12530, 906, 12555, 928, 12582, 953, 12611, 979, 12643, 1008, 12676, 1039, 12712, 1073, 12751, 1109, 12791, 1148, 12834, 1189, 12879, 1231, 12926, 1275, 12974, 1321, 13023, 1367, 13072, 1413, 13122, 1460, 13172, 1506, 13222, 1552, 13272, 1598, 13321, 1643, 13371, 1688, 13421, 1733, 13471, 1778, 13524, 1823, 13578, 1869, 13634, 1916, 13694, 1964, 13758, 2014, 13826, 2065, 13898, 2118, 13975, 2173, 14057, 2229, 14142, 2286, 14232, 2344, 14324, 2402, 14419, 2460, 14514, 2516, 14610, 2570, 14704, 2622, 14795, 2671, 14883, 2717, 14965, 2759, 15043, 2797, 15113, 2831, 15177, 2861, 15235, 2887, 15285, 2910, 15329, 2930, 15367, 2948, 15401, 2964, 15430, 2979, 15456, 2994, 15481, 3008, 15505, 3022, 15529, 3037, 15554, 3053, 15580, 3069, 15609, 3086, 15640, 3103, 15674, 3120, 15709, 3135, 15746, 3150, 15783, 3162, 15821, 3172, 15857, 3178, 15891, 3181, 15922, 3180, 15949, 3175, 15972, 3166, 15989, 3153, 16001, 3136, 16007, 3116, 16007, 3093, 16001, 3068, 15990, 3041, 15975, 3013, 15955, 2984, 15932, 2956, 15907, 2928, 15880, 2902, 15851, 2876, 15822, 2851, 15793, 2828, 15763, 2805, 15734, 2783, 15705, 2760, 15675, 2738, 15645, 2714, 15614, 2689, 15582, 2662, 15549, 2632, 15513, 2600, 15476, 2565, 15436, 2528, 15393, 2487, 15348, 2445, 15300, 2400, 15251, 2353, 15200, 2306, 15147, 2257, 15094, 2209, 15041, 2161, 14988, 2114, 14936, 2068, 14884, 2023, 14834, 1980, 14784, 1938, 14735, 1898, 14687, 1858, 14639, 1820, 14590, 1781, 14540, 1743, 14489, 1704, 14434, 1663, 14377, 1622, 14316, 1579, 14252, 1534, 14183, 1488, 14110, 1440, 14033, 1390, 13953, 1339, 13870, 1288, 13785, 1236, 13699, 1184, 13613, 1134, 13528, 1084, 13445, 1037, 13364, 992, 13287, 950, 13215, 911, 13148, 875, 13086, 842, 13030, 812, 12979, 785, 12933, 761, 12892, 740, 12855, 721, 12821, 703, 12791, 687, 12762, 672, 12735, 659, 12708, 646, 12682, 635, 12656, 624, 12629, 614, 12602, 606, 12574, 599, 12547, 593, 12519, 590, 12492, 588, 12467, 589, 12443, 592, 12422, 598, 12403, 606, 12387, 616, 12375, 628, 12366, 642, 12361, 658, 12359, 675, 12360, 693, 12365, 712, 12373, 731, 12383, 750, 12396, 769, 12411, 788, 12427, 807, 12445, 826, 12465, 845, 12485, 865, 12508, 885, 12531, 907, 12557, 930, 12584, 954, 12613, 981, 12644, 1010, 12678, 1041, 12714, 1075, 12752, 1111, 12793, 1150, 12836, 1191, 12881, 1233, 12928, 1277, 12976, 1323, 13025, 1369, 13075, 1415, 13125, 1462, 13175, 1508, 13225, 1555, 13274, 1600, 13324, 1645, 13373, 1690, 13423, 1735, 13474, 1780, 13526, 1825, 13580, 1871, 13637, 1918, 13697, 1967, 13761, 2016, 13829, 2068, 13902, 2121, 13979, 2176, 14061, 2232, 14147, 2289, 14236, 2347, 14329, 2405, 14423, 2462, 14519, 2519, 14614, 2573, 14708, 2625, 14799, 2674, 14887, 2719, 14969, 2761, 15046, 2799, 15117, 2832, 15180, 2862, 15237, 2888, 15287, 2911, 15331, 2931, 15369, 2949, 15402, 2965, 15431, 2980, 15458, 2994, 15482, 3009, 15506, 3023, 15530, 3038, 15555, 3054, 15582, 3070, 15611, 3087, 15642, 3104, 15675, 3120, 15711, 3136, 15748, 3150, 15785, 3162, 15822, 3172, 15859, 3178, 15893, 3181, 15923, 3180, 15951, 3175, 15973, 3166, 15990, 3152, 16001, 3135, 16007, 3115, 16007, 3092, 16001, 3067, 15990, 3039, 15974, 3011, 15954, 2983, 15931, 2955, 15906, 2927, 15878, 2900, 15850, 2875, 15821, 2850, 15791, 2827, 15762, 2804, 15732, 2782, 15703, 2759, 15674, 2737, 15644, 2713, 15613, 2687, 15581, 2660, 15547, 2631, 15512, 2598, 15474, 2563, 15433, 2526, 15391, 2485, 15345, 2442, 15298, 2398, 15248, 2351, 15197, 2303, 15145, 2255, 15092, 2207, 15038, 2159, 14985, 2112, 14933, 2066, 14882, 2021, 14831, 1978, 14782, 1936, 14733, 1896, 14685, 1856, 14637, 1818, 14588, 1779, 14538, 1741, 14486, 1702, 14432, 1661, 14374, 1620, 14313, 1577, 14248, 1532, 14179, 1486, 14106, 1437, 14029, 1388, 13949, 1337, 13866, 1285, 13781, 1233, 13695, 1182, 13609, 1131, 13524, 1082, 13440, 1035, 13360, 990, 13284, 948, 13212, 909, 13145, 873, 13083, 840, 13027, 811, 12976, 784, 12931, 760, 12890, 739, 12853, 720, 12820, 702, 12789, 686, 12761, 672, 12734, 658, 12707, 646, 12681, 634, 12654, 623, 12628, 614, 12600, 605, 12573, 598, 12545, 593, 12518, 590, 12491, 588, 12466, 589, 12442, 592, 12421, 598, 12402, 606, 12386, 616, 12374, 629, 12365, 643, 12360, 659, 12359, 676, 12361, 694, 12366, 713, 12374, 732, 12384, 751, 12397, 770, 12412, 789, 12428, 808, 12446, 827, 12466, 846, 12486, 866, 12509, 886, 12532, 908, 12558, 931, 12585, 955, 12614, 982, 12646, 1011, 12680, 1043, 12716, 1077, 12754, 1113, 12795, 1152, 12839, 1193, 12884, 1235, 12931, 1280, 12979, 1325, 13028, 1371, 13077, 1418, 13127, 1464, 13177, 1511, 13227, 1557, 13277, 1602, 13326, 1648, 13376, 1693, 13426, 1737, 13477, 1782, 13529, 1827, 13583, 1873, 13640, 1921, 13700, 1969, 13765, 2019, 13833, 2071, 13906, 2124, 13983, 2179, 14065, 2235, 14151, 2292, 14241, 2350, 14334, 2408, 14428, 2465, 14524, 2521, 14619, 2575, 14713, 2627, 14804, 2676, 14891, 2721, 14973, 2763, 15050, 2800, 15120, 2834, 15183, 2864, 15240, 2890, 15290, 2912, 15333, 2932, 15371, 2950, 15404, 2966, 15433, 2981, 15459, 2995, 15483, 3009, 15507, 3024, 15531, 3039, 15556, 3055, 15583, 3071, 15612, 3088, 15644, 3105, 15677, 3121, 15713, 3137, 15750, 3151, 15787, 3163, 15824, 3172, 15860, 3179, 15894, 3181, 15925, 3180, 15952, 3175, 15974, 3165, 15991, 3152, 16002, 3135, 16007, 3114, 16006, 3091, 16000, 3065, 15989, 3038, 15973, 3010, 15953, 2982, 15930, 2953, 15904, 2926, 15877, 2899, 15848, 2873, 15819, 2849, 15790, 2825, 15760, 2803, 15731, 2781, 15702, 2758, 15672, 2735, 15642, 2712, 15611, 2686, 15579, 2659, 15545, 2629, 15510, 2597, 15472, 2562, 15431, 2524, 15388, 2483, 15343, 2440, 15295, 2395, 15246, 2349, 15194, 2301, 15142, 2253, 15089, 2204, 15036, 2156, 14983, 2109, 14930, 2063, 14879, 2019, 14829, 1976, 14779, 1934, 14731, 1894, 14682, 1854, 14634, 1816, 14585, 1777, 14535, 1739, 14483, 1700, 14429, 1659, 14371, 1618, 14310, 1575, 14245, 1530, 14176, 1483, 14103, 1435, 14026, 1385, 13945, 1334, 13862, 1282, 13777, 1231, 13691, 1179, 13605, 1129, 13519, 1080, 13436, 1032, 13356, 988, 13280, 946, 13208, 907, 13141, 871, 13080, 839, 13024, 809, 12974, 783, 12929, 759, 12888, 738, 12851, 719, 12818, 701, 12788, 686, 12759, 671, 12732, 658, 12706, 645, 12680, 634, 12653, 623, 12626, 613, 12599, 605, 12571, 598, 12544, 593, 12516, 590, 12490, 588, 12465, 589, 12441, 593, 12420, 598, 12401, 607, 12386, 617, 12374, 629, 12365, 644, 12360, 660, 12359, 677, 12361, 695, 12366, 714, 12374, 733, 12385, 752, 12398, 771, 12412, 790, 12429, 809, 12447, 828, 12467, 847, 12488, 867, 12510, 887, 12534, 909, 12559, 932, 12587, 957, 12616, 984, 12647, 1013, 12681, 1044, 12718, 1078, 12756, 1115, 12798, 1154, 12841, 1195, 12886, 1238, 12933, 1282, 12981, 1327, 13030, 1374, 13080, 1420, 13130, 1467, 13180, 1513, 13229, 1559, 13279, 1605, 13329, 1650, 13378, 1695, 13428, 1740, 13479, 1784, 13531, 1830, 13586, 1876, 13643, 1923, 13704, 1971, 13768, 2021, 13836, 2073, 13909, 2127, 13987, 2181, 14069, 2238, 14155, 2295, 14245, 2353, 14338, 2411, 14433, 2468, 14529, 2524, 14624, 2578, 14717, 2630, 14808, 2678, 14895, 2724, 14977, 2765, 15054, 2802, 15123, 2836, 15186, 2865, 15243, 2891, 15292, 2913, 15335, 2933, 15373, 2951, 15405, 2967, 15434};
    size_t measures_read = 2048*2;      
    u16_t sample = 0;                   // Generic variable to momently store sample value converted to mV
    // # Loop on measures to convert and save on buffer
    for (u16_t i = 0; i < measures_read; i ++)
    {
        // Get channels offset on buffer
            //printf("Channel %d \n", buf[i] >> 12);
            switch (buf[i] >> 12){
                case ADC_CURRENT_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                    //sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_current, sample);
                    break;
                case ADC_VOLTAGE_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                    //sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_voltage, sample);
                    break; 
                default:
                    ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[i]>>12);

            }
    }
    if(buf_voltage.size != buf_current.size){
        ESP_LOGE(TAG_SM, "# Voltage buffer and current doesn't have the same size [%d/%d]", buf_voltage.size, buf_current.size);
    } else {

        // Verifica se os buffers chegaram ao limite
        if(buffers_verify()){
            // Debug
            int64_t start_time = 0;             // Start time tick [us]
            int64_t end_time = 0;               // End time tick [us]
            float time_spent = 0;	            // Time spent in the execution [s]
            start_time = esp_timer_get_time();      // Debug
            // # Compute RMS and remove DC offset from signals;
            /*for(uint16_t x = 0; x < buf_voltage.size ; x++){
                printf("%u,%d;", x, buf_voltage.data[x]);
            }*/
            do_rms(&buf_voltage, &buf_current);
            // # Apply goertzel to voltage signal
            ESP_LOGI(TAG_SM, "- Executing IPDFT for voltage");
            ipdft_handle = do_ipdft(&buf_voltage, &ip_dft_data);
            if(ipdft_handle != 0){
                // # Store result into main data block
                sm_data.voltage_amp = ip_dft_data.amplitude;
                sm_data.voltage_phase = ip_dft_data.phase_ang;
                sm_data.voltage_freq = ip_dft_data.frequency;
                ESP_LOGI(TAG_SM, "- Executing IPDFT for current");
                // # Apply goertzel to current signal
                ipdft_handle = do_ipdft(&buf_current, &ip_dft_data);
                if(ipdft_handle != 0){
                    // # Store result into main data block
                    sm_data.current_amp = ip_dft_data.amplitude;
                    sm_data.current_phase = ip_dft_data.phase_ang;
                    sm_data.current_freq = ip_dft_data.frequency;

                    ESP_LOGI(TAG_SM, "- Executing THD and general calculations");
                    do_thd();

                } else {
                    ESP_LOGE(TAG_SM, "# Error when computing IpDFT for current");
                }
            } else {
                ESP_LOGE(TAG_SM, "# Error when computing IpDFT for voltage");
            }

            buffer_clean(&buf_current);
            buffer_clean(&buf_voltage);
            // # Debug 
            // Obtain current time
            end_time = esp_timer_get_time();

            // Calculating time spent
            time_spent = (float)(end_time - start_time) / 1000;
            printf("smart meter execution time = %f ms\n", time_spent);
            //loop_ctrl = 1;
        }
    }
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
		test_functions,					// Function executed in the task
		"SRT",					            // Task name (for debug)
		32000,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&sample_read_task_handle,			// Used to pass back a handle by which the created task can be referenced
		0);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating sample_read task"); }


    
    
}
