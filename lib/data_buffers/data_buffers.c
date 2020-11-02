#include "data_buffers.h"             // Include buffer header

TimeSerieSM sm_ts;

// Function used to insert data into timeseries struct 
int sm_push (SmartMeter *data){
    uint16_t pt = sm_ts.pointer;

    // Store data into timeseries data block
    sm_ts.data[pt].v_rms = data->v_rms;
	sm_ts.data[pt].i_rms = data->i_rms;
	sm_ts.data[pt].aparent_power = data->aparent_power;
	sm_ts.data[pt].active_power = data->active_power;
    sm_ts.data[pt].reactive_power = data->reactive_power;
    sm_ts.data[pt].frequency = data->frequency;
    sm_ts.data[pt].fp = data->fp;
    sm_ts.data[pt].THD_V = data->THD_V;
    sm_ts.data[pt].THD_I = data->THD_I;
    sm_ts.pointer++;

    // Verify if timeserires size is not full
    if(sm_ts.pointer >= SM_TIMESERIE_SIZE){
        sm_ts.pointer = 0;
    }
    return 1;
}
	
	
	
	
    
    
    
    
    



// Function used to insert data into buffer and contabilize it size;
int buffer_push (Buffer *buf, int16_t value){
    // Verify if buffer size is not full
    if(buf->size == BUFFER_SIZE){
        return -1;
    }
    // Insert data into buffer and increment it counter
    buf->data[buf->size] = value;
    // Compute max and min values
    buf->max = (value > buf->max ) ? value : buf->max;
    buf->min = (value < buf->min ) ? value : buf->min;

    buf->size++;


    return 1;
}
void buffer_clean (Buffer *buf){
    for (uint16_t i = 0; i < buf->size ; i++){
        buf->data[i] = 0;
    }
    buf->size = 0;
    buf->max = 0;
    buf->min = 65535;
}
int is_buffer_full(Buffer *buf){
    // Verify if buffer size is not full
    if(buf->size == BUFFER_SIZE){
        return 1;
    } 
    return 0;
}
/* // Test routine
int main(){
    Buffer voltage;
	int buffer_handle = 0;


	for(uint16_t k = 0; k < BUFFER_SIZE+1; k++){
		buffer_handle = buffer_push(&voltage, k);
		if(buffer_handle >= 0){
			printf("Buffer [%d] = %d [Buffer size %d]\n", k, voltage.data[k], voltage.size);
		} else {
			printf("Error while inserting\n");
		}
	}
	buffer_clean(&voltage);
	printf("Buffer size: %d\n", voltage.size);
	for(uint16_t k = 0; k < BUFFER_SIZE; k++){
		printf("Buffer [%d] = %d\n", k, voltage.data[k]);
	}
}
*/