#include "buffer.h"             // Include buffer header

// Function used to insert data into buffer and contabilize it size;
int buffer_push (Buffer *buf, int16_t value){
    // Verify if buffer size is not full
    if(buf->size == BUFFER_SIZE){
        return -1;
    }
    // Insert data into buffer and increment it counter
    buf->data[buf->size] = value;
    buf->size++;
    return 1;
}
void buffer_clean (Buffer *buf){
    for (uint16_t i = 0; i < buf->size ; i++){
        buf->data[i] = 0;
    }
    buf->size = 0;
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