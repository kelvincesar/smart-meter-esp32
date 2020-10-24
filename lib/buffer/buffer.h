// Inclusion guard, to prevent multiple includes of the same header
#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>                     // C standard library

#define BUFFER_SIZE     2048            // Define buffer size for current and voltage

typedef struct {
    int16_t data[BUFFER_SIZE];
    uint16_t size;
} Buffer;

int buffer_push (Buffer *buf, int16_t value);
int is_buffer_full(Buffer *buf);
void buffer_clean (Buffer *buf);
#endif
