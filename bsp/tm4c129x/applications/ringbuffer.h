/**
 * @brief   a simple ringbuffer, DO NOT support dynamic expanded memory
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdlib.h>

#define MAX_RINGBUFFER_LEN 2048 - 6

struct RingBuffer_t
{
    size_t rb_capacity;
    char  *rb_head;
    char  *rb_tail;
    char  rb_buff[MAX_RINGBUFFER_LEN];
};
//struct RingBuffer;

// RingBuffer* rb_new(size_t capacity);
void rb_new(struct RingBuffer_t* rb);
void rb_free(struct RingBuffer_t *rb);

size_t rb_capacity(struct RingBuffer_t *rb);
size_t rb_can_read(struct RingBuffer_t *rb);
size_t rb_can_write(struct RingBuffer_t *rb);

size_t rb_read(struct RingBuffer_t *rb, void *data, size_t count);
size_t rb_write(struct RingBuffer_t *rb, const void *data, size_t count);

#endif
