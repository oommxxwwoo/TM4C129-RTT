//#include <assert.h>
#include <rtthread.h>
#include <components.h>
#include <stdlib.h>
#include "ringbuffer.h"

#define min(a, b) (a)<(b)?(a):(b)

//static char buff[512] = {0};
void rb_new(struct RingBuffer_t* rb)
{
    //RingBuffer *rb = (RingBuffer *)buff;//malloc(sizeof(RingBuffer) + capacity);
    //if (rb == NULL) return NULL;
    
    rb->rb_capacity = MAX_RINGBUFFER_LEN;//-sizeof(RingBuffer);//capacity;
    //rb->rb_buff     = buff+sizeof(RingBuffer);//(char*)rb + sizeof(RingBuffer);
    rb->rb_head     = rb->rb_buff;
    rb->rb_tail     = rb->rb_buff;
	
		//return rb;
};

void rb_free(struct RingBuffer_t *rb)
{
    //free((char*)rb);
}

size_t rb_capacity(struct RingBuffer_t *rb)
{
    //assert(rb != NULL);
    return rb->rb_capacity;
}
size_t rb_can_read(struct RingBuffer_t *rb)
{
    //assert(rb != NULL);
    if (rb->rb_head == rb->rb_tail) return 0;
    if (rb->rb_head < rb->rb_tail) return rb->rb_tail - rb->rb_head;
    return rb_capacity(rb) - (rb->rb_head - rb->rb_tail);
}
size_t rb_can_write(struct RingBuffer_t *rb)
{
    //assert(rb != NULL);
    return rb_capacity(rb) - rb_can_read(rb);
}

size_t rb_read(struct RingBuffer_t *rb, void *data, size_t count)
{
    //assert(rb != NULL);
    //assert(data != NULL);
    if (rb->rb_head < rb->rb_tail)
    {
        int copy_sz = min(count, rb_can_read(rb));
        rt_memcpy(data, rb->rb_head, copy_sz);
        rb->rb_head += copy_sz;
        return copy_sz;
    }
    else
    {
        if (count < rb_capacity(rb)-(rb->rb_head - rb->rb_buff))
        {
            int copy_sz = count;
            rt_memcpy(data, rb->rb_head, copy_sz);
            rb->rb_head += copy_sz;
            return copy_sz;
        }
        else
        {
            int copy_sz = rb_capacity(rb) - (rb->rb_head - rb->rb_buff);
            rt_memcpy(data, rb->rb_head, copy_sz);
            rb->rb_head = rb->rb_buff;
            copy_sz += rb_read(rb, (char*)data+copy_sz, count-copy_sz);
            return copy_sz;
        }
    }
}

size_t rb_write(struct RingBuffer_t *rb, const void *data, size_t count)
{
    //assert(rb != NULL);
    //assert(data != NULL);
    
    if (count >= rb_can_write(rb)) 
			return 0;
    
    if (rb->rb_head <= rb->rb_tail)
    {
        int tail_avail_sz = rb_capacity(rb) - (rb->rb_tail - rb->rb_buff);
        if (count <= tail_avail_sz)
        {
            rt_memcpy(rb->rb_tail, data, count);
            rb->rb_tail += count;
            if (rb->rb_tail == rb->rb_buff+rb_capacity(rb))
                rb->rb_tail = rb->rb_buff;
            return count;
        }
        else
        {
            rt_memcpy(rb->rb_tail, data, tail_avail_sz);
            rb->rb_tail = rb->rb_buff;
            
            return tail_avail_sz + rb_write(rb, (char*)data+tail_avail_sz, count-tail_avail_sz);
        }
    }
    else
    {
        rt_memcpy(rb->rb_tail, data, count);
        rb->rb_tail += count;
        return count;
    }
}
