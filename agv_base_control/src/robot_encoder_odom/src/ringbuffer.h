
#pragma once

#include <strings.h>
#include <stdbool.h>

#include "logging.h"

typedef struct
{
	float *buffer;
	size_t head, tail, size, capacity;
} ringbuffer_handle_t;

typedef enum
{
	RINGBUFFER_OK,
	RINGBUFFER_EMPTY,
	RINGBUFFER_FULL,
	RINGBUFFER_ARGUMENT_ERROR,
	RINGBUFFER_MAX,
} ringbuffer_state_t;

static const char __attribute__((unused)) * RINGBUFFER_STATE_STRING[] = {
	"RINGBUFFER_OK",
	"RINGBUFFER_EMPTY",
	"RINGBUFFER_FULL",
	"RINGBUFFER_ARGUMENT_ERROR",
	"RINGBUFFER_MAX"};

void ringbuffer_init(ringbuffer_handle_t *handle, float *buffer, size_t capacity);
void ringbuffer_clear(ringbuffer_handle_t *handle);

size_t ringbuffer_get_size(ringbuffer_handle_t *handle);
size_t ringbuffer_get_capacity(ringbuffer_handle_t *handle);
bool ringbuffer_is_full(ringbuffer_handle_t *handle);

ringbuffer_state_t ringbuffer_put(ringbuffer_handle_t *handle, float data_in);
ringbuffer_state_t ringbuffer_fill(ringbuffer_handle_t *handle, float data_in);
ringbuffer_state_t ringbuffer_peek(ringbuffer_handle_t *handle, float *data_out);
ringbuffer_state_t ringbuffer_get(ringbuffer_handle_t *handle, float *data_out);

// A differentiator implemtation using ringbuffer
ringbuffer_state_t differentiator_update(ringbuffer_handle_t *handle, float data_in, float *data_out);