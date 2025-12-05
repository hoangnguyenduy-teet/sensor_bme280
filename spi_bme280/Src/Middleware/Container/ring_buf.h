#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>

// Ring buffer struct
typedef struct {
	uint8_t *p_ui8Buf; // pointer to the start of the ring buffer
	uint32_t ui32End; // index of the end of the ring buffer

	// atomic index to where next element will be inserted
	uint32_t ui32Head;

	// atomic index to where next element will be removed
	uint32_t ui32Tail;
}RingBuf_t;

void ring_buf_init(RingBuf_t * const me, uint8_t p_ui8Sto[], uint32_t ui32StoLen);
uint32_t ring_buf_num_free(RingBuf_t * const me);
uint32_t ring_buf_put(RingBuf_t * const me, uint8_t const ui8Element);
uint32_t ring_buf_get(RinbBuf_t * const me, uint8_t *pui8Element);

// Ring buffer callback function for ring_buf_process_all()

// The callback processes one element and runs in the context of ring_buf_process_all()
typedef void (*RingBufHandler)(uint8_t const ui8Element);

void ring_buf_process_all(RingBuf_t * const me, RingBufHandler handler);

#endif // RING_BUF_H
