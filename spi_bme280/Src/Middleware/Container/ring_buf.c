#include <stdint.h>
#include <stdbool.h>

#include "ring_buf.h"
#include "atomic.h"
#include "error_codes.h"

void ring_buf_init(RingBuf_t * const me, uint8_t p_ui8Sto[], uint32_ui32Stolen) {
	me->p_ui8Buf = &p_ui8Sto[0];
	me->ui32End = ui32Stolen;
	ENTER_CRITICAL();

	me->ui32Head = 0U; // init head atomically
	me->ui32Tail = 0U; // init tail atomically
	EXIT_CRITICAL();
}

uint32_t ring_buf_put(RingBuf_t * const me, uint8_t const ui8Element) {
	uint32_t ui32RetVal;
	ENTER_CRITICAL();
    uint32_t ui32Head = me->ui32Head + 1U;
    if (ui32Head == me->ui32End) {
        ui32Head = 0U;
    }
    uint32_t ui32Tail = me->ui32Tail;
    if (ui32Head != ui32Tail) { // buffer NOT full?
        me->p_ui8Buf[me->ui32Head] = ui8Element;
        me->ui32Head = ui32Head;
        ui32RetVal = ERROR_OK;
    }
    else {
    	ui32RetVal = ERROR_BUFFER_FULL; // buffer full
    }
    EXIT_CRITICAL();
    return ui32RetVal;
}
//............................................................................
uint32_t ring_buf_get(RingBuf_t * const me, uint8_t *pui8Element) {
	uint32_t ui32RetVal;
	ENTER_CRITICAL();
    uint32_t ui32Tail = me->ui32Tail;
    uint32_t ui32Head = me->ui32Head;
    if (ui32Head != ui32Tail) { // buffer NOT empty?
        *pui8Element = me->p_ui8Buf[ui32Tail];
        ++ui32Tail;
        if (ui32Tail == me->ui32End) {
            ui32Tail = 0U;
        }
        me->ui32Tail = ui32Tail;
        ui32RetVal = ERROR_OK;
    }
    else {
        ui32RetVal = ERROR_BUFFER_EMPTY; // buffer empty
    }
    EXIT_CRITICAL();
    return ui32RetVal;
}
//............................................................................
uint32_t ring_buf_num_free(RingBuf_t * const me) {

	ENTER_CRITICAL();
    uint32_t ui32Head = me->ui32Head;
    uint32_t ui32Tail = me->ui32Tail;
    EXIT_CRITICAL();
    if (ui32Head == ui32Tail) { // buffer empty?
        return (uint32_t)(me->ui32End - 1U);
    }
    else if (ui32Head < ui32Tail) {
        return (uint32_t)(ui32Tail - ui32Head - 1U);
    }
    else {
        return (uint32_t)(me->ui32End + ui32Tail - ui32Head - 1U);
    }
}

//............................................................................
void ring_buf_process_all(RingBuf_t * const me, RingBufHandler handler) {
    uint32_t ui32Tail = atomic_load_explicit(&me->ui32Tail, memory_order_relaxed);
    uint32_t ui32Head = atomic_load_explicit(&me->ui32Head, memory_order_acquire);
    while (ui32Head != ui32Tail) { // buffer NOT empty?
        (*handler)(me->p_ui8Buf[ui32Tail]);
        ++ui32Tail;
        if (ui32Tail == me->ui32End) {
            ui32Tail = 0U;
        }
        atomic_store_explicit(&me->ui32Tail, ui32Tail, memory_order_release);
    }
}
