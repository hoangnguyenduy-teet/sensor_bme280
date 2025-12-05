#include "do.h"
#include "stdint.h"
#include "stddef.h"
#include "error_codes.h"

uint32_t do_init(do_t *me, const doInit_t * const init)
{
	if (me == NULL) return ERROR_INVALID_PARAM;
	me->hGPIO = init->hGPIOInit;
	me->ui32PinMask = (1 << init->ui32PinNumInit);
	me->bStatus = init->bStatusInit;
	if (me->bStatus) do_set(me);
	else do_reset(me);
	return ERROR_OK;

}

void do_set(do_t *me)
{
	me->hGPIO->BSRR =  me->ui32PinMask;
	me->bStatus = true;
}

void do_reset(do_t *me)
{
	me->hGPIO->BSRR =  me->ui32PinMask << 16;
	me->bStatus = false;
}

void do_toggle(do_t *me)
{
	if (me->bStatus) do_reset(me);
	else do_set(me);
}
