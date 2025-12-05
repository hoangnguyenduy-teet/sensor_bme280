#ifndef DRIVER_COMPONENTS_DIO_DO_H_
#define DRIVER_COMPONENTS_DIO_DO_H_
#include "stdint.h"
#include "stdbool.h"
#include "stm32f1xx.h"

typedef struct
{
	GPIO_TypeDef * hGPIO;
	uint32_t		ui32PinMask;
	bool			bStatus;
}do_t;

typedef struct
{
	GPIO_TypeDef * hGPIOInit;
	uint32_t		ui32PinNumInit;
	bool			bStatusInit;
}doInit_t;

uint32_t do_init(do_t *me, const doInit_t * const init);
void do_set(do_t *me);
void do_reset(do_t *me);
void do_toggle(do_t *me);

#endif /* DRIVER_COMPONENTS_DIO_DO_H_ */
