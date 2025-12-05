#ifndef BSP_BOARD_H_
#define BSP_BOARD_H_

enum {
	LED_GREEN = 0,
		LED_BLUE ,
		LED_RED

};

#define STM32F4_DISCO
//#define MY_HARDWARE

# ifdef STM32F4_DISCO
	#include "stm32f1xx.h"
	#include "stm32f1xx_hal_spi.h"

	#define	RTC_SPI			SPI1
	#define RTC_CS1_Pin 		 8
	#define RTC_CS1_GPIO_Port GPIOA
	#define RTC_CS2_Pin 		 9
	#define RTC_CS2_GPIO_Port GPIOA


#endif

# ifdef MY_HARDWARE_STM32F1
	#include "stm32f1xx.h"
	#include "stm32f1xx_hal_spi.h"
	#define CONSOLE_SPI SPI1

#endif

#define VERSION_MAJOR      (1)
#define VERSION_MINOR      (0)
#define VERSION_PATCH      (0)

#endif /* BSP_BOARD_H_ */
