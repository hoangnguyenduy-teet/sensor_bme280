#ifndef BSP_BOARD_CONFIGS_H_
#define BSP_BOARD_CONFIGS_H_

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif

#ifndef UNUSED_FUNC
#define UNUSED_FUNC __attribute__((unused))
#endif

#define CONFIG_CONSOLE_TX_BUFFER_SIZE 256
#define CONFIG_CONSOLE_RX_BUFFER_SIZE 256

#endif /* BSP_BOARD_CONFIGS_H_ */
