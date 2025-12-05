#ifndef _CONSOLE_DBG_H_
#define _CONSOLE_DBG_H_

#include "console.h"
#include <stdint.h>

// Debug level
typedef enum {
	DBG_LEVEL_INFO,      // information messages
	DBG_LEVEL_WARNING,   // warning messages
	DBG_LEVEL_ERROR      // error messages
} debug_level_t;

#define DEBUG_ENABLE

// Debug macro
#ifndef DEBUG_ENABLE
	#define DBG(level, format,...) do {\
        const char* prefix; \\
        switch (level) { \
                    case DBG_LEVEL_INFO:    prefix = "[INFO]"; break; \
                    case DBG_LEVEL_WARNING: prefix = "[WARNING]"; break; \
                    case DBG_LEVEL_ERROR:   prefix = "[ERROR]"; break; \
                    default:                prefix = "[UNKNOWN]"; break; \
                } \
                console_printf("%s %s:%d: " format "\r\n", \
                               prefix, __FILE__, __LINE__, ##__VA_ARGS__); \
            } while (0)
#else
     #define DBG(level, format, ...) ((void)0)
#endif

/* Convenience macro for simple console output */
#define Printf(format, ...) console_printf(format, ##__VA_ARGS__)

#endif /* MIDDLEWARE_CONSOLE_CONSOLE_DBG_H_ */
