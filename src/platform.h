#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdint.h>

uint32_t platform_tick_ms(void);
void platform_watchdog_kick(void);
void platform_reboot(void);

#endif
