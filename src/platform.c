#include "platform.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

uint32_t platform_tick_ms(void)
{
	return k_uptime_get_32();
}

void platform_watchdog_kick(void)
{
	/* IWDG surucusu bilerek arm edilmiyor. Cagri noktalari Cube'daki
	 * IWDG->KR = 0xAAAA yerlerinde duruyor; watchdog acilinca burada
	 * wdt_feed() baglanir. */
}

void platform_reboot(void)
{
	sys_reboot(SYS_REBOOT_COLD);
}
