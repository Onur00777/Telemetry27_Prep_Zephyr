#include "modem_internal.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(modem_fsm, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_TELEMETRY_SIM)

void Modem_InitPowerPins(void)
{
	LOG_INF("sim PWRKEY/RESET idle dusuk (PC6/PC7)");
}

uint8_t Modem_ProbeOn(void)
{
	Modem_RingFlush();
	if (modem_port_write((const uint8_t *)"AT\r\n", 4) != 0) {
		return 0u;
	}
	uint32_t t0 = platform_tick_ms();

	while ((platform_tick_ms() - t0) < MODEM_PROBE_MS) {
		platform_watchdog_kick();
		Modem_RxPump();
		if (Modem_RingContains("OK")) {
			return 1u;
		}
		k_msleep(50);
	}
	return 0u;
}

uint8_t Modem_PowerOnVerified(void)
{
	LOG_INF("sim PWRKEY 700 ms");
	k_msleep(50);
	return Modem_ProbeOn();
}

void Modem_EnsurePoweredDown(void)
{
	Modem_RingFlush();
	(void)modem_port_write((const uint8_t *)"AT+QPOWD=1\r\n", 12);
	k_msleep(50);
	Modem_RingFlush();
}

void Modem_HwReset(void)
{
	LOG_INF("sim RESET pulse");
	k_msleep(20);
	(void)modem_port_rearm();
}

#else

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec modem_pwrkey =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), modem_pwrkey_gpios);
static const struct gpio_dt_spec modem_reset =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), modem_reset_gpios);

static void pwrkey_press(void)
{
	(void)gpio_pin_set_dt(&modem_pwrkey, 1);
}

static void pwrkey_release(void)
{
	(void)gpio_pin_set_dt(&modem_pwrkey, 0);
}

static void rst_assert(void)
{
	(void)gpio_pin_set_dt(&modem_reset, 1);
}

static void rst_release(void)
{
	(void)gpio_pin_set_dt(&modem_reset, 0);
}

void Modem_InitPowerPins(void)
{
	if (!gpio_is_ready_dt(&modem_pwrkey) || !gpio_is_ready_dt(&modem_reset)) {
		LOG_ERR("modem GPIO hazir degil");
		return;
	}
	(void)gpio_pin_configure_dt(&modem_pwrkey, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&modem_reset, GPIO_OUTPUT_INACTIVE);
	rst_release();
	pwrkey_release();
}

uint8_t Modem_ProbeOn(void)
{
	Modem_RingFlush();
	if (modem_port_write((const uint8_t *)"AT\r\n", 4) != 0) {
		return 0u;
	}
	uint32_t t0 = platform_tick_ms();

	while ((platform_tick_ms() - t0) < MODEM_PROBE_MS) {
		platform_watchdog_kick();
		Modem_RxPump();
		if (Modem_RingContains("OK")) {
			return 1u;
		}
		k_msleep(50);
	}
	return 0u;
}

uint8_t Modem_PowerOnVerified(void)
{
	for (int attempt = 0; attempt < 3; attempt++) {
		pwrkey_press();
		k_msleep(700);
		pwrkey_release();
		uint32_t t0 = platform_tick_ms();

		while ((platform_tick_ms() - t0) < 15000u) {
			platform_watchdog_kick();
			k_msleep(1000);
			if (Modem_ProbeOn()) {
				return 1u;
			}
		}
	}
	return 0u;
}

void Modem_EnsurePoweredDown(void)
{
	if (!Modem_ProbeOn()) {
		Modem_RingFlush();
		return;
	}
	Modem_RingFlush();
	(void)modem_port_write((const uint8_t *)"AT+QPOWD=1\r\n", 12);
	uint32_t t0 = platform_tick_ms();

	while ((platform_tick_ms() - t0) < MODEM_QPOWD_WAIT_MS) {
		platform_watchdog_kick();
		Modem_RxPump();
		if (Modem_RingContains("DOWN")) {
			break;
		}
		k_msleep(50);
	}
	uint8_t silent = 0;

	t0 = platform_tick_ms();
	while (silent < 2 && (platform_tick_ms() - t0) < 20000u) {
		platform_watchdog_kick();
		if (Modem_ProbeOn()) {
			silent = 0;
		} else {
			silent++;
		}
	}
	k_msleep(3000);
	Modem_RingFlush();
}

void Modem_HwReset(void)
{
	rst_release();
	pwrkey_release();
	k_msleep(50);
	rst_assert();
	k_msleep(300);
	rst_release();
	k_msleep(12000);
	(void)modem_port_rearm();
}

#endif
