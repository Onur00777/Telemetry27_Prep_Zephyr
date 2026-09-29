#include "app_config.h"
#include "modem.h"
#include "platform.h"
#include "telemetry_buffer.h"
#include "telemetry_protocol.h"
#if defined(CONFIG_TELEMETRY_SIM)
#include "sim_vehicle.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(telemetry, LOG_LEVEL_INF);

#define TELEMETRY_STACK_SIZE 2048
#if defined(CONFIG_TELEMETRY_SIM)
#define TELEMETRY_PRIORITY   6
#else
#define TELEMETRY_PRIORITY   5
#endif

#if !defined(CONFIG_TELEMETRY_SIM)
static void log_pack(void)
{
	int volts_x10 = (int)(MainBuffer.f[BMS_TOTAL_VOLTAGE_f] * 10.0f);
	int temp_x100 = (int)(MainBuffer.f[BMS_MAX_CELL_TEMP_f] * 100.0f);
	int soc_x1000 = (int)(MainBuffer.f[BMS_ESTIMATED_SoC_f] * 1000.0f);

	LOG_INF("paket hiz=%u km/h paket=%d.%d V tmax=%d.%02d C soc=%d.%03d",
		MainBuffer.u8[VCU_VEHICLE_SPEED_u8],
		volts_x10 / 10, volts_x10 % 10,
		temp_x100 / 100, temp_x100 % 100,
		soc_x1000 / 1000, soc_x1000 % 1000);
}
#endif

static void telemetry_thread(void *p1, void *p2, void *p3)
{
	uint32_t ticks = 0u;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	telemetry_lock();
	telemetry_seed_timeouts();
	telemetry_unlock();
	LOG_INF("telemetry thread %u ms", APP_TELEMETRY_PERIOD_MS);

	while (1) {
#if defined(CONFIG_TELEMETRY_SIM)
		uint8_t pace = 0u;
#endif

		platform_watchdog_kick();
		if (telemetry_lock_timeout(100) == 0) {
			telemetry_check_timeouts();
			if (Modem_IsReady()) {
#if APP_TELEMETRY_BENCH_TEST
				Telemetry_BenchFill();
#endif
				Telemetry_TransmitFrame();
				if (Modem_RequestUdpSend(TransmitBuffer, TELEMETRY_BUFFER_LENGTH)) {
					ticks++;
#if defined(CONFIG_TELEMETRY_SIM)
					if (sim_race_armed) {
						pace = 1u;
					}
#else
					if ((ticks % 10u) == 0u) {
						log_pack();
					}
#endif
				}
			}
			telemetry_unlock();
		} else {
			Telemetry_SetError(ERR_MUTEX_TIMEOUT);
		}
#if defined(CONFIG_TELEMETRY_SIM)
		if (pace) {
			sim_pace_signal();
		}
#endif
		k_msleep(APP_TELEMETRY_PERIOD_MS);
	}
}

K_THREAD_DEFINE(telemetry_tid, TELEMETRY_STACK_SIZE, telemetry_thread,
		NULL, NULL, NULL, TELEMETRY_PRIORITY, 0, 0);
