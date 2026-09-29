#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("Telemetry27 Zephyr");
#if IS_ENABLED(CONFIG_TELEMETRY_SIM)
	LOG_INF("simulasyon: sahte CAN + sahte Quectel AT");
#else
	LOG_INF("hedef STM32G474, USART3 / FDCAN2 / FDCAN3");
#endif
	return 0;
}
