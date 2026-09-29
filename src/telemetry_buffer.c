#include "telemetry_buffer.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>

MainBuffer_t MainBuffer;
uint8_t TransmitBuffer[TELEMETRY_BUFFER_LENGTH];
volatile PeripheralError_t can_timeout_error = ERR_NONE;
volatile uint8_t evt_overflow_cnt;

static volatile PeripheralError_t runtime_error = ERR_NONE;
static K_MUTEX_DEFINE(buffer_mu);

static uint32_t bms_rx_ms;
static uint32_t inv_rx_ms;

void telemetry_lock(void)
{
	k_mutex_lock(&buffer_mu, K_FOREVER);
}

void telemetry_unlock(void)
{
	k_mutex_unlock(&buffer_mu);
}

int telemetry_lock_timeout(uint32_t timeout_ms)
{
	return k_mutex_lock(&buffer_mu, K_MSEC(timeout_ms)) == 0 ? 0 : -1;
}

void telemetry_on_can_id(uint32_t id)
{
	uint32_t now = k_uptime_get_32();

	if (id == 0x060u) {
		bms_rx_ms = now;
		if (can_timeout_error == ERR_BMS_TIMEOUT) {
			can_timeout_error = ERR_NONE;
		}
	} else if (id == 0x0B0u) {
		inv_rx_ms = now;
		if (can_timeout_error == ERR_INV_TIMEOUT) {
			can_timeout_error = ERR_NONE;
		}
	}
}

void telemetry_seed_timeouts(void)
{
	uint32_t now = k_uptime_get_32();

	bms_rx_ms = now;
	inv_rx_ms = now;
}

void telemetry_check_timeouts(void)
{
	uint32_t now = k_uptime_get_32();

	if ((now - bms_rx_ms) > 1000u) {
		can_timeout_error = ERR_BMS_TIMEOUT;
	}
	if ((now - inv_rx_ms) > 500u) {
		can_timeout_error = ERR_INV_TIMEOUT;
	}
}

uint8_t Telemetry_ErrorPriority(PeripheralError_t err)
{
	switch (err) {
	case ERR_FDCAN2_RUNTIME:
	case ERR_FDCAN3_RUNTIME:
	case ERR_UART3_RUNTIME:
		return 1u;
	case ERR_GSM_SEND_FAIL:
	case ERR_GSM_AT_TIMEOUT:
		return 2u;
	case ERR_GSM_SIM_REJECT:
	case ERR_GSM_NO_NETWORK:
		return 3u;
	case ERR_BMS_CELL_VOLTAGE:
		return 4u;
	case ERR_BMS_TIMEOUT:
	case ERR_BMS_VOLT_STALE:
		return 5u;
	case ERR_INV_TIMEOUT:
		return 6u;
	case ERR_DASHBOARD_TIMEOUT:
		return 7u;
	case ERR_BRAKE_TIMEOUT:
		return 8u;
	case ERR_IMU_TIMEOUT:
		return 9u;
	case ERR_DAMPER_TIMEOUT:
		return 10u;
	case ERR_GPS_NO_FIX:
		return 11u;
	case ERR_LVBMS_TIMEOUT:
		return 12u;
	case ERR_QUEUE_FULL:
	case ERR_MUTEX_TIMEOUT:
		return 13u;
	default:
		return 99u;
	}
}

void Telemetry_SetError(PeripheralError_t err)
{
	unsigned int key;

	if (err == ERR_NONE) {
		return;
	}
	key = irq_lock();
	if (runtime_error == ERR_NONE ||
	    Telemetry_ErrorPriority(err) < Telemetry_ErrorPriority(runtime_error)) {
		runtime_error = err;
	}
	irq_unlock(key);
}

void Telemetry_ClearError(PeripheralError_t err)
{
	unsigned int key = irq_lock();

	if (runtime_error == err) {
		runtime_error = ERR_NONE;
	}
	irq_unlock(key);
}

PeripheralError_t Telemetry_GetActiveError(void)
{
	PeripheralError_t active;
	unsigned int key = irq_lock();

	active = runtime_error;
	irq_unlock(key);
	if (active == ERR_NONE ||
	    (can_timeout_error != ERR_NONE &&
	     Telemetry_ErrorPriority(can_timeout_error) < Telemetry_ErrorPriority(active))) {
		active = can_timeout_error;
	}
	return active;
}

void Telemetry_ConsumeRuntimeError(void)
{
	unsigned int key = irq_lock();

	runtime_error = ERR_NONE;
	irq_unlock(key);
}
