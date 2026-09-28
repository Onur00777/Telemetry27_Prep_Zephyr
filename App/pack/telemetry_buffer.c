#include "telemetry_buffer.h"
#include "stm32g4xx.h"

MainBuffer_t MainBuffer;
uint8_t TransmitBuffer[TELEMETRY_BUFFER_LENGTH];
volatile PeripheralError_t can_timeout_error = ERR_NONE;
volatile uint8_t evt_overflow_cnt = 0;

static volatile PeripheralError_t runtime_error = ERR_NONE;

uint8_t Telemetry_ErrorPriority(PeripheralError_t err)
{
    switch (err) {
    case ERR_FDCAN2_RUNTIME:
    case ERR_FDCAN3_RUNTIME:
    case ERR_UART3_RUNTIME: return 1u;
    case ERR_GSM_SEND_FAIL:
    case ERR_GSM_AT_TIMEOUT: return 2u;
    case ERR_GSM_SIM_REJECT:
    case ERR_GSM_NO_NETWORK: return 3u;
    case ERR_BMS_CELL_VOLTAGE: return 4u;
    case ERR_BMS_TIMEOUT:
    case ERR_BMS_VOLT_STALE: return 5u;
    case ERR_INV_TIMEOUT: return 6u;
    case ERR_DASHBOARD_TIMEOUT: return 7u;
    case ERR_BRAKE_TIMEOUT: return 8u;
    case ERR_IMU_TIMEOUT: return 9u;
    case ERR_DAMPER_TIMEOUT: return 10u;
    case ERR_GPS_NO_FIX: return 11u;
    case ERR_LVBMS_TIMEOUT: return 12u;
    case ERR_QUEUE_FULL:
    case ERR_MUTEX_TIMEOUT: return 13u;
    default: return 99u;
    }
}

void Telemetry_SetError(PeripheralError_t err)
{
    if (err == ERR_NONE) return;
    __disable_irq();
    if (runtime_error == ERR_NONE ||
        Telemetry_ErrorPriority(err) < Telemetry_ErrorPriority(runtime_error)) {
        runtime_error = err;
    }
    __enable_irq();
}

void Telemetry_ClearError(PeripheralError_t err)
{
    __disable_irq();
    if (runtime_error == err) runtime_error = ERR_NONE;
    __enable_irq();
}

PeripheralError_t Telemetry_GetActiveError(void)
{
    PeripheralError_t active;
    __disable_irq();
    active = runtime_error;
    __enable_irq();
    if (active == ERR_NONE ||
        (can_timeout_error != ERR_NONE &&
         Telemetry_ErrorPriority(can_timeout_error) < Telemetry_ErrorPriority(active))) {
        active = can_timeout_error;
    }
    return active;
}

void Telemetry_ConsumeRuntimeError(void)
{
    __disable_irq();
    runtime_error = ERR_NONE;
    __enable_irq();
}
