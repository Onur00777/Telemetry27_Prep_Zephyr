#include "app_tasks.h"
#include "app_config.h"
#include "modem.h"
#include "modem_internal.h"
#include "telemetry_protocol.h"
#include "telemetry_buffer.h"
#include "can_decode.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern IWDG_HandleTypeDef hiwdg;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

osMutexId_t bufferMutexHandle;
static const osMutexAttr_t bufferMutexAttr = { .name = "bufferMutex" };

typedef enum {
    PERIPHERAL_BMS = 0,
    PERIPHERAL_INV,
    NUM_PERIPHERALS
} CAN_Peripherals_t;

static volatile uint32_t peripherals_last_rx_time[NUM_PERIPHERALS];
static const uint32_t peripheral_timeout_limits[NUM_PERIPHERALS] = {1000, 500};
static const PeripheralError_t peripheral_error_codes[NUM_PERIPHERALS] = {ERR_BMS_TIMEOUT, ERR_INV_TIMEOUT};

void App_CanRxCallback(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    CAN_ProcessRxFrame(hfdcan, header, data);
    CAN_Peripherals_t peripheral;
    uint8_t found = 0;
    switch (header->Identifier) {
    case CANID_BMS:
        peripheral = PERIPHERAL_BMS;
        found = 1;
        break;
    case CANID_INV_HIGHSPEED:
        peripheral = PERIPHERAL_INV;
        found = 1;
        break;
    default:
        break;
    }
    if (found) {
        peripherals_last_rx_time[peripheral] = HAL_GetTick();
        if (can_timeout_error == peripheral_error_codes[peripheral]) {
            can_timeout_error = ERR_NONE;
        }
    }
}

static void TelemetryTask(void *argument)
{
    (void)argument;
    for (int i = 0; i < NUM_PERIPHERALS; i++) {
        peripherals_last_rx_time[i] = HAL_GetTick();
    }
    last_modem_rx_tick = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

#if APP_IWDG_ENABLE_REFRESH
        IWDG->KR = 0x0000AAAAu;
#endif
#if APP_GSM_SILENCE_WATCHDOG
        if ((now - last_modem_rx_tick) > APP_GSM_SILENCE_RESET_MS) {
            NVIC_SystemReset();
        }
#endif
        for (int i = 0; i < NUM_PERIPHERALS; i++) {
            if ((now - peripherals_last_rx_time[i]) > peripheral_timeout_limits[i]) {
                can_timeout_error = peripheral_error_codes[i];
            }
        }

        if (Modem_IsReady()) {
            if (osMutexAcquire(bufferMutexHandle, 100) == osOK) {
#if APP_TELEMETRY_BENCH_TEST
                Telemetry_BenchFill();
#endif
                Telemetry_TransmitFrame();
                Modem_RequestUdpSend(TransmitBuffer, TELEMETRY_BUFFER_LENGTH);
                osMutexRelease(bufferMutexHandle);
            }
        }
        osDelay(APP_TELEMETRY_PERIOD_MS);
    }
}

static void ModemTask(void *argument)
{
    (void)argument;
    osDelay(APP_GSM_BOOT_DELAY_MS);
    __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);
    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
    Modem_Init(&huart3);
    for (;;) {
        Modem_Poll();
        osDelay(1);
    }
}

/* Final26: telemetry High, GSM Normal — modem must not starve pack/mutex. */
static const osThreadAttr_t telemetryTaskAttr = {
    .name = "telemetry_task",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 512 * 4
};

static const osThreadAttr_t modemTaskAttr = {
    .name = "modem_task",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 512 * 4
};

void App_TasksStart(void)
{
    bufferMutexHandle = osMutexNew(&bufferMutexAttr);
    modem_buffer_mutex = bufferMutexHandle;
    (void)osThreadNew(TelemetryTask, NULL, &telemetryTaskAttr);
    (void)osThreadNew(ModemTask, NULL, &modemTaskAttr);
}
