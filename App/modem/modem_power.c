#include "modem_internal.h"
#include "main.h"

#define PWRKEY_PRESS()    HAL_GPIO_WritePin(MCU_PWRK_GPIO_Port, MCU_PWRK_Pin, GPIO_PIN_SET)
#define PWRKEY_RELEASE()  HAL_GPIO_WritePin(MCU_PWRK_GPIO_Port, MCU_PWRK_Pin, GPIO_PIN_RESET)
#define RST_ASSERT()      HAL_GPIO_WritePin(MCU_RST_GPIO_Port, MCU_RST_Pin, GPIO_PIN_SET)
#define RST_RELEASE()     HAL_GPIO_WritePin(MCU_RST_GPIO_Port, MCU_RST_Pin, GPIO_PIN_RESET)

uint8_t Modem_ProbeOn(void)
{
    Modem_RingFlush();
    modem_huart->gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit(modem_huart, (uint8_t *)"AT\r\n", 4, 100);
    uint32_t t0 = osKernelGetTickCount();
    while ((osKernelGetTickCount() - t0) < MODEM_PROBE_MS) {
        IWDG->KR = 0x0000AAAAu;
        Modem_RxPump();
        if (Modem_RingContains("OK")) return 1;
        osDelay(50);
    }
    return 0;
}

uint8_t Modem_PowerOnVerified(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        PWRKEY_PRESS();
        osDelay(700);
        PWRKEY_RELEASE();
        uint32_t t0 = osKernelGetTickCount();
        while ((osKernelGetTickCount() - t0) < 15000u) {
            IWDG->KR = 0x0000AAAAu;
            osDelay(1000);
            Modem_EnsureRxAlive();
            if (Modem_ProbeOn()) return 1;
        }
    }
    return 0;
}

/* Warm-flash / PSU already on: soft power-down without cutting VBAT.
 * Final26 GSM_EnsurePoweredDown: QPOWD → wait silent ×2 → Toff-on delay.
 * Do NOT PWRKEY-toggle while still answering (toggle trap). */
void Modem_EnsurePoweredDown(void)
{
    Modem_EnsureRxAlive();
    if (!Modem_ProbeOn()) {
        Modem_RingFlush();
        return;
    }
    Modem_RingFlush();
    modem_huart->gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit(modem_huart, (uint8_t *)"AT+QPOWD=1\r\n", 12, 200);
    uint32_t t0 = osKernelGetTickCount();
    while ((osKernelGetTickCount() - t0) < MODEM_QPOWD_WAIT_MS) {
        IWDG->KR = 0x0000AAAAu;
        Modem_RxPump();
        if (Modem_RingContains("DOWN")) break;
        osDelay(50);
    }
    /* Confirm silent: two consecutive failed probes. */
    uint8_t silent = 0;
    t0 = osKernelGetTickCount();
    while (silent < 2 && (osKernelGetTickCount() - t0) < 20000u) {
        IWDG->KR = 0x0000AAAAu;
        if (Modem_ProbeOn()) {
            silent = 0;
        } else {
            silent++;
        }
    }
    osDelay(3000); /* Toff-on margin before PWRKEY */
    Modem_RingFlush();
}

void Modem_HwReset(void)
{
    RST_RELEASE();
    PWRKEY_RELEASE();
    osDelay(50);
    RST_ASSERT();
    osDelay(300);
    RST_RELEASE();
    osDelay(12000);
    HAL_UART_AbortReceive(modem_huart);
    __HAL_UART_CLEAR_FLAG(modem_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    modem_huart->RxState = HAL_UART_STATE_READY;
    Modem_ArmRxDma();
}

void Modem_InitPowerPins(void)
{
    RST_RELEASE();
    PWRKEY_RELEASE();
}
