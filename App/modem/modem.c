#include "modem.h"
#include "modem_internal.h"

volatile modem_live_t modem_live;
volatile uint32_t last_modem_rx_tick;
volatile uint8_t modem_net_lost;
volatile uint8_t modem_rf_collapse;

static modem_fsm_state_t fsm_state;
static uint8_t send_pending;
uint8_t recovery_count;
static uint8_t send_retry_count;
static uint8_t qiopen_fail_count;
static uint32_t last_send_tick;
static uint32_t send_backoff_ms;

void Modem_InitPowerPins(void);
void Modem_HwReset(void);
void Modem_SetPendingPayload(const uint8_t *buf, uint16_t len);
uint8_t Modem_IsUartBusy(void);

void Modem_UpdateLive(void)
{
    modem_live.rx_fill = modem_rx_head;
    modem_live.rx_tail = modem_rx_tail;
    modem_live.dma_old_pos = modem_dma_old_pos;
    modem_live.send_pending = send_pending;
    modem_live.send_retry_count = send_retry_count;
    modem_live.send_backoff_ms = (uint16_t)send_backoff_ms;
    modem_live.fsm = (modem_live_fsm_t)fsm_state;
    if (modem_huart != NULL) {
        modem_live.uart_rx_busy =
            (uint8_t)(modem_huart->RxState == HAL_UART_STATE_BUSY_RX ? 1u : 0u);
    }
}

void Modem_Init(UART_HandleTypeDef *huart)
{
    modem_huart = huart;
    memset((void *)&modem_live, 0, sizeof(modem_live));
    modem_live.csq = -1; /* unset — not "99" until AT+CSQ says so */
    modem_live.cereg = -1;
    modem_live.cfun = -1;
    modem_live.airplane_ctl = -1;
    modem_live.airplane_status = -1;
    fsm_state = MODEM_FSM_INIT_AT;
    send_pending = 0;
    recovery_count = 0;
    send_retry_count = 0;
    qiopen_fail_count = 0;
    last_send_tick = 0;
    send_backoff_ms = 0;
    modem_net_lost = 0;
    modem_rf_collapse = 0;
    last_modem_rx_tick = HAL_GetTick();

    Modem_InitPowerPins();
    osDelay(100);
    Modem_UartBringup();

    /* Final26 GSM_Init — warm flash (PSU on): AT OK → DO NOT QPOWD/PWRKEY.
     * Mandatory QPOWD was leaving cfun=1 cpin_ok=1 but csq=99 / cereg=0 forever. */
    modem_live.state = MODEM_STATE_BOOTING;
#if APP_MODEM_WARM_FLASH_QPOWD
    Modem_EnsurePoweredDown();
    Modem_UartBringup();
    if (!Modem_ProbeOn()) {
        if (!Modem_PowerOnVerified()) {
            uint8_t filler[64];
            memset(filler, 'U', sizeof(filler));
            for (int k = 0; k < 8; k++) {
                HAL_UART_Transmit(modem_huart, filler, sizeof(filler), 200);
            }
            HAL_UART_Transmit(modem_huart, (uint8_t *)"\r\n", 2, 100);
            osDelay(1000);
            (void)Modem_PowerOnVerified();
        }
    }
#else
    if (!Modem_ProbeOn()) {
        uint8_t filler[64];
        memset(filler, 'U', sizeof(filler));
        for (int k = 0; k < 8; k++) {
            HAL_UART_Transmit(modem_huart, filler, sizeof(filler), 200);
        }
        HAL_UART_Transmit(modem_huart, (uint8_t *)"\r\n", 2, 100);
        osDelay(1000);
        if (!Modem_ProbeOn()) {
            (void)Modem_PowerOnVerified();
        }
    } else {
        modem_live.warm_skip_qpowd = 1;
    }
#endif
    HAL_UART_Transmit(modem_huart, (uint8_t *)"AT\r\n", 4, 100);
    Modem_UartBringup();
    modem_live.state = MODEM_STATE_BOOTING;
}

bool Modem_RequestUdpSend(const uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0 || len > TELEMETRY_BUFFER_LENGTH) return false;
    Modem_SetPendingPayload(buf, len);
    send_pending = 1;
    return true;
}

bool Modem_IsReady(void)
{
    return modem_live.ready != 0;
}

modem_state_t Modem_GetState(void)
{
    return modem_live.state;
}

const modem_live_t *Modem_GetLive(void)
{
    return (const modem_live_t *)&modem_live;
}

static uint8_t rf_good_live(void)
{
    return (uint8_t)(modem_live.csq >= 10 && modem_live.csq <= 31);
}

static uint8_t net_up_live(void)
{
    return (uint8_t)((modem_live.cereg == 1 || modem_live.cereg == 5) &&
                     modem_live.pdp_active);
}

void Modem_Poll(void)
{
    /* Soft RX keep-alive (rate-limited inside). Do not thrash ArmRxDma. */
    Modem_EnsureRxAlive();
    Modem_RxPump();
    Modem_UpdateLive();
    Modem_ParseResponses();

    switch (fsm_state) {
    case MODEM_FSM_HW_RESET:
        modem_live.state = MODEM_STATE_RECOVERING;
        modem_live.ready = 0;
        Modem_HwReset(); /* already soft-rearms USART3 RX DMA */
        fsm_state = MODEM_FSM_INIT_AT;
        break;

    case MODEM_FSM_INIT_AT:
        modem_live.state = MODEM_STATE_BOOTING;
        if (Modem_InitModemConfig()) {
            fsm_state = MODEM_FSM_CHECK_NET;
        } else {
            if (recovery_count++ >= MODEM_MAX_RECOVERY) {
                recovery_count = 0;
                fsm_state = MODEM_FSM_HW_RESET;
            } else {
                fsm_state = MODEM_FSM_RECOVERY_WAIT;
            }
        }
        break;

    case MODEM_FSM_CHECK_NET:
        modem_live.state = MODEM_STATE_LINKING;
        modem_live.ready = 0;
        if (!Modem_ProbeOn()) {
            if (Modem_PowerOnVerified()) {
                fsm_state = MODEM_FSM_INIT_AT;
            } else {
                fsm_state = MODEM_FSM_HW_RESET;
            }
            break;
        }
        if (net_up_live()) {
            /* Şebeke + PDP ayakta: QICSGP/QIACT/CEREG tekrar etme — sadece soket (Final26). */
            if (Modem_EnsureSocketUp()) {
                recovery_count = 0;
                qiopen_fail_count = 0;
                fsm_state = MODEM_FSM_IDLE;
            } else {
                modem_live.recovery_count++;
                osDelay(2000u);
            }
        } else if (Modem_BringLinkUp()) {
            recovery_count = 0;
            qiopen_fail_count = 0;
            fsm_state = MODEM_FSM_IDLE;
        } else if (!rf_good_live() && ++qiopen_fail_count >= 20u) {
            /* RESET_N cannot create coverage, so bad RF is the weakest possible
             * reason to pulse it — CHECK_NET already hard-resets when AT dies.
             * Kept only as a very-last-resort unwedge after ~30 min of no service. */
            qiopen_fail_count = 0;
            fsm_state = MODEM_FSM_HW_RESET;
        } else {
            modem_live.recovery_count++;
            osDelay(rf_good_live() ? 500u : 5000u);
        }
        break;

    case MODEM_FSM_IDLE:
        /* Final26: CEREG drop → leave IDLE before burning QISEND ladder. */
        if (modem_net_lost) {
            modem_net_lost = 0;
            modem_live.ready = 0;
            modem_live.socket_open = 0;
            modem_live.state = MODEM_STATE_LINKING;
            fsm_state = MODEM_FSM_CHECK_NET;
            break;
        }
        /* Soket düştü ama şebeke ayakta: soft reopen (Final26 send-fail path). */
        if (!modem_live.socket_open) {
            modem_live.ready = 0;
            modem_live.state = MODEM_STATE_RECOVERING;
            if (!Modem_EnsureSocketUp()) {
                modem_live.state = MODEM_STATE_LINKING;
                fsm_state = MODEM_FSM_CHECK_NET;
                break;
            }
            modem_live.socket_open = 1;
        }
        modem_live.ready = 1;
        modem_live.state = MODEM_STATE_READY;

        if (send_pending && !Modem_IsUartBusy()) {
            uint32_t now = osKernelGetTickCount();
            if (last_send_tick != 0 &&
                (now - last_send_tick) < (MODEM_SEND_PACE_MS + send_backoff_ms)) {
                break;
            }
            last_send_tick = now;
            send_pending = 0;
            modem_live.ready = 0;
            modem_live.state = MODEM_STATE_SENDING;

            if (Modem_UdpSendNow()) {
                modem_live.qisend_ok++;
                send_retry_count = 0;
                send_backoff_ms = 0;
                recovery_count = 0;
                modem_live.ready = 1;
                modem_live.state = MODEM_STATE_READY;
                break;
            }

            /* Final26 IDLE fail: EnsureCommandMode, same-socket retry.
             * Reopen only when QISTATE says down — never ForceRxArm thrash. */
            modem_live.qisend_fail++;
            send_pending = 1;
            last_send_tick = 0;
            Modem_EnsureCommandMode();
            if (send_retry_count < 255u) {
                send_retry_count++;
            }
            send_backoff_ms = 0;
            if (send_retry_count >= MODEM_SEND_RETRY_MAX) {
                send_retry_count = 0;
                if (!Modem_QuerySocketUp()) {
                    if (!Modem_SyncReopenSocket()) {
                        modem_live.ready = 0;
                        modem_live.socket_open = 0;
                        modem_live.state = MODEM_STATE_LINKING;
                        fsm_state = MODEM_FSM_CHECK_NET;
                        break;
                    }
                    modem_live.socket_open = 1;
                }
            }
            modem_live.ready = 1;
            modem_live.state = MODEM_STATE_READY;
        }
        break;

    case MODEM_FSM_RECOVERY_WAIT:
        /* INIT_AT failed — retry config, do not leap into CHECK_NET with
         * cpin/csq/cfun still unset (looks like "linking" while AT is dead). */
        modem_live.state = MODEM_STATE_RECOVERING;
        modem_live.recovery_count++;
        osDelay(1000u << (recovery_count > 4 ? 4 : recovery_count));
        fsm_state = MODEM_FSM_INIT_AT;
        break;

    default:
        fsm_state = MODEM_FSM_INIT_AT;
        break;
    }
}
