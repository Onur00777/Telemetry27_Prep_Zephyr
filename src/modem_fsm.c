#include "modem.h"
#include "modem_internal.h"
#include <stdlib.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(modem_fsm, LOG_LEVEL_INF);

void Modem_SimulationStop(void)
{
#if defined(CONFIG_ARCH_POSIX)
	exit(0);
#elif defined(CONFIG_QEMU_TARGET)
	/* QEMU semihosting SYS_EXIT / ADP_Stopped_ApplicationExit */
	register unsigned int cmd __asm__("r0") = 0x18;
	register unsigned int arg __asm__("r1") = 0x20026;

	__asm__ volatile("bkpt 0xAB" : : "r"(cmd), "r"(arg) : "memory");
#else
	k_sleep(K_FOREVER);
#endif
}

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
    modem_live.send_pending = send_pending;
    modem_live.send_retry_count = send_retry_count;
    modem_live.send_backoff_ms = (uint16_t)send_backoff_ms;
    modem_live.fsm = (modem_live_fsm_t)fsm_state;
    modem_live.rx_fill = (uint16_t)ring_buf_size_get(&modem_rx_ring);
    modem_live.rx_dma_circular = 1u;
    modem_live.uart_rx_busy = 1u;
}

void Modem_Init(void)
{
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
    last_modem_rx_tick = platform_tick_ms();

    if (modem_port_init() != 0) {
        LOG_ERR("modem portu acilmadi");
    }
    Modem_InitPowerPins();
    k_msleep(100);
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
                (void)modem_port_write(filler, sizeof(filler));
            }
            (void)modem_port_write((const uint8_t *)"\r\n", 2);
            k_msleep(1000);
            (void)Modem_PowerOnVerified();
        }
    }
#else
    if (!Modem_ProbeOn()) {
        uint8_t filler[64];
        memset(filler, 'U', sizeof(filler));
        for (int k = 0; k < 8; k++) {
            (void)modem_port_write(filler, sizeof(filler));
        }
        (void)modem_port_write((const uint8_t *)"\r\n", 2);
        k_msleep(1000);
        if (!Modem_ProbeOn()) {
            (void)Modem_PowerOnVerified();
        }
    } else {
        modem_live.warm_skip_qpowd = 1;
    }
#endif
    (void)modem_port_write((const uint8_t *)"AT\r\n", 4);
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
    static modem_fsm_state_t logged = MODEM_FSM_HW_RESET;
    static const char *const name[] = {
        "HW_RESET", "INIT_AT", "CHECK_NET", "IDLE", "RECOVERY"
    };

    if (fsm_state != logged && fsm_state <= MODEM_FSM_RECOVERY_WAIT) {
        LOG_INF("durum %s", name[fsm_state]);
        logged = fsm_state;
    }

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
                k_msleep(2000u);
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
            k_msleep(rf_good_live() ? 500u : 5000u);
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
            uint32_t now = platform_tick_ms();
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
                LOG_DBG("QISEND ok #%u len=%u cereg=%d csq=%d",
                        modem_live.qisend_ok, modem_live.last_send_len,
                        modem_live.cereg, modem_live.csq);
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
        k_msleep(1000u << (recovery_count > 4 ? 4 : recovery_count));
        fsm_state = MODEM_FSM_INIT_AT;
        break;

    default:
        fsm_state = MODEM_FSM_INIT_AT;
        break;
    }
}

#if IS_ENABLED(CONFIG_TELEMETRY_SIM)
#define MODEM_THREAD_DELAY_MS 300
#else
#define MODEM_THREAD_DELAY_MS APP_GSM_BOOT_DELAY_MS
#endif

#define MODEM_STACK_SIZE 4096
#define MODEM_PRIORITY   7

static void modem_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	Modem_Init();
	LOG_INF("modem thread hazir, poll 1 ms");
	while (1) {
		uint32_t now = platform_tick_ms();

		platform_watchdog_kick();
#if APP_GSM_SILENCE_WATCHDOG
		if ((now - last_modem_rx_tick) > APP_GSM_SILENCE_RESET_MS) {
			LOG_ERR("GSM sessizligi, reset");
			platform_reboot();
		}
#endif
		Modem_Poll();
		k_msleep(1);
	}
}

K_THREAD_DEFINE(modem_tid, MODEM_STACK_SIZE, modem_thread,
		NULL, NULL, NULL, MODEM_PRIORITY, 0, MODEM_THREAD_DELAY_MS);
