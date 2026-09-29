#include "modem_internal.h"

static char at_buf[MODEM_AT_BUF_SIZE];
static volatile uint8_t uart_tx_busy;
static uint8_t qisend_need_flush;

uint8_t Modem_SendAt(const char *cmd)
{
    uart_tx_busy = 1;
    strncpy(at_buf, cmd, sizeof(at_buf) - 1);
    at_buf[sizeof(at_buf) - 1] = '\0';
    int st = modem_port_write((const uint8_t *)at_buf, (uint16_t)strlen(at_buf));
    uart_tx_busy = 0;
    return (uint8_t)(st == 0);
}

uint8_t Modem_WaitEvent(modem_event_t want, uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        Modem_RxPump();
        Modem_ParseResponses();
        modem_event_t e = Modem_EventPull();
        if (e == want) return 1;
        if (e == MODEM_EVT_ERROR && want != MODEM_EVT_ERROR) return 0;
        k_msleep(10);
    }
    return 0;
}

uint8_t Modem_WaitOkOrErr(uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        Modem_RxPump();
        Modem_ParseResponses();
        modem_event_t e = Modem_EventPull();
        if (e == MODEM_EVT_OK || e == MODEM_EVT_ERROR) {
            Modem_EventDrain();
            return 1;
        }
        k_msleep(10);
    }
    return 0;
}

/* Final26 WaitPrompt + RxPump: circular DMA stays armed for the whole window.
 * NEVER HAL_UART_DMAStop / byte-poll here — that desyncs RX after a few sends
 * (prompt_miss ≈ qisend_fail storm while socket_open stays 1).
 * RxPump copies NDTR without waiting for IDLE so a lone '>' is not stranded. */
uint8_t Modem_WaitPrompt(uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        Modem_RxPump();
        Modem_UpdateLive();
        {
            int at = Modem_RingFindByte('>');
            if (at >= 0) {
                Modem_RingDrop((uint16_t)(at + 1));
                qisend_need_flush = 0;
                Modem_EventDrain();
                return 1;
            }
        }
        /* Final26: only framed ERROR — bare "ERROR" false-positives on ring noise. */
        if (Modem_RingContains("SEND FAIL") ||
            Modem_RingContains("\nERROR") ||
            Modem_RingContains("\rERROR")) {
            qisend_need_flush = 0;
            Modem_EventDrain();
            return 2;
        }
        /* First 40 ms busy-poll so osDelay cannot miss a single-byte IDLE. */
        if ((platform_tick_ms() - t0) >= 40u) {
            k_msleep(1);
        }
    }
    Modem_RxPump();
    Modem_SnapshotRing((char *)modem_live.last_line, sizeof(modem_live.last_line));
    qisend_need_flush = 1;
    return 0;
}

static uint8_t Modem_ConsumeFromRing(const char *needle)
{
    return Modem_RingConsume(needle);
}

uint8_t Modem_WaitSendOk(uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        Modem_RxPump();
        if (Modem_ConsumeFromRing("SEND OK")) {
            Modem_ParseResponses();
            Modem_EventDrain();
            return 1;
        }
        if (Modem_RingContains("SEND FAIL")) {
            Modem_ParseResponses();
            Modem_EventDrain();
            modem_live.send_fail++;
            return 0;
        }
        k_msleep(5);
    }
    return 0;
}

void Modem_AbortQisend(void)
{
    uint8_t esc = 0x1Bu;
    /* Final26: single ESC. Multi-ESC + DMA rearm thrashed command mode. */
    (void)modem_port_write(&esc, 1);
    qisend_need_flush = 0;
    (void)Modem_WaitOkOrErr(300);
    Modem_EventDrain();
}

/* 1 = AT→OK görüldü. 0 = cevap yok (yine de QISEND denenebilir). */
uint8_t Modem_EnsureCommandMode(void)
{
    Modem_EventDrain();
    if (qisend_need_flush) Modem_AbortQisend();

    /* Final26: no RingFlush here — flush races with late '>' / URCs. */
    (void)modem_port_write((const uint8_t *)"AT\r\n", 4);
    if (Modem_WaitEvent(MODEM_EVT_OK, 400)) {
        Modem_EventDrain();
        modem_live.at_ok_count++;
        return 1;
    }
    Modem_EventDrain();
    (void)modem_port_write((const uint8_t *)"AT\r\n", 4);
    if (Modem_WaitEvent(MODEM_EVT_OK, 400)) {
        Modem_EventDrain();
        modem_live.at_ok_count++;
        return 1;
    }
    modem_live.at_fail_count++;
    Modem_EventDrain();
    return 0;
}

#if APP_MODEM_RF_PROFILE_ENABLE
static void Modem_ApplyRfProfile(void)
{
    char cmd[96];
    (void)snprintf(cmd, sizeof(cmd),
        "AT+QCFG=\"sarcfg\",\"lte\",%u,100,0,%u\r\n",
        (unsigned)APP_MODEM_SARCFG_MAX_POWER,
        (unsigned)APP_MODEM_SARCFG_BAND);
    (void)Modem_SendAt(cmd);
    (void)Modem_WaitOkOrErr(2000);
}
#endif

uint8_t Modem_InitModemConfig(void)
{
    Modem_RingFlush();
    (void)modem_port_write((const uint8_t *)"AT\r\n", 4);
    k_msleep(100);
    if (!Modem_SendAt("AT\r\n") || !Modem_WaitEvent(MODEM_EVT_OK, MODEM_INIT_AT_MS)) {
        if (!Modem_SendAt("AT\r\n")) {
            modem_live.at_fail_count++;
            return 0;
        }
        if (!Modem_WaitEvent(MODEM_EVT_OK, MODEM_INIT_AT_MS)) {
            modem_live.at_fail_count++;
            return 0;
        }
    }
    modem_live.at_ok_count++;
    Modem_RingFlush();
    (void)Modem_SendAt("ATE0\r\n");
    (void)Modem_WaitOkOrErr(500);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QSCLK=0\r\n");
    (void)Modem_WaitOkOrErr(500);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CMER=3,0,0,2\r\n");
    (void)Modem_WaitOkOrErr(500);
#if APP_MODEM_RF_PROFILE_ENABLE
    Modem_RingFlush();
    Modem_ApplyRfProfile();
#endif
    /* W_DISABLE# (pin 18) is tied to GND on this board while Quectel specifies
     * "if unused, keep it open". The pin only takes effect when airplanecontrol
     * is enabled, and that setting lives in module NV — so read it back once:
     * airplane_ctl=1 means the module is held in airplane mode by hardware. */
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QCFG=\"airplanecontrol\"\r\n");
    (void)Modem_WaitOkOrErr(1000);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CFUN=1\r\n");
    (void)Modem_WaitOkOrErr(2000);
    Modem_SampleVbat();
    for (uint8_t i = 0; i < 12u; i++) {
        platform_watchdog_kick();
        Modem_RingFlush();
        (void)Modem_SendAt("AT+CPIN?\r\n");
        if (Modem_WaitEvent(MODEM_EVT_SIM_READY, 1500)) {
            Modem_EventDrain();
            Modem_SampleVbat();
            (void)Modem_WaitRfReady(20000u);
            return 1;
        }
        k_msleep(500);
    }
    return 0;
}

static uint8_t net_registered(void)
{
    return (uint8_t)(modem_live.cereg == 1 || modem_live.cereg == 5);
}

void Modem_QueryRfStatus(void)
{
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CFUN?\r\n");
    k_msleep(250);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+CPIN?\r\n");
    k_msleep(350);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+CSQ\r\n");
    k_msleep(250);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+CEREG?\r\n");
    k_msleep(350);
    Modem_ParseResponses();
}

uint8_t Modem_WaitRfReady(uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        (void)Modem_SendAt("AT+CSQ\r\n");
        k_msleep(400);
        Modem_ParseResponses();
        (void)Modem_SendAt("AT+CEREG?\r\n");
        k_msleep(400);
        Modem_ParseResponses();
        Modem_SampleVbat();
        if (net_registered()) return 1;
        if (modem_live.cereg == 2) return 1;
        if (modem_live.csq >= 0 && modem_live.csq <= 31) return 1;
        k_msleep(1000);
    }
    return (uint8_t)(modem_live.csq >= 0 && modem_live.csq <= 31);
}

static uint8_t rf_good(void)
{
    return (uint8_t)(modem_live.csq >= 10 && modem_live.csq <= 31);
}

/* VBAT as the module measures it — the only reading taken after FB1 and the
 * local bulk caps. EG915N HW design: Vmin 3.4 V, Vnom 3.8 V. */
void Modem_SampleVbat(void)
{
    (void)Modem_SendAt("AT+CBC\r\n");
    k_msleep(150);
    Modem_ParseResponses();
}

/* Why the network refused: +CEER carries the attach reject cause, +COPS? the
 * PLMN actually selected. Only queried while we are failing to register. */
void Modem_QueryAttachDiag(void)
{
    (void)Modem_SendAt("AT+CEER\r\n");
    k_msleep(200);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+COPS?\r\n");
    k_msleep(250);
    Modem_ParseResponses();
}

/* AT+CFUN=1,1 is <rst>=1 — a full module reboot, not a radio restart. A cold
 * LTE cell search plus attach needs an uninterrupted 30-90 s, so firing this on
 * every csq=99 retry restarted the search each time and attach could never
 * finish. Last resort only, widely spaced, and the volatile URC config has to be
 * re-sent once the module answers AT again. */
static uint8_t Modem_UeReset(void)
{
    static uint32_t last_ue_reset_tick;
    uint32_t now = platform_tick_ms();
    if (last_ue_reset_tick != 0u && (now - last_ue_reset_tick) < MODEM_UE_RESET_GAP_MS) {
        return 0;
    }
    last_ue_reset_tick = now;
    modem_live.ue_reset_count++;
    modem_live.socket_open = 0;
    modem_live.pdp_active = 0;
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CFUN=1,1\r\n");
    (void)Modem_WaitOkOrErr(5000);
    for (uint8_t i = 0; i < 24u; i++) {
        platform_watchdog_kick();
        if (Modem_ProbeOn()) break;
        k_msleep(500);
    }
    modem_live.csq = -1;
    modem_live.cereg = -1;
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QSCLK=0\r\n");
    (void)Modem_WaitOkOrErr(500);
    (void)Modem_SendAt("AT+CEREG=2\r\n");
    (void)Modem_WaitOkOrErr(500);
    return 1;
}

/* NEVER AT+CFUN=0 — if CFUN=1 fails after CFUN=0, radio stays OFF forever
 * (csq=99, cereg=0). Soft recover: force CFUN=1 and re-query. */
static uint8_t Modem_EnsureRadioOn(void)
{
    Modem_EnsureCommandMode();
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CFUN=1\r\n");
    (void)Modem_WaitOkOrErr(15000);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CFUN?\r\n");
    k_msleep(300);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+CPIN?\r\n");
    k_msleep(300);
    Modem_ParseResponses();
    (void)Modem_SendAt("AT+CSQ\r\n");
    k_msleep(200);
    Modem_ParseResponses();
    return (uint8_t)(modem_live.cfun == 1);
}

/* Former RadioRestart used CFUN=0→1; mid-fail left fun=0. Now only EnsureRadioOn. */
static uint8_t Modem_RadioRestart(void)
{
    static uint32_t last_rf_restart_tick;
    uint32_t now = platform_tick_ms();
    if (last_rf_restart_tick != 0u && (now - last_rf_restart_tick) < 45000u) {
        return 0;
    }
    last_rf_restart_tick = now;
    modem_live.rf_restart_count++;
    modem_rf_collapse = 0;
    modem_live.socket_open = 0;
    uint8_t ok = Modem_EnsureRadioOn();
    if (ok) {
        (void)Modem_SendAt("AT+QSCLK=0\r\n");
        (void)Modem_WaitOkOrErr(500);
        (void)Modem_SendAt("AT+CEREG=2\r\n");
        (void)Modem_WaitEvent(MODEM_EVT_OK, 500);
        modem_live.csq = -1;
    }
    return ok;
}

static uint8_t wait_net_ready(uint32_t ms)
{
    uint32_t t0 = platform_tick_ms();
    uint8_t csq99_streak = 0;
    uint8_t did_radio_restart = 0;
    while ((platform_tick_ms() - t0) < ms) {
        platform_watchdog_kick();
        Modem_ParseResponses();
        if (net_registered()) {
            Modem_EventDrain();
            return 1;
        }
        /* Radio off ⇒ cereg stays 0 forever — re-assert CFUN=1. */
        if (modem_live.cfun == 0) {
            (void)Modem_EnsureRadioOn();
        }
        (void)Modem_SendAt("AT+CEREG?\r\n");
        uint32_t t1 = platform_tick_ms();
        while ((platform_tick_ms() - t1) < 1500u) {
            Modem_ParseResponses();
            if (net_registered() || Modem_EventPull() == MODEM_EVT_NET_READY) {
                Modem_EventDrain();
                return 1;
            }
            k_msleep(50);
        }
        (void)Modem_SendAt("AT+CSQ\r\n");
        k_msleep(200);
        Modem_ParseResponses();
        Modem_SampleVbat();
        if (modem_live.csq == 99) {
            if (csq99_streak < 255u) csq99_streak++;
            if (!did_radio_restart && csq99_streak >= 8u &&
                (platform_tick_ms() - t0) >= 10000u) {
                did_radio_restart = 1;
                (void)Modem_RadioRestart();
                csq99_streak = 0;
            }
        } else {
            csq99_streak = 0;
            if (rf_good() && (platform_tick_ms() - t0) >= 8000u) {
                return net_registered();
            }
        }
    }
    return net_registered();
}

uint8_t Modem_QuerySocketUp(void)
{
    /* Final26: no RingFlush; flushing dropped QISTATE replies → false "socket down"
     * → full BringLinkUp while cereg/pdp still up (user snapshot: csq=21, socket=0). */
    Modem_EventDrain();
    modem_live.socket_open = 0;
    (void)Modem_SendAt("AT+QISTATE=1,0\r\n");
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < 500u) {
        platform_watchdog_kick();
        Modem_RxPump();
        Modem_ParseResponses();
        if (modem_live.socket_open) {
            Modem_EventDrain();
            return 1;
        }
        k_msleep(20);
    }
    Modem_EventDrain();
    return modem_live.socket_open;
}

uint8_t Modem_SyncReopenSocket(void)
{
    char cmd[MODEM_UDP_CMD_SIZE];
    modem_live.reopen_count++;
    modem_live.socket_open = 0;
    Modem_EnsureCommandMode();
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QICLOSE=0\r\n");
    /* ERROR = zaten kapalı — Final26: sorun değil */
    (void)Modem_WaitOkOrErr(MODEM_QICLOSE_WAIT_MS);
    Modem_RingFlush();
    (void)snprintf(cmd, sizeof(cmd),
        "AT+QIOPEN=1,0,\"UDP\",\"%s\",%d,0,0\r\n", APP_UDP_HOST, APP_UDP_PORT);
    if (!Modem_SendAt(cmd)) return 0;
    uint32_t t0 = platform_tick_ms();
    while ((platform_tick_ms() - t0) < MODEM_QIOPEN_WAIT_MS) {
        platform_watchdog_kick();
        Modem_RxPump();
        Modem_ParseResponses();
        modem_event_t e = Modem_EventPull();
        if (modem_live.socket_open || e == MODEM_EVT_UDP_OPEN_OK) {
            modem_live.socket_open = 1;
            Modem_EventDrain();
            return 1;
        }
        if (e == MODEM_EVT_ERROR) break;
        k_msleep(20);
    }
    return Modem_QuerySocketUp();
}

/* Final26 Gsm_EnsureSocketUp: önce reopen; RF iyiyse PDP+reopen; full BringLinkUp değil. */
uint8_t Modem_EnsureSocketUp(void)
{
    if (modem_live.socket_open) {
        if (Modem_QuerySocketUp()) return 1;
    }
    if (Modem_SyncReopenSocket()) return 1;
    if (rf_good()) {
        Modem_EnsureCommandMode();
        Modem_RingFlush();
        (void)Modem_SendAt("AT+QIACT=1\r\n");
        (void)Modem_WaitOkOrErr(10000);
        return Modem_SyncReopenSocket();
    }
    return 0;
}

uint8_t Modem_BringLinkUp(void)
{
    char apn_cmd[96];
    modem_live.ready = 0;
    modem_live.state = MODEM_STATE_LINKING;

    /* Fast path: LTE+PDP already up (post-send recovery) — socket only. */
    if (net_registered() && modem_live.pdp_active) {
        if (Modem_QuerySocketUp()) {
            modem_live.socket_open = 1;
            return 1;
        }
        return Modem_SyncReopenSocket();
    }

    Modem_EnsureCommandMode();
    Modem_RingFlush();
    (void)Modem_EnsureRadioOn();
    (void)Modem_SendAt("AT+QSCLK=0\r\n");
    (void)Modem_WaitOkOrErr(500);
    if (modem_rf_collapse) {
        (void)Modem_RadioRestart();
    }
    Modem_RingFlush();
    (void)Modem_SendAt("AT+CEREG=2\r\n");
    (void)Modem_WaitOkOrErr(500);
    (void)Modem_SendAt("AT+CEREG?\r\n");
    k_msleep(400);
    Modem_ParseResponses();
    if (!net_registered()) {
        /* Fresh boot / warm QPOWD: allow full CEREG window (csq often 99 for 10–30 s). */
        if (!wait_net_ready(MODEM_CEREG_GIVEUP_MS)) {
            Modem_QueryAttachDiag();
            /* Only after a full patient window has failed is a module reboot
             * worth the cost of restarting cell search from cold. */
            if (modem_live.cpin_ok && Modem_UeReset()) {
                if (!wait_net_ready(MODEM_CEREG_GIVEUP_MS)) {
                    Modem_QueryAttachDiag();
                    return 0;
                }
            } else {
                return 0;
            }
        }
    }
    Modem_RingFlush();
    (void)snprintf(apn_cmd, sizeof(apn_cmd),
        "AT+QICSGP=1,1,\"%s\",\"\",\"\",1\r\n", APP_APN);
    (void)Modem_SendAt(apn_cmd);
    (void)Modem_WaitOkOrErr(3000);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QIACT=1\r\n");
    (void)Modem_WaitOkOrErr(10000);
    Modem_RingFlush();
    (void)Modem_SendAt("AT+QIACT?\r\n");
    k_msleep(400);
    Modem_ParseResponses();
    if (Modem_SyncReopenSocket()) {
        modem_live.socket_open = 1;
        return 1;
    }
    return 0;
}

static uint16_t pending_send_len;
static uint8_t udp_snapshot[TELEMETRY_BUFFER_LENGTH];

/* Final26 Gsm_UdpSendNow — flush → QISEND → wait '>' → payload → SEND OK.
 * No mid-send ForceRxArm: circular DMA stays armed for the whole session. */
uint8_t Modem_UdpSendNow(void)
{
    char cmd[MODEM_UDP_CMD_SIZE];

    Modem_ParseResponses();
    if (!modem_live.socket_open) return 0;
    if (pending_send_len == 0 || pending_send_len > TELEMETRY_BUFFER_LENGTH) return 0;

    modem_live.last_send_len = pending_send_len;
    Modem_RingFlush();
    (void)snprintf(cmd, sizeof(cmd), "AT+QISEND=0,%u\r\n", (unsigned)pending_send_len);
    if (!Modem_SendAt(cmd)) return 0;
    Modem_EventDrain();

    uint8_t pr = Modem_WaitPrompt(MODEM_PROMPT_TIMEOUT_MS);
    if (pr == 2u) {
        modem_live.prompt_err++;
        Modem_SnapshotRing((char *)modem_live.last_line, sizeof(modem_live.last_line));
        Modem_AbortQisend();
        return 0;
    }
    if (pr != 1u) {
        modem_live.prompt_miss++;
        Modem_SnapshotRing((char *)modem_live.last_line, sizeof(modem_live.last_line));
        Modem_AbortQisend();
        return 0;
    }

    if (modem_port_write(udp_snapshot, pending_send_len) != 0) {
        return 0;
    }
    if (!Modem_WaitSendOk(MODEM_QISEND_OK_MS)) {
        modem_live.sendok_timeout++;
        Modem_AbortQisend();
        return 0;
    }
    return 1;
}

void Modem_SetPendingPayload(const uint8_t *buf, uint16_t len)
{
    pending_send_len = len;
    memcpy(udp_snapshot, buf, len);
}

uint8_t Modem_IsUartBusy(void)
{
    return uart_tx_busy;
}
