#include "modem_internal.h"

static modem_event_t evt_queue[MODEM_EVT_QUEUE_SIZE];
static volatile uint8_t evt_head;
static volatile uint8_t evt_tail;
static char line_buf[MODEM_LINE_SIZE];

#if APP_MODEM_DEBUG_LOG
char modem_log[MODEM_LOG_LINES][MODEM_LOG_LEN];
static volatile uint8_t modem_log_idx;
#endif

/* Read reply:  +CEREG: <n>,<stat>[,<tac>,<ci>[,<AcT>]]  — both leading fields are digits.
 * URC (n=2):   +CEREG: <stat>[,<tac>,<ci>[,<AcT>]]      — <tac>/<ci> are quoted strings.
 * Taking the field after the first comma unconditionally read <tac> on the URC and
 * clobbered cereg to -1 exactly when the module announced it had registered. */
static int parse_cereg_stat(const char *line)
{
    const char *p = strchr(line, ':');
    if (!p) return -1;
    p++;
    while (*p == ' ') p++;
    if (*p < '0' || *p > '9') return -1;
    int stat = (int)(*p - '0');
    const char *comma = strchr(p, ',');
    if (comma) {
        const char *q = comma + 1;
        while (*q == ' ') q++;
        if (*q >= '0' && *q <= '9') {
            stat = (int)(*q - '0'); /* read reply: first field was <n> */
        }
    }
    return (stat <= 5) ? stat : -1;
}

void Modem_EventPush(modem_event_t evt)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t next = (uint8_t)((evt_head + 1u) % MODEM_EVT_QUEUE_SIZE);
    if (next != evt_tail) {
        evt_queue[evt_head] = evt;
        evt_head = next;
    }
    __set_PRIMASK(primask);
}

modem_event_t Modem_EventPull(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (evt_head == evt_tail) {
        __set_PRIMASK(primask);
        return MODEM_EVT_NONE;
    }
    modem_event_t e = evt_queue[evt_tail];
    evt_tail = (uint8_t)((evt_tail + 1u) % MODEM_EVT_QUEUE_SIZE);
    __set_PRIMASK(primask);
    return e;
}

void Modem_EventDrain(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    evt_tail = evt_head;
    __set_PRIMASK(primask);
}

static void handle_qiopen_line(const char *line)
{
    const char *comma = strchr(line, ',');
    int err = -1;
    if (comma) {
        const char *ep = comma + 1;
        while (*ep == ' ') ep++;
        if (*ep >= '0' && *ep <= '9') {
            err = 0;
            while (*ep >= '0' && *ep <= '9') {
                err = err * 10 + (*ep - '0');
                ep++;
            }
        }
    }
    modem_live.last_qiopen_err = (int16_t)err;
    if (err == 0 || err == 563) {
        modem_live.socket_open = 1;
        Modem_EventPush(MODEM_EVT_UDP_OPEN_OK);
    } else {
        Telemetry_SetError(ERR_GSM_SEND_FAIL);
        Modem_EventPush(MODEM_EVT_ERROR);
    }
}

void Modem_ParseResponses(void)
{
    while (Modem_ExtractLine(line_buf, sizeof(line_buf))) {
        size_t len = strlen(line_buf);
        while (len > 0 && (line_buf[len - 1] == '\r' || line_buf[len - 1] == '\n')) {
            line_buf[--len] = '\0';
        }
        if (len == 0) continue;

        strncpy((char *)modem_live.last_line, line_buf, sizeof(modem_live.last_line) - 1);
        ((char *)modem_live.last_line)[sizeof(modem_live.last_line) - 1] = '\0';
#if APP_MODEM_DEBUG_LOG
        strncpy(modem_log[modem_log_idx], line_buf, MODEM_LOG_LEN - 1);
        modem_log[modem_log_idx][MODEM_LOG_LEN - 1] = '\0';
        modem_log_idx = (uint8_t)((modem_log_idx + 1u) % MODEM_LOG_LINES);
#endif

        if (strncmp(line_buf, "+CFUN:", 6) == 0) {
            const char *p = line_buf + 6;
            while (*p == ' ') p++;
            if (*p >= '0' && *p <= '9') {
                modem_live.cfun = (int8_t)(*p - '0');
            }
        } else if (strncmp(line_buf, "+CPIN: READY", 12) == 0) {
            modem_live.cpin_ok = 1;
            Modem_EventPush(MODEM_EVT_SIM_READY);
        } else if (strncmp(line_buf, "+CPIN: NOT", 10) == 0) {
            modem_live.cpin_ok = 0;
            Telemetry_SetError(ERR_GSM_SIM_REJECT);
        } else if (strncmp(line_buf, "+CME ERROR", 10) == 0) {
            Modem_EventPush(MODEM_EVT_ERROR);
        } else if (strncmp(line_buf, "+QIURC: \"closed\"", 16) == 0) {
            modem_live.socket_open = 0;
            Telemetry_SetError(ERR_GSM_SEND_FAIL);
        } else if (strncmp(line_buf, "+QIURC: \"pdpdeact\"", 18) == 0) {
            modem_live.socket_open = 0;
            modem_live.pdp_active = 0;
            Telemetry_SetError(ERR_GSM_SEND_FAIL);
        } else if (strncmp(line_buf, "+QIACT:", 7) == 0) {
            const char *p = strchr(line_buf, ',');
            if (p) {
                p++;
                while (*p == ' ') p++;
                modem_live.pdp_active = (uint8_t)(*p == '1');
            }
        } else if (strncmp(line_buf, "+QISTATE:", 9) == 0) {
            /* +QISTATE: <id>,<svc>,<ip>,<rport>,<lport>,<state>,...  state 2=connected */
            int field = 0;
            const char *p = line_buf;
            while (*p && field < 5) {
                if (*p == ',') field++;
                p++;
            }
            while (*p == ' ') p++;
            if (field == 5 && *p == '2') {
                modem_live.socket_open = 1;
                Modem_EventPush(MODEM_EVT_UDP_OPEN_OK);
            }
        } else if (strncmp(line_buf, "+CEREG:", 7) == 0 || strncmp(line_buf, "+CGREG:", 7) == 0) {
            int stat = parse_cereg_stat(line_buf);
            if (stat < 0) continue; /* unparsable — keep the last known state */
            modem_live.cereg = (int8_t)stat;
            if (stat == 1 || stat == 5) {
                modem_net_lost = 0;
                Modem_EventPush(MODEM_EVT_NET_READY);
            } else if (stat == 0 || stat == 3 || stat == 4) {
                /* Final26: kayıt düştü → IDLE CHECK_NET'e iner. stat=2 arıyor, dokunma. */
                modem_net_lost = 1;
                Telemetry_SetError(ERR_GSM_NO_NETWORK);
            } else if (stat == 2) {
                Telemetry_SetError(ERR_GSM_NO_NETWORK);
            }
        } else if (strncmp(line_buf, "+CSQ:", 5) == 0) {
            const char *p = line_buf + 5;
            while (*p == ' ') p++;
            int v = 0;
            while (*p >= '0' && *p <= '9') {
                v = v * 10 + (*p - '0');
                p++;
            }
            /* Final26: iyi CSQ → ani 99 = RF collapse. */
            if (modem_live.csq >= 0 && modem_live.csq <= 31 && v == 99) {
                modem_rf_collapse = 1;
                modem_net_lost = 1;
                modem_live.socket_open = 0;
            }
            modem_live.csq = (int8_t)v;
        } else if (strncmp(line_buf, "+QCFG: \"airplanecontrol\"", 24) == 0) {
            /* +QCFG: "airplanecontrol",<enable>,<status> */
            const char *p = strchr(line_buf, ',');
            if (p) {
                p++;
                while (*p == ' ') p++;
                if (*p >= '0' && *p <= '9') modem_live.airplane_ctl = (int8_t)(*p - '0');
                p = strchr(p, ',');
                if (p) {
                    p++;
                    while (*p == ' ') p++;
                    if (*p >= '0' && *p <= '9') modem_live.airplane_status = (int8_t)(*p - '0');
                }
            }
        } else if (strncmp(line_buf, "+CBC:", 5) == 0) {
            /* +CBC: <bcs>,<bcl>,<voltage_mV> — VBAT as the module itself sees it,
             * i.e. after FB1 and the local bulk caps. */
            const char *p = strchr(line_buf, ',');
            if (p) p = strchr(p + 1, ',');
            if (p) {
                p++;
                while (*p == ' ') p++;
                uint32_t mv = 0;
                while (*p >= '0' && *p <= '9') {
                    mv = mv * 10u + (uint32_t)(*p - '0');
                    p++;
                }
                if (mv > 0u && mv < 6000u) {
                    modem_live.vbat_mv = (uint16_t)mv;
                    if (modem_live.vbat_min_mv == 0u || mv < modem_live.vbat_min_mv) {
                        modem_live.vbat_min_mv = (uint16_t)mv;
                    }
                }
            }
        } else if (strncmp(line_buf, "+CEER:", 6) == 0) {
            const char *p = line_buf + 6;
            while (*p == ' ') p++;
            strncpy((char *)modem_live.ceer, p, sizeof(modem_live.ceer) - 1);
            ((char *)modem_live.ceer)[sizeof(modem_live.ceer) - 1] = '\0';
        } else if (strncmp(line_buf, "+COPS:", 6) == 0) {
            const char *p = line_buf + 6;
            while (*p == ' ') p++;
            strncpy((char *)modem_live.oper, p, sizeof(modem_live.oper) - 1);
            ((char *)modem_live.oper)[sizeof(modem_live.oper) - 1] = '\0';
        } else if (strstr(line_buf, "+QIOPEN:") != NULL) {
            handle_qiopen_line(line_buf);
        } else if (strncmp(line_buf, "SEND OK", 7) == 0) {
            Modem_EventPush(MODEM_EVT_UDP_SEND_OK);
        } else if (strncmp(line_buf, "SEND FAIL", 9) == 0) {
            Telemetry_SetError(ERR_GSM_SEND_FAIL);
            Modem_EventPush(MODEM_EVT_ERROR);
        } else if (strcmp(line_buf, "OK") == 0) {
            Modem_EventPush(MODEM_EVT_OK);
        } else if (strncmp(line_buf, "ERROR", 5) == 0) {
            Modem_EventPush(MODEM_EVT_ERROR);
        } else if (line_buf[0] == '>') {
            Modem_EventPush(MODEM_EVT_PROMPT);
        }
    }
}
