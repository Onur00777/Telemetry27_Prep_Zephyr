#include "modem_internal.h"

#include <stdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(modem_fsm, LOG_LEVEL_INF);

static bool expecting_payload;
static uint32_t last_tx_ms;
static uint8_t have_tx;
static uint32_t qisend_n;
static uint32_t qisend_gap;
static char qisend_cmd[48];

static void sim_reply(const char *text)
{
	Modem_RingPush((const uint8_t *)text, (uint16_t)strlen(text));
}

static void reply_one_line(const char *text, char *out, size_t out_len)
{
	size_t w = 0u;

	if (out_len == 0u) {
		return;
	}
	while (*text != '\0' && w + 1u < out_len) {
		char c = *text++;

		if (c == '\r' || c == '\n') {
			if (w == 0u || (w >= 3u && out[w - 1u] == ' ' &&
					out[w - 2u] == '/' && out[w - 3u] == ' ')) {
				continue;
			}
			if (w + 4u >= out_len) {
				break;
			}
			out[w++] = ' ';
			out[w++] = '/';
			out[w++] = ' ';
			continue;
		}
		out[w++] = c;
	}
	while (w >= 3u && out[w - 1u] == ' ' && out[w - 2u] == '/' && out[w - 3u] == ' ') {
		w -= 3u;
	}
	out[w] = '\0';
}

static uint32_t mark_tx(void)
{
	uint32_t now = k_uptime_get_32();
	uint32_t gap = have_tx ? (now - last_tx_ms) : 0u;

	have_tx = 1u;
	last_tx_ms = now;
	return gap;
}

static void trace_tx(const char *cmd)
{
	LOG_INF("TX %s  | onceki TX %u ms", cmd, mark_tx());
}

static void trace_rx(const char *text)
{
	char line[96];

	reply_one_line(text, line, sizeof(line));
	LOG_INF("RX %s", line[0] != '\0' ? line : text);
}

int modem_port_init(void)
{
	LOG_INF("AT iz: TX firmware, RX modul. 'onceki TX' iki komut arasi bekleme");
	return 0;
}

int modem_port_rearm(void)
{
	return 0;
}

void modem_port_rx_pump(void)
{
}

int modem_port_write(const uint8_t *data, uint16_t len)
{
	char cmd[160];
	uint16_t n;

	if (data == NULL || len == 0u) {
		return -1;
	}

	if (expecting_payload) {
		expecting_payload = false;
		sim_reply("\r\nSEND OK\r\n");
		if (qisend_n <= 6u) {
			char yuk[40];

			snprintf(yuk, sizeof(yuk), "yuk %u crc=0x%02x", len, data[len - 1u]);
			trace_tx(yuk);
			trace_rx("SEND OK");
		} else {
			LOG_INF("TX %s -> > -> yuk %u crc=0x%02x -> SEND OK  | onceki TX %u ms",
				qisend_cmd, len, data[len - 1u], qisend_gap);
		}
		return 0;
	}

	n = len < (sizeof(cmd) - 1u) ? len : (uint16_t)(sizeof(cmd) - 1u);
	memcpy(cmd, data, n);
	cmd[n] = '\0';
	for (char *p = cmd; *p != '\0'; p++) {
		if (*p == '\r' || *p == '\n') {
			*p = '\0';
			break;
		}
	}
	if (cmd[0] == '\0') {
		return 0;
	}

	if (strncmp(cmd, "AT+QISEND", 9) == 0) {
		qisend_n++;
		qisend_gap = mark_tx();
		{
			size_t c = strlen(cmd);

			if (c >= sizeof(qisend_cmd)) {
				c = sizeof(qisend_cmd) - 1u;
			}
			memcpy(qisend_cmd, cmd, c);
			qisend_cmd[c] = '\0';
		}
		expecting_payload = true;
		sim_reply("\r\n> ");
		if (qisend_n <= 6u) {
			LOG_INF("TX %s  | onceki TX %u ms", cmd, qisend_gap);
			trace_rx(">");
		}
		return 0;
	}

	trace_tx(cmd);

	if (strncmp(cmd, "AT+CPIN?", 8) == 0) {
		trace_rx("\r\n+CPIN: READY\r\nOK\r\n");
		sim_reply("\r\n+CPIN: READY\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CSQ", 6) == 0) {
		trace_rx("\r\n+CSQ: 21,0\r\nOK\r\n");
		sim_reply("\r\n+CSQ: 21,0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CEREG?", 9) == 0 || strncmp(cmd, "AT+CGREG?", 9) == 0) {
		trace_rx("\r\n+CEREG: 2,1\r\nOK\r\n");
		sim_reply("\r\n+CEREG: 2,1\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CBC", 6) == 0) {
		trace_rx("\r\n+CBC: 0,75,3850\r\nOK\r\n");
		sim_reply("\r\n+CBC: 0,75,3850\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CFUN?", 8) == 0) {
		trace_rx("\r\n+CFUN: 1\r\nOK\r\n");
		sim_reply("\r\n+CFUN: 1\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QCFG", 7) == 0) {
		trace_rx("\r\n+QCFG: \"airplanecontrol\",0,0\r\nOK\r\n");
		sim_reply("\r\n+QCFG: \"airplanecontrol\",0,0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QIACT?", 9) == 0) {
		trace_rx("\r\n+QIACT: 1,1,1,\"10.64.0.2\"\r\nOK\r\n");
		sim_reply("\r\n+QIACT: 1,1,1,\"10.64.0.2\"\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QIOPEN", 9) == 0) {
		trace_rx("\r\nOK\r\n+QIOPEN: 0,0\r\n");
		sim_reply("\r\nOK\r\n+QIOPEN: 0,0\r\n");
	} else if (strncmp(cmd, "AT+QISTATE", 10) == 0) {
		trace_rx("\r\n+QISTATE: 0,\"UDP\",\"135.125.196.63\",5010,0,2,1,0,0,\"uart1\"\r\nOK\r\n");
		sim_reply("\r\n+QISTATE: 0,\"UDP\",\"135.125.196.63\",5010,0,2,1,0,0,\"uart1\"\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+COPS?", 8) == 0) {
		trace_rx("\r\n+COPS: 0,0,\"SIM\",7\r\nOK\r\n");
		sim_reply("\r\n+COPS: 0,0,\"SIM\",7\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CEER", 7) == 0) {
		trace_rx("\r\n+CEER: 0\r\nOK\r\n");
		sim_reply("\r\n+CEER: 0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QPOWD", 8) == 0) {
		trace_rx("\r\nOK\r\nPOWERED DOWN\r\n");
		sim_reply("\r\nOK\r\nPOWERED DOWN\r\n");
	} else {
		trace_rx("\r\nOK\r\n");
		sim_reply("\r\nOK\r\n");
	}
	return 0;
}
