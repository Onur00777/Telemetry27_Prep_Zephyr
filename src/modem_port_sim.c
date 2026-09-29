#include "modem_internal.h"

#include <stdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(modem_fsm, LOG_LEVEL_INF);

static bool expecting_payload;

static void sim_reply(const char *text)
{
	Modem_RingPush((const uint8_t *)text, (uint16_t)strlen(text));
}

static void log_cmd(const char *cmd)
{
	char shown[80];
	size_t n = strlen(cmd);

	if (n >= sizeof(shown)) {
		n = sizeof(shown) - 1u;
	}
	memcpy(shown, cmd, n);
	shown[n] = '\0';
	LOG_DBG("modem << %s", shown);
}

int modem_port_init(void)
{
	LOG_INF("sim modem portu, AT yanitlari halkaya yazilir");
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
		LOG_DBG("modem << yuk %u bayt crc=0x%02x", len, data[len - 1u]);
		sim_reply("\r\nSEND OK\r\n");
		LOG_DBG("modem >> SEND OK");
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
	log_cmd(cmd);

	if (strncmp(cmd, "AT+QISEND", 9) == 0) {
		expecting_payload = true;
		sim_reply("\r\n> ");
		LOG_DBG("modem >> >");
	} else if (strncmp(cmd, "AT+CPIN?", 8) == 0) {
		sim_reply("\r\n+CPIN: READY\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CSQ", 6) == 0) {
		sim_reply("\r\n+CSQ: 21,0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CEREG?", 9) == 0 || strncmp(cmd, "AT+CGREG?", 9) == 0) {
		sim_reply("\r\n+CEREG: 2,1\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CBC", 6) == 0) {
		sim_reply("\r\n+CBC: 0,75,3850\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CFUN?", 8) == 0) {
		sim_reply("\r\n+CFUN: 1\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QCFG", 7) == 0) {
		sim_reply("\r\n+QCFG: \"airplanecontrol\",0,0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QIACT?", 9) == 0) {
		sim_reply("\r\n+QIACT: 1,1,1,\"10.64.0.2\"\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QIOPEN", 9) == 0) {
		sim_reply("\r\nOK\r\n+QIOPEN: 0,0\r\n");
		LOG_INF("modem >> QIOPEN UDP %s:%d", APP_UDP_HOST, APP_UDP_PORT);
	} else if (strncmp(cmd, "AT+QISTATE", 10) == 0) {
		sim_reply("\r\n+QISTATE: 0,\"UDP\",\"135.125.196.63\",5010,0,2,1,0,0,\"uart1\"\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+COPS?", 8) == 0) {
		sim_reply("\r\n+COPS: 0,0,\"SIM\",7\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+CEER", 7) == 0) {
		sim_reply("\r\n+CEER: 0\r\nOK\r\n");
	} else if (strncmp(cmd, "AT+QPOWD", 8) == 0) {
		sim_reply("\r\nOK\r\nPOWERED DOWN\r\n");
	} else {
		sim_reply("\r\nOK\r\n");
	}
	return 0;
}
