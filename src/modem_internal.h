#ifndef MODEM_INTERNAL_H_
#define MODEM_INTERNAL_H_

#include "modem.h"
#include "app_config.h"
#include "telemetry_buffer.h"
#include "platform.h"
#include "modem_port.h"

#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#define MODEM_RING_SIZE           1024u
#define MODEM_LINE_SIZE           256u
#define MODEM_AT_BUF_SIZE         256u
#define MODEM_EVT_QUEUE_SIZE      64u
#define MODEM_UDP_CMD_SIZE        64u

#define MODEM_MAX_RECOVERY        6u
#define MODEM_SEND_RETRY_MAX      3u
#define MODEM_SEND_PACE_MS        100u
#define MODEM_QIOPEN_SETTLE_MS    0u

#define MODEM_INIT_AT_MS          3000u
#define MODEM_PROMPT_TIMEOUT_MS   1000u
#define MODEM_QISEND_OK_MS        1200u
#define MODEM_QIOPEN_WAIT_MS      10000u
#define MODEM_QICLOSE_WAIT_MS     2000u
#define MODEM_CEREG_POLL_MS       2000u
#define MODEM_CEREG_GIVEUP_MS     90000u
#define MODEM_UE_RESET_GAP_MS     300000u
#define MODEM_PROBE_MS            1500u
#define MODEM_QPOWD_WAIT_MS       8000u

typedef enum {
	MODEM_EVT_NONE = 0,
	MODEM_EVT_OK,
	MODEM_EVT_ERROR,
	MODEM_EVT_PROMPT,
	MODEM_EVT_SIM_READY,
	MODEM_EVT_NET_READY,
	MODEM_EVT_UDP_SEND_OK,
	MODEM_EVT_UDP_OPEN_OK,
	MODEM_EVT_UART_ERROR
} modem_event_t;

typedef enum {
	MODEM_FSM_HW_RESET,
	MODEM_FSM_INIT_AT,
	MODEM_FSM_CHECK_NET,
	MODEM_FSM_IDLE,
	MODEM_FSM_RECOVERY_WAIT
} modem_fsm_state_t;

extern struct ring_buf modem_rx_ring;

extern volatile uint8_t modem_net_lost;
extern volatile uint8_t modem_rf_collapse;

void Modem_EventPush(modem_event_t evt);
modem_event_t Modem_EventPull(void);
void Modem_EventDrain(void);

void Modem_RingPush(const uint8_t *data, uint16_t len);
uint8_t Modem_RingContains(const char *needle);
uint8_t Modem_RingConsume(const char *needle);
int Modem_RingFindByte(char c);
void Modem_RingDrop(uint16_t n);
void Modem_RingFlush(void);

uint8_t Modem_ExtractLine(char *out, uint16_t out_len);
void Modem_ParseResponses(void);

uint8_t Modem_SendAt(const char *cmd);
uint8_t Modem_WaitEvent(modem_event_t want, uint32_t ms);
uint8_t Modem_WaitOkOrErr(uint32_t ms);
uint8_t Modem_WaitPrompt(uint32_t ms);
uint8_t Modem_WaitSendOk(uint32_t ms);
void Modem_AbortQisend(void);
uint8_t Modem_EnsureCommandMode(void);

void Modem_RxPump(void);
void Modem_SnapshotRing(char *out, uint16_t out_len);
void Modem_UartBringup(void);

uint8_t Modem_ProbeOn(void);
uint8_t Modem_PowerOnVerified(void);
void Modem_EnsurePoweredDown(void);

uint8_t Modem_InitModemConfig(void);
void Modem_QueryRfStatus(void);
uint8_t Modem_WaitRfReady(uint32_t ms);
void Modem_SampleVbat(void);
void Modem_QueryAttachDiag(void);
uint8_t Modem_QuerySocketUp(void);
uint8_t Modem_SyncReopenSocket(void);
uint8_t Modem_EnsureSocketUp(void);
uint8_t Modem_BringLinkUp(void);
uint8_t Modem_UdpSendNow(void);

void Modem_InitPowerPins(void);
void Modem_HwReset(void);
void Modem_SetPendingPayload(const uint8_t *buf, uint16_t len);
uint8_t Modem_IsUartBusy(void);

void Modem_UpdateLive(void);

#endif
