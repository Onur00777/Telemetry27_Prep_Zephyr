#include "modem_internal.h"

#include <zephyr/irq.h>

RING_BUF_DECLARE(modem_rx_ring, MODEM_RING_SIZE);

volatile uint32_t modem_dma_restart_err_cnt;

static uint8_t peek_buf[MODEM_RING_SIZE];
static uint8_t discard_buf[MODEM_RING_SIZE];

static uint32_t ring_copy(void)
{
	unsigned int key = irq_lock();
	uint32_t n = ring_buf_peek(&modem_rx_ring, peek_buf, sizeof(peek_buf));

	irq_unlock(key);
	return n;
}

void Modem_RingPush(const uint8_t *data, uint16_t len)
{
	unsigned int key;
	uint32_t wrote;

	if (data == NULL || len == 0u) {
		return;
	}
	key = irq_lock();
	wrote = ring_buf_put(&modem_rx_ring, data, len);
	if (wrote > 0u) {
		last_modem_rx_tick = platform_tick_ms();
	}
	if (wrote < len) {
		evt_overflow_cnt++;
	}
	irq_unlock(key);
}

void Modem_RingFlush(void)
{
	unsigned int key = irq_lock();

	ring_buf_reset(&modem_rx_ring);
	irq_unlock(key);
}

uint8_t Modem_RingContains(const char *needle)
{
	uint16_t nlen = (uint16_t)strlen(needle);
	uint32_t n;
	uint32_t i;

	if (nlen == 0u) {
		return 1u;
	}
	n = ring_copy();
	if (n < nlen) {
		return 0u;
	}
	for (i = 0u; i + nlen <= n; i++) {
		if (memcmp(&peek_buf[i], needle, nlen) == 0) {
			return 1u;
		}
	}
	return 0u;
}

uint8_t Modem_RingConsume(const char *needle)
{
	uint16_t nlen = (uint16_t)strlen(needle);
	uint32_t n;
	uint32_t i;
	unsigned int key;

	if (nlen == 0u) {
		return 1u;
	}
	n = ring_copy();
	for (i = 0u; i + nlen <= n; i++) {
		if (memcmp(&peek_buf[i], needle, nlen) == 0) {
			key = irq_lock();
			(void)ring_buf_get(&modem_rx_ring, discard_buf, i + nlen);
			irq_unlock(key);
			return 1u;
		}
	}
	return 0u;
}

int Modem_RingFindByte(char c)
{
	uint32_t n = ring_copy();
	uint32_t i;

	for (i = 0u; i < n; i++) {
		if (peek_buf[i] == (uint8_t)c) {
			return (int)i;
		}
	}
	return -1;
}

void Modem_RingDrop(uint16_t count)
{
	unsigned int key = irq_lock();

	if (count > MODEM_RING_SIZE) {
		count = MODEM_RING_SIZE;
	}
	(void)ring_buf_get(&modem_rx_ring, discard_buf, count);
	irq_unlock(key);
}

void Modem_SnapshotRing(char *out, uint16_t out_len)
{
	uint32_t n;
	uint32_t i;
	uint32_t w = 0u;

	if (out == NULL || out_len == 0u) {
		return;
	}
	memset(out, 0, out_len);
	n = ring_copy();
	for (i = 0u; i < n && w + 1u < out_len; i++) {
		char c = (char)peek_buf[i];

		out[w++] = (c >= 32 && c < 127) ? c : '.';
	}
	out[w] = '\0';
}

void Modem_RxPump(void)
{
	modem_port_rx_pump();
}

void Modem_UartBringup(void)
{
	Modem_RingFlush();
	(void)modem_port_rearm();
}

uint8_t Modem_ExtractLine(char *out, uint16_t out_len)
{
	uint32_t n = ring_copy();
	uint32_t i;

	if (out == NULL || out_len == 0u) {
		return 0u;
	}
	for (i = 0u; i < n; i++) {
		char c = (char)peek_buf[i];

		if (c == '\n' || c == '\r') {
			uint32_t end = i + 1u;
			uint32_t copy = i;
			unsigned int key;

			if (c == '\r' && end < n && peek_buf[end] == '\n') {
				end++;
			}
			if (copy >= out_len) {
				copy = (uint32_t)out_len - 1u;
			}
			memcpy(out, peek_buf, copy);
			out[copy] = '\0';
			key = irq_lock();
			(void)ring_buf_get(&modem_rx_ring, discard_buf, end);
			irq_unlock(key);
			return 1u;
		}
	}
	return 0u;
}
