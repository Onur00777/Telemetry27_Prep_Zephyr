#include "modem_internal.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(modem_fsm, LOG_LEVEL_INF);

#define RX_CHUNK 256

static const struct device *const modem_uart = DEVICE_DT_GET(DT_NODELABEL(usart3));
static uint8_t rx_buf[2][RX_CHUNK];
static int rx_next;
static bool rx_active;
static K_SEM_DEFINE(tx_done, 0, 1);

static int start_rx(void)
{
	int rc;

	rx_next = 1;
	rc = uart_rx_enable(modem_uart, rx_buf[0], RX_CHUNK, 1000);
	if (rc == 0) {
		rx_active = true;
	}
	return rc;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user)
{
	ARG_UNUSED(user);

	switch (evt->type) {
	case UART_RX_RDY:
		Modem_RingPush(&evt->data.rx.buf[evt->data.rx.offset],
			       (uint16_t)evt->data.rx.len);
		break;
	case UART_RX_BUF_REQUEST: {
		int idx = rx_next;

		rx_next ^= 1;
		(void)uart_rx_buf_rsp(dev, rx_buf[idx], RX_CHUNK);
		break;
	}
	case UART_RX_DISABLED:
		rx_active = false;
		(void)start_rx();
		break;
	case UART_RX_STOPPED:
		modem_dma_restart_err_cnt++;
		Telemetry_SetError(ERR_UART3_RUNTIME);
		Modem_EventPush(MODEM_EVT_UART_ERROR);
		rx_active = false;
		break;
	case UART_TX_DONE:
	case UART_TX_ABORTED:
		k_sem_give(&tx_done);
		break;
	default:
		break;
	}
}

int modem_port_init(void)
{
	int rc;

	if (!device_is_ready(modem_uart)) {
		return -1;
	}
	rc = uart_callback_set(modem_uart, uart_cb, NULL);
	if (rc != 0) {
		LOG_ERR("uart callback %d", rc);
		return rc;
	}
	rc = start_rx();
	if (rc != 0) {
		LOG_ERR("uart rx %d", rc);
		return rc;
	}
	LOG_INF("usart3 async RX, halka %u bayt", ring_buf_capacity_get(&modem_rx_ring));
	return 0;
}

int modem_port_rearm(void)
{
	if (rx_active) {
		(void)uart_rx_disable(modem_uart);
		return 0;
	}
	return start_rx();
}

int modem_port_write(const uint8_t *data, uint16_t len)
{
	int rc;

	if (data == NULL || len == 0u) {
		return -1;
	}
	while (k_sem_take(&tx_done, K_NO_WAIT) == 0) {
	}
	rc = uart_tx(modem_uart, (uint8_t *)data, len, 200U * USEC_PER_MSEC);
	if (rc != 0) {
		return rc;
	}
	return k_sem_take(&tx_done, K_MSEC(1000)) == 0 ? 0 : -1;
}

void modem_port_rx_pump(void)
{
	if (!rx_active) {
		(void)start_rx();
	}
}
