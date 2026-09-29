#include "can_bus.h"
#include "can_decode.h"
#include "telemetry_buffer.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_bus, LOG_LEVEL_INF);

struct can_rx_msg {
	uint8_t bus;
	uint8_t len;
	uint16_t pad;
	uint32_t id;
	uint8_t data[64];
};

K_MSGQ_DEFINE(can_rx_q, sizeof(struct can_rx_msg), 24, 4);

void can_rx_submit(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len)
{
	struct can_rx_msg msg;

	memset(&msg, 0, sizeof(msg));
	msg.bus = bus;
	msg.id = id;
	msg.len = len > sizeof(msg.data) ? (uint8_t)sizeof(msg.data) : len;
	if (data != NULL && msg.len > 0u) {
		memcpy(msg.data, data, msg.len);
	}
	if (k_msgq_put(&can_rx_q, &msg, K_NO_WAIT) != 0) {
		evt_overflow_cnt++;
	}
}

#if !IS_ENABLED(CONFIG_TELEMETRY_SIM)

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

static const struct device *const can_classic = DEVICE_DT_GET(DT_NODELABEL(fdcan2));
static const struct device *const can_fd = DEVICE_DT_GET(DT_NODELABEL(fdcan3));

static void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user)
{
	uint8_t len;

	ARG_UNUSED(dev);
	len = can_dlc_to_bytes(frame->dlc);
	can_rx_submit((uint8_t)(uintptr_t)user, frame->id, frame->data, len);
}

static int add_accept_all(const struct device *dev, uintptr_t bus)
{
	const struct can_filter std = {
		.id = 0u,
		.mask = 0u,
		.flags = 0u,
	};
	const struct can_filter ext = {
		.id = 0u,
		.mask = 0u,
		.flags = CAN_FILTER_IDE,
	};
	int id;

	id = can_add_rx_filter(dev, can_rx_cb, (void *)bus, &std);
	if (id < 0) {
		LOG_ERR("std filtre %d", id);
		return id;
	}
	id = can_add_rx_filter(dev, can_rx_cb, (void *)bus, &ext);
	if (id < 0) {
		LOG_WRN("ext filtre %d", id);
	}
	return 0;
}

static int start_controller(const struct device *dev, uintptr_t bus, const char *name)
{
	int rc;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s hazir degil", name);
		return -1;
	}
	rc = add_accept_all(dev, bus);
	if (rc != 0) {
		return rc;
	}
	rc = can_start(dev);
	if (rc != 0) {
		LOG_ERR("%s start %d", name, rc);
		return rc;
	}
	LOG_INF("%s dinleniyor", name);
	return 0;
}

static void can_hw_start(void)
{
	(void)start_controller(can_classic, CAN_BUS_CLASSIC, "fdcan2");
	(void)start_controller(can_fd, CAN_BUS_FD, "fdcan3");
}

#else

static void can_hw_start(void)
{
	LOG_INF("CAN donanimi yok, cerceveler sim_vehicle'dan gelecek");
}

#endif

static void can_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	can_hw_start();
	while (1) {
		struct can_rx_msg msg;

		if (k_msgq_get(&can_rx_q, &msg, K_FOREVER) != 0) {
			continue;
		}
		telemetry_lock();
		CAN_ProcessRxFrame(msg.bus, msg.id, msg.data, msg.len);
		telemetry_on_can_id(msg.id);
		telemetry_unlock();
	}
}

K_THREAD_DEFINE(can_tid, 2048, can_thread, NULL, NULL, NULL, 4, 0, 0);
