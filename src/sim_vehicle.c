#include "can_bus.h"
#include "can_decode.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sim_vehicle, LOG_LEVEL_INF);

static void put_u16(uint8_t *d, uint8_t off, uint16_t v)
{
	d[off] = (uint8_t)(v & 0xffu);
	d[off + 1u] = (uint8_t)(v >> 8);
}

static void put_i16(uint8_t *d, uint8_t off, int16_t v)
{
	put_u16(d, off, (uint16_t)v);
}

static void sim_vehicle_thread(void *p1, void *p2, void *p3)
{
	uint32_t step = 0u;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("sahte BMS 0x060 ve inverter 0x0B0 akisi");

	while (1) {
		uint8_t bms[32];
		uint8_t inv[8];
		uint8_t speed = (uint8_t)(step % 121u);
		uint16_t pack_raw = (uint16_t)(40000u + (step % 20u) * 50u);
		int16_t tmax_c100 = (int16_t)(3500 + (int)(step % 15u) * 100);
		int16_t rpm = (int16_t)((float)speed / 0.02394f);

		memset(bms, 0, sizeof(bms));
		put_i16(bms, 0, tmax_c100);
		put_i16(bms, 2, (int16_t)(tmax_c100 - 400));
		put_i16(bms, 4, (int16_t)(tmax_c100 - 200));
		put_i16(bms, 6, (int16_t)(tmax_c100 + 200));
		put_u16(bms, 8, 415);
		put_u16(bms, 10, 390);
		bms[12] = 10;
		bms[13] = 3;
		put_u16(bms, 14, 405);
		put_u16(bms, 16, pack_raw);
		put_u16(bms, 18, 8500);
		bms[20] = 0x20;
		bms[21] = 0x4e;
		bms[22] = 0x00;
		put_i16(bms, 23, 500);
		bms[29] = 0x07;

		memset(inv, 0, sizeof(inv));
		put_i16(inv, 2, (int16_t)(speed * 2u));
		put_i16(inv, 4, rpm);
		put_i16(inv, 6, (int16_t)(pack_raw / 10u));

		can_rx_submit(CAN_BUS_FD, 0x060u, bms, sizeof(bms));
		can_rx_submit(CAN_BUS_CLASSIC, 0x0B0u, inv, sizeof(inv));
		step++;
		k_msleep(80);
	}
}

K_THREAD_DEFINE(sim_vehicle_tid, 2048, sim_vehicle_thread,
		NULL, NULL, NULL, 6, 0, 0);
