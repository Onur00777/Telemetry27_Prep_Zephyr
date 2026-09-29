/*
 * Formula Student endurance: 50 s tur, 36 tur, 18 000 x 100 ms.
 * Hiz ve akim sektor komutlari; SoC coulomb sayaci; V = Vocv - I*R;
 * hucre sicakligi yavas I^2, inverter hizli isinip virajda 1-2 C dinlenir.
 * CAN cerceveleri gercek cozucunun olcekleriyle basilir.
 */

#include "can_bus.h"
#include "can_decode.h"
#include "modem.h"
#include "sim_vehicle.h"
#include "telemetry_buffer.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sim_vehicle, LOG_LEVEL_INF);

#define RACE_PACKETS     18000u
#define LAP_TICKS        500u
#define DT_S             0.1f
#define PACK_AH          62.87f
#define R_INT_OHM        0.080f
#define CELL_COUNT       96.0f
#define RPM_PER_KMH      (5500.0f / 105.0f)
#define T_AMB_C          26.0f
#define K_CELL_I2        1.05e-6f
#define TAU_CELL_S       20000.0f

volatile uint8_t sim_race_armed;

static K_SEM_DEFINE(pace_sem, 0, 1);

void sim_pace_signal(void)
{
	(void)k_sem_give(&pace_sem);
}

static float clampf(float x, float lo, float hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

static float smooth01(float u)
{
	u = clampf(u, 0.0f, 1.0f);
	return u * u * (3.0f - 2.0f * u);
}

/* Turlar arasi sifir ortalamali, tekrar edilebilir sapma. */
static float noise_unit(uint32_t lap, uint32_t tick, uint32_t salt)
{
	float p = (float)tick * 0.17f + (float)lap * 1.3f + (float)salt;

	return sinf(p) * 0.6f + sinf(p * 0.37f) * 0.4f;
}

static float speed_cmd(float t)
{
	if (t < 10.0f) {
		return 30.0f + 75.0f * smooth01(t / 10.0f);
	}
	if (t < 16.0f) {
		float u = (t - 10.0f) / 6.0f;

		return 105.0f + (32.0f - 105.0f) * smooth01(u);
	}
	if (t < 26.0f) {
		float u = (t - 16.0f) / 10.0f;
		float osc = 52.5f + 12.5f * sinf(2.0f * 3.14159265f * 4.0f * u);

		if (u < 0.12f) {
			return 32.0f + (osc - 32.0f) * smooth01(u / 0.12f);
		}
		if (u > 0.88f) {
			return 50.0f + (osc - 50.0f) * smooth01((1.0f - u) / 0.12f);
		}
		return osc;
	}
	if (t < 34.0f) {
		return 50.0f + 45.0f * smooth01((t - 26.0f) / 8.0f);
	}
	if (t < 45.0f) {
		float u = (t - 34.0f) / 11.0f;
		float cruise = 52.5f + 7.5f * sinf(2.0f * 3.14159265f * 2.0f * u);

		if (u < 0.14f) {
			return 95.0f + (cruise - 95.0f) * smooth01(u / 0.14f);
		}
		if (u > 0.88f) {
			return 52.0f + (cruise - 52.0f) * smooth01((1.0f - u) / 0.12f);
		}
		return cruise;
	}

	{
		float u = (t - 45.0f) / 5.0f;

		if (u < 0.55f) {
			return 52.0f + (35.0f - 52.0f) * smooth01(u / 0.55f);
		}
		return 35.0f + (30.0f - 35.0f) * smooth01((u - 0.55f) / 0.45f);
	}
}

static float current_cmd(float t)
{
	if (t < 10.0f) {
		return 180.0f + 20.0f * sinf(3.14159265f * (t / 10.0f));
	}
	if (t < 16.0f) {
		return -40.0f * sinf(3.14159265f * ((t - 10.0f) / 6.0f));
	}
	if (t < 26.0f) {
		return 75.0f + 15.0f * sinf(2.0f * 3.14159265f * 3.0f * ((t - 16.0f) / 10.0f));
	}
	if (t < 34.0f) {
		return 145.0f + 10.0f * sinf(3.14159265f * ((t - 26.0f) / 8.0f));
	}
	if (t < 45.0f) {
		return 65.0f + 15.0f * sinf(2.0f * 3.14159265f * 2.0f * ((t - 34.0f) / 11.0f));
	}
	{
		float u = (t - 45.0f) / 5.0f;

		if (u < 0.55f) {
			return -35.0f * sinf(3.14159265f * (u / 0.55f));
		}
		return 40.0f * smooth01((u - 0.55f) / 0.45f);
	}
}

static void lateral(float t, float speed, float *steer, float *ay, float *yaw_dps)
{
	float s = 0.0f;

	*steer = 0.0f;
	*ay = 0.0f;
	if (t >= 10.0f && t < 16.0f) {
		*steer = 22.0f;
		*ay = 5.0f;
	} else if (t >= 16.0f && t < 26.0f) {
		s = sinf(2.0f * 3.14159265f * 4.0f * ((t - 16.0f) / 10.0f));
		*steer = 26.0f * s;
		*ay = 7.5f * s;
	} else if (t >= 34.0f && t < 45.0f) {
		s = sinf(2.0f * 3.14159265f * 2.0f * ((t - 34.0f) / 11.0f));
		*steer = 14.0f * s;
		*ay = 3.8f * s;
	} else if (t >= 45.0f) {
		*steer = 16.0f;
		*ay = 3.2f;
	}
	if (speed > 8.0f) {
		*yaw_dps = (*ay) / (speed / 3.6f) * 57.2958f;
	} else {
		*yaw_dps = 0.0f;
	}
}

static void put_u16(uint8_t *d, uint8_t off, uint16_t v)
{
	d[off] = (uint8_t)(v & 0xffu);
	d[off + 1u] = (uint8_t)(v >> 8);
}

static void put_i16(uint8_t *d, uint8_t off, int16_t v)
{
	put_u16(d, off, (uint16_t)v);
}

static void put_i32_le(uint8_t *d, int32_t v)
{
	uint32_t u = (uint32_t)v;

	d[0] = (uint8_t)u;
	d[1] = (uint8_t)(u >> 8);
	d[2] = (uint8_t)(u >> 16);
	d[3] = (uint8_t)(u >> 24);
}

static void put_i32_be(uint8_t *d, int32_t v)
{
	uint32_t u = (uint32_t)v;

	d[0] = (uint8_t)(u >> 24);
	d[1] = (uint8_t)(u >> 16);
	d[2] = (uint8_t)(u >> 8);
	d[3] = (uint8_t)u;
}

static int16_t amps_to_raw(float a)
{
	float x = a * 10.0f;

	x = clampf(x, -32000.0f, 32000.0f);
	return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static void cell_bounds(float pack_v, float *vmin, float *vmax, float *vavg)
{
	float avg = pack_v / CELL_COUNT;
	float mn = avg - 0.012f;
	float mx = avg + 0.010f;

	if (mx > 4.24f) {
		float shift = mx - 4.24f;

		mx -= shift;
		mn -= shift;
	}
	if (mn < 2.60f) {
		float shift = 2.60f - mn;

		mn += shift;
		mx += shift;
	}
	*vmin = mn;
	*vmax = mx;
	*vavg = (mn + mx) * 0.5f;
}

static void emit_damper(uint32_t id, float rest, float delta)
{
	uint8_t d[2];
	int raw = (int)rest + (int)delta;

	if (raw < 0) {
		raw = 0;
	}
	if (raw > 4095) {
		raw = 4095;
	}
	put_u16(d, 0, (uint16_t)raw);
	can_rx_submit(CAN_BUS_CLASSIC, id, d, 2);
}

static void emit_car(float speed, float amps, float volts, float soc, float t_cell,
		     float t_inv, float lv, float ax, float ay, float yaw_dps,
		     float heading, float rpm, float brake, float throttle)
{
	uint8_t bms[32];
	uint8_t cells[32];
	uint8_t temps[64];
	uint8_t lvframe[24];
	uint8_t imu[48];
	uint8_t vcu[12];
	uint8_t gps[16];
	uint8_t inv[8];
	uint8_t frame[8];
	float vmin;
	float vmax;
	float vavg;
	float tmin = t_cell - 1.6f;
	float tmot = t_inv + 1.8f;
	int i;
	int32_t watts;
	uint16_t spd_u = (uint16_t)clampf(speed + 0.5f, 0.0f, 255.0f);

	cell_bounds(volts, &vmin, &vmax, &vavg);

	memset(bms, 0, sizeof(bms));
	put_i16(bms, 0, (int16_t)(t_cell * 100.0f));
	put_i16(bms, 2, (int16_t)(tmin * 100.0f));
	put_i16(bms, 4, (int16_t)((t_cell - 0.7f) * 100.0f));
	put_i16(bms, 6, (int16_t)((t_cell + 0.4f) * 100.0f));
	put_u16(bms, 8, (uint16_t)(vmax * 100.0f));
	put_u16(bms, 10, (uint16_t)(vmin * 100.0f));
	bms[12] = 12;
	bms[13] = 40;
	put_u16(bms, 14, (uint16_t)(vavg * 100.0f));
	put_u16(bms, 16, (uint16_t)clampf(volts * 100.0f, 0.0f, 65535.0f));
	put_u16(bms, 18, (uint16_t)clampf(soc * 10000.0f, 0.0f, 65535.0f));
	watts = (int32_t)(volts * amps);
	if (watts < 0) {
		watts = 0;
	}
	bms[20] = (uint8_t)((uint32_t)watts & 0xffu);
	bms[21] = (uint8_t)(((uint32_t)watts >> 8) & 0xffu);
	bms[22] = (uint8_t)(((uint32_t)watts >> 16) & 0xffu);
	put_i16(bms, 23, amps_to_raw(amps));
	bms[29] = 0x07;
	can_rx_submit(CAN_BUS_FD, CANID_BMS, bms, sizeof(bms));

	for (i = 0; i < 96; i++) {
		float cell = vmin + (float)(i % 5) * 0.004f;
		uint8_t delta = (uint8_t)clampf((cell - vmin) * 100.0f, 0.0f, 20.0f);

		cells[i % 32] = delta;
		if ((i % 32) == 31) {
			uint32_t id = CANID_BMS_VOLT1;

			if (i >= 64) {
				id = CANID_BMS_VOLT3;
			} else if (i >= 32) {
				id = CANID_BMS_VOLT2;
			}
			can_rx_submit(CAN_BUS_FD, id, cells, 32);
		}
	}

	for (i = 0; i < 96; i++) {
		float tc = tmin + 1.6f * (float)(i % 16) / 15.0f;
		uint16_t delta = (uint16_t)clampf((tc - tmin) * 100.0f, 0.0f, 400.0f);
		uint8_t slot = (uint8_t)((i % 32) * 2);

		put_u16(temps, slot, delta);
		if ((i % 32) == 31) {
			uint32_t id = CANID_BMS_TEMP1;

			if (i >= 64) {
				id = CANID_BMS_TEMP3;
			} else if (i >= 32) {
				id = CANID_BMS_TEMP2;
			}
			can_rx_submit(CAN_BUS_FD, id, temps, 64);
		}
	}

	memset(lvframe, 0, sizeof(lvframe));
	put_u16(lvframe, 0, (uint16_t)(lv * 100.0f));
	put_u16(lvframe, 2, (uint16_t)((lv / 4.0f + 0.02f) * 100.0f));
	put_u16(lvframe, 4, (uint16_t)((lv / 4.0f + 0.01f) * 100.0f));
	put_u16(lvframe, 6, (uint16_t)((lv / 4.0f) * 100.0f));
	put_u16(lvframe, 8, (uint16_t)((lv / 4.0f - 0.01f) * 100.0f));
	put_u16(lvframe, 10, (uint16_t)((lv / 4.0f + 0.02f) * 100.0f));
	put_u16(lvframe, 12, (uint16_t)((lv / 4.0f - 0.01f) * 100.0f));
	lvframe[14] = 2;
	lvframe[15] = 0;
	put_i16(lvframe, 16, amps_to_raw(2.2f + 0.4f * sinf(volts * 0.01f)));
	put_i16(lvframe, 18, (int16_t)(3200 + (int)(t_cell)));
	put_i16(lvframe, 20, (int16_t)(2800 + (int)(t_cell * 0.5f)));
	can_rx_submit(CAN_BUS_FD, CANID_LVBMS, lvframe, sizeof(lvframe));

	memset(imu, 0, sizeof(imu));
	put_i16(imu, 0, (int16_t)clampf(ax * 100.0f, -32000.0f, 32000.0f));
	put_i16(imu, 2, (int16_t)clampf(ay * 100.0f, -32000.0f, 32000.0f));
	put_i16(imu, 4, 981);
	put_i16(imu, 10, (int16_t)clampf(yaw_dps * 16.0f, -32000.0f, 32000.0f));
	put_i16(imu, 12, (int16_t)clampf(heading * 16.0f, -32000.0f, 32000.0f));
	imu[24] = 3;
	imu[25] = 3;
	imu[26] = 3;
	imu[27] = 3;
	put_i16(imu, 32, 16384);
	can_rx_submit(CAN_BUS_FD, CANID_IMU, imu, sizeof(imu));

	memset(vcu, 0, sizeof(vcu));
	vcu[0] = 3;
	vcu[1] = 1;
	vcu[4] = (uint8_t)spd_u;
	vcu[5] = (uint8_t)clampf(throttle + 0.5f, 0.0f, 100.0f);
	vcu[6] = 4;
	put_u16(vcu, 8, (uint16_t)clampf(amps * 0.72f, 0.0f, 400.0f));
	vcu[10] = (uint8_t)spd_u;
	vcu[11] = (uint8_t)spd_u;
	can_rx_submit(CAN_BUS_FD, CANID_VCU_STATUS, vcu, sizeof(vcu));

	memset(gps, 0, sizeof(gps));
	{
		float ang = heading * 0.0174533f;
		int32_t lat = (int32_t)((41.0150f + 0.00035f * sinf(ang)) * 10000000.0f);
		int32_t lon = (int32_t)((28.9790f + 0.00055f * cosf(ang)) * 10000000.0f);
		uint16_t gspd = (uint16_t)(speed * 100.0f);

		put_i32_be(&gps[0], lat);
		put_i32_be(&gps[4], lon);
		gps[12] = (uint8_t)(gspd >> 8);
		gps[13] = (uint8_t)gspd;
	}
	can_rx_submit(CAN_BUS_FD, CANID_GPS, gps, 14);

	memset(inv, 0, sizeof(inv));
	put_i16(inv, 2, amps_to_raw(amps * 0.72f));
	put_i16(inv, 4, (int16_t)clampf(rpm, -32000.0f, 32000.0f));
	put_i16(inv, 6, amps_to_raw(volts));
	can_rx_submit(CAN_BUS_CLASSIC, CANID_INV_HIGHSPEED, inv, sizeof(inv));

	memset(frame, 0, sizeof(frame));
	put_i16(frame, 0, amps_to_raw(t_inv));
	put_i16(frame, 2, amps_to_raw(t_inv - 0.8f));
	put_i16(frame, 4, amps_to_raw(t_inv - 0.4f));
	put_i16(frame, 6, amps_to_raw(t_inv - 2.0f));
	can_rx_submit(CAN_BUS_CLASSIC, CANID_INV_TEMP1, frame, sizeof(frame));

	memset(frame, 0, sizeof(frame));
	put_i16(frame, 0, amps_to_raw(t_inv - 3.0f));
	can_rx_submit(CAN_BUS_CLASSIC, CANID_INV_TEMP2, frame, 2);

	memset(frame, 0, sizeof(frame));
	put_i16(frame, 4, amps_to_raw(tmot));
	can_rx_submit(CAN_BUS_CLASSIC, CANID_INV_TEMP3, frame, 6);

	memset(frame, 0, sizeof(frame));
	put_i16(frame, 6, amps_to_raw(amps));
	can_rx_submit(CAN_BUS_CLASSIC, CANID_INV_CURRENT, frame, sizeof(frame));

	emit_damper(CANID_DAMPER_FL, DAMPER_REST_FL, -ay * 5.0f + ax * 3.0f);
	emit_damper(CANID_DAMPER_FR, DAMPER_REST_FR, ay * 5.0f + ax * 3.0f);
	emit_damper(CANID_DAMPER_RL, DAMPER_REST_RL, -ay * 4.0f - ax * 2.0f);
	emit_damper(CANID_DAMPER_RR, DAMPER_REST_RR, ay * 4.0f - ax * 2.0f);

	{
		uint8_t br[4];
		int32_t front = (int32_t)(brake * 1000.0f);
		int32_t rear = (int32_t)(brake * 650.0f);

		put_i32_le(br, front);
		can_rx_submit(CAN_BUS_CLASSIC, CANID_BRAKE_FRONT, br, 4);
		put_i32_le(br, rear);
		can_rx_submit(CAN_BUS_CLASSIC, CANID_BRAKE_REAR, br, 4);
	}

	memset(frame, 0, sizeof(frame));
	put_u16(frame, 0, (uint16_t)(400.0f + throttle * 28.0f));
	put_u16(frame, 2, (uint16_t)(420.0f + throttle * 27.0f));
	frame[4] = (uint8_t)spd_u;
	frame[5] = (uint8_t)spd_u;
	can_rx_submit(CAN_BUS_CLASSIC, CANID_DASHBOARD, frame, 6);
}

static void log_tick(uint32_t step)
{
	int v_x10;
	int i_x10;
	int soc_x1000;
	int tc_x100;
	int tinv;
	char isign;
	uint32_t lap = step / LAP_TICKS + 1u;
	uint32_t tenth = step % LAP_TICKS;
	uint8_t spd;

	telemetry_lock();
	v_x10 = (int)(MainBuffer.f[BMS_TOTAL_VOLTAGE_f] * 10.0f + 0.5f);
	{
		float ia = MainBuffer.f[BMS_CURRENT_f];

		i_x10 = (int)(ia >= 0.0f ? ia * 10.0f + 0.5f : ia * 10.0f - 0.5f);
	}
	soc_x1000 = (int)(MainBuffer.f[BMS_ESTIMATED_SoC_f] * 1000.0f + 0.5f);
	tc_x100 = (int)(MainBuffer.f[BMS_MAX_CELL_TEMP_f] * 100.0f + 0.5f);
	tinv = (int)MainBuffer.i16[INV_PWRSTG_TEMP_i16];
	spd = MainBuffer.u8[VCU_VEHICLE_SPEED_u8];
	telemetry_unlock();

	isign = (i_x10 < 0) ? '-' : '+';
	if (i_x10 < 0) {
		i_x10 = -i_x10;
	}
	LOG_INF("tur=%u t=%u.%us hiz=%u km/h V=%d.%d I=%c%d.%dA soc=%d.%03d Tcell=%d.%02d Tinv=%d",
		lap, tenth / 10u, tenth % 10u, spd,
		v_x10 / 10, v_x10 % 10,
		isign, i_x10 / 10, i_x10 % 10,
		soc_x1000 / 1000, soc_x1000 % 1000,
		tc_x100 / 100, tc_x100 % 100,
		tinv);
}

static void sim_vehicle_thread(void *p1, void *p2, void *p3)
{
	float i_filt = 180.0f;
	float soc = 0.98f;
	float t_cell = T_AMB_C;
	float t_inv = 30.0f;
	float heading = 0.0f;
	float speed_prev = 30.0f;
	uint32_t step;
	uint32_t send0;
	uint32_t sent;
	bool locked = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("endurance model: 36 tur x 50 s, grid");
	while (!Modem_IsReady()) {
		emit_car(0.0f, 0.0f, 405.0f, 0.98f, T_AMB_C, 30.0f, 12.60f,
			 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f);
		k_msleep(100);
	}

	send0 = Modem_GetLive()->qisend_ok;
	sim_race_armed = 1u;
	LOG_INF("endurance basladi");

	for (step = 0u; step < RACE_PACKETS; step++) {
		uint32_t lap = step / LAP_TICKS;
		uint32_t tick = step % LAP_TICKS;
		float t = (float)tick * DT_S;
		float speed = speed_cmd(t) + 0.8f * noise_unit(lap, tick, 1u);
		float icmd = current_cmd(t) + 2.0f * noise_unit(lap, tick, 2u);
		float vocv;
		float volts;
		float ax;
		float steer;
		float ay;
		float yaw;
		float rpm;
		float brake;
		float thr;
		float tau;
		float teq;
		int pace;

		speed = clampf(speed, 0.0f, 140.0f);
		i_filt += (icmd - i_filt) * 0.45f;
		soc -= i_filt * DT_S / (PACK_AH * 3600.0f);
		soc = clampf(soc, 0.05f, 1.0f);
		vocv = 360.0f + 45.0f * (soc - 0.275f) / (0.98f - 0.275f);
		volts = clampf(vocv - i_filt * R_INT_OHM, 300.0f, 430.0f);

		t_cell += (K_CELL_I2 * i_filt * i_filt - (t_cell - T_AMB_C) / TAU_CELL_S) * DT_S;
		teq = 72.2f + 3.0f * (fabsf(i_filt) / 200.0f);
		if (i_filt < -5.0f) {
			teq = 68.8f;
		}
		tau = (t_inv < 68.0f) ? 3.2f : 14.0f;
		t_inv += (teq - t_inv) / tau * DT_S;

		ax = (speed - speed_prev) / DT_S / 3.6f;
		speed_prev = speed;
		lateral(t, speed, &steer, &ay, &yaw);
		heading += yaw * DT_S;
		if (heading > 180.0f) {
			heading -= 360.0f;
		} else if (heading < -180.0f) {
			heading += 360.0f;
		}
		rpm = speed * RPM_PER_KMH;
		brake = (i_filt < 0.0f) ? (-i_filt * 0.85f) : 0.4f;
		thr = (i_filt > 15.0f) ? clampf(i_filt * 0.50f, 0.0f, 100.0f) : 0.0f;

		{
			float lv = 12.62f - 0.00085f * fmaxf(i_filt, 0.0f) +
				   0.045f * sinf(0.13f * (float)step);

			lv = clampf(lv, 12.40f, 12.80f);
			emit_car(speed, i_filt, volts, soc, t_cell, t_inv, lv,
				 ax, ay, yaw, heading, rpm, brake, thr);
		}
		(void)steer;
		log_tick(step);

		pace = k_sem_take(&pace_sem, K_MSEC(500));
		if (pace != 0) {
			locked = true;
			LOG_ERR("telemetri yanit vermedi, adim %u", step);
			break;
		}
	}

	sim_race_armed = 0u;
	k_msleep(300);
	sent = Modem_GetLive()->qisend_ok - send0;
	LOG_INF("ozet adim=%u qisend=%u overflow=%u err=%u",
		step, sent, evt_overflow_cnt, (unsigned)Telemetry_GetActiveError());
	if (!locked && step == RACE_PACKETS && sent >= RACE_PACKETS &&
	    evt_overflow_cnt == 0u && Telemetry_GetActiveError() == ERR_NONE) {
		LOG_INF("ENDURANCE TEST COMPLETED SUCCESSFULLY");
	} else {
		LOG_ERR("ENDURANCE TEST FAILED");
	}
	Modem_SimulationStop();
}

K_THREAD_DEFINE(sim_vehicle_tid, 4096, sim_vehicle_thread,
		NULL, NULL, NULL, 5, 0, 0);
