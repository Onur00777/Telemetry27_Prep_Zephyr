/*
 * can_decode.c
 *
 *  Created on: 18 Tem 2026
 *      Author: onury
 */

/*
 * Faz 3: CAN -> MainBuffer decode. Tek giris fonksiyonu (CAN_ProcessRxFrame)
 * her cerceveyi bus + ID'ye gore cozer ve fiziksel degerleri MainBuffer'a yazar.
 *
 * Referans: vcu26_firmware/CM7 decode kodlari (bms.cpp, lvbms.cpp, imu.cpp,
 * GPS.cpp, damper.cpp, brake.cpp, dashboard.cpp, inverter.cpp) ve can_decode.h.
 * Olcek/endianness/bayt-offset degerleri VCU decode ile birebir aynidir.
 *
 * NOT (birim): Telemetry_TransmitFrame her alani kendi olcegiyle paketler
 * (or. f[...]*100 -> i16). Bu yuzden burada FIZIKSEL deger yazilir; VCU'nun
 * ayni fiziksel degeri urettigi dogrulandi. Tek istisna damper: VCU metre
 * uretir ama telemetri paketlemesi *10 oldugundan burada mm saklanir (asagida).
 */

#include "can_decode.h"
#include "telemetry_buffer.h"
#include <string.h>

extern MainBuffer_t MainBuffer;

static uint16_t damper_raw_fl = (uint16_t)DAMPER_REST_FL;
static uint16_t damper_raw_fr = (uint16_t)DAMPER_REST_FR;
static uint16_t damper_raw_rl = (uint16_t)DAMPER_REST_RL;
static uint16_t damper_raw_rr = (uint16_t)DAMPER_REST_RR;

static int16_t inv_dcbus_voltage_scaled = 0; /* V (int16, /10 uygulanmis) */
/* 0x060 geldi → hücre temp/volt delta referansı (min) geçerli (VCU/dash birebir). */
static uint8_t bms_cell_min_valid = 0;
/* 0x256 d4 araç hızı otoritesi (dashboard ile aynı). Yoksa 0x0B0 RPM fallback. */
static uint8_t vcu_speed_from_status = 0;

#define BMS_CELL_V_MIN  2.50f
#define BMS_CELL_V_MAX  4.25f

/* FDCAN DataLength kodu -> gercek bayt sayisi */
static uint8_t can_payload_len(uint32_t dlc_code)
{
    static const uint8_t map[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
        8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
    };
    return map[dlc_code & 0xFu];
}

static uint8_t encode_temp_u8(float temp_c)
{
    float enc = temp_c + 40.0f;
    if (enc < 0.0f)   enc = 0.0f;
    if (enc > 255.0f) enc = 255.0f;
    return (uint8_t)enc;
}

static void diff_checker(const uint8_t *d, uint8_t count)
{
    const float min_v = MainBuffer.f[BMS_MIN_CELL_VOLTAGE_f];
    if (min_v <= 0.0f) {
        return;
    }

    for (uint8_t i = 0; i < count; i++) {
        const float cell_v = min_v + ((float)d[i] / 100.0f);
        if (cell_v < BMS_CELL_V_MIN || cell_v > BMS_CELL_V_MAX) {
            Telemetry_SetError(ERR_BMS_CELL_VOLTAGE);
            return; /* baska bank OK diye temizleme — sticky */
        }
    }
}

int32_t CAN_DecodeRaw(const uint8_t *d, uint8_t start, uint8_t width, bool sgned, bool bigendian){
    uint32_t raw = 0;
    if (bigendian == true) {
        for (uint8_t i = 0; i < width; i++)
            raw = (raw << 8) | d[start + i];          /* d[start] = MSB */
    } else {
        for (uint8_t i = 0; i < width; i++)
            raw |= (uint32_t)d[start + i] << (8 * i);  /* d[start] = LSB */
    }
    if (sgned == true && width < 4){                            /* isaret genisletme */
        uint32_t signbit = 1u << (8 * width - 1);
        if (raw & signbit)
            raw |= ~((1u << (8 * width)) - 1);
    }
    return (int32_t)raw;
}

float CAN_Decode(const uint8_t *d, uint8_t start, uint8_t width, bool sgned, bool bigendian, float scale){
    int32_t raw = CAN_DecodeRaw(d, start, width, sgned, bigendian);
    return (scale != 0.0f) ? (float)raw / scale : (float)raw;
}

static float damper_angle_dev(uint16_t raw, float rest)
{
    float diff = (float)raw - rest;
    while (diff >  DAMPER_RAW_HALF) diff -= DAMPER_RAW_FULL;
    while (diff < -DAMPER_RAW_HALF) diff += DAMPER_RAW_FULL;
    return diff / DAMPER_RAW_PER_DEG;
}

static void damper_recompute(void)
{
    /* kose aci sapmasi (derece) * mirror isareti ("sikisma = +") */
    float devFL = damper_angle_dev(damper_raw_fl, DAMPER_REST_FL) * DAMPER_SIGN_FL;
    float devFR = damper_angle_dev(damper_raw_fr, DAMPER_REST_FR) * DAMPER_SIGN_FR;
    float devRL = damper_angle_dev(damper_raw_rl, DAMPER_REST_RL) * DAMPER_SIGN_RL;
    float devRR = damper_angle_dev(damper_raw_rr, DAMPER_REST_RR) * DAMPER_SIGN_RR;

    /* derece -> mm  (VCU: derece->metre; biz mm sakliyoruz ki TransmitFrame'in
     * *10 paketlemesi cozunurlugu korusun. Motion ratio VCU'da placeholder.) */
    const float k = DAMPER_M_PER_DEG * DAMPER_MM_PER_M; /* mm/derece */
    float zFL = devFL * k, zFR = devFR * k, zRL = devRL * k, zRR = devRR * k;

    /* heave = ortalama dusey hareket, roll = sol-sag farki / 2 (VCU modal ayrisim) */
    MainBuffer.f[DAMPER_COMPRESSION_FH_f] = (zFL + zFR) * 0.5f; /* on heave  (mm) */
    MainBuffer.f[DAMPER_COMPRESSION_FR_f] = (zFL - zFR) * 0.5f; /* on roll   (mm) */
    MainBuffer.f[DAMPER_COMPRESSION_RH_f] = (zRL + zRR) * 0.5f; /* arka heave(mm) */
    MainBuffer.f[DAMPER_COMPRESSION_RR_f] = (zRL - zRR) * 0.5f; /* arka roll (mm) */
    /* per-kose dusey hareket (mm) */
    MainBuffer.f[WHEEL_TRAVEL_FL_f] = zFL;
    MainBuffer.f[WHEEL_TRAVEL_FR_f] = zFR;
    MainBuffer.f[WHEEL_TRAVEL_RL_f] = zRL;
    MainBuffer.f[WHEEL_TRAVEL_RR_f] = zRR;
}

/* ======================= Tek decode/dispatch fonksiyonu ======================= */
void CAN_ProcessRxFrame(FDCAN_HandleTypeDef *hfdcan,
                        const FDCAN_RxHeaderTypeDef *header,
                        const uint8_t *d)
{
    const uint32_t id = header->Identifier;
    const uint8_t len = can_payload_len(header->DataLength);

    /* ================= FD hatti (FDCAN3): BMS, LV-BMS, IMU, GPS ================= */
    if (hfdcan->Instance == FDCAN3) {
        switch (id) {

        /* ---- HV BMS ozet (LE, >=30B) — Excel(7) / VCU bms.cpp / dash Decode_BMS_0x60 ---- */
        case CANID_BMS:
            if (len < 30u) break;
            MainBuffer.f[BMS_MAX_CELL_TEMP_f]    = CAN_Decode(d,  0, 2, true,  false, 100.0f);
            MainBuffer.f[BMS_MIN_CELL_TEMP_f]    = CAN_Decode(d,  2, 2, true,  false, 100.0f);
            MainBuffer.f[BMS_AVG_CELL_TEMP_f]    = CAN_Decode(d,  4, 2, true,  false, 100.0f);
            MainBuffer.f[BMS_MAX_SLAVE_TEMP_f]   = CAN_Decode(d,  6, 2, true,  false, 100.0f);
            MainBuffer.f[BMS_MAX_CELL_VOLTAGE_f] = CAN_Decode(d,  8, 2, false, false, 100.0f);
            MainBuffer.f[BMS_MIN_CELL_VOLTAGE_f] = CAN_Decode(d, 10, 2, false, false, 100.0f);
            /* d[12]=maxCellNo, d[13]=minCellNo -> telemetri'de alan yok, atla */
            MainBuffer.f[BMS_AVG_CELL_VOLTAGE_f] = CAN_Decode(d, 14, 2, false, false, 100.0f);
            MainBuffer.f[BMS_TOTAL_VOLTAGE_f]    = CAN_Decode(d, 16, 2, false, false, 100.0f);
            MainBuffer.f[BMS_ESTIMATED_SoC_f]    = CAN_Decode(d, 18, 2, false, false, 10000.0f);
            MainBuffer.u32[BMS_POWER_u32]        = (uint32_t)CAN_DecodeRaw(d, 20, 3, false, false);
            MainBuffer.f[BMS_CURRENT_f]          = CAN_Decode(d, 23, 2, true,  false, 10.0f);
            MainBuffer.u16[BMS_FAULTS_u16]       = (uint16_t)CAN_DecodeRaw(d, 25, 2, false, false);
            /* d[27..28]=leftChargeSec -> telemetri'de alan yok, atla */
            MainBuffer.u8[BMS_CONTRACTORS_u8]    = d[29];
            bms_cell_min_valid = 1u;
            break;

        /* ---- Hucre gerilimleri: her bayt MIN hucreye gore DELTA (ham byte saklanir) ----
         * Sunucu: hucre_V = BMS_MIN_CELL_VOLTAGE + delta/100. cell_voltages[] u8. */
        case CANID_BMS_VOLT1:
            if (!bms_cell_min_valid || len < 32u) break;
            memcpy(&MainBuffer.cell_voltages[0],  d, 32);
            diff_checker(d, 32);
            break;
        case CANID_BMS_VOLT2:
            if (!bms_cell_min_valid || len < 32u) break;
            memcpy(&MainBuffer.cell_voltages[32], d, 32);
            diff_checker(d, 32);
            break;
        case CANID_BMS_VOLT3:
            if (!bms_cell_min_valid || len < 32u) break;
            memcpy(&MainBuffer.cell_voltages[64], d, 32);
            diff_checker(d, 32);
            break;

        /* ---- Hucre sicakliklari: u16 LE delta /100; abs = MIN_CELL_TEMP + delta
         * (VCU bms.cpp + dash Decode_BMS_CellTemps). Paket: °C+40 → u8. ---- */
        case CANID_BMS_TEMP1:
            if (!bms_cell_min_valid || len < 64u) break;
            for (uint8_t i = 0; i < 32; i++) {
                float t = MainBuffer.f[BMS_MIN_CELL_TEMP_f] +
                          CAN_Decode(d, (uint8_t)(i * 2u), 2, false, false, 100.0f);
                MainBuffer.cell_temperatures[i] = encode_temp_u8(t);
            }
            break;

        case CANID_BMS_TEMP2:
            if (!bms_cell_min_valid || len < 64u) break;
            for (uint8_t i = 0; i < 32; i++) {
                float t = MainBuffer.f[BMS_MIN_CELL_TEMP_f] +
                          CAN_Decode(d, (uint8_t)(i * 2u), 2, false, false, 100.0f);
                MainBuffer.cell_temperatures[i + 32] = encode_temp_u8(t);
            }
            break;

        case CANID_BMS_TEMP3:
            if (!bms_cell_min_valid || len < 64u) break;
            for (uint8_t i = 0; i < 32; i++) {
                float t = MainBuffer.f[BMS_MIN_CELL_TEMP_f] +
                          CAN_Decode(d, (uint8_t)(i * 2u), 2, false, false, 100.0f);
                MainBuffer.cell_temperatures[i + 64] = encode_temp_u8(t);
            }
            break;

        /* ---- LV BMS (LE, >=23B) ---- */
        case CANID_LVBMS:
            if (len < 23u) break;
            MainBuffer.f[LVBMS_VOLTAGE_f]          = CAN_Decode(d,  0, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_CELL_REAL0_f]       = CAN_Decode(d,  2, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_CELL_REAL1_f]       = CAN_Decode(d,  4, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_CELL_REAL2_f]       = CAN_Decode(d,  6, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_CELL_REAL3_f]       = CAN_Decode(d,  8, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_MAX_CELL_VOLTAGE_f] = CAN_Decode(d, 10, 2, false, false, 100.0f);
            MainBuffer.f[LVBMS_MIN_CELL_VOLTAGE_f] = CAN_Decode(d, 12, 2, false, false, 100.0f);
            MainBuffer.u8[LVBMS_CELL_MIN_NUMBER_u8] = d[14];
            MainBuffer.u8[LVBMS_CELL_MAX_NUMBER_u8] = d[15];
            MainBuffer.f[LVBMS_CURRENT_f]          = CAN_Decode(d, 16, 2, true,  false, 10.0f);
            MainBuffer.f[LVBMS_MAX_TEMP_f]         = CAN_Decode(d, 18, 2, true,  false, 100.0f);
            MainBuffer.f[LVBMS_MIN_TEMP_f]         = CAN_Decode(d, 20, 2, true,  false, 100.0f);
            MainBuffer.u8[LVBMS_FAULTBYTE_u8]      = d[22];
            /* LVBMS_ESTIMATED_SoC: 0x131'de yok -> yazilmaz */
            break;

        /* ---- IMU 0x220 (LE): Excel(7) / VCU imu.cpp / dash Decode_IMU_0x220 ----
         * accel/gyro/euler + calib@24-27 + health@28-30; mag reserved=0;
         * quat@32-39 sadece DLC>=48. */
        case CANID_IMU:
            if (len < 28u) break; /* accel..calib icin minimum */
            MainBuffer.f[IMU_ACCEL_X_f]     = CAN_Decode(d,  0, 2, true, false, 100.0f);
            MainBuffer.f[IMU_ACCEL_Y_f]     = CAN_Decode(d,  2, 2, true, false, 100.0f);
            MainBuffer.f[IMU_ACCEL_Z_f]     = CAN_Decode(d,  4, 2, true, false, 100.0f);
            MainBuffer.f[IMU_GYRO_X_f]      = CAN_Decode(d,  6, 2, true, false, 16.0f);
            MainBuffer.f[IMU_GYRO_Y_f]      = CAN_Decode(d,  8, 2, true, false, 16.0f);
            MainBuffer.f[IMU_GYRO_Z_f]      = CAN_Decode(d, 10, 2, true, false, 16.0f);
            MainBuffer.f[YAW_RATE_f]        = MainBuffer.f[IMU_GYRO_Z_f]; /* gyroZ = yaw hizi */
            MainBuffer.f[IMU_EULER_YAW_f]   = CAN_Decode(d, 12, 2, true, false, 16.0f);
            MainBuffer.f[IMU_EULER_ROLL_f]  = CAN_Decode(d, 14, 2, true, false, 16.0f);
            MainBuffer.f[IMU_EULER_PITCH_f] = CAN_Decode(d, 16, 2, true, false, 16.0f);
            /* Mag bytes 18-23: imu26 artik basmiyor — stale birikmesin diye sifirla. */
            MainBuffer.f[IMU_MAG_X_f] = 0.0f;
            MainBuffer.f[IMU_MAG_Y_f] = 0.0f;
            MainBuffer.f[IMU_MAG_Z_f] = 0.0f;
            MainBuffer.u8[IMU_CALIB_SYS_u8]         = d[24];
            MainBuffer.u8[IMU_CALIB_GYRO_u8]        = d[25];
            MainBuffer.u8[IMU_CALIB_ACCEL_u8]       = d[26];
            MainBuffer.u8[IMU_CALIB_MAG_u8]         = d[27];
            /* Quat WXYZ @32-39 (DLC 48). Eski DLC32'de quat yok — onceki degeri koru. */
            if (len >= 48u) {
                MainBuffer.f[IMU_QUAT_W_f] = CAN_Decode(d, 32, 2, true, false, 16384.0f);
                MainBuffer.f[IMU_QUAT_X_f] = CAN_Decode(d, 34, 2, true, false, 16384.0f);
                MainBuffer.f[IMU_QUAT_Y_f] = CAN_Decode(d, 36, 2, true, false, 16384.0f);
                MainBuffer.f[IMU_QUAT_Z_f] = CAN_Decode(d, 38, 2, true, false, 16384.0f);
            }
            break;

        /* ---- VCU status 0x256 (LE, >=12B) — dash Decode_VCU_Status_0x256 ----
         * d[4] = vehicle speed otoritesi (dashboard ile ayni). */
        case CANID_VCU_STATUS:
            if (len < 12u) break;
            MainBuffer.u8[VCU_VEHICLE_STATE_u8] = d[0];
            MainBuffer.u8[VCU_DRIVE_MODE_u8]   = d[1];
            MainBuffer.u8[VCU_VEHICLE_SPEED_u8] = d[4];
            MainBuffer.u8[THROTTLE_PERCENT_u8]  = d[5];
            MainBuffer.u8[VCU_APP_STATE_REQ_u8] = d[6];
            MainBuffer.u16[VCU_TORQUE_NM_REQ_u16] =
                (uint16_t)(d[8] | ((uint16_t)d[9] << 8));
            MainBuffer.u8[TIRE_SPEED_RR_u8] = d[10];
            MainBuffer.u8[TIRE_SPEED_RL_u8] = d[11];
            vcu_speed_from_status = 1u;
            break;

        /* ---- GPS (BIG-ENDIAN, >=14B for lat/lon/speed) ---- */
        case CANID_GPS:
            if (len < 14u) break;
            MainBuffer.i32[GPS_LATITUDE_i32]   = CAN_DecodeRaw(d, 0, 4, true, true); /* ham (x1e-7) */
            MainBuffer.i32[GPS_LONGTITUDE_i32] = CAN_DecodeRaw(d, 4, 4, true, true);
            /* yer hizi: byte12-13 UNSIGNED BE, /100 -> km/h, u8'e sigdir */
            {
                float kmh = CAN_Decode(d, 12, 2, false, true, 100.0f);
                if (kmh < 0.0f)   kmh = 0.0f;
                if (kmh > 255.0f) kmh = 255.0f;
                MainBuffer.u8[GPS_SPEED_u8] = (uint8_t)kmh;
            }
            break;

        default:
            break;
        }
        return;
    }

    /* ================= Classic hatti (FDCAN2): Inverter, Damper, Fren, Dashboard ================= */
    if (hfdcan->Instance == FDCAN2) {
        switch (id) {

        /* ---------------- Inverter (Cascadia tarzi, LE, 8B) ---------------- */
        case CANID_INV_HIGHSPEED: { /* 0x0B0 */
            if (len < 8u) break;
            MainBuffer.i16[INV_TORQUE_EST_NM_i16] = (int16_t)CAN_Decode(d, 2, 2, true, false, 10.0f);
            int32_t motor_speed = CAN_DecodeRaw(d, 4, 2, true, false);      /* ham RPM */
            MainBuffer.i32[INV_EMACHINE_SPEED_ERPM_i32] = motor_speed;
            /* Dashboard: araç hızı 0x256 d4. VCU status yoksa RPM→km/h fallback. */
            if (!vcu_speed_from_status) {
                int32_t v = (int32_t)((float)motor_speed * 0.02394f);
                if (v < 0)   v = 0;
                if (v > 255) v = 255;
                MainBuffer.u8[VCU_VEHICLE_SPEED_u8] = (uint8_t)v;
            }
            inv_dcbus_voltage_scaled = (int16_t)CAN_Decode(d, 6, 2, true, false, 10.0f);
            MainBuffer.u16[INV_DCBUS_VOLTAGE_u16] = (uint16_t)inv_dcbus_voltage_scaled;
            break;
        }
        case CANID_INV_TEMP1: { /* 0x0A0: faz sicakliklari + gate driver */
            if (len < 8u) break;
            int16_t pa = (int16_t)CAN_Decode(d, 0, 2, true, false, 10.0f);
            int16_t pb = (int16_t)CAN_Decode(d, 2, 2, true, false, 10.0f);
            int16_t pc = (int16_t)CAN_Decode(d, 4, 2, true, false, 10.0f);
            int16_t mx = pa; if (pb > mx) mx = pb; if (pc > mx) mx = pc;
            MainBuffer.i16[INV_PWRSTG_TEMP_i16]  = mx;                      /* max(faz A/B/C) */
            MainBuffer.i16[INV_BOARD_1_TEMP_i16] = (int16_t)CAN_Decode(d, 6, 2, true, false, 10.0f);
            break;
        }
        case CANID_INV_TEMP2: /* 0x0A1: kontrol karti sicakligi */
            if (len < 2u) break;
            MainBuffer.i16[INV_BOARD_2_TEMP_i16] = (int16_t)CAN_Decode(d, 0, 2, true, false, 10.0f);
            break;
        case CANID_INV_TEMP3: /* 0x0A2: motor sicakligi */
            if (len < 6u) break;
            MainBuffer.i16[INV_EMACHINE_TEMP_1_i16] = (int16_t)CAN_Decode(d, 4, 2, true, false, 10.0f);
            break;
        case CANID_INV_CURRENT: { /* 0x0A6: DC akim + AC guc */
            if (len < 8u) break;
            int16_t dc_current = (int16_t)CAN_Decode(d, 6, 2, true, false, 10.0f); /* A */
            /* AC guc = Idc * Vdc / 100 (kW*10). Vdc 0x0B0'den. (faz akimlari telemetri'de yok) */
            MainBuffer.i16[INV_ACBUS_POWER_i16] =
                (int16_t)(((int32_t)dc_current * inv_dcbus_voltage_scaled) / 100);
            break;
        }
        case CANID_INV_FLUX: /* 0x0A8: Id/Iq feedback */
            if (len < 8u) break;
            MainBuffer.f[INV_CURRENT_D_A_f] = CAN_Decode(d, 4, 2, true, false, 10.0f);
            MainBuffer.f[INV_CURRENT_Q_A_f] = CAN_Decode(d, 6, 2, true, false, 10.0f);
            break;
        case CANID_INV_STATES: /* 0x0AA: VSM/inverter durum + bayrak kelimesi */
            if (len < 8u) break;
            if (d[0] < 16) { /* VCU ile ayni gecerlilik kosulu */
                MainBuffer.u8[INV_APP_STATE_APP_u8]       = d[0]; /* VSM_State  */
                MainBuffer.u8[INV_EMCTRL_FOC_BITSTATE_u8] = d[2]; /* Inverter_State */
                uint16_t flags = (uint16_t)(d[7] & 0x7F);
                flags |= (uint16_t)(d[6] & 0x01)        << 7;  /* Enable      */
                flags |= (uint16_t)((d[6] >> 7) & 0x01) << 8;  /* Lockout     */
                flags |= (uint16_t)((d[6] >> 6) & 0x01) << 9;  /* Start_Mode  */
                flags |= (uint16_t)(d[4] & 0x01)        << 10; /* Run_Mode    */
                flags |= (uint16_t)(d[5] & 0x01)        << 11; /* Command_Mode*/
                MainBuffer.u16[INV_PWRSTG_BITSTATE_u16]   = flags;
            }
            break;
        case CANID_INV_FAULTS: /* 0x0AB: POST(b0-3) / Run(b4-7) hata kelimeleri - VCU ile birebir */
            if (len < 8u) break;
            MainBuffer.u32[INV_DEM1_u32] =
                (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
            MainBuffer.u32[INV_DEM2_u32] =
                (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            break;

        /* ---------------- Damper (LE, byte0-1 ham aci u16) ---------------- */
        case CANID_DAMPER_FL:
            if (len < 2u) break;
            damper_raw_fl = (uint16_t)(d[0] | (d[1] << 8)); damper_recompute(); break;
        case CANID_DAMPER_FR:
            if (len < 2u) break;
            damper_raw_fr = (uint16_t)(d[0] | (d[1] << 8)); damper_recompute(); break;
        case CANID_DAMPER_RL:
            if (len < 2u) break;
            damper_raw_rl = (uint16_t)(d[0] | (d[1] << 8)); damper_recompute(); break;
        case CANID_DAMPER_RR:
            if (len < 2u) break;
            damper_raw_rr = (uint16_t)(d[0] | (d[1] << 8)); damper_recompute(); break;

        /* ---------------- Fren basinci (CANopen DS404, TPDO1) ---------------- */
        case CANID_BRAKE_FRONT: {
            if (len < 4u) break;
            uint32_t rawp = (uint32_t)CAN_DecodeRaw(d, 0, 4, true, false); /* int32 LE */
            float bar = (float)(int32_t)rawp * BRAKE_PRESSURE_CONST;
            if (bar < 0.0f)   bar = 0.0f;
            if (bar > 255.0f) bar = 255.0f;
            MainBuffer.u8[BRAKE_PRESSURE_FRONT_u8] = (uint8_t)bar;
            break;
        }
        case CANID_BRAKE_REAR: {
            if (len < 4u) break;
            uint32_t rawp = (uint32_t)CAN_DecodeRaw(d, 0, 4, true, false);
            float bar = (float)(int32_t)rawp * BRAKE_PRESSURE_CONST;
            if (bar < 0.0f)   bar = 0.0f;
            if (bar > 255.0f) bar = 255.0f;
            MainBuffer.u8[BRAKE_PRESSURE_REAR_u8] = (uint8_t)bar;
            break;
        }

        /* ---------------- Dashboard / gaz (0x210, LE) ---------------- */
        case CANID_DASHBOARD:
            if (len < 6u) break;
            MainBuffer.u16[POT1_U16] = (uint16_t)(d[0] | (d[1] << 8)); /* APPS1 ham ADC */
            MainBuffer.u16[POT2_U16] = (uint16_t)(d[2] | (d[3] << 8)); /* APPS2 ham ADC */
            MainBuffer.u8[TIRE_SPEED_FR_u8] = d[4];
            MainBuffer.u8[TIRE_SPEED_FL_u8] = d[5];
            /* d[6]=buttons -> telemetri'de yok */
            break;

        default:
            break;
        }
        return;
    }
}
