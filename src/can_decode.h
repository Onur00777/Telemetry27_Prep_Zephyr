/*
 * can_decode.h
 *
 * Faz 3: CAN hattindan gelen ham cerceveleri cozup MainBuffer'a yazan tek
 * decode modulunun arayuzu.
 *
 * Kaynak: vcu26_firmware (CM7) decode kodu esas alinmistir. Cozum kurallari
 * VCU'nun Decode::canDecode(...) yardimcisiyla birebir aynidir; her cihazin
 * bayt duzeni/olcek/endianness'i VCU decode dosyalarindan cikarilmistir.
 *
 * Bus atamasi (VCU numaralandirmasi bizde ters):
 *   - Classic 1 Mbit  -> FDCAN2 : Inverter, Damper x4, Fren F/R, Dashboard(0x210), (TMS kapali)
 *   - CAN-FD 1M/2M    -> FDCAN3 : HV BMS, LV-BMS, IMU, GPS, (VCU-status placeholder)
 */

#ifndef INC_CAN_DECODE_H_
#define INC_CAN_DECODE_H_

#include <stdint.h>
#include <stdbool.h>

#define CAN_BUS_CLASSIC 2u
#define CAN_BUS_FD      3u

/* ======================= CAN mesaj ID'leri ======================= */
/* --- FD hatti (FDCAN3) --- */
#define CANID_BMS            0x060u   /* HV BMS ozet (32B)            */
#define CANID_BMS_VOLT1      0x100u   /* hucre 0-31   delta byte      */
#define CANID_BMS_VOLT2      0x110u   /* hucre 32-63  delta byte      */
#define CANID_BMS_VOLT3      0x120u   /* hucre 64-95  delta byte      */
/* Hucre sicakliklari (32 x i16 LE /100, 64B FD). BMS master ID'leri. */
#define CANID_BMS_TEMP1      0x130u   /* hucre 0-31   temp            */
#define CANID_BMS_TEMP2      0x140u   /* hucre 32-63  temp            */
#define CANID_BMS_TEMP3      0x150u   /* hucre 64-95  temp            */
#define CANID_LVBMS          0x131u   /* LV BMS (24B)                 */
#define CANID_IMU            0x220u   /* IMU DLC48: quat@32-39 (Excel7/VCU/dash) */
#define CANID_GPS            0x230u   /* GPS (32B, BIG-ENDIAN)        */
#define CANID_VCU_STATUS     0x256u   /* VCU → Dash/Telemetry MSG-1   */
#define CANID_VCU_DIAG       0x257u   /* VCU → Dash MSG-2 (diag)      */

/* --- Classic hatti (FDCAN2) --- */
#define CANID_INV_TEMP1      0x0A0u   /* faz sicakliklari + gate driver */
#define CANID_INV_TEMP2      0x0A1u   /* kontrol karti sicakligi        */
#define CANID_INV_TEMP3      0x0A2u   /* motor sicakligi                */
#define CANID_INV_CURRENT    0x0A6u   /* faz/DC akim                    */
#define CANID_INV_FLUX       0x0A8u   /* Id/Iq feedback                 */
#define CANID_INV_STATES     0x0AAu   /* VSM/inverter durum + bayrak    */
#define CANID_INV_FAULTS     0x0ABu   /* POST/Run hata kelimeleri       */
#define CANID_INV_HIGHSPEED  0x0B0u   /* torque/hiz/DC bus V (hizli)    */
#define CANID_DAMPER_FL      0x1F5u   /* Sol On   (front-left)          */
#define CANID_DAMPER_FR      0x1F4u   /* Sag On   (front-right)         */
#define CANID_DAMPER_RL      0x1F1u   /* Sol Arka (rear-left)           */
#define CANID_DAMPER_RR      0x1F2u   /* Sag Arka (rear-right)          */
#define CANID_BRAKE_FRONT    0x181u   /* CANopen node1 TPDO1            */
#define CANID_BRAKE_REAR     0x182u   /* CANopen node2 TPDO1            */
#define CANID_DASHBOARD      0x210u   /* APPS pot1/pot2 + on teker hizi */

/* ======================= Damper sabitleri (damper.h ile ayni) ======================= */
#define DAMPER_RAW_FULL      4096.0f
#define DAMPER_RAW_HALF      2048.0f
#define DAMPER_RAW_PER_DEG   (4095.0f / 360.0f)  /* 11.375 ham/derece */
#define DAMPER_M_PER_DEG     0.001f              /* motion ratio (VCU'da placeholder) */
#define DAMPER_MM_PER_M      1000.0f
#define DAMPER_SIGN_FL       (+1.0f)
#define DAMPER_SIGN_FR       (-1.0f)
#define DAMPER_SIGN_RL       (+1.0f)
#define DAMPER_SIGN_RR       (-1.0f)
/* Kalibrasyon modu KAPALI -> sabit rest referanslari (damper.h) */
#define DAMPER_REST_FL       1950.0f
#define DAMPER_REST_FR       1223.0f
#define DAMPER_REST_RL       3551.0f
#define DAMPER_REST_RR       2697.0f

/* ======================= Fren sabiti (CANopen DS404) ======================= */
#define BRAKE_PRESSURE_CONST (1.0f / 1000.0f)    /* ham int32 -> bar */

/* ======================= Cozum yardimcilari (VCU Decode:: ile ayni) ======================= */
/*
 * be=true  -> big-endian (d[start] = MSB)
 * be=false -> little-endian (d[start] = LSB)
 * sgn=true -> 2's complement isaret genisletme (width<4)
 * canDecode: fiziksel = ham / scale   (scale==0 -> ham deger dondurulur)
 */
int32_t CAN_DecodeRaw(const uint8_t *d, uint8_t start, uint8_t width, bool sgn, bool be);
float   CAN_Decode(const uint8_t *d, uint8_t start, uint8_t width, bool sgn, bool be, float scale);

/* ======================= Tek decode/dispatch fonksiyonu ======================= */
/*
 * CAN is parcacigi bu fonksiyonu cagirir. bus CAN_BUS_CLASSIC (FDCAN2) veya
 * CAN_BUS_FD (FDCAN3). len bayt cinsindendir. Paketleme Telemetry_TransmitFrame'dedir.
 */
void CAN_ProcessRxFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len);

#endif /* INC_CAN_DECODE_H_ */
