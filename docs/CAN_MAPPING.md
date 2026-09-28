# Telemetry26 — CAN → MainBuffer Eşleme Spec'i (Faz 3)

Kaynak (çapraz doğrulama, 2026-09):
- **Excel:** `vcu26_firmware/CAN Hattı ve SDCARD (7).xlsx` (6 değil — quat/DLC48)
- **VCU decode:** `vcu26_firmware` CM7 (`bms.cpp`, `imu.cpp`, …) — `main` / track tip
- **Dashboard decode:** `ITU-RACING-DASHBOARD` `dash_decode.c` (`cursor/can-tx-boot-delay-c7a6`)
- **Eski telemetri:** `Final26` `can_decode.c` (bu rewrite’ın tabanı)

Bu doküman, telemetri firmware'inin CAN'dan hangi mesajı nasıl çözüp `MainBuffer`'ın
hangi alanına yazacağını tanımlar. Mock kapalı: `APP_TELEMETRY_BENCH_TEST=0`.

---

## 0. Bus ataması (kesin)

VCU'nun `HAL_FDCAN_RxFifo0Callback`'i iki bus'ı ayrı çözüyor:

| Fiziksel hat | VCU peripheral | **Telemetri peripheral** | Frame | ECU'lar |
|---|---|---|---|---|
| **Classic 1 Mbit** | FDCAN1 | **FDCAN2** | Classic | İnverter, Damper×4, Fren F/R, TMS×4, Dashboard(0x210) |
| **CAN-FD 1M/2M** | FDCAN2 | **FDCAN3** | FD-BRS | HV BMS, LV-BMS, IMU, GPS, VCU-status |

> Dikkat: VCU'nun numaralandırması ters (VCU FDCAN1=classic; bizde FDCAN2=classic). ID'ler hep **standart 11-bit**.

## Decode kuralları
- **Endianness mesaja göre:** her şey **little-endian**, **tek istisna GPS `0x230` = big-endian.**
- **Ölçek yönü:** VCU `canDecode` **böler** (physical = raw / scale, offset 0). Yani ör. scale=100 → raw × 0.01.
- **FD frame'leri 24–32 bayt** → telemetri callback'inde `RxData[64]` olmalı (mevcut `[8]` FD'de payload'ı keser). DLC koddan (`FDCAN_DLC_BYTES_*`) çözülmeli.
- Değerler fiziksel olarak `MainBuffer.f[]` vb. yazılır; `telemetry.c` sonra kendi ölçeğiyle paketler (uyum tablolarda doğrulandı).

---

## 1. FD hattı (FDCAN3)

### HV BMS `0x060` (LE, 32B) — *layout VCU decode'una göre*
| Byte | Sinyal | Tip | → MainBuffer | Dönüşüm |
|---|---|---|---|---|
| 0–1 | max cell temp | i16 | `f[BMS_MAX_CELL_TEMP_f]` | /100 °C |
| 2–3 | min cell temp | i16 | `f[BMS_MIN_CELL_TEMP_f]` | /100 |
| 4–5 | avg cell temp | i16 | `f[BMS_AVG_CELL_TEMP_f]` | /100 |
| 6–7 | max slave temp | i16 | `f[BMS_MAX_SLAVE_TEMP_f]` | /100 |
| 8–9 | max cell V | u16 | `f[BMS_MAX_CELL_VOLTAGE_f]` | /100 V |
| 10–11 | min cell V | u16 | `f[BMS_MIN_CELL_VOLTAGE_f]` | /100 |
| 14–15 | avg cell V | u16 | `f[BMS_AVG_CELL_VOLTAGE_f]` | /100 |
| 16–17 | pack total V | u16 | `f[BMS_TOTAL_VOLTAGE_f]` | /100 |
| 18–19 | SOC | u16 | `f[BMS_ESTIMATED_SoC_f]` | /10000 (0–1) |
| 20–22 | power | u24 | `u32[BMS_POWER_u32]` | raw W |
| 23–24 | current | i16 | `f[BMS_CURRENT_f]` | /10 A (+boşalım) |
| 25–26 | faults | u16 | `u16[BMS_FAULTS_u16]` | bitfield |
| 29 | contractors | u8 | `u8[BMS_CONTRACTORS_u8]` | bits |
| (12,13,27–28) | maxCellNo/minCellNo/leftChargeSec | — | — | telemetri'de yok, atla |

> ⚠ AÇIK SORU: BMS sender kartı okuması bu layout'tan **2 bayt kaymış** (current@14, faults@23) rapor etti; VCU decode (12/13'te hücre no) daha detaylı ve alıcı-referansı. **VCU decode'unu esas alıyorum** — gerçek BMS ile bir kez doğrula.

### HV BMS hücre voltajları `0x100`/`0x110`/`0x120` (LE, 32B)
Her byte = bir hücrenin **delta**'sı; `hücre_V = min_cell_V + byte/100`. → `cell_voltages[0..31]`, `[32..63]`, `[64..95]` = **ham delta byte** (server min+delta ile toplar). İlk `0x060` gelene kadar decode edilmez (`bms_cell_min_valid`).

### HV BMS hücre sıcaklıkları `0x130`/`0x140`/`0x150` (LE, 64B FD)
Her hücre **u16 LE delta** (`/100` °C); `hücre_T = BMS_MIN_CELL_TEMP + delta`. Pakette `cell_temperatures[]` = `(°C + 40)` u8. İlk `0x060` gelene kadar decode edilmez.

### LV-BMS `0x131` (LE, 24B)
| Byte | Sinyal | Tip | → MainBuffer | Dönüşüm |
|---|---|---|---|---|
| 0–1 | total V | u16 | `f[LVBMS_VOLTAGE_f]` | /100 V |
| 10–11 | max cell V | u16 | `f[LVBMS_MAX_CELL_VOLTAGE_f]` | /100 |
| 12–13 | min cell V | u16 | `f[LVBMS_MIN_CELL_VOLTAGE_f]` | /100 |
| 16–17 | current | i16 | `f[LVBMS_CURRENT_f]` | /10 A |
| 18–19 | max temp | i16 | `f[LVBMS_MAX_TEMP_f]` | /100 °C |
| 20–21 | min temp | i16 | `f[LVBMS_MIN_TEMP_f]` | /100 |
| (2–9 cell0–3, 14–15 cellNo, 22 fault) | — | — | telemetri'de yok / SoC gönderilmiyor | |

### IMU `0x220` (LE, **DLC 48** — Excel7 / VCU / dash)
| Byte | Sinyal | Tip | → MainBuffer | Dönüşüm |
|---|---|---|---|---|
| 0–5 | accel X/Y/Z | i16 | `f[IMU_ACCEL_*]` | /100 m/s² (yerçekimi dahil) |
| 6–11 | gyro X/Y/Z | i16 | `f[IMU_GYRO_*]` + `YAW_RATE`=gyroZ | /16 °/s |
| 12–17 | euler yaw/roll/pitch | i16 | `f[IMU_EULER_*]` | /16 ° |
| 18–23 | mag | — | **0** (imu26 reserved) | — |
| 24–27 | calib sys/gyro/accel/mag | u8 | `u8[IMU_CALIB_*]` | ×1 |
| 28–30 | health | u8 | telemetri buffer’da alan yok | — |
| **32–39** | quat W/X/Y/Z | i16 | `f[IMU_QUAT_*]` | /16384; **sadece DLC≥48** |

### GPS `0x230` (**BIG-ENDIAN**, 32B)
| Byte | Sinyal | Tip | → MainBuffer | Dönüşüm |
|---|---|---|---|---|
| 0–3 | latitude | i32 BE | `i32[GPS_LATITUDE_i32]` | ham (×1e-7 °) — doğrudan kopya |
| 4–7 | longitude | i32 BE | `i32[GPS_LONGTITUDE_i32]` | ham |
| 12–13 | ground speed | u16 BE | `u8[GPS_SPEED_u8]` | /100 km/h → u8 |
| (velN/E/D, altitude, fix/numSV) | — | — | telemetri'de yok | |

### VCU status `0x256` (FD, LE, 16B) — dash/VCU MSG-1
| Byte | Sinyal | → MainBuffer |
|---|---|---|
| 0 | vehicle_state | `u8[VCU_VEHICLE_STATE_u8]` |
| 1 | drive_mode | `u8[VCU_DRIVE_MODE_u8]` |
| 4 | **vehicle_speed (km/h)** | `u8[VCU_VEHICLE_SPEED_u8]` — **hız otoritesi** |
| 5 | throttle % | `u8[THROTTLE_PERCENT_u8]` |
| 6 | apps_status | `u8[VCU_APP_STATE_REQ_u8]` |
| 8–9 | torque_req (Nm) | `u16[VCU_TORQUE_NM_REQ_u16]` |
| 10 | tire_speed_RR | `u8[TIRE_SPEED_RR_u8]` |
| 11 | tire_speed_RL | `u8[TIRE_SPEED_RL_u8]` |

> `0x0B0` RPM→km/h yalnızca henüz `0x256` görülmediyse fallback (dashboard: inverter hız yazmaz).

---

## 2. Classic hattı (FDCAN2)

### İnverter (Cascadia/PM tarzı, LE, 8B) — ID `0x0A0–0x0AF`, `0x0B0`
| ID | Byte | Sinyal | → MainBuffer | Dönüşüm |
|---|---|---|---|---|
| `0x0B0` | 2–3 | torque feedback | `i16[INV_TORQUE_EST_NM_i16]` | /10 Nm ⚠ (telemetri i16 ham mı ×10 mu?) |
| `0x0B0` | 4–5 | motor speed | `i32[INV_EMACHINE_SPEED_ERPM_i32]` | ham RPM |
| `0x0B0` | 6–7 | DC bus V | `u16[INV_DCBUS_VOLTAGE_u16]` | /10 V |
| `0x0A0` | 0–5 | phase A/B/C temp | `i16[INV_PWRSTG_TEMP_i16]` | max(A,B,C)/10 °C |
| `0x0A0` | 6–7 | gate driver temp | `i16[INV_BOARD_1_TEMP_i16]` | /10 |
| `0x0A1` | 0–1 | control board temp | `i16[INV_BOARD_2_TEMP_i16]` | /10 |
| `0x0A2` | 4–5 | motor temp | `i16[INV_EMACHINE_TEMP_1_i16]` | /10 |
| `0x0A6` | 6–7 | DC bus current | (→ INV_ACBUS_POWER hesabı) | /10 A |
| `0x0A6` | — | AC power = Idc×Vdc | `i16[INV_ACBUS_POWER_i16]` | kW×10 |
| `0x0A8` | 4–5 / 6–7 | Id / Iq feedback | `f[INV_CURRENT_D_A_f]` / `f[INV_CURRENT_Q_A_f]` | /10 A |
| `0x0AB` | 0–3 / 4–7 | POST / Run fault word | `u16[INV_DEM_u16]` (16-bit'e sığdır) ⚠ | u32 → truncate |
| `0x0AA` | bit-packed | VSM/Inverter/durum bit'leri | `u16[INV_PWRSTG_BITSTATE_u16]` / `u8[INV_EMCTRL_FOC_BITSTATE_u8]` / `u8[INV_APP_STATE_APP_u8]` ⚠ | bit eşleme netleşmeli |

> ⚠ İnverter alanları telemetri `INV_*` alanlarıyla 1:1 örtüşmüyor; yukarısı en makul eşleme. Telemetri'nin hangi INV alanını gerçekten istediğini birlikte kesinleştirelim.

### Damper (LE, byte0–1 ham açı u16 12-bit)
| ID | Köşe | → MainBuffer ⚠ |
|---|---|---|
| `0x1F5` | FL | `f[WHEEL_TRAVEL_FL_f]` veya `f[DAMPER_COMPRESSION_FH_f]` |
| `0x1F4` | FR | `f[WHEEL_TRAVEL_FR_f]` / `DAMPER_COMPRESSION_FR` |
| `0x1F1` | RL | `f[WHEEL_TRAVEL_RL_f]` / `DAMPER_COMPRESSION_RH` |
| `0x1F2` | RR | `f[WHEEL_TRAVEL_RR_f]` / `DAMPER_COMPRESSION_RR` |

> ⚠ Ham açı (0–4095) mı saklansın, yoksa VCU'nun kalibrasyonlu heave/travel dönüşümü mü? Telemetri'de hem `DAMPER_COMPRESSION_*` hem `WHEEL_TRAVEL_*` var — hangisi hangisi?

### Fren basıncı (CANopen DS404, LE)
| ID | Byte | → MainBuffer | Dönüşüm |
|---|---|---|---|
| `0x181` (front) | 0–3 | `u8[BRAKE_PRESSURE_FRONT_u8]` | i32/1000 bar → u8 |
| `0x182` (rear) | 0–3 | `u8[BRAKE_PRESSURE_REAR_u8]` | i32/1000 bar → u8 |

### Dashboard / gaz `0x210` (LE, classic)
| Byte | Sinyal | → MainBuffer |
|---|---|---|
| 0–1 | pot1 (APPS1 ham ADC) | `u16[POT1_U16]` |
| 2–3 | pot2 (APPS2 ham ADC) | `u16[POT2_U16]` |
| 4 | tire_speed_FR | `u8[TIRE_SPEED_FR_u8]` |
| 5 | tire_speed_FL | `u8[TIRE_SPEED_FL_u8]` |
| 6 | buttons | (telemetri'de yok) |

### TMS lastik sıcaklık `0x300/0x310/0x320/0x330` (LE) — ⚠ VCU'da default KAPALI
Zone frame (base+9): byte0–1 inner, 2–3 mid, 4–5 outer, int16 centi-°C (/100) → `f[TIRE_TEMP_FL/FR/RL/RR_f]` (bir bölge, ör. mid). ⚠ Etkinleştirilecek mi + hangi bölge?

---

## 3. CAN'da OLMAYAN alanlar (kaynak netleşmeli) ⚠
Telemetri `MainBuffer`'da olup CAN haritasında karşılığı belirsiz olanlar:
- `SLIP_RATIO_f`, `RTC_MILLISEC_u16` — VCU türetir; CAN'da ayrı yayın var mı?
- `FLUID_TEMP/PRESSURE_*` (radyatör) — VCU'ya göre CAN'da değil (VCU'nun kendi ADC'si). Telemetri'de nereden?
- `IMU_SPEED_u8` — IMU hız göndermiyor.
- Telemetri kartının **kendi sensörleri**: `P_SGNL1/2` (PA0/PA1 ADC, INA826) ve 4× ABS teker-hız timer'ı. Bunlar hangi fiziksel ölçüm? (Fren? Damper? Teker hız?) — bunları CAN yerine yerel okuyacağız; eşlemesi lazım.

---

---

## Faz 3 — UYGULANAN (kod: `Core/Src/can_decode.c`, `Core/Inc/can_decode.h`)

Tek giriş fonksiyonu `CAN_ProcessRxFrame(hfdcan, header, data)` RX FIFO0 callback'inden
çağrılır; `hfdcan->Instance` (FDCAN2=classic / FDCAN3=FD) ve mesaj ID'sine göre çözer ve
**fiziksel** değeri `MainBuffer`'a yazar. Çözüm ilkeleri VCU `Decode::canDecode` ile birebir
(`CAN_Decode`/`CAN_DecodeRaw`). Ayrıca callback'te `RxData[8] → [64]` düzeltildi (FD payload'ı
kesiyordu) ve FDCAN2/3'e "eşleşmeyeni FIFO0'a kabul et" global filtresi eklendi.

**Kaynağı VCU decode olan ve tam çözülen mesajlar (kesin):**
- HV BMS `0x060` + hücre `0x100/0x110/0x120` (ham delta byte → `cell_voltages[]`)
- LV-BMS `0x131`, IMU `0x220` (gyroZ→`YAW_RATE_f`), GPS `0x230` (BE; yer hızı **unsigned**)
- Fren `0x181/0x182` (int32/1000 bar), Dashboard `0x210` (pot1/pot2, teker hızı FR/FL)
- İnverter: `0x0B0` (torque_est, motor hızı→`INV_EMACHINE_SPEED_ERPM`, **araç hızı = RPM×0.02394**,
  DC bus V), `0x0A0` (max faz sıc.→`INV_PWRSTG_TEMP`, gate→`INV_BOARD_1`), `0x0A1`→`INV_BOARD_2`,
  `0x0A2`→`INV_EMACHINE_TEMP_1`, `0x0A6` (AC güç=Idc×Vdc/100), `0x0A8` (Id/Iq→`INV_CURRENT_D/Q`),
  `0x0AA` (durum + bit-bayrak→`INV_PWRSTG_BITSTATE`), `0x0AB` (Run hata düşük16→`INV_DEM_u16`).
- **Damper** `0x1F5/1F4/1F1/1F2`: VCU heave/roll matematiği **birebir** (kalibrasyon KAPALI →
  sabit rest 1950/1223/3551/2697, wrap-around, mirror işaret). VCU metre üretir; biz **mm**
  saklıyoruz (`m×1000`) ki `TransmitFrame`'in `×10` paketlemesi çözünürlüğü korusun.
  `DAMPER_COMPRESSION_FH/FR/RH/RR` = ön/arka heave/roll; `WHEEL_TRAVEL_*` = köşe düşey (mm).

**VCU'da kaynağı OLMAYAN telemetri alanları (0 kalır):** `INV_EMACHINE_TEMP_2`,
`INV_SETPOINT_APP_Q/D`, `INV_VOLT_MODULUS_PERMIL`, `INV_TORQUE_MAX_FEAS_NDM`,
`LVBMS_ESTIMATED_SoC` (0x131'de yok), IMU `MAG`/`IMU_SPEED`, `FLUID_*`, `SLIP_RATIO`,
`TIRE_TEMP_*` (TMS VCU'da kapalı), `RTC_MILLISEC`.

**VCU-status (0x256/0x257) alanları (0 kalır — placeholder ID):** `VCU_VEHICLE_STATE`,
`VCU_DRIVE_MODE`, `THROTTLE_PERCENT`, `VCU_TORQUE_NM_REQ`, `VCU_APP_STATE_REQ`, `BSPD_PLAUSIBILITY`.
(Not: `VCU_VEHICLE_SPEED` inverter `0x0B0`'den doluyor.)

## Kalan açık sorular (kullanıcı bilgisi gerektiren)
1. **VCU yayın ID'leri** (`0x256/0x257` placeholder) — gerçek ID + byte düzeni gelince VCU-status
   alanları da doldurulacak.
2. **Damper birimi/motion-ratio** — VCU `DAMPER_M_PER_DEG=0.001` placeholder; biz mm sakladık.
   Sunucunun beklediği birim/ölçek netleşince ayarlanır.
3. ~~İnverter `INV_DEM`~~ **ÇÖZÜLDÜ:** VCU ile birebir yapıldı — `MainBuffer`'da iki 32-bit hata
   kelimesi (`INV_DEM1_u32`=POST, `INV_DEM2_u32`=Run). Kablo formatı bozulmasın diye mevcut 15-bit
   `INV_DEM` slotuna Run kelimesinin düşük bitleri gidiyor; tam 32-bit kabloya taşımak istenirse
   sunucu parser'ı + frame boyu koordineli değişmeli.
4. **TMS** lastik sıcaklığı — VCU'da kapalı; etkinleştirilecek mi (telemetri `TIRE_TEMP_*` u8 mi f mi)?
5. **Telemetri kartının kendi sensörleri** (P_SGNL1/2 ADC + 4 teker-hız timer) — CAN'dan bağımsız,
   ayrı ele alınacak; hangi fiziksel ölçüm, hangi `MainBuffer` alanı?
6. **BMS `0x060`** layout — VCU decode esas alındı; gerçek BMS ile bir kez doğrula.
</content>
