# Telemetry26 — Sistem İnceleme & Düzeltme Planı

**Kart:** Formula Student telemetri PCB'si
**MCU:** STM32G474RCT6 (FreeRTOS, CMSIS-V2)
**GSM modülü:** Quectel EG915N-EU (LTE Cat 1)
**Veri yolu:** Araç CAN hattı (FDCAN2/FDCAN3) → MainBuffer → bit-packing → UART3 (DMA/idle) → EG915N → **UDP** → sunucu
**Referans belgeler (repoda):**
- `...eg915n_series_hardware_design_v1-4...pdf` (HW Design)
- `...tcpip_application_note_v1-4.pdf` (TCP/IP Note)
- `...at_commands_manual_v1-4...pdf` (AT komut kılavuzu)
- `telemetry26.pdf` (şematik)

> Bu doküman, yapılan incelemenin bulgularını, alınan kararları ve fazlı düzeltme
> planını içerir. **Faz 1 uygulanmıştır** (bu commit). Faz 2/3 onay bekliyor.

---

## 0. Genel Değerlendirme

Mimari doğru: olay-kuyruğu (event queue) tabanlı AT state-machine, ring buffer +
`HAL_UARTEx_ReceiveToIdle_DMA` ile satır parser'ı, iki FreeRTOS task'ı (telemetri +
gsm), mutex korumalı paylaşımlı buffer ve CRC8. Bit-packing katmanı düzgün.

Ancak inceleme öncesi sistem **uçtan uca çalışmıyordu**: firmware'de akışı kilitleyen
2 kritik hata ve şematik↔firmware arası donanımsal bir polarite uyumsuzluğu vardı.
Bunlar Faz 1'de giderildi.

---

## 1. Bulgular (önem sırasına göre) ve datasheet teyitleri

### 🔴 Kritik — Faz 1'de düzeltildi

**B1. GSM state-machine `INIT_AT`'te kilitleniyordu.**
`INIT_AT`, `expected_event` (=EVT_NONE) ve `next_state` (=INIT_AT) ayarlamadan
`WAIT_RESPONSE`'a geçiyordu. `WAIT_RESPONSE` içinde `EVT_NONE == EVT_NONE` anında
"eşleşme" sayıldığı için tekrar `INIT_AT`'e dönüyordu → `DISABLE_ECHO`, APN, PDP,
`QIOPEN` state'lerine **hiç girilmiyordu**. Soket asla açılmıyordu.

**B2. PWRKEY/RESET_N polaritesi ters + `MCU_RST` hiç bırakılmıyordu → modül kalıcı reset'te.**
HW Design §3.5.1 (Figure 11) ve §3.7 (Figure 15): PWRKEY ve RESET_N **açık-drain/kollektör
(evirici)** sürücüyle sürülür — şematikteki 2N7002 (Q1/Q2, source=GND) tam bu referans devre.
Dolayısıyla **MCU pini HIGH ⇒ modül pini LOW (aktif)**.
- HW Design §3.5.1: Açmak için PWRKEY **≥500 ms LOW** tutulup bırakılır (opsiyon②: ≥700 ms), VBAT ≥30 ms stabil, boot ~10 s (Figure 13).
- HW Design §3.7: RESET_N **aktif-low**, **≥300 ms LOW** tutulup bırakılır; yalnız baseband'i resetler, "son çare" kullanılır.

Firmware ise tersini varsayıyordu ve `MCU_RST` yalnızca `SET` (=RESET_N LOW = reset aktif)
yazılıyor, **hiçbir yerde `RESET` (serbest) yapılmıyordu** → modül hiç boot etmiyordu.

**B3. `AT+QIOPEN` sonucu (`+QIOPEN` URC) beklenmiyordu.**
TCP/IP Note §2.3.5: `QIOPEN` önce `OK`, sonra **`+QIOPEN: <connectID>,<err>`** URC'si döner
(`err=0` = başarı, 150 s'ye kadar). Firmware yalnız `OK` bekleyip `IDLE`'a geçiyordu; soket
açılamasa bile "hazır" sanıyordu.

**B4. UART hata callback'i RX DMA'yı durdurup yeniden başlatmıyordu.**
Bir overrun (ORE) sonrası `HAL_UART_DMAStop` çağrılıp RX yeniden kurulmuyordu → modülden
gelen cevaplar parse edilemez hale geliyor, sistem ancak tam `HW_RESET`'e kadar (~15 s+) sağır kalıyordu.

**B5. `gsm_ready` soket açılmadan set ediliyordu** (`GSM_Init` biter bitmez) → semantik yanlış.

### 🟠 Önemli — kısmen Faz 1, kalanı Faz 2

- **B6.** Recovery'deki `AT+QICLOSE` timeout'u 500 ms idi; TCP/IP Note §2.3.6: `QICLOSE` **max 10 s**. *(Faz 1'de 10 s yapıldı.)*
- **B7.** Ağa kayıt (`AT+CEREG?`) tek-atış + 500 ms timeout ile yapılıyor; LTE kaydı 10–30 s sürebilir. *(Faz 2)*
- **B8.** Heap 8 KB + `configUSE_NEWLIB_REENTRANT=1` + `printf` → yığın/heap sınırda olabilir. *(Faz 2)*
- **B9.** Düşük-bant paketinde CRC, gönderilen 25 baytın sonunda değil 21. baytta; iç tutarsızlık. *(Faz 2 — nihai düzen sunucu spec'ine bağlı.)*

### 🟡 Şematik notları (kod dışı — donanım ekibinin doğrulaması)

- **CAN sonlandırması:** CAN1 ve CAN2'nin **her ikisinde** kart üstünde ~120 Ω split-termination var
  (2×59 Ω + 4n7). CAN hattında **yalnız iki fiziksel uç** 120 Ω olmalı. Kart araç hattına
  **ara nokta (tap)** olarak bağlanıyorsa buradaki termination üçüncü terminatör olur ve hattı bozar
  → **DNP/jumper ile opsiyonel** yapılmalı. Kart hattın ucundaysa sorun yok.
- FDCAN'de açık filtre yok ama STM32G4'te `RXGFC.ANFS` reset=0 olduğundan eşleşmeyen tüm standart ID'ler
  FIFO0'a kabul edilir → alım çalışır (yoğun hatta filtre eklemek CPU yükünü azaltır).

### 🟢 Ertelenen (bilgi) — Faz 3

`MainBuffer` CAN'dan doldurulmuyor; ADC okunmuyor; teker-hız input-capture callback'leri boş;
SIM PIN'i yok varsayılıyor. (Kullanıcı tarafından bilinçli olarak ertelendi.)

---

## 2. Alınan Kararlar

| Konu | Karar | Etki |
|---|---|---|
| Veri yönü | **Tek yön** (yalnız telemetri gönderimi) | `QIOPEN` **buffer access mode (0)**; gelen veri (`+QIURC "recv"`) parse yolu **eklenmedi** |
| SIM | **PIN yok, APN=`internet`** | Mevcut `CPIN?`/`QICSGP` akışı korundu; ek PIN kodu yok |
| CAN matrisi | **Ertelendi** | FDCAN bit-timing/ID/filtre Faz 3'te |
| Sunucu paket spec'i | **Elde yok** | telemetry.c mevcut düzeni "doğru protokol" kabul edildi; nihai doğrulama spec gelince |

---

## 3. Faz 0 — Dal hazırlığı  ✅

- Çalışma dalı `claude/formula-student-telemetry-pcb-h3k8ix`, `origin/main`'e ileri sarıldı
  (yüklenen 2 datasheet dahil). Düzeltmeler bunun üstüne commit'lendi.

---

## 4. Faz 1 — Blocker düzeltmeleri  ✅ (bu commit)

Modülü fiilen açığa çıkarıp ilk UDP paketini gönderten minimum set. Dosya:değişiklik:

### 4.1 GPIO polarite + güç dizisi (B2)
- `gsm.c`: Anlamsal makrolar eklendi — evirici sürücüye göre:
  `PWRKEY_PRESS()`=MCU HIGH (PWRKEY LOW), `PWRKEY_RELEASE()`=MCU LOW,
  `RST_ASSERT()`=MCU HIGH (RESET_N LOW), `RST_RELEASE()`=MCU LOW.
- `GSM_Init()`: Açılış dizisi datasheet'e göre yazıldı — `RST_RELEASE()` → PWRKEY **700 ms** press →
  release → ~10 s boot bekleme (HW Design Figure 13).
- `GSM_STATE_HW_RESET`: Deep-recovery olarak yeniden tasarlandı — RESET_N **300 ms** darbesi + RX DMA yeniden kurulumu (HW Design §3.7).
- `main.c` `MX_GPIO_Init`: başlangıç `MCU_RST` seviyesi `SET`→`RESET` (RESET_N serbest).
- `Telemetry26.ioc`: `PC7.PinState=GPIO_PIN_SET`→`GPIO_PIN_RESET` (CubeMX regen'de de doğru üretsin diye).

### 4.2 State-machine dead-end (B1)
- `GSM_STATE_INIT_AT`: sonunda `expected_event=EVT_OK`, `next_state=GSM_STATE_DISABLE_ECHO`,
  `post_tx_state`, `state_timeout_start`, `current_timeout_limit=INIT_AT_WAIT_LIMIT (3 s)` ayarlanıyor.
- `WAIT_RESPONSE` ve `WAIT_PROMPT`: `expected_event != EVT_NONE` guard'ı eklendi (EVT_NONE artık asla "eşleşme" saymaz).

### 4.3 +QIOPEN URC + timeout'lar (B3, B6)
- Parser'a `+QIOPEN: 0,0` → yeni `EVT_UDP_OPEN_OK`, `+QIOPEN: 0,<≠0>` → `EVT_ERROR` eklendi.
- `GSM_STATE_UDP_OPEN`: artık `EVT_UDP_OPEN_OK` bekliyor (yalnız OK değil), timeout `QIOPEN_WAIT_LIMIT (60 s)`.
- `access_mode` `1`→`0` (buffer, tek-yön kararı gereği).
- Recovery `QICLOSE` timeout'u `AT_WAIT_LIMIT`→`QICLOSE_WAIT_LIMIT (10 s)`.

### 4.4 UART hata → RX DMA restart (B4)
- `main.c` `HAL_UART_ErrorCallback`: `DMAStop` sonrası flag temizleme + `ReceiveToIdle_DMA` yeniden kurulumu
  (başarısızsa `dma_rx_restart_err_cnt++`).

### 4.5 `gsm_ready` semantiği (B5)
- Erken `gsm_ready=1` (`StartGSMTask`) kaldırıldı; artık ilk kez `GSM_STATE_IDLE`'a (soket açık) ulaşınca set ediliyor.

**Not (regen güvenliği):** `MX_GPIO_Init` CubeMX üretimi olduğundan `.ioc`'deki `PC7.PinState` de
düzeltildi; ayrıca gerçek kontrol `gsm.c`'de (kullanıcı kodu) olduğundan regen sonrası da doğru kalır.

---

## 5. Faz 2 — Sağlamlaştırma (onay bekliyor)

- **2.1** ✔ karar verildi: `QIOPEN` buffer access (0) — Faz 1'de uygulandı; RX parse eklenmeyecek.
- **2.2** CEREG kayıt beklemeyi iyileştir (`AT+CEREG=2` + URC veya periyodik poll), 500 ms tek-atış yerine.
- **2.3** Recovery'yi katmanla: önce `QICLOSE→QIACT→QIOPEN`; yalnız tekrarlı başarısızlıkta deep-recovery
  (RESET_N/PWRKEY), datasheet "RESET_N son çare" notuna uygun.
- **2.4** `printf`'i (`GSM_Parse_Response`) debug makrosu arkasına al; `uxTaskGetStackHighWaterMark` ile
  yığın kullanımını ölç, gerekirse heap'i büyüt.
- **2.5** Düşük-bant paketinde CRC'yi fiilen gönderilen uzunluğun sonuna koy; `is_low_bandwidth_mode`'un
  frame-doldurma ↔ gönderim arasında değişme yarışını gider. *(Nihai düzen sunucu spec'ine bağlı.)*

---

## 6. Faz 3 — Ertelenen (CAN & sunucu doğrulaması)

- FDCAN bit-timing'i geçerli hıza çek (`.ioc`'de şu an ~3.54 Mbps geçersiz); gerekirse ID filtreleri.
- `MainBuffer`'ı CAN Rx callback'inde ID'lere göre doldur; ADC okuma; teker-hız input-capture callback'leri.
- telemetry.c paket düzeni (endianness, offset, ölçek, CRC yeri) ↔ sunucu ayrıştırıcısı doğrulaması.

---

## 7. Gri Noktalar (netleşmesi gerekenler)

1. **Sunucu UDP paket spec'i:** bit-packing/endianness/CRC'nin sunucu tarafıyla birebir uyumu. *(Elde yok.)*
2. **CAN hattı:** hız (250k/500k/1M) ve klasik/FD; BMS (0x6B0) & INV (0x6A0) mesaj düzenleri. *(Faz 3.)*
3. **Sunucu portu:** 135.125.196.63:5010'un **UDP** dinlediğinin teyidi.
4. **`AT+CMER`/`+CIEV` sinyal URC'si:** düşük-bant tespitinin çalışması için modülün sinyal göstergesini yayınladığının doğrulanması.

---

## 8. Donanımda Doğrulama Adımları (Faz 1 sonrası)

1. CubeIDE'de derle, karta flash'la.
2. UART3 TX/RX'i (veya modülün DBG portunu) logic analyzer/USB-UART ile izle.
3. Açılışta beklenen: PWRKEY 700 ms LOW darbesi → ~10 s sonra modülden `RDY` → `AT`→`OK` →
   `ATE0`→`OK` → `CFUN=1`→`OK` → `CPIN?`→`+CPIN: READY` → `CEREG?`→`+CEREG: 0,1` →
   `QICSGP`→`OK` → `QIACT=1`→`OK` → `QIOPEN`→`OK` + `+QIOPEN: 0,0` → `IDLE`.
4. `IDLE` sonrası `QISEND=0,<len>` → `>` → payload → `SEND OK` döngüsü.
5. Sunucuda UDP paketlerinin geldiğini doğrula.
6. Takılırsa hangi AT komutunda durduğunu paylaş → o state'e odaklanırız.
