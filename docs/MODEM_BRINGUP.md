# EG915N Modem Bring-Up Guide

## AT sequence (normal boot)

```
AT → ATE0 → AT+QSCLK=0 → AT+CMER=3,0,0,2
AT+QCFG="airplanecontrol"          (read-back: 1 = W_DISABLE# holds RF off)
AT+CFUN=1 → AT+CPIN? → AT+CEREG=2 → AT+CEREG? (poll)
AT+QICSGP=1,1,"internet","","",1 → AT+QIACT=1
AT+QIOPEN=1,0,"UDP","135.125.196.63",5010,0,0
  → OK + URC +QIOPEN: 0,0
```

`AT+QCFG="sarcfg"` and `AT+QCFG="lbooster"` do **not exist** on EG915N — the AT manual
lists only `band`, `nwscanmode`, `nwscanseq`, `roamserviceex`, `servicedomain`,
`gprsattach`, `airplanecontrol`, `urcdelay`, `usbnet`, `ims`, `nat`, `risignaltype`,
`sleepind/drx`, `urc/*`, `fota/times`, `BIP`, `ppp/termframe`, `uart2ipr`. Earlier
firmware sent `sarcfg`; it returned ERROR, so no TX-power limit was ever written to
module NV. Do not chase it as a cause of weak uplink.

## Attach (registration) path

Registration is the slow part, not the socket. A cold LTE cell search plus attach on a
marginal indoor cell routinely needs **30–90 s**, and every UE reset restarts it from
scratch. Two rules follow:

- `AT+CFUN=1,1` is `<rst>=1` — a **full module reboot**, not a radio restart. It is a
  last resort, spaced by `MODEM_UE_RESET_GAP_MS` (5 min), and the volatile config
  (`QSCLK`, `CEREG=2`) must be re-sent after the module answers AT again.
- `RESET_N` cannot create coverage. `CHECK_NET` already hard-resets when AT stops
  answering; bad RF alone only escalates after `qiopen_fail_count >= 20`.

`+CEREG` has two shapes and they must be told apart:

| Source | Shape | Stat field |
|--------|-------|-----------|
| `AT+CEREG?` | `+CEREG: <n>,<stat>[,<tac>,<ci>[,<AcT>]]` | second (after comma, a digit) |
| URC, `n=2` | `+CEREG: <stat>[,"<tac>","<ci>"[,<AcT>]]` | first (comma is followed by `"`) |

`<stat>`: 0 = not searching, 1 = registered home, 2 = searching, **3 = registration
denied**, 4 = unknown, 5 = roaming.

`cereg=3` is the network actively refusing — read `modem_live.ceer` (`AT+CEER`) and
`modem_live.oper` (`AT+COPS?`). It is a subscription/PLMN answer, not an RF level
problem. `csq=99` with `cereg=0` for minutes is the opposite: no cell is being camped.

## UDP send

```
AT+QISEND=0,373 → > → <373 bytes> → SEND OK
```

Firmware notes (aligned to Final26 `gsm.c` @ 83abbac):
- USART3 RX DMA stays **circular**; never `HAL_UART_DMAStop` mid-QISEND.
- IDLE callback only copies delta — **no rearm** (Final26).
- QISEND path: flush → AT+QISEND → wait `>` → payload → SEND OK (no ForceRxArm per send).
- Send fail: `EnsureCommandMode` + same-socket retry; reopen only if `QISTATE` down (`SEND_RETRY_MAX=3`).
- Boot: if AT answers, **do not QPOWD** (keeps LTE/PDP). Silent → filler → PWRKEY.

## PWRKEY / RESET (2N7002 inverted)

| MCU pin | Module pin | Action |
|---------|------------|--------|
| HIGH | LOW (active) | Press PWRKEY or assert RESET |
| LOW | HIGH (released) | Idle state |

- PWRKEY power-on: 700 ms active, then ~10 s boot wait
- RESET_N: last resort, 300 ms active-low
- PWRKEY is a **toggle** — probe AT before power-on if module may already be on
- **Warm flash (PSU left on):** Final26 style — if module answers AT, leave it on. Only filler/PWRKEY when silent.

## Server vs firmware

UDP telemetry is one-way. Dashboard **"Sinyal yok"** means *no recent packets arrived* — it is a **symptom**, not a cause of `prompt_miss`. Server cannot make the modem skip `>`. Check server only if `qisend_ok` climbs but the site stays dark.

## Live Expressions — minimal set (her koşuda açık tut)

CubeIDE → Live Expressions. Hepsi `modem_live.` prefix ile (global `modem_live`):

### Link health
- `modem_live.state` — 0=OFF 1=BOOTING 2=LINKING 3=READY 4=SENDING 5=RECOVERING
- `modem_live.fsm` — 0=HW_RESET 1=INIT_AT 2=CHECK_NET 3=IDLE 4=RECOVERY_WAIT
- `modem_live.ready` — 1 = TelemetryTask send isteyebilir
- `modem_live.csq` — 10..31 iyi; 99 = sinyal yok
- `modem_live.cereg` — 1 veya 5 = registered
- `modem_live.pdp_active` — 1 = PDP up
- `modem_live.socket_open` — 1 = UDP socket up
- `modem_live.last_qiopen_err` — 0 = OK; 566 = no PDP

### Send path (~5 paket sonra durma)
- `modem_live.qisend_ok` — sunucuya giden başarılı SEND OK sayısı
- `modem_live.qisend_fail` — UdpSendNow fail (outer)
- `modem_live.prompt_miss` — `>` gelmedi (WaitPrompt timeout)
- `modem_live.prompt_err` — WaitPrompt ERROR/SEND FAIL gördü
- `modem_live.sendok_timeout` — `>` geldi ama SEND OK gelmedi
- `modem_live.send_fail` — modem `SEND FAIL` dedi
- `modem_live.send_pending` — 1 = kuyrukta payload var
- `modem_live.send_retry_count` — outer fail sayacı; **≥3 → QISTATE/reopen** (`MODEM_SEND_RETRY_MAX`, Final26)
- `modem_live.send_backoff_ms` — pace üstüne ek (Final26 fail path'te 0)
- `modem_live.last_send_len` — son QISEND uzunluğu (373 beklenir)
- `modem_live.last_line` — son satır / miss snapshot (ASCII)

### RX / DMA
- `modem_live.rx_dma_circular` — **1 olmalı**
- `modem_live.uart_rx_busy` — linked iken çoğu zaman **1** (BUSY_RX)
- `modem_live.dma_rearm_count` — yavaş artabilir; saniyede onlarca = storm
- `modem_live.rx_fill` / `modem_live.rx_tail` — ring head/tail
- `modem_live.dma_old_pos` — son pump edilen DMA index (0..255)
- `last_modem_rx_tick` — (global) son RX tick; donarsa artmaz

### Attach / RF (kayıt olmuyorsa bu 8'i not et)
- `modem_live.vbat_mv` — **AT+CBC**: modülün kendi ölçtüğü VBAT (mV). FB1 ve yerel
  bulk kapasitörlerden *sonra*ki tek gerçek okuma. EG915N Vmin = **3400**, Vnom = 3800
- `modem_live.vbat_min_mv` — boot'tan beri en düşük CBC değeri. **< 3400 → donanım**
- `modem_live.ceer` — AT+CEER reject/release cause metni (cereg=3 ise buraya bak)
- `modem_live.oper` — AT+COPS? seçilen PLMN
- `modem_live.airplane_ctl` — AT+QCFG="airplanecontrol". **1 = W_DISABLE# pini RF'i
  kapatıyor** (bu kartta pin 18 GND'ye bağlı → 1 görürsen sebep bu). -1 = okunamadı
- `modem_live.airplane_status` — 1 = modül şu an airplane mode'da
- `modem_live.ue_reset_count` — AT+CFUN=1,1 modül reboot sayısı. Kayıt beklerken
  **artmamalı**; artıyorsa hücre araması sürekli baştan başlıyor
- `modem_live.rf_restart_count` — CFUN=1 soft restart (çoğunlukla no-op)

### Recovery thrash
- `modem_live.reopen_count` — SyncReopenSocket çağrıları
- `modem_live.recovery_count` — CHECK_NET / RECOVERY_WAIT sayacı
- `modem_live.at_ok_count` / `modem_live.at_fail_count` — Init AT + EnsureCommandMode

**AT ölü (bu oturumda modem cevap vermiyor):** `at_ok_count=0`, `cpin_ok=0`, `vbat_mv=0`,
`csq/cereg/cfun=-1`, `rx_fill==rx_tail` (ring boş). `fsm=CHECK_NET` + `state=LINKING`
görünse bile bu RF değil — önce AT/SIM/güç. `uart_rx_busy=1` sadece MCU DMA'nın
armed olduğunu gösterir; RX gelmiyorsa yine de 1 kalır.

**Sağlıklı:** `state=3` READY, `fsm=3` IDLE, `ready=1`, `socket_open=1`, `cereg=1|5`,
`rx_dma_circular=1`, `uart_rx_busy=1`, `qisend_ok`↑, `prompt_miss` düz, `send_retry_count=0`,
`reopen_count` sabit, `dma_rearm_count` sakin.

**"~5 paket sonra durdu" ilk bakış:** `qisend_ok` kaçta kaldı? `prompt_miss` / `qisend_fail` /
`send_retry_count` / `fsm` / `state` / `uart_rx_busy` / `reopen_count` — bu 7'yi not et.

## Breakpoint protokolü (sıralı)

Durunca **Continue** etme; Live Expressions snapshot al, sonra bir sonraki BP.

| # | Location | When | Check (healthy → failing) |
|---|----------|------|---------------------------|
| 1 | `Modem_UdpSendNow` @ `modem_at.c:497` | Her QISEND denemesi | `socket_open==1`, `uart_rx_busy==1`, `last_send_len≈373`. Fail sonrası: `send_retry_count` 0..2 |
| 2 | `Modem_WaitPrompt` timeout `return 0` @ `modem_at.c:89` | `>` gelmedi | `last_line` boş/çöp mü? `rx_fill==rx_tail` (ring boş=RX sağır). `dma_old_pos` donuk mu? |
| 3 | `Modem_WaitPrompt` `return 2` @ `modem_at.c:79` | Modem ERROR | `prompt_err`↑; `last_line` içinde ERROR? → soket/komut modu, RX değil |
| 4 | `modem_live.prompt_miss++` @ `modem_at.c:519` | Soft miss | `prompt_miss` adımı; inner retry sonrası yine miss → outer fail |
| 5 | `modem_live.qisend_fail++` @ `modem.c:245` | Outer fail | `send_retry_count` kaç? `state` 4→5→3 mi yoksa 2 LINKING mi? |
| 6 | `send_retry_count >= MODEM_SEND_RETRY_MAX` @ `modem.c:253` | **3. outer fail** | QISTATE/reopen dalı. Sonra QuerySocketUp mı EnsureSocketUp mı? |
| 7 | `Modem_QuerySocketUp` @ `modem_at.c:361` (fonksiyon çıkışı) | 3. fail sonrası | return 1 + `socket_open=1` → RX miss, soket sağlam. return 0 → reopen/CHECK_NET |
| 8 | `Modem_SyncReopenSocket` @ `modem_at.c:383` | Reopen | `reopen_count`↑. `last_line` hâlâ `+QIOPEN: 0,0` ise reopen RX'i düzeltmez |
| 9 | `fsm_state = MODEM_FSM_CHECK_NET` @ `modem.c:260` | EnsureSocketUp fail | Telemetry **durur** (`ready=0`). `cereg`/`csq`/`pdp_active` |
| 10 | `HAL_UARTEx_RxEventCallback` @ `main.c:811` | USART3 RX event | `Size` artıyor mu? Artmıyorsa DMA/IDLE ölü |
| 11 | `Modem_ForceRxArm` → `Modem_ArmRxDma` @ `modem_uart.c:119` | Soft rearm fail | `dma_rearm_count` fırtınası; `rx_dma_circular` 0 olursa kritik |

Opsiyonel: `Modem_WaitSendOk` return 0 → `sendok_timeout` (prompt var, SEND OK yok).

## Decision tree (durduktan sonra)

Snapshot: `qisend_ok`, `prompt_miss`, `qisend_fail`, `send_retry_count`, `fsm`, `state`,
`socket_open`, `uart_rx_busy`, `rx_dma_circular`, `reopen_count`, `last_line`, `csq`.

```
A) qisend_ok≈5 (veya N), sonra qisend_ok donuk
   ├─ prompt_miss≈qisend_fail, socket_open=1, uart_rx_busy=0
   │     → RX READY/sağır. Cause: DMA IDLE sonrası rearm kaçtı / BUSY değil.
   │     Fix yönü: ForceRxArm / circular / IdleDone. BP #2+#10.
   ├─ prompt_miss≈qisend_fail, socket_open=1, uart_rx_busy=1, rx_fill ilerliyor ama '>' yok
   │     → Modem '>' vermiyor (komut modu / QISEND reddi). last_line'a bak.
   │     BP #3 (prompt_err) vs #2 (timeout).
   ├─ send_retry_count her fail'de 1..3, 3'te QuerySocketUp=1, reopen_count düz
   │     → Bilinçli "3 fail sonra settle" path. Soket OK; RX miss thrash.
   │     Confirm: BP #6→#7 return 1. Fix: RX path, reopen değil.
   ├─ send_retry_count→3, sonra fsm=2 CHECK_NET, ready=0, reopen_count↑
   │     → EnsureSocketUp fail. Cause: QISTATE silent / QIOPEN fail.
   │     Confirm: last_qiopen_err, cereg, pdp. BP #8+#9.
   ├─ state=4 SENDING uzun süre (>2s) donuk
   │     → WaitPrompt/WaitSendOk içinde. BP #1 leave etmeden #2/#WaitSendOk.
   ├─ prompt_miss=0, sendok_timeout↑ veya send_fail↑
   │     → `>` OK; ağ/UDP tarafı. PSU/operator; RX değil.
   ├─ csq→0/1/99 veya cereg≠1/5 send durunca
   │     → VBAT sag / RF. Scope VBAT @ QISEND.
   └─ qisend_ok artıyor ama dashboard "Sinyal yok"
         → Sunucu/firewall; firmware send path OK.
```

### "~5" hipotezleri → hangi sayaç doğrular

| Hipotez | Confirm counters |
|---------|------------------|
| Outer retry tavanı (`MODEM_SEND_RETRY_MAX=3`) | `send_retry_count` 1→3, BP #6 hit; `qisend_fail`+=3 |
| Prompt miss / RX sağır | `prompt_miss`↑ ≈ `qisend_fail`; `uart_rx_busy=0` veya ring boş |
| QISTATE false-up, reopen skip | BP #7 returns 1; `reopen_count` düz; miss devam |
| Reopen thrash | `reopen_count`↑; `last_line=+QIOPEN: 0,0`; `prompt_miss` hâlâ↑ |
| Rate-limit / pace (yanlış alarm) | `send_pending=1`, `send_backoff_ms=150|300`, `fsm=3`; kısa süre sonra tekrar send |
| Stuck CHECK_NET | `fsm=2`, `ready=0`, `state=2` LINKING; TelemetryTask send etmez |
| DMA rearm storm | `dma_rearm_count` saniyede çok↑; `rx_dma_circular` 0'a düşerse kritik |
| SEND OK timeout | `prompt_miss` düz, `sendok_timeout`↑ |

## Troubleshooting (kısa)

```
never registers (cereg stays 0/2, csq 99)?
  ├─ airplane_ctl=1 → W_DISABLE# (pin 18, GND'ye bağlı) RF'i kapatıyor
  │     → AT+QCFG="airplanecontrol",0  ve pin 18'i GND'den ayır
  ├─ vbat_min_mv < 3400 → donanım: TPS54331 girişi (C4/C5 boş) + FB1 üzerindeki düşüş
  ├─ ue_reset_count artıyor → attach penceresi kesiliyor (bu commit'in düzelttiği şey)
  └─ vbat_min_mv ≥ 3600 ve ue_reset_count sabit → gerçekten kapsama/anten/SIM

cereg=3 (registration denied)?
  └─ ceer + oper oku. Abonelik/PLMN cevabı; RF seviyesi değil.
     AT+COPS=? tarama, AT+COPS=1,2,"<plmn>" manuel seçim (FPLMN'i bypass eder),
     AT+CRSM=176,28539,0,0,12 ile EF_FPLMN oku (hepsi FF = temiz)

qisend_ok not increasing?
  ├─ cereg not in {1,5} → network (antenna/SIM)
  ├─ pdp_active=0 → APN/PDP
  ├─ last_qiopen_err≠0 → socket (566=no PDP)
  ├─ prompt_miss≈qisend_fail, socket_open=1 → UART RX (`>` miss)
  ├─ send_retry_count hits 3 → QISTATE/reopen branch (BP #6)
  ├─ fsm=2 CHECK_NET, ready=0 → link recovery; send durur
  ├─ csq drop after N pkts → measure VBAT during TX
  └─ qisend_ok↑ but site dark → server only
```

### External factors (bench)

| Factor | Fits when | Check |
|--------|-----------|-------|
| PSU / VBAT sag | OK for tens–hundreds then CSQ/prompt_miss | Scope VBAT @ QISEND |
| Antenna / indoor | csq low/99, cereg flaps | Reseat; outdoor compare |
| Operator / server | sendok_timeout or SEND FAIL; prompt_miss=0 | not RX |

## Configuration (`App/config/app_config.h`)

- `APP_TELEMETRY_PERIOD_MS`: **500 = 2 Hz** default (bench-stable); 100 = 10 Hz vehicle
- `APP_MODEM_RF_PROFILE_ENABLE`: leave **0** — `sarcfg` is not an EG915N command
- `APP_GSM_SILENCE_RESET_MS`: 120000 (must exceed QIOPEN timeout)
- `MODEM_SEND_RETRY_MAX` (`modem_internal.h`): **3** outer fails before QISTATE/reopen — "~5 paket" ile karıştırma: o gözlem **başarılı** `qisend_ok` sayısı; bu sabit **fail** sayacı
