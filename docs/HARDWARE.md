# Telemetry26 Hardware Summary

## Power architecture

Schematic netlist (`telemetry26.pdf`) includes USB-C + TPS2116; **this assembled board does not populate them**. Bench/vehicle feed is used as follows:

```
Connector +5V ══ short ══ +5VOUT ──→ TPS54331 (U1) VIN
                              C4 47µF ‖ C5 47µF ‖ C6 100nF   ← buck INPUT bulk
                                         │
                    U1 SW → L1 → +3V8 (3.8 V; R4 75k / R11 20k → 0.8·4.75)
                              C2 47µF ‖ C3 47µF, C11 1µF, C14 10µF
                                         │
                    ├──────────────────→ TLV75733 (U2) IN+EN → +3V3 (STM32, CAN, shifter)
                    └── FB1 120Ω@100MHz → +3V8FB → VBAT_BB (32,33) + VBAT_RF (52,53)
                              C15 100µF, C19 100µF, 100nF/33pF/10pF each, D6 TVS
```

### Assembled board vs schematic (DNP)

| Schematic part | This board |
|----------------|------------|
| USB-C connector | **Empty / not used** |
| TPS2116 power mux (IC1) | **Empty / not used** |
| +5V ↔ +5VOUT | **Shorted** (mux bypass) |
| Power source | Bench/vehicle **connector +5V only** (≥2 A PSU recommended) |

Do **not** diagnose this board as “USB VBUS stealing the rail” — that path is unpopulated.

### EG915N power requirements (HW Design v1.4, ch. 2.5 / 3.4)

| Item | Quectel spec | This board |
|------|--------------|-----------|
| VBAT | Vmin **3.4 V**, Vnom 3.8 V, Vmax 4.5 V | 3.8 V → only **400 mV** droop budget |
| VBAT_RF current | ≥ **2.2 A** | TPS54331, 3 A rated |
| VBAT_BB current | ≥ 0.8 A | shared rail |
| Supply capability | ≥ **3.0 A** | PSU / vehicle +5V (mux unused) |
| Bulk at VBAT | **~100 µF, ESR ≤ 0.7 Ω**, plus 100 nF/33 pF/10 pF | C15/C19 100 µF ✓ |
| VBAT → BB/RF split | star, **0 Ω 1/10 W** each leg (Fig. 10) | **FB1 ferrite bead** ✗ |
| Reference design | 470 µF + 470 µF (Fig. 8) | 94 µF on +3V8 |

Two deviations that still matter when a DMM shows “+3V8 = 3.8 V” but attach fails:

1. **FB1 is a ferrite bead where Quectel specifies 0 Ω.** Its DCR is in series with the
   full RF burst current, and a probe on the `+3V8` test point sits on the *wrong side*
   of it. Measure at the module side, or read `modem_live.vbat_mv` (`AT+CBC`), which is
   the module's own ADC.
2. **C4/C5 are the buck's input bulk, not output.** With them depopulated the TPS54331
   has little input capacitance and may not source pulsed current without the **+5VOUT**
   rail dipping — often invisible on a slow DMM, bad for TX/search bursts. Use
   **ceramic** (X5R/X7R), not polarized. TI wants ~10 µF ceramic at VIN plus bulk when
   the source is not very close; TPS54331 stops regulating below **VIN ≈ 3.5 V**.

`AT+CBC` (`+CBC: <bcs>,<bcl>,<mV>`) is the cheapest VBAT diagnostic; firmware polls it
during attach and tracks the session minimum in `modem_live.vbat_min_mv`. Anything below
3400 is a hardware answer (even if a DMM on +3V8 looked fine).

### Module pins tied to GND — check against "keep it open"

`telemetry26.pdf` grounds several module pins. Most are harmless, one is not:

- **W_DISABLE# (pin 18) → GND.** This is *airplane mode control*, active low; Quectel
  says "if unused, keep it open". The pin is only honoured when
  `AT+QCFG="airplanecontrol"` is 1, and that setting is **saved in module NV**. Default
  is 0, so today the ground tie is inert — but any write of that key, or a module
  firmware whose default differs, puts the modem into permanent airplane mode
  (`csq=99`, `cereg=0`, `+CFUN: 4`) that no amount of STM32 reflashing can fix. The
  firmware now reads the key back at boot into `modem_live.airplane_ctl`. Prefer
  lifting pin 18.
- MAIN_RI (39), MAIN_DCD (38), GRFC_1/2 (76, 77), SLEEP_IND (1) → GND. These are module
  **outputs**; grounding them shorts the driver whenever the module asserts them. Not
  the current failure, but they should be left open on the next board revision.
- USIM1_DET (42) → GND: fine, hot-plug detect is off by default (`AT+QSIMDET`).
- USB_BOOT (75), AP_READY (19), WAKEUP_IN (96), MAIN_DTR (30): grounding is correct/safe.

## MCU ↔ Modem

| MCU | Net | Function |
|-----|-----|----------|
| PB10/PB11 | USART3 | 115200 baud, 1.8V level shifted |
| PC6 | MCU_PWRK | PWRKEY via 2N7002 inverter |
| PC7 | MCU_RST | RESET_N via 2N7002 inverter |

## CAN buses

| Peripheral | Bus | Type | ECUs |
|------------|-----|------|------|
| FDCAN3 | CAN1 (FD) | 1M/2M BRS | BMS, LV-BMS, IMU, GPS, VCU |
| FDCAN2 | CAN2 (Classic) | 1 Mbit | Inverter, damper, brake, dashboard |

## Shared +5V concern

Dashboard + telemetry may share the same vehicle +5V rail. LTE TX bursts can cause brownout. Mitigations:

- Firmware: 2 Hz telemetry rate, optional RF power reduction (`APP_MODEM_RF_PROFILE_ENABLE`)
- Hardware: adequate wire gauge, PSU/vehicle current ≥2 A, C4/C5 ceramic input bulk fitted

## CAN termination

On-board ~120Ω split termination on both buses. If card is a mid-bus tap (not physical end), DNP termination resistors to avoid triple termination.

## References

- Schematic: `telemetry26.pdf`, `itu-racing-elektronik-test/Schematics/Telemetry26 (2).pdf`
- EG915N HW Design v1.4 in repo root
