# SIL Endurance Report

Software-in-the-loop Formula Student endurance on QEMU (`qemu_cortex_m3`). No car, no modem, no CAN controller: the same telemetry thread, packer, and Quectel state machine run against a deterministic lap model. Full console log is not committed; the excerpts below are from that run.

## Result

| Metric | Value |
| --- | --- |
| Race packets | 18 000 (36 laps × 50 s × 10 Hz) |
| UDP `QISEND` completions | 18 002 |
| Queue overflow | 0 |
| Active telemetry error | 0 |
| Verdict | `ENDURANCE TEST COMPLETED SUCCESSFULLY` |

The two extra `QISEND`s are the grid-to-race handoff, not a dropped or duplicated race step. The endurance counter itself stopped at 18 000. A stalled telemetry thread would have failed the run: each model step waits on the real 100 ms UDP queue.

## What the stream did

| | Start (lap 1, t = 0) | Finish (lap 36, t = 49.9 s) |
| --- | --- | --- |
| SoC | 0.980 | 0.275 |
| Cell temperature | 26.00 °C | 49.91 °C |
| Inverter temperature | 31 °C, climbing | 72 °C, settled |
| Pack voltage | 390.5 V at +181 A | 357.0 V at +38 A |

Cell temperature rose with I² and a long thermal time constant: 26.00 °C → 49.91 °C across 30 minutes of race time, inside the 48–52 °C finish band. The inverter is faster. It reached about 70 °C on the first straight and then sat at 70–73 °C, giving back about 1 °C in the hairpin.

Pack voltage is `Vocv − I × 0.08 Ω`, so the trace sags on throttle and rises on regen. Open-circuit voltage itself falls with SoC, from the 405 V class toward about 360 V.

| Moment | Speed | Current | Pack voltage |
| --- | --- | --- | --- |
| Lap 1, mid straight (t = 5.0 s) | 67 km/h | +198.6 A | 388.8 V (sag under peak current) |
| Lap 1, end of straight (t = 9.9 s) | 105 km/h | +181.6 A | 389.9 V |
| Lap 1, hairpin (t = 13.0 s) | 68 km/h | −40.4 A | 407.7 V (regen lift) |
| Lap 36, finish (t = 49.9 s) | 29 km/h | +37.9 A | 357.0 V (SoC 0.275) |

## Log excerpts

Bring-up, then the first straight. ANSI color codes are removed.

```text
[00:00:00.000,000] <inf> app: Telemetry27 Zephyr
[00:00:00.000,000] <inf> app: endurance sim: 30 dk, 18000 paket
[00:00:00.420,000] <inf> modem_fsm: durum INIT_AT
[00:00:01.870,000] <inf> modem_fsm: durum CHECK_NET
[00:00:03.540,000] <inf> modem_fsm: modem >> QIOPEN UDP 135.125.196.63:5010
[00:00:03.560,000] <inf> modem_fsm: durum IDLE
[00:00:03.600,000] <inf> sim_vehicle: endurance basladi
[00:00:03.610,000] <inf> sim_vehicle: tur=1 t=0.0s hiz=31 km/h V=390.5 I=+180.7A soc=0.980 Tcell=26.00 Tinv=31
[00:00:04.710,000] <inf> sim_vehicle: tur=1 t=1.0s hiz=33 km/h V=390.1 I=+185.8A soc=0.979 Tcell=26.03 Tinv=43
[00:00:09.510,000] <inf> sim_vehicle: tur=1 t=5.0s hiz=67 km/h V=388.8 I=+198.6A soc=0.976 Tcell=26.19 Tinv=66
[00:00:15.390,000] <inf> sim_vehicle: tur=1 t=9.9s hiz=105 km/h V=389.9 I=+181.6A soc=0.971 Tcell=26.38 Tinv=70
[00:00:19.110,000] <inf> sim_vehicle: tur=1 t=13.0s hiz=68 km/h V=407.7 I=-40.4A soc=0.972 Tcell=26.39 Tinv=69
```

Halfway and the last seconds of lap 36.

```text
[00:18:09.970,000] <inf> sim_vehicle: tur=18 t=0.0s hiz=30 km/h V=375.5 I=+102.6A soc=0.647 Tcell=37.56 Tinv=72
[00:37:36.130,000] <inf> sim_vehicle: tur=36 t=0.0s hiz=30 km/h V=353.2 I=+100.6A soc=0.294 Tcell=49.28 Tinv=72
[00:38:40.510,000] <inf> sim_vehicle: tur=36 t=49.7s hiz=30 km/h V=357.2 I=+35.0A soc=0.275 Tcell=49.91 Tinv=72
[00:38:40.640,000] <inf> sim_vehicle: tur=36 t=49.8s hiz=29 km/h V=357.1 I=+36.6A soc=0.275 Tcell=49.91 Tinv=72
[00:38:40.770,000] <inf> sim_vehicle: tur=36 t=49.9s hiz=29 km/h V=357.0 I=+37.9A soc=0.275 Tcell=49.91 Tinv=72
[00:38:41.200,000] <inf> sim_vehicle: ozet adim=18000 qisend=18002 overflow=0 err=0
[00:38:41.200,000] <inf> sim_vehicle: ENDURANCE TEST COMPLETED SUCCESSFULLY
```

Timestamps are guest race time. QEMU on a desktop finishes the 30 minutes faster than a wall clock.

## Reproduce

From the Zephyr workspace, with the virtualenv active:

```powershell
west build -b qemu_cortex_m3 -d C:\Embedded\Telemetry27_Prep_Zephyr\build-sim C:\Embedded\Telemetry27_Prep_Zephyr
west build -t run -d C:\Embedded\Telemetry27_Prep_Zephyr\build-sim
```

The model is `src/sim_vehicle.c`. Hardware builds for `nucleo_g474re` do not include it.
