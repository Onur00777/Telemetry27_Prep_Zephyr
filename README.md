# Telemetry27 (Telemetry26 rewrite branch)

Clean-architecture rewrite of Formula Student telemetry firmware for STM32G474 + Quectel EG915N-EU.

## Architecture

```
App/
├── config/     app_config.h — UDP, APN, rate, RF profile flags
├── modem/      Quectel EG915N library (power, UART, AT, link, UDP)
├── pack/       373-byte telemetry protocol + MainBuffer
├── can/        CAN decode → MainBuffer
└── app/        FreeRTOS tasks (telemetry + modem)
Core/           CubeMX generated (unchanged pin/peripheral config)
```

## Default configuration

- UDP: `135.125.196.63:5010`
- APN: `internet`
- Rate: **10 Hz** (`APP_TELEMETRY_10HZ=1` → 100 ms). Set `APP_TELEMETRY_10HZ` to `0` for proven **2 Hz** fallback
- RF: Quectel default (~23 dBm); optional profile via `APP_MODEM_RF_PROFILE_ENABLE`
- **Vehicle mode:** `APP_TELEMETRY_BENCH_TEST=0` — MainBuffer only from CAN decode (no mock overwrite)
- CAN map: Excel **(7)** + VCU CM7 + Dashboard `dash_decode` (see `docs/CAN_MAPPING.md`)

## Build

Open in STM32CubeIDE. `App/` folder is added to include paths and source entries in `.cproject`.

## Docs

- [MODEM_BRINGUP.md](docs/MODEM_BRINGUP.md)
- [TELEMETRY_PROTOCOL.md](docs/TELEMETRY_PROTOCOL.md)
- [HARDWARE.md](docs/HARDWARE.md)
- [CAN_MAPPING.md](docs/CAN_MAPPING.md)

## Branch

`cursor/telemetry-rewrite-e290` — rewrite based on Final26 (`83abbac`) modem behavior.
