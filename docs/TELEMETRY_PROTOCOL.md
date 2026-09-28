# Telemetry Protocol (frozen server spec)

## Packet format

- **373 bytes** total: 372 payload + 1 CRC8 at offset 372
- CRC-8: polynomial `0x07`, init `0x00`, over first 372 bytes
- Bit layout defined in `App/pack/telemetry_protocol.h` (`START_*` / `LENGTH_*`)

## Critical fields (bits 0-183)

| Field | Start bit | Length |
|-------|-----------|--------|
| VCU torque | 0 | 9 |
| Vehicle state | 9 | 2 |
| Throttle | 11 | 7 |
| BMS faults | 21 | 16 |
| BMS voltage/current/temp/SoC | 37-84 | various |
| Inverter temps/DC bus | 94-110 | various |
| Brake front, slip, speed | 120-136 | various |

## Cell data

- Voltages: bit 1202, 96 × 8-bit deltas (server adds min cell V)
- Temperatures: bit 1970, 96 × 8-bit encoded (°C + 40)

## Scaling examples

| Signal | Wire format |
|--------|-------------|
| BMS voltage | `f * 10` → i16 |
| BMS SoC | `f * 1000` → u16 |
| IMU accel/gyro | `f * 100` → i16 |
| Quaternions | `f * 16384` → i16 |
| Inverter temps | `i16 + 50` → u8 |

## Low-bandwidth mode

Disabled by default (`APP_TELEMETRY_LOW_BW_ENABLE=0`). Server expects full 373-byte packets.
