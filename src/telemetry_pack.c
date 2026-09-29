/*
 * telemetry_pack.c — bit-pack MainBuffer into 373B TransmitBuffer + CRC8
 */
#include "telemetry_protocol.h"
#include "telemetry_buffer.h"
#include "app_config.h"
#include "modem.h"
#include <stdint.h>
#include <string.h>

#define CELL_NUMBER 96
#define CELL_START 1202
#define TEMP_START 1970

static void Telemetry_ClearBuffer(void)
{
    memset(TransmitBuffer, 0, sizeof(TransmitBuffer));
}

static uint8_t Telemetry_CRC_8(uint8_t *data, uint16_t datalength)
{
    uint8_t crc = 0x00;
    uint8_t poly = 0x07;

    for (int i = 0; i < datalength; i++) {
        uint8_t b = *(data + i);
        crc ^= b;
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ poly);
            else
                crc <<= 1;
        }
    }
    return crc;
}

#define TELEM_U8_MASK(len)   ((uint8_t)((1u << (len)) - 1u))
#define TELEM_U16_MASK(len)  ((uint16_t)((1u << (len)) - 1u))
#define TELEM_U32_MASK(len)  ((uint32_t)((1u << (len)) - 1u))
#define TELEM_BIT_CAP        (TELEMETRY_BUFFER_LENGTH << 3)

#if (CELL_START + (CELL_NUMBER * 8)) > (TELEMETRY_BUFFER_LENGTH * 8)
#error "Cell voltage fields exceed TransmitBuffer bit capacity"
#endif

static inline int Telemetry_BitsFit(int start_index, int length)
{
    return (unsigned)(start_index + length) <= TELEM_BIT_CAP;
}

static void Telemetry_FillTheBuffer_u8_len(uint8_t *buffer, uint8_t value, int start_index, int length)
{
    if (length <= 0 || length > 8) return;
    if (!Telemetry_BitsFit(start_index, length)) return;

    value &= TELEM_U8_MASK(length);

    const int bit_off = start_index & 7;
    const int byte_i  = start_index >> 3;

    if (bit_off + length <= 8) {
        buffer[byte_i] |= (uint8_t)(value << (8 - length - bit_off));
    } else {
        const int rshift = length + bit_off - 8;
        buffer[byte_i]     |= (uint8_t)(value >> rshift);
        buffer[byte_i + 1] |= (uint8_t)(value << (16 - length - bit_off));
    }
}

static void Telemetry_FillTheBuffer_u16_len(uint8_t *buffer, uint16_t value, int start_index, int length)
{
    if (length <= 0 || length > 16) return;
    if (!Telemetry_BitsFit(start_index, length)) return;

    value &= (length == 16) ? 0xFFFFu : TELEM_U16_MASK(length);

    int shift = length & 7;
    if (shift == 0) shift = 8;

    const uint8_t temp2 = (uint8_t)(value & TELEM_U8_MASK(shift));
    const uint8_t temp1 = (uint8_t)(value >> shift);

    if (length <= 8) {
        Telemetry_FillTheBuffer_u8_len(buffer, temp2, start_index, length);
    } else {
        Telemetry_FillTheBuffer_u8_len(buffer, temp1, start_index, 8);
        Telemetry_FillTheBuffer_u8_len(buffer, temp2, start_index + 8, length - 8);
    }
}

static void Telemetry_FillTheBuffer_u32_len(uint8_t *buffer, uint32_t value, int start_index, int length)
{
    if (length <= 0 || length > 32) return;
    if (!Telemetry_BitsFit(start_index, length)) return;

    value &= (length == 32) ? 0xFFFFFFFFu : TELEM_U32_MASK(length);

    int shift = length & 15;
    if (shift == 0) shift = 16;

    const uint16_t temp2 = (uint16_t)(value & TELEM_U16_MASK(shift));
    const uint16_t temp1 = (uint16_t)(value >> shift);

    if (length <= 16) {
        Telemetry_FillTheBuffer_u16_len(buffer, temp2, start_index, length);
    } else {
        Telemetry_FillTheBuffer_u16_len(buffer, temp1, start_index, 16);
        Telemetry_FillTheBuffer_u16_len(buffer, temp2, start_index + 16, length - 16);
    }
}

static void Telemetry_FillTheBuffer_i8(uint8_t *buffer, int8_t value, int start_index)
{
    if (!Telemetry_BitsFit(start_index, 8)) return;

    const uint8_t temp = (uint8_t)value;
    const int bit_off  = start_index & 7;
    const int byte_i   = start_index >> 3;

    if (bit_off == 0) {
        buffer[byte_i] |= temp;
    } else {
        buffer[byte_i]     |= (uint8_t)(temp >> bit_off);
        buffer[byte_i + 1] |= (uint8_t)(temp << (8 - bit_off));
    }
}

static void Telemetry_FillTheBuffer_i16(uint8_t *buffer, int16_t value, int start_index)
{
    if (!Telemetry_BitsFit(start_index, 16)) return;

    if ((start_index & 7) == 0) {
        const int byte_i = start_index >> 3;
        buffer[byte_i]     |= (uint8_t)((value >> 8) & 0xFF);
        buffer[byte_i + 1] |= (uint8_t)(value & 0xFF);
        return;
    }

    const int8_t temp1 = (int8_t)((value >> 8) & 0xFF);
    const int8_t temp2 = (int8_t)(value & 0xFF);
    Telemetry_FillTheBuffer_i8(buffer, temp1, start_index);
    Telemetry_FillTheBuffer_i8(buffer, temp2, start_index + 8);
}

static void Telemetry_FillTheBuffer_i32(uint8_t *buffer, int32_t value, int start_index)
{
    if (!Telemetry_BitsFit(start_index, 32)) return;

    if ((start_index & 7) == 0) {
        const int byte_i = start_index >> 3;
        buffer[byte_i]     |= (uint8_t)((value >> 24) & 0xFF);
        buffer[byte_i + 1] |= (uint8_t)((value >> 16) & 0xFF);
        buffer[byte_i + 2] |= (uint8_t)((value >> 8) & 0xFF);
        buffer[byte_i + 3] |= (uint8_t)(value & 0xFF);
        return;
    }

    const int16_t temp1 = (int16_t)((value >> 16) & 0xFFFF);
    const int16_t temp2 = (int16_t)(value & 0xFFFF);
    Telemetry_FillTheBuffer_i16(buffer, temp1, start_index);
    Telemetry_FillTheBuffer_i16(buffer, temp2, start_index + 16);
}

void Telemetry_TransmitFrame(void)
{
    uint8_t temp_error = Telemetry_GetActiveError();
    Telemetry_ConsumeRuntimeError();

    Telemetry_ClearBuffer();

    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[VCU_TORQUE_NM_REQ_u16], START_VCU_TORQUE_NM_REQ, LENGTH_VCU_TORQUE_NM_REQ);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[VCU_VEHICLE_STATE_u8], START_VCU_VEHICLE_STATE, LENGTH_VCU_VEHICLE_STATE);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[THROTTLE_PERCENT_u8], START_THROTTLE_PERCENT, LENGTH_THROTTLE_PERCENT);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[VCU_DRIVE_MODE_u8], START_DRIVE_MODE, LENGTH_DRIVE_MODE);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[BMS_FAULTS_u16], START_BMS_FAULTS, LENGTH_BMS_FAULTS);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[BMS_TOTAL_VOLTAGE_f]*10), START_BMS_TOTAL_VOLTAGE, LENGTH_BMS_TOTAL_VOLTAGE);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[BMS_CURRENT_f]*10), START_BMS_CURRENT);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[BMS_MAX_CELL_TEMP_f]*100), START_BMS_MAX_CELL_TEMP);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[BMS_ESTIMATED_SoC_f]*1000), START_BMS_ESTIMATED_SoC, LENGTH_BMS_ESTIMATED_SoC);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.i16[INV_EMACHINE_TEMP_1_i16]+50), START_INV_EMACHINE_TEMP_1, LENGTH_INV_EMACHINE_TEMP_1);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.i16[INV_PWRSTG_TEMP_i16]+50), START_INV_PWRSTG_TEMP, LENGTH_INV_PWRSTG_TEMP);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[INV_DCBUS_VOLTAGE_u16], START_INV_DCBUS_VOLTAGE_V, LENGTH_INV_DCBUS_VOLTAGE_V);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[BRAKE_PRESSURE_FRONT_u8], START_BRAKE_PRESSURE_FRONT, LENGTH_BRAKE_PRESSURE_FRONT);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.f[SLIP_RATIO_f] * 100), START_SLIP_RATIO, LENGTH_SLIP_RATIO);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[VCU_VEHICLE_SPEED_u8], START_VCU_VEHICLE_SPEED, LENGTH_VCU_VEHICLE_SPEED);

    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)modem_dma_restart_err_cnt, START_DMA_RX_RESTART_ERR_CNT, LENGTH_DMA_RX_RESTART_ERR_CNT);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)recovery_count, START_RECOVERY_COUNTER, LENGTH_RECOVERY_COUNTER);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_ACCEL_X_f]*100), START_IMU_X_ACC);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_ACCEL_Y_f]*100), START_IMU_Y_ACC);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_ACCEL_Z_f]*100), START_IMU_Z_ACC);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_GYRO_X_f]*100), START_IMU_X_GYRO);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_GYRO_Y_f]*100), START_IMU_Y_GYRO);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_GYRO_Z_f]*100), START_IMU_Z_GYRO);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_MAG_X_f]*100), START_IMU_X_MAG);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_MAG_Y_f]*100), START_IMU_Y_MAG);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_MAG_Z_f]*100), START_IMU_Z_MAG);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[IMU_SPEED_u8], START_IMU_SPEED, LENGTH_IMU_SPEED);

    Telemetry_FillTheBuffer_i32(TransmitBuffer, (int32_t)MainBuffer.i32[GPS_LATITUDE_i32], START_GPS_LATITUDE);
    Telemetry_FillTheBuffer_i32(TransmitBuffer, (int32_t)MainBuffer.i32[GPS_LONGTITUDE_i32], START_GPS_LONGTITUDE);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[GPS_SPEED_u8], START_GPS_SPEED, LENGTH_GPS_SPEED);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)0, START_GPS_X_MAG, LENGTH_GPS_X_MAG);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)0, START_GPS_Y_MAG, LENGTH_GPS_Y_MAG);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)0, START_GPS_Z_MAG, LENGTH_GPS_Z_MAG);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[DAMPER_COMPRESSION_RR_f]*10), START_DAMPER_COMPRESSION_RR);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[DAMPER_COMPRESSION_RH_f]*10), START_DAMPER_COMPRESSION_RH);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[DAMPER_COMPRESSION_FR_f]*10), START_DAMPER_COMPRESSION_FR);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[DAMPER_COMPRESSION_FH_f]*10), START_DAMPER_COMPRESSION_FH);

    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_VOLTAGE_f]*100), START_LVBMS_VOLTAGE, LENGTH_LVBMS_VOLTAGE);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.f[LVBMS_CURRENT_f]*10), START_LVBMS_CURRENT, LENGTH_LVBMS_CURRENT);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[LVBMS_MAX_TEMP_f]*100), START_LVBMS_MAX_TEMP);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[LVBMS_MIN_TEMP_f]*100), START_LVBMS_MIN_TEMP);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_MAX_CELL_VOLTAGE_f]*100), START_LVBMS_MAX_CELL_VOLTAGE, LENGTH_LVBMS_MAX_CELL_VOLTAGE);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_MIN_CELL_VOLTAGE_f]*100), START_LVBMS_MIN_CELL_VOLTAGE, LENGTH_LVBMS_MIN_CELL_VOLTAGE);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_ESTIMATED_SoC_f]*1000), START_LVBMS_ESTIMATED_SoC, LENGTH_LVBMS_ESTIMATED_SoC);

    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[TIRE_SPEED_FR_u8], START_TIRE_SPEED_FR, LENGTH_TIRE_SPEED_FR);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[TIRE_SPEED_FL_u8], START_TIRE_SPEED_FL, LENGTH_TIRE_SPEED_FL);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[TIRE_SPEED_RR_u8], START_TIRE_SPEED_RR, LENGTH_TIRE_SPEED_RR);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[TIRE_SPEED_RL_u8], START_TIRE_SPEED_RL, LENGTH_TIRE_SPEED_RL);

    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[BRAKE_PRESSURE_REAR_u8], START_BRAKE_PRESSURE_REAR, LENGTH_BRAKE_PRESSURE_REAR);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[FLUID_PRESSURE_AFTER_RAD_u8], START_FLUID_PRESSURE_AFTER_RAD, LENGTH_FLUID_PRESSURE_AFTER_RAD);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[FLUID_PRESSURE_BEFORE_RAD_u8], START_FLUID_PRESSURE_BEFORE_RAD, LENGTH_FLUID_PRESSURE_BEFORE_RAD);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[FLUID_TEMP_AFTER_RAD_f]*100), START_FLUID_TEMP_AFTER_RAD);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[FLUID_TEMP_BEFORE_RAD_f]*100), START_FLUID_TEMP_BEFORE_RAD);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[POT1_U16], START_POT1, LENGTH_POT1);

    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[BMS_MIN_CELL_VOLTAGE_f]*100), START_BMS_MIN_CELL_VOLTAGE, LENGTH_BMS_MIN_CELL_VOLTAGE);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[BMS_MAX_CELL_VOLTAGE_f]*100), START_BMS_MAX_CELL_VOLTAGE, LENGTH_BMS_MAX_CELL_VOLTAGE);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[BMS_AVG_CELL_VOLTAGE_f]*100), START_BMS_AVG_CELL_VOLTAGE, LENGTH_BMS_AVG_CELL_VOLTAGE);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[BMS_MIN_CELL_TEMP_f]*100), START_BMS_MIN_CELL_TEMP);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[BMS_AVG_CELL_TEMP_f]*100), START_BMS_AVG_CELL_TEMP);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[BMS_MAX_SLAVE_TEMP_f]*100), START_BMS_MAX_SLAVE_TEMP);
    Telemetry_FillTheBuffer_u32_len(TransmitBuffer, (uint32_t)MainBuffer.u32[BMS_POWER_u32], START_BMS_POWER, LENGTH_BMS_POWER);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[POT2_U16], START_POT2, LENGTH_POT2);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[BMS_CONTRACTORS_u8], START_BMS_CONTRACTORS, LENGTH_BMS_CONTRACTORS);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[INV_CURRENT_Q_A_f]*10), START_INV_CURRENT_Q_A);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[INV_CURRENT_D_A_f]*10), START_INV_CURRENT_D_A);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.i16[INV_BOARD_1_TEMP_i16]+50), START_INV_BOARD_1_TEMP, LENGTH_INV_BOARD_1_TEMP);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.i16[INV_BOARD_2_TEMP_i16]+50), START_INV_BOARD_2_TEMP, LENGTH_INV_BOARD_2_TEMP);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)(MainBuffer.i16[INV_EMACHINE_TEMP_2_i16]+50), START_INV_EMACHINE_TEMP_2, LENGTH_INV_EMACHINE_TEMP_2);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.u32[INV_DEM2_u32] & 0xFFFFu), START_INV_DEM, LENGTH_INV_DEM);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)MainBuffer.i16[INV_ACBUS_POWER_i16], START_INV_ACBUS_POWER_W);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)MainBuffer.i16[INV_SETPOINT_APP_Q_i16], START_INV_SETPOINT_APP_Q);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)MainBuffer.i16[INV_SETPOINT_APP_D_i16], START_INV_SETPOINT_APP_D);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[INV_PWRSTG_BITSTATE_u16], START_INV_PWRSTG_BITSTATE, LENGTH_INV_PWRSTG_BITSTATE);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[INV_EMCTRL_FOC_BITSTATE_u8], START_INV_EMCTRL_FOC_BITSTATE, LENGTH_INV_EMCTRL_FOC_BITSTATE);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[INV_APP_STATE_APP_u8], START_INV_APP_STATE_APP, LENGTH_INV_APP_STATE_APP);
    Telemetry_FillTheBuffer_i32(TransmitBuffer, (int32_t)MainBuffer.i32[INV_EMACHINE_SPEED_ERPM_i32], START_INV_EMACHINE_SPEED_ERPM);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[INV_VOLT_MODULUS_PERMIL_u16], START_INV_VOLT_MODULUS_PERMIL, LENGTH_INV_VOLT_MODULUS_PERMIL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)MainBuffer.i16[INV_TORQUE_MAX_FEAS_NDM_i16], START_INV_TORQUE_MAX_FEAS_NDM);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)MainBuffer.i16[INV_TORQUE_EST_NM_i16], START_INV_TORQUE_EST_NM);

    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[VCU_APP_STATE_REQ_u8], START_VCU_APP_STATE_REQ, LENGTH_VCU_APP_STATE_REQ);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)MainBuffer.u8[BSPD_PLAUSIBILITY_u8], START_BSPD_PLAUSIBILITY, LENGTH_BSPD_PLAUSIBILITY);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)MainBuffer.u16[RTC_MILLISEC_u16], START_RTC_MILLISEC, LENGTH_RTC_MILLISEC);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[YAW_RATE_f] * 100), START_YAW_RATE);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[WHEEL_TRAVEL_FL_f] * 10), START_WHEEL_TRAVEL_FL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[WHEEL_TRAVEL_FR_f] * 10), START_WHEEL_TRAVEL_FR);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[WHEEL_TRAVEL_RL_f] * 10), START_WHEEL_TRAVEL_RL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[WHEEL_TRAVEL_RR_f] * 10), START_WHEEL_TRAVEL_RR);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[TIRE_TEMP_FL_f]), START_TIRE_TEMP_FL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[TIRE_TEMP_FR_f]), START_TIRE_TEMP_FR);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[TIRE_TEMP_RL_f]), START_TIRE_TEMP_RL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[TIRE_TEMP_RR_f]), START_TIRE_TEMP_RR);

    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)evt_overflow_cnt, START_EVT_OVERFLOW_CNT, LENGTH_EVT_OVERFLOW_CNT);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, (uint8_t)temp_error, START_PERIPHERAL_ERROR, LENGTH_PERIPHERAL_ERROR);

    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, 0x00, START_SECURITY_BIT_LOW_BANDWIDTH, LENGTH_SECURITY_BIT_LOW_BANDWIDTH);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, 0x00, START_SECURITY_BIT_NORMAL_BANDWIDTH, LENGTH_SECURITY_BIT_NORMAL_BANDWIDTH);

    {
        uint8_t *const myBuffer = TransmitBuffer;
        int bit = CELL_START;
        for (int i = 0; i < CELL_NUMBER; i++, bit += 8) {
            const uint8_t v = MainBuffer.cell_voltages[i];
            const int byte_index = bit >> 3;
            const int byte_offset = bit & 7;
            if (byte_offset == 0) {
                myBuffer[byte_index] |= v;
            } else {
                myBuffer[byte_index]     |= (uint8_t)(v >> byte_offset);
                myBuffer[byte_index + 1] |= (uint8_t)(v << (8 - byte_offset));
            }
        }
    }

    {
        uint8_t *const buf = TransmitBuffer;
        int bit = TEMP_START;
        for (int i = 0; i < CELL_NUMBER; i++, bit += 8) {
            const uint8_t t = MainBuffer.cell_temperatures[i];
            const int byte_index = bit >> 3;
            const int byte_offset = bit & 7;

            if (byte_offset == 0) {
                buf[byte_index] = t;
            } else {
                buf[byte_index]   &= ~(0xFF >> byte_offset);
                buf[byte_index]   |= (uint8_t)(t >> byte_offset);
                buf[byte_index+1] &= ~(0xFF << (8 - byte_offset));
                buf[byte_index+1] |= (uint8_t)(t << (8 - byte_offset));
            }
        }
    }

    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_CELL_REAL0_f]*100), START_LVBMS_CELL_REAL0, LENGTH_LVBMS_CELL_REAL);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_CELL_REAL1_f]*100), START_LVBMS_CELL_REAL1, LENGTH_LVBMS_CELL_REAL);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_CELL_REAL2_f]*100), START_LVBMS_CELL_REAL2, LENGTH_LVBMS_CELL_REAL);
    Telemetry_FillTheBuffer_u16_len(TransmitBuffer, (uint16_t)(MainBuffer.f[LVBMS_CELL_REAL3_f]*100), START_LVBMS_CELL_REAL3, LENGTH_LVBMS_CELL_REAL);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[LVBMS_CELL_MIN_NUMBER_u8], START_LVBMS_CELL_MIN_NUMBER, LENGTH_LVBMS_CELL_MIN_NUMBER);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[LVBMS_CELL_MAX_NUMBER_u8], START_LVBMS_CELL_MAX_NUMBER, LENGTH_LVBMS_CELL_MAX_NUMBER);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[LVBMS_FAULTBYTE_u8], START_LVBMS_FAULTBYTE, LENGTH_LVBMS_FAULTBYTE);

    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_EULER_YAW_f]*100), START_IMU_EULER_YAW);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_EULER_ROLL_f]*100), START_IMU_EULER_ROLL);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_EULER_PITCH_f]*100), START_IMU_EULER_PITCH);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[IMU_CALIB_SYS_u8], START_IMU_CALIB_SYS, LENGTH_IMU_CALIB_SYS);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[IMU_CALIB_GYRO_u8], START_IMU_CALIB_GYRO, LENGTH_IMU_CALIB_GYRO);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[IMU_CALIB_ACCEL_u8], START_IMU_CALIB_ACCEL, LENGTH_IMU_CALIB_ACCEL);
    Telemetry_FillTheBuffer_u8_len(TransmitBuffer, MainBuffer.u8[IMU_CALIB_MAG_u8], START_IMU_CALIB_MAG, LENGTH_IMU_CALIB_MAG);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_QUAT_W_f]*16384.0f), START_IMU_QUAT_W);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_QUAT_X_f]*16384.0f), START_IMU_QUAT_X);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_QUAT_Y_f]*16384.0f), START_IMU_QUAT_Y);
    Telemetry_FillTheBuffer_i16(TransmitBuffer, (int16_t)(MainBuffer.f[IMU_QUAT_Z_f]*16384.0f), START_IMU_QUAT_Z);

    TransmitBuffer[TELEMETRY_CRC_OFFSET] = Telemetry_CRC_8(TransmitBuffer, TELEMETRY_CRC_OFFSET);
}

#if APP_TELEMETRY_BENCH_TEST
void Telemetry_BenchFill(void)
{
    static uint32_t bench_tick = 0;
    bench_tick++;
    uint8_t  saw100 = (uint8_t)(bench_tick % 101);
    uint8_t  saw120 = (uint8_t)(bench_tick % 121);
    float    soc    = (float)(bench_tick % 101) / 100.0f;
    uint16_t rtc    = (uint16_t)((bench_tick % 10) * 100);

    MainBuffer.u16[VCU_TORQUE_NM_REQ_u16] = 245;
    MainBuffer.u8[VCU_VEHICLE_STATE_u8]   = 3;
    MainBuffer.u8[THROTTLE_PERCENT_u8]    = saw100;
    MainBuffer.u8[VCU_DRIVE_MODE_u8]      = 2;
    MainBuffer.u8[VCU_VEHICLE_SPEED_u8]   = saw120;
    MainBuffer.u8[VCU_APP_STATE_REQ_u8]   = 1;
    MainBuffer.u8[BSPD_PLAUSIBILITY_u8]   = 1;
    MainBuffer.u16[RTC_MILLISEC_u16]      = rtc;

    MainBuffer.f[BMS_TOTAL_VOLTAGE_f]     = 400.0f;
    MainBuffer.f[BMS_CURRENT_f]           = 123.4f;
    MainBuffer.f[BMS_MAX_CELL_TEMP_f]     = 42.50f;
    MainBuffer.f[BMS_MIN_CELL_TEMP_f]     = 38.00f;
    MainBuffer.f[BMS_AVG_CELL_TEMP_f]     = 40.00f;
    MainBuffer.f[BMS_MAX_SLAVE_TEMP_f]    = 45.00f;
    MainBuffer.f[BMS_MAX_CELL_VOLTAGE_f]  = 4.15f;
    MainBuffer.f[BMS_MIN_CELL_VOLTAGE_f]  = 3.90f;
    MainBuffer.f[BMS_AVG_CELL_VOLTAGE_f]  = 4.05f;
    MainBuffer.f[BMS_ESTIMATED_SoC_f]     = soc;
    MainBuffer.u32[BMS_POWER_u32]         = 50000u;
    MainBuffer.u16[BMS_FAULTS_u16]        = 0x0000;
    MainBuffer.u8[BMS_CONTRACTORS_u8]     = 0x07;
    for (int i = 0; i < 96; i++) MainBuffer.cell_voltages[i] = (uint8_t)(5 + (i % 15));

    MainBuffer.f[LVBMS_VOLTAGE_f]          = 24.00f;
    MainBuffer.f[LVBMS_CURRENT_f]          = 5.0f;
    MainBuffer.f[LVBMS_MAX_TEMP_f]         = 35.00f;
    MainBuffer.f[LVBMS_MIN_TEMP_f]         = 28.00f;
    MainBuffer.f[LVBMS_MAX_CELL_VOLTAGE_f] = 4.10f;
    MainBuffer.f[LVBMS_MIN_CELL_VOLTAGE_f] = 3.95f;
    MainBuffer.f[LVBMS_ESTIMATED_SoC_f]    = 0.90f;
    MainBuffer.f[LVBMS_CELL_REAL0_f]       = 4.00f;
    MainBuffer.f[LVBMS_CELL_REAL1_f]       = 4.01f;
    MainBuffer.f[LVBMS_CELL_REAL2_f]       = 3.99f;
    MainBuffer.f[LVBMS_CELL_REAL3_f]       = 4.02f;
    MainBuffer.u8[LVBMS_CELL_MIN_NUMBER_u8] = 2;
    MainBuffer.u8[LVBMS_CELL_MAX_NUMBER_u8] = 3;
    MainBuffer.u8[LVBMS_FAULTBYTE_u8]       = 0x00;

    MainBuffer.f[IMU_ACCEL_X_f] = 0.50f;
    MainBuffer.f[IMU_ACCEL_Y_f] = -0.30f;
    MainBuffer.f[IMU_ACCEL_Z_f] = 9.81f;
    MainBuffer.f[IMU_GYRO_X_f]  = 10.0f;
    MainBuffer.f[IMU_GYRO_Y_f]  = -5.0f;
    MainBuffer.f[IMU_GYRO_Z_f]  = 2.0f;
    MainBuffer.f[IMU_EULER_YAW_f]   = 45.0f;
    MainBuffer.f[IMU_EULER_ROLL_f]  = 1.5f;
    MainBuffer.f[IMU_EULER_PITCH_f] = -2.0f;
    MainBuffer.f[IMU_MAG_X_f]   = 25.0f;
    MainBuffer.f[IMU_MAG_Y_f]   = -12.0f;
    MainBuffer.f[IMU_MAG_Z_f]   = 40.0f;
    MainBuffer.u8[IMU_SPEED_u8] = 0;
    MainBuffer.u8[IMU_CALIB_SYS_u8]        = 3;
    MainBuffer.u8[IMU_CALIB_GYRO_u8]       = 3;
    MainBuffer.u8[IMU_CALIB_ACCEL_u8]      = 3;
    MainBuffer.u8[IMU_CALIB_MAG_u8]        = 2;
    MainBuffer.f[IMU_QUAT_W_f] = 1.0f;
    MainBuffer.f[IMU_QUAT_X_f] = 0.0f;
    MainBuffer.f[IMU_QUAT_Y_f] = 0.0f;
    MainBuffer.f[IMU_QUAT_Z_f] = 0.0f;
    MainBuffer.f[YAW_RATE_f]    = 12.5f;

    MainBuffer.i32[GPS_LATITUDE_i32]   = 410123456;
    MainBuffer.i32[GPS_LONGTITUDE_i32] = 290987654;
    MainBuffer.u8[GPS_SPEED_u8]        = saw100;

    MainBuffer.f[DAMPER_COMPRESSION_FH_f] = 5.0f;
    MainBuffer.f[DAMPER_COMPRESSION_FR_f] = 2.0f;
    MainBuffer.f[DAMPER_COMPRESSION_RH_f] = 4.5f;
    MainBuffer.f[DAMPER_COMPRESSION_RR_f] = 1.8f;
    MainBuffer.f[WHEEL_TRAVEL_FL_f] = 3.0f;
    MainBuffer.f[WHEEL_TRAVEL_FR_f] = 3.1f;
    MainBuffer.f[WHEEL_TRAVEL_RL_f] = 2.8f;
    MainBuffer.f[WHEEL_TRAVEL_RR_f] = 2.9f;

    MainBuffer.u8[TIRE_SPEED_FR_u8] = 50;
    MainBuffer.u8[TIRE_SPEED_FL_u8] = 51;
    MainBuffer.u8[TIRE_SPEED_RR_u8] = 52;
    MainBuffer.u8[TIRE_SPEED_RL_u8] = 53;
    MainBuffer.u8[BRAKE_PRESSURE_FRONT_u8]      = 30;
    MainBuffer.u8[BRAKE_PRESSURE_REAR_u8]       = 25;
    MainBuffer.u8[FLUID_PRESSURE_AFTER_RAD_u8]  = 15;
    MainBuffer.u8[FLUID_PRESSURE_BEFORE_RAD_u8] = 16;
    MainBuffer.f[FLUID_TEMP_AFTER_RAD_f]  = 60.00f;
    MainBuffer.f[FLUID_TEMP_BEFORE_RAD_f] = 55.00f;
    MainBuffer.u16[POT1_U16] = 1500;
    MainBuffer.u16[POT2_U16] = 1520;
    MainBuffer.f[SLIP_RATIO_f] = 0.15f;

    MainBuffer.i16[INV_TORQUE_EST_NM_i16]       = 245;
    MainBuffer.i32[INV_EMACHINE_SPEED_ERPM_i32] = 5000;
    MainBuffer.u16[INV_DCBUS_VOLTAGE_u16]       = 400;
    MainBuffer.i16[INV_PWRSTG_TEMP_i16]         = 55;
    MainBuffer.i16[INV_EMACHINE_TEMP_1_i16]     = 60;
    MainBuffer.i16[INV_EMACHINE_TEMP_2_i16]     = 65;
    MainBuffer.i16[INV_BOARD_1_TEMP_i16]        = 45;
    MainBuffer.i16[INV_BOARD_2_TEMP_i16]        = 40;
    MainBuffer.i16[INV_ACBUS_POWER_i16]         = 800;
    MainBuffer.f[INV_CURRENT_Q_A_f]             = 150.0f;
    MainBuffer.f[INV_CURRENT_D_A_f]             = -20.0f;
    MainBuffer.i16[INV_SETPOINT_APP_Q_i16]      = 100;
    MainBuffer.i16[INV_SETPOINT_APP_D_i16]      = -10;
    MainBuffer.i16[INV_TORQUE_MAX_FEAS_NDM_i16] = 300;
    MainBuffer.u16[INV_VOLT_MODULUS_PERMIL_u16] = 950;
    MainBuffer.u16[INV_PWRSTG_BITSTATE_u16]     = 0x0A5;
    MainBuffer.u8[INV_EMCTRL_FOC_BITSTATE_u8]   = 4;
    MainBuffer.u8[INV_APP_STATE_APP_u8]         = 5;
    MainBuffer.u32[INV_DEM1_u32] = 0x11223344u;
    MainBuffer.u32[INV_DEM2_u32] = 0x55667788u;

    MainBuffer.f[TIRE_TEMP_FL_f] = 80.0f;
    MainBuffer.f[TIRE_TEMP_FR_f] = 81.0f;
    MainBuffer.f[TIRE_TEMP_RL_f] = 82.0f;
    MainBuffer.f[TIRE_TEMP_RR_f] = 83.0f;
    for (int i = 0; i < 96; i++) {
        MainBuffer.cell_voltages[i] = (uint8_t)(5 + (i % 15));
        MainBuffer.cell_temperatures[i] = (uint8_t)(60 + ((i + bench_tick) % 15));
    }
}
#else
void Telemetry_BenchFill(void) {}
#endif /* APP_TELEMETRY_BENCH_TEST */
