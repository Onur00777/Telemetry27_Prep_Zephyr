#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* Network */
#define APP_UDP_HOST              "135.125.196.63"
#define APP_UDP_PORT              5010
#define APP_APN                   "internet"

/* Pack rate: 10 Hz for vehicle/site trial; flip APP_TELEMETRY_10HZ to 0
 * for the proven 2 Hz (500 ms) TX-safe bench/vehicle fallback. */
#define APP_TELEMETRY_10HZ            1
#if APP_TELEMETRY_10HZ
#define APP_TELEMETRY_PERIOD_MS       100u   /* 10 Hz — Final26 vehicle rate */
#else
#define APP_TELEMETRY_PERIOD_MS       500u   /* 2 Hz — shared-rail / weak PSU */
#endif

/* Modem boot delay before GSM task starts link bring-up */
#define APP_GSM_BOOT_DELAY_MS     3000u

/* Warm flash: 0 = Final26 (AT OK → leave modem on, no QPOWD). 1 = force soft power-down. */
#define APP_MODEM_WARM_FLASH_QPOWD    0

/* RF: off = Quectel default (~23 dBm). Set 1 + 1800 for shared-+5V bench. */
#define APP_MODEM_RF_PROFILE_ENABLE   0
#define APP_MODEM_SARCFG_MAX_POWER    1800u   /* 18.0 dBm when profile on */
#define APP_MODEM_SARCFG_BAND         20u

/* Feature flags */
#define APP_TELEMETRY_LOW_BW_ENABLE   0
/* 0 = vehicle: MainBuffer filled only from CAN decode (no mock overwrite). */
#define APP_TELEMETRY_BENCH_TEST      0
#define APP_MODEM_DEBUG_LOG           1

/* Watchdog / recovery */
#define APP_GSM_SILENCE_WATCHDOG      1
#define APP_GSM_SILENCE_RESET_MS      120000u

#define APP_IWDG_ENABLE_REFRESH       1

#endif /* APP_CONFIG_H_ */
