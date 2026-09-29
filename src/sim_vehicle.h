#ifndef SIM_VEHICLE_H_
#define SIM_VEHICLE_H_

#include <stdint.h>

/* Telemetri thread'i her basarili UDP kuyrugunda bir kez verir.
 * Endurance dongusu bu semaforla 100 ms pakete kilitlenir. */
void sim_pace_signal(void);

extern volatile uint8_t sim_race_armed;

#endif
