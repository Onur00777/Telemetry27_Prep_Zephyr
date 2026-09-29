#ifndef MODEM_H_
#define MODEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "modem_live.h"

void Modem_Init(void);
void Modem_Poll(void);
void Modem_SimulationStop(void);
bool Modem_IsReady(void);
bool Modem_RequestUdpSend(const uint8_t *buf, uint16_t len);
modem_state_t Modem_GetState(void);
const modem_live_t *Modem_GetLive(void);

extern volatile uint32_t last_modem_rx_tick;
extern volatile uint32_t modem_dma_restart_err_cnt;
extern uint8_t recovery_count;

#endif
