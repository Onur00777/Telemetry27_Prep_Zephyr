#ifndef MODEM_H_
#define MODEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "modem_live.h"

void Modem_Init(UART_HandleTypeDef *huart);
void Modem_Poll(void);
bool Modem_IsReady(void);
bool Modem_RequestUdpSend(const uint8_t *buf, uint16_t len);
modem_state_t Modem_GetState(void);
const modem_live_t *Modem_GetLive(void);

void Modem_OnUartRxEvent(uint16_t size);
void Modem_OnUartRxIdleDone(void);
void Modem_OnUartError(void);

#endif /* MODEM_H_ */
