#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include "stm32g4xx_hal.h"

void App_TasksStart(void);
void App_CanRxCallback(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *header, uint8_t *data);

#endif /* APP_TASKS_H_ */
