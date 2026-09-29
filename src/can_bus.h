#ifndef CAN_BUS_H_
#define CAN_BUS_H_

#include <stdint.h>

void can_rx_submit(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t len);

#endif
