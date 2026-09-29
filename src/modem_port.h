#ifndef MODEM_PORT_H_
#define MODEM_PORT_H_

#include <stdint.h>

int modem_port_init(void);
int modem_port_rearm(void);
int modem_port_write(const uint8_t *data, uint16_t len);
void modem_port_rx_pump(void);

#endif
