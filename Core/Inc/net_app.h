#ifndef NET_APP_H
#define NET_APP_H

#include <stdint.h>

void NET_Init(void);
void NET_Poll(void);
uint16_t NET_StatusColor(void);

#endif
