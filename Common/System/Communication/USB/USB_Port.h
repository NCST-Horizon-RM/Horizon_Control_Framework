#ifndef HORIZON_USB_PORT_H
#define HORIZON_USB_PORT_H

#include "USB_Router.h"

/* Internal platform boundary. Critical sections also provide compiler barriers. */
uint32_t USB_Port_Lock(void);
void USB_Port_Unlock(uint32_t key);
uint32_t USB_Port_Now(void);
bool USB_Port_InISR(void);
bool USB_Port_CanTransmit(USB_Port_t port);
USB_Status_t USB_Port_Transmit(USB_Port_t port, uint8_t *data, uint16_t len);
USB_Status_t USB_Port_Rearm(USB_Port_t port, uint8_t *data);
void USB_ServiceNotify(void);

#endif
