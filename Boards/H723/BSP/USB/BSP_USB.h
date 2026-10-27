#ifndef HORIZON_BSP_USB_H
#define HORIZON_BSP_USB_H

#include "hal.h"
#include "USB_Router.h"

/* Native CDC compatibility entry and receive rearm with retry bookkeeping. */
uint8_t BSP_USB_TransmitLegacy(uint8_t *data, uint16_t len);
void BSP_USB_RearmRx(uint8_t *data);

#endif

