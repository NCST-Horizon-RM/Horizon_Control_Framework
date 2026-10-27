#ifndef HORIZON_USB_SERVICE_H
#define HORIZON_USB_SERVICE_H

#include "USB_Router.h"

/* Call after osKernelInitialize(), once. Uses a static TCB and stack. */
USB_Status_t USB_ServiceStart(void);

#endif
