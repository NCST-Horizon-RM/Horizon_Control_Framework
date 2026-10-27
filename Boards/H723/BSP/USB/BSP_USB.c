#include "BSP_USB.h"
#include "USB_Port.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define USB_DEVICE hUsbDeviceHS
extern USBD_HandleTypeDef hUsbDeviceHS;

_Static_assert(USB_RX_PACKET_SIZE >= CDC_DATA_FS_OUT_PACKET_SIZE,
               "USB_RX_PACKET_SIZE must hold one native OUT packet");
_Static_assert(APP_RX_DATA_SIZE >= USB_RX_PACKET_SIZE,
               "Native CDC RX buffer is too small");

uint32_t USB_Port_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}

void USB_Port_Unlock(uint32_t key)
{
    __DMB();
    __set_PRIMASK(key);
}

uint32_t USB_Port_Now(void) { return HAL_GetTick(); }
bool USB_Port_InISR(void) { return __get_IPSR() != 0U; }

bool USB_Port_CanTransmit(USB_Port_t port)
{
    return port == USB_PORT_CDC1 && USB_DEVICE.dev_state == USBD_STATE_CONFIGURED &&
        USB_DEVICE.pClassDataCmsit[USB_DEVICE.classId] != NULL;
}

USB_Status_t USB_Port_Transmit(USB_Port_t port, uint8_t *data, uint16_t len)
{
    if (!USB_Port_CanTransmit(port)) return USB_ERR_NOT_READY;
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *)USB_DEVICE.pClassDataCmsit[USB_DEVICE.classId];
    if (cdc->TxState != 0U) return USB_ERR_BUSY;
    if (USBD_CDC_SetTxBuffer(&USB_DEVICE, data, len) != USBD_OK) return USB_ERR_NOT_READY;
    uint8_t result = USBD_CDC_TransmitPacket(&USB_DEVICE);
    return result == USBD_OK ? USB_OK :
        (result == USBD_BUSY ? USB_ERR_BUSY : USB_ERR_NOT_READY);
}

USB_Status_t USB_Port_Rearm(USB_Port_t port, uint8_t *data)
{
    if (data == NULL || !USB_Port_CanTransmit(port)) return USB_ERR_NOT_READY;
    if (USBD_CDC_SetRxBuffer(&USB_DEVICE, data) != USBD_OK) return USB_ERR_NOT_READY;
    return USBD_CDC_ReceivePacket(&USB_DEVICE) == USBD_OK ? USB_OK : USB_ERR_NOT_READY;
}

void BSP_USB_RearmRx(uint8_t *data)
{
    uint32_t key = USB_Port_Lock();
    USB_Status_t status = USB_Port_Rearm(USB_PORT_CDC1, data);
    USB_Port_Unlock(key);
    if (status != USB_OK) USB_RearmFailedISR(USB_PORT_CDC1, data);
}

uint8_t BSP_USB_TransmitLegacy(uint8_t *data, uint16_t len)
{
    USB_Status_t status = USB_SendLegacy(USB_PORT_CDC1, data, len);
    return status == USB_OK ? USBD_OK : (status == USB_ERR_BUSY ? USBD_BUSY : USBD_FAIL);
}

