# USB CDC 接入与使用

F407、H723 已接入同一套 CDC 路由。USB 初始化时自动扫描 `USB_RX_NODE`，RTOS 初始化时自动启动静态分配的 `USB_CDC` 处理任务。应用不需要修改 CubeMX 接收回调，也不需要逐个 App 添加初始化调用。

**接收数据结构体由应用自己定义。框架没有固定的 `Host_USB_Data_t`，也不会直接把 USB 字节强转为业务结构体。**

## 1. 按现有 UART/CAN 风格注册

配置方式与 `UART_RX_NODE(...)` 一样：接收参数直接填写在注册宏中，由宏自动生成静态配置和注册节点。应用不再单独定义或导出 `USB_RxConfig_t` 配置对象。

将数据类型与解析函数放在应用自己的协议模块。例如协议头文件：

```c
#include "BSP_USB.h"

typedef struct {
    uint8_t command;
    uint16_t target;
} My_USB_Data_t;

void My_USB_Resolve(uint8_t *data, void *device_ptr, uint16_t size);
```

协议源文件中的最小示例（3 字节：命令字、目标值低字节、目标值高字节）：

```c
void My_USB_Resolve(uint8_t *data, void *device_ptr, uint16_t size)
{
    if (size != 3) return;
    My_USB_Data_t *device = device_ptr;
    device->command = data[0];
    device->target = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
}
```

`Robot_Config.c` 只定义实例并注册：

```c
#include "BSP_USB.h"
#include "My_USB_Protocol.h"  // 你自己的协议头文件

My_USB_Data_t my_usb = {0};
static uint8_t MY_USB_RX_DATA[3];

USB_RX_NODE(USB_PORT_CDC1, USB_RX_FIXED, 3, 64, 100, 500,
            NULL, MY_USB_RX_DATA, sizeof(MY_USB_RX_DATA),
            &my_usb, My_USB_Resolve);
```

带应用接收缓存的十一个参数按顺序为：

```c
USB_RX_NODE(端口, 接收模式, 帧长, 缓存容量, 半帧超时ms, 离线超时ms,
            校验函数, 接收数组, 接收数组容量, 设备指针, 解析函数);
```

校验函数可传 `NULL`；超时传 `0` 禁用相应计时；缓存容量传 `0` 使用编译期默认值。RAW 和内置变长帧模式的帧长传 `0`。CDC 不需要 UART 的波特率、校验位、停止位、UART DMA 句柄等参数。

固定模式每拼出一帧且通过校验后，框架自动将完整原始帧复制到 `MY_USB_RX_DATA`，再调用 `My_USB_Resolve`；回调的 `data` 就是这个数组的地址，`device_ptr` 仍是 `&my_usb`。接收数组与解析结果结构体各有用途，框架不会自动解码业务字段。CDC 的这次复制在 USB 任务内进行，不是 UART DMA 直接写数组。

原九参数写法继续使用内部回调缓存，无需修改：

```c
USB_RX_NODE(USB_PORT_CDC1, USB_RX_FIXED, 3, 64, 100, 500,
            NULL, &my_usb, My_USB_Resolve);
```

宏生成的配置在内部仍使用 `USB_RxConfig_t`，但业务文件无需创建这个对象。旧的四参数 `USB_RX_NODE(port, &config, device, callback)` 写法不支持；有额外字段时使用 `USB_RX_NODE_EX`。每个节点单独占一条宏调用，不要在同一物理行注册两个节点。

当前 `F407/8_dart_rack` 已在 `Robot_Config.c` 注册 12 字节的 `VISION_RX_DATA`。其现有 `Vision_USB_Rx_Callback` 还未填入坐标解码；接收数组自动填充与业务坐标解析是两个步骤。

实例需要被其他文件访问时，在应用自己的头文件中添加 `extern My_USB_Data_t my_usb;`。多字段数据被其他任务读取时，应由应用使用短临界区、互斥或快照接口保证一致性；`volatile` 不提供这些保证。

上述 3 字节示例只说明接口。固定长度无帧头/校验时，丢字节后不能自动知道正确起点。正式协议建议使用帧头、校验或变长帧模式。

## 2. 接收模式与长度

回调参数顺序与 UART 一致：`uint8_t *data, void *device_ptr, uint16_t size`。它们在 USB 任务中执行，不在 USB 中断中执行。使用内部缓存时，`data` 只在本次回调期间有效，需要留存时自行复制。指定应用数组时，数组必须在节点使用期间一直有效，通常定义为文件作用域 `static` 数组；下一次回调前会被新数据覆盖。只允许访问本次的前 `size` 字节，不自动清零剩余空间或追加 `\0`。回调应简短，不执行长时间控制动作或等待。

外部数组在完整帧/RAW 批次交付前自动填充；半帧、校验失败、半帧超时和断线不会清除或覆盖数组中的上一次结果。断线后的旧数据不能当作新帧，需结合回调通知或 `USB_GetRxState` 判断有效性。外部数组由 USB 任务写入，其他任务并发读取多字节数据时需由应用提供同步或快照。

| 模式 | 回调数据 |
|---|---|
| `USB_RX_RAW` | 一个原生 CDC 接收批次的独立可写副本；不代表电脑的一次发送或完整帧 |
| `USB_RX_FIXED` | 累计 `frame_len` 字节，经过可选的 `check_frame` 校验 |
| `USB_RX_DELIMITER` | 收到分隔符后提供一行，可选择是否包含分隔符 |
| `USB_RX_FRAMED` | 下文定义的 A5 5A 变长帧，包含帧头与 CRC |
| `USB_RX_CUSTOM` | 由 `custom_parser` 确认的完整帧 |

普通模式直接填写注册宏。分隔符、自定义解帧、事件回调等高级配置使用 `USB_RX_NODE_EX(port, device, callback, .字段 = 值, ...)`，同样在注册处填写，无需单独声明配置对象。相关字段约定：

- `cache_size = 0`：使用 `USB_MAX_FRAME_SIZE`；指定值不能超过此编译期上限。
- `max_frame_len = 0`：使用节点缓存容量；否则不能超过缓存容量。
- `rx_buf = NULL, rx_buf_size = 0`：使用内部回调缓存；两字段也可在 `USB_RX_NODE_EX` 中直接填写。
- 指定 `rx_buf` 时，`rx_buf_size` 必须填写数组实际容量。固定模式至少容纳 `frame_len` 字节，RAW 至少容纳 `USB_RX_PACKET_SIZE`，分隔符/内置变长/自定义模式至少容纳 `max_frame_len`（为 0 时使用缓存容量）。不满足时初始化返回 `USB_ERR_CONFIG`，不会截断数据交给回调。每个节点应使用独立数组。
- `cache_size` 仍是框架内部拼帧缓存的容量限制，与应用接收数组容量独立；例如拼帧缓存 64、应用数组 12 可以用于 12 字节固定帧。
- 固定模式的 `frame_len` 必须大于 0 且不超过最大帧长。
- RAW 模式缓存至少能保存 `USB_RX_PACKET_SIZE` 字节，忽略拼帧和 `check_frame` 设置。
- `partial_timeout_ms` 是半帧接收间隔超时，0 禁用；先按队列中的实际接收时间处理数据，再判断墙上时间超时。
- `offline_timeout_ms` 从最近有效帧的接收时间计算，0 表示有效帧不会按时间过期；断线/会话变化仍会清除在线状态。
- `check_frame` 返回 false 时，固定长度/变长/自定义帧丢弃一个字节重新同步；分隔符模式丢弃整行。
- 分隔符长度为 1～8 字节。超长行会一直丢弃到下一分隔符，避免把剩余后缀当成正常新行。
- 多个节点收到相同字节流，各自拥有拼帧缓存。一个 `(port, device_ptr)` 只能注册一次；实例、自定义解析上下文应具有静态生命周期，配置对象由宏自动管理。

19 字节固定帧示例：

```c
// My_Frame_Check 类型：bool(const uint8_t *, uint16_t)，需在此处可见。
USB_RX_NODE(USB_PORT_CDC1, USB_RX_FIXED, 19, 64, 100, 500,
            My_Frame_Check, &my_usb, My_USB_Resolve);
```

`7 + 12` 字节触发一次回调，38 字节触发两次，40 字节触发两次并保留 2 字节。校验函数应按真实协议检查，不能凭空给既有协议添加 CRC。

文本示例：

```c
static const uint8_t newline[] = {'\r', '\n'};
USB_RX_NODE_EX(USB_PORT_CDC1, &text_device, Text_Resolve,
    .mode = USB_RX_DELIMITER,
    .cache_size = 128,
    .delimiter = newline,
    .delimiter_len = sizeof(newline),
    .include_delimiter = false,
    .partial_timeout_ms = 1000,
);
```

回调收到的是有长度的字节数组，不自动补 `\0`，可使用长度解析或复制到多一个字节的文本数组后补零。

其他模式示例：

```c
// 原始接收批次：不拼帧，按实际长度回调。
USB_RX_NODE(USB_PORT_CDC1, USB_RX_RAW, 0, 64, 0, 500,
            NULL, &raw_device, Raw_Resolve);

// 内置变长帧：缓存/最大帧长为 512，帧长由协议头读取。
USB_RX_NODE(USB_PORT_CDC1, USB_RX_FRAMED, 0, 512, 100, 500,
            NULL, &framed_device, Framed_Resolve);

// 自定义协议：仅填写需要覆盖的字段，其余字段自动置零。
USB_RX_NODE_EX(USB_PORT_CDC1, &custom_device, Custom_Resolve,
    .mode = USB_RX_CUSTOM,
    .cache_size = 128,
    .max_frame_len = 64,
    .partial_timeout_ms = 100,
    .offline_timeout_ms = 500,
    .custom_parser = My_Parser,
    .parser_reset = My_Parser_Reset,
    .parser_context = &my_parser_state,
    .event_callback = My_Event_Callback
);
```

自定义模式：`custom_parser(data, available, context)` 返回 `USB_ParseResult_t`。

- `USB_PARSE_WAIT`：其余字段必须为 0，继续等待。
- `USB_PARSE_DROP`：`consumed` 必须在 1～available 之间。
- `USB_PARSE_FRAME`：`consumed` 在 1～available 之间，`frame_offset + frame_len <= consumed`，帧长不能超过配置上限。

解帧器只负责查找/验证帧，业务回调再解释数据。`parser_reset` 用于清除应用提供的解析状态，在会话改变、序号不连续或半帧超时时由 USB 任务调用。错误的自定义返回值会被统计并丢弃一个字节，不允许死循环或越界。

## 3. 发送与旧接口

```c
uint8_t reply[] = {0xAA, 0x01, 0x55};
USB_Status_t status = USB_SendCopy(USB_PORT_CDC1, reply, sizeof(reply));
```

`USB_OK` 表示消息已复制进队列，此后可以复用 `reply`；不表示电脑应用已收到或解析。接口不阻塞，完整入队或整体失败。用于主循环或任务上下文，ISR 中调用返回 `USB_ERR_BUSY`，应通知任务发送。其他返回包括 `USB_ERR_NOT_READY`、`USB_ERR_FULL`、`USB_ERR_SIZE`、`USB_ERR_ARG`。

需要完成通知时：

```c
uint32_t ticket;
USB_SendCopyEx(USB_PORT_CDC1, reply, sizeof(reply), 1000, &ticket);
```

`timeout_ms` 包括排队等待时间，0 禁用。事件中的 `ticket` 标识该条消息。消息的发送顺序由成功预留队列槽位的顺序决定。

- `USB_EVENT_TX_DONE`：原生 USB 传输完成，包括库所需的 ZLP 阶段；不等于应用层 ACK。
- `USB_EVENT_TX_TIMEOUT`：消息超时。若底层仍在传输，缓冲区保持占用，直到完成或端点停止。不会直接回收正在被硬件使用的内存，也不会自动重置整个 USB。
- `USB_EVENT_TX_CANCELLED`：会话结束或排队消息失效。

`CDC_Transmit_FS/HS` 的原签名保留，并与队列共用发送仲裁。旧接口仍为直接发送：`USBD_OK` 后，调用者必须保持原缓冲区不变直到原生发送完成；忙时返回 `USBD_BUSY`，未配置时返回 `USBD_FAIL`。同一协议应统一使用队列，以得到顺序保证；旧直接发送不属于队列的全局 FIFO。

VOFA 的 USB 分支已经使用复制队列，编码格式和 UART 分支保持原有行为。VOFA 的编码函数仍使用其原有静态编码缓冲区，应由一个任务调用或由应用串行化；发送队列自身支持多个生产者。

## 4. 状态、事件和流控

```c
USB_LinkState_t link;
USB_Stats_t stats;
USB_RxState_t rx;

USB_GetLinkState(USB_PORT_CDC1, &link);
USB_GetStats(USB_PORT_CDC1, &stats);
USB_GetRxState(USB_PORT_CDC1, &my_usb, &rx);
```

`link.ready` 表示当前可提交发送；它综合设备配置状态、挂起和可选 DTR 门控，不能代表电脑业务在线。

只有收到过当前会话的有效帧，`rx.online` 才可能为 true。RAW 模式收到字节本身不算有效帧：业务回调实际完成协议校验后，在该回调内调用 `USB_RxMarkValid(USB_PORT_CDC1, device_ptr)`。

在 `USB_RX_NODE_EX` 中填写 `.event_callback = My_Event_Callback` 可接收本节点事件及所在端口的连接/控制/发送事件。没有接收节点、只需要发送事件时，可注册：

```c
USB_SetPortEventCallback(USB_PORT_CDC1, My_Event_Callback, &my_context);
```

事件回调签名为 `void(void *context, const USB_Event_t *event)`，在 USB 任务执行；事件指针只在回调内有效。端口回调接收端口级事件。状态变化会更新当前状态，即使有限事件队列已满；丢失的通知记入 `events_lost`，统计量和状态查询仍可用于诊断。

端口配置可在初始化后设置，示例：

```c
USB_PortConfig_t options = {
    .overflow_policy = USB_OVERFLOW_PAUSE,
    .require_dtr = false,
    .tx_timeout_ms = 1000,
    .rx_pause_timeout_ms = 100,
};
USB_ConfigurePort(USB_PORT_CDC1, &options);
```

- 默认 `USB_OVERFLOW_DROP_NEW`：队列满时丢弃新接收批次并报告；后续批次序号跳变会使各节点重新同步。
- `USB_OVERFLOW_PAUSE`：保留当前原生缓冲区，暂停 OUT 接收。任务腾出空间后先复制该批数据，再恢复接收。暂停超过配置时限时丢弃该批数据并尝试恢复；0 表示禁用暂停时限。
- DTR 门控默认关闭。关闭串口、设置波特率不会自动重启 MCU。
- Line Coding 的七字节 SET/GET、DTR/RTS 和 SEND_BREAK 通知均已接入。波特率只保存为 CDC 配置信息，不改变 USB 总线速率。
- 两块板当前关闭 VBUS 检测，不能保证拔线瞬间收到断开事件；应结合挂起、USB 状态、有效业务心跳超时判断在线。

复位/重新配置会生成新的会话编号。旧会话的 RX 半帧、待发送消息不会拼入新会话。USB/业务超时使用同一个 HAL 毫秒时基，采用无符号差值处理 tick 回绕；配置超时必须小于 `2^31` 毫秒。

## 5. 内置变长协议，可选使用

`USB_RX_FRAMED` 使用以下字节布局，回调得到含头和 CRC 的整帧：

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 2 | A5 5A |
| 2 | 1 | 版本，由业务模块决定是否接受 |
| 3 | 1 | 命令字 |
| 4 | 2 | 负载长度，小端，不含头和 CRC |
| 6 | 2 | 序号，小端 |
| 8 | N | 负载 |
| 8+N | 2 | CRC16，小端 |

总长 `N + 10`。CRC 覆盖偏移 2 到负载末尾；poly `0x1021`、init `0xFFFF`、无反射、xorout `0`。`"123456789"` 的校验结果为 `0x29B1`。

```c
#include "USB_Framer.h"
uint8_t frame[USB_MAX_FRAME_SIZE];
uint16_t size;
if (USB_BuildFrame(1, command, sequence, payload, payload_size,
                   frame, sizeof(frame), &size) == USB_OK) {
    USB_SendCopy(USB_PORT_CDC1, frame, size);
}
```

电脑端等价编码（Python 标准库）：

```python
import binascii
import struct

frame = struct.pack('<2sBBHH', b'\xA5\x5A', 1, command, len(payload), sequence) + payload
frame += struct.pack('<H', binascii.crc_hqx(frame[2:], 0xFFFF))
```

两端都要按字节流拼帧。ACK、重发和重复命令去重属于业务协议，框架不自动重发控制命令。

## 6. 容量与调度

编译期默认上限在 `USB_Router.h`：4 个节点、512 字节单帧、每节点最多 512 字节缓存、16 个 64 字节 RX 槽、4 个 512 字节 TX 槽。`USB_Service.c` 默认静态分配 1024 字节任务栈，任务优先级 Normal，每轮处理约 512 接收字节并至少每个 RTOS tick 检查一次状态。预算以完整 USB 接收批次处理，最多超出不足一个批次。

这些宏如需改变，应在对应板卡 CMake 中同时给可执行目标和 Common 目标设置相同的编译定义，不能只在某个 `.c` 中局部 `#define`。三个队列深度必须是 2 的幂，以保证计数器回绕后的索引一致。实际 RAM/栈余量需看 `.map` 和板上栈高水位。当前 F407、H723 均为全速、USB DMA 关闭；未来改成真正高速或 DMA 时，需要重新配置包容量、内存区域和 Cache 一致性。

裸机可直接循环调用 `USB_Process(HAL_GetTick(), budget)`，不要同时启动 USB 服务任务；全系统只能有一个消费者。`USB_Router_InitNodes()` 是不使用链接器段时的显式静态注册入口。注册只允许初始化一次。

