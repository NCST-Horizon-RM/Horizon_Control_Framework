# 板级支持包(BSP)扩展说明

本文档说明如何在 `Horizon_Control_Framework` 中新增一块目标板卡(MCU 平台)。框架采用「单仓库、多板卡、共享上层」结构:上层逻辑统一位于 `Common/`,板级实现独立于 `Boards/<board>/`。新增板卡的实质工作是**实现一套与既有板卡签名一致的中性接口,并编写一份板级构建脚本**。

## 1. 板卡目录结构

每块板卡目录自包含,不与其它板卡共享任何编译单元:

```
Boards/<board>/
├── CMakeLists.txt              # 板级构建配置(CPU 参数、源收集、编译/链接、子目录)
├── Core/                       # CubeMX 生成:HAL 初始化、FreeRTOS、main.c、中断、启动文件
├── Drivers/                    # ST HAL 与 CMSIS(芯片相关)
├── Middlewares/                # FreeRTOS、CMSIS-DSP、USB 设备库
├── USB_DEVICE/                 # USB CDC 应用层
├── BSP/                        # 硬件抽象层(见第 2 节)
│   ├── CAN/                    # CAN 总线抽象(bxCAN 或 FDCAN)
│   ├── SPI/                    # SPI 抽象
│   ├── TIM/                    # 定时器/PWM 抽象
│   ├── UART/                   # 串口抽象
│   ├── DWT/                    # 周期计数器计时
│   └── Indicator/              # 指示灯(LED/WS2812)、蜂鸣器、System_Indicator
├── App/                        # 机器人型号(见第 3、4 节)
├── startup_*.s                 # 启动文件
├── *.ld                        # 链接脚本
├── *.ioc / .mxproject          # CubeMX 工程文件
└── cmake/
    ├── gcc-arm-none-eabi.cmake # 工具链描述(编译器、链接脚本路径)
    └── stm32cubemx/            # CubeMX 生成的 HAL/RTOS/USB 编译规则
```

参考实现:`Boards/F407`(bxCAN)与 `Boards/H723`(FDCAN)。新增板卡时,应选择 CAN 外设类型相同的一块作为模板复制。

## 2. 板级接口契约

`Common/` 仅通过以下头文件与板级交互。新增板卡必须**提供同名、同签名的接口**,否则上层无法编译。头文件本身即契约,建议自模板板卡复制后仅修改实现(`.c`)与芯片相关的类型定义。

| 头文件 | 必须提供的符号 |
|---|---|
| `BSP_CAN.h` | `hcan_t`、`CAN_Send_Msg`、`CAN_Config(hcan_t *, uint32_t fifo)`、`CAN_GetTxFreeLevel`、`CAN_RX_NODE` 宏、`BSP_CAN_Auto_Init`、`CAN_App_Frame_Dispatch`、`BSP_CAN_Register_Slot` |
| `BSP_UART.h` | `UART_RX_NODE` 宏、`BSP_UART_Register_Slot`、`Auto_UART_Router_Init`、`UART_ReceiveToIdle_DMA` |
| `BSP_SPI.h` | `BSP_SPI_RegisterIRQCallback` |
| `BSP_TIM.h` | `BSP_PWM_t` 及 PWM 控制函数 |
| `BSP_DWT.h` | `DWT_Init`、`DWT_GetDeltaT`、`DWT_SysTimeUpdate`、`DWT_Profile_*` |
| `Indicator/` | 指示灯(LED 或 WS2812 二选一)、`Buzzer`、`System_Indicator.c/h` |

### 关键约定

1. `hcan_t` 为 CAN 句柄的中性别名,在 `BSP_CAN.h` 内以 `typedef` 映射到芯片实际的 CAN/FDCAN 句柄类型(如 `typedef FDCAN_HandleTypeDef hcan_t;`)。
2. `hcan_t` 的 `typedef` 必须位于头文件**顶部**(`#include` 之后、任何使用该类型的声明之前),否则编译报 `unknown type name`。
3. 引用 `hcan_t` 的头文件必须自行 `#include "BSP_CAN.h"`,不得依赖其它头文件传递。
4. USB CDC 发送函数名因芯片而异(全速 `CDC_Transmit_FS`,高速 `CDC_Transmit_HS`),统一经 `Common/Utils/Inc/hal.h` 中的 `CDC_Transmit` 宏屏蔽差异。

## 3. 数据所有者约定

设备数据(遥控、裁判、图传、电容等)不定义在 `Common/`,而是由每块板的 `Robot_Config.c` 以 `static` 实例私有持有,再经发布/订阅对外提供:

- `Robot_Config.c` 定义 `static` 数据实例与接收缓冲,并实例化发布订阅槽位表 `g_topics[]`(`PubSub.h` 中的 `TOPIC_*` 枚举集中命名,topic 拼错即编译错误)。
- 设备解析函数(`Common/` 中的纯函数)通过 `device_ptr` 写入该实例,再由 `Robot_Config.c` 里的包装回调 `PubSub_Publish` 发布到总线。
- 上层模块一律通过 `PubSub_Read(&g_topics[TOPIC_XXX], ...)` 读取最新快照,无任何头文件暴露实例。

新增板卡时,须按自身接线在 `Robot_Config.c` 中声明所需数据实例并注册,而非沿用旧板卡的头文件声明。

## 4. 新增板卡流程

### 4.1 生成芯片骨架

使用 STM32CubeMX 生成工程,输出 `Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/`、启动文件与链接脚本。外设配置须与机器人 App 的实际需求一致(CAN、UART、SPI、TIM 的数量与引脚)。

### 4.2 建立板卡目录

将 CubeMX 输出置于 `Boards/<board>/` 下,并建立 `BSP/`、`App/` 与 `CMakeLists.txt`。

### 4.3 实现 BSP

以 CAN 外设类型相同的既有板卡为模板:

- 复制模板的 `BSP/` 目录,保留全部 `.h`,重写 `.c` 内实现,将 HAL API 替换为芯片对应版本。
- 修改 `BSP_CAN.h` 中的 `typedef` 与芯片相关包含。
- `Indicator/` 依据芯片实际指示灯硬件选择 LED 或 WS2812,连同 `System_Indicator.c/h` 一并实现。

### 4.4 配置链接脚本

CubeMX 生成的链接脚本不包含自定义注册段。须在 `.rodata` 之后追加 `.custom_registry` 段,否则声明式设备注册(`CAN_RX_NODE`、`UART_RX_NODE`、`OFFLINE_NODE`)无法生效:

```ld
.custom_registry :
{
  . = ALIGN(4);
  PROVIDE(__start_UART_Reg_Sec = .);    KEEP(*(UART_Reg_Sec))    PROVIDE(__stop_UART_Reg_Sec = .);
  PROVIDE(__start_CAN_Reg_Sec = .);     KEEP(*(CAN_Reg_Sec))     PROVIDE(__stop_CAN_Reg_Sec = .);
  PROVIDE(__start_Offline_Reg_Sec = .); KEEP(*(Offline_Reg_Sec)) PROVIDE(__stop_Offline_Reg_Sec = .);
  . = ALIGN(4);
} >FLASH
```

`KEEP` 用于防止 `--gc-sections` 回收未被直接引用的注册段;缺失将导致离线检测静默失效。

### 4.5 扩展 hal.h

在 `Common/Utils/Inc/hal.h` 中追加芯片分支,映射到对应 HAL 头,并在使用 USB CDC 时补充 `CDC_Transmit` 宏:

```c
#elif defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#define CDC_Transmit CDC_Transmit_FS   /* 若启用 USB CDC */
```

### 4.6 配置 App

- `Robot_Config.c`:声明所需设备数据 `static` 实例与接收缓冲;将 `CAN_RX_NODE`、`UART_RX_NODE`、`OFFLINE_NODE` 中的实例名与句柄替换为芯片实际符号;实例化 `g_topics[]` 槽位表,并为各设备添加"解析 + `PubSub_Publish`"的包装回调。
- `System_Init.c`:实现外设初始化、电源 GPIO 控制、`DWT_Init(主频MHz)`、`CAN_Config`、`Auto_UART_Router_Init`、指示灯与传感器初始化,并在订阅方之前调用 `Robot_Config_Init()`。
- `Core/Src/main.c`:保留 `extern void System_Init(void);` 与 `extern void MY_TIM_PeriodElapsedCallback(...)`,并在外设初始化后调用 `System_Init()`。

## 5. 构建系统接入

### 5.1 板级 CMakeLists.txt

新建 `Boards/<board>/CMakeLists.txt`,以既有板卡为模板,设置 CPU 参数、芯片宏、源文件收集、编译/链接选项、子目录与后处理:

```cmake
set(CPU_FLAGS      -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(BOARD_CHIP_DEF STM32G431xx)
set(BOARD_MATH_DEF ARM_MATH_CM4)
# ... 源收集、add_executable、include、compile/link options、
#     add_subdirectory(cmake/stm32cubemx)、add_subdirectory(App/${ACTIVE_APP}) ...
```

顶层 `CMakeLists.txt` 通过 `add_subdirectory(Boards/${ACTIVE_BOARD})` 委托,**无需新增分支**。

### 5.2 工具链描述文件

新建 `Boards/<board>/cmake/gcc-arm-none-eabi.cmake`,设置 `TARGET_FLAGS`(CPU 参数)与链接脚本路径(路径大小写须与磁盘实际文件名一致):

```cmake
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/Boards/G431/STM32G431xx_FLASH.ld\"")
```

### 5.3 预设

不要手动维护 `CMakePresets.json` 中的 App 列表。新增板卡或 App 后运行:

```bash
python scripts/generate_cmake_presets.py
```

脚本会扫描 `Boards/*/App/*/CMakeLists.txt`,为每个板卡与 App 组合生成 Debug/Release 预设。`toolchainFile` 指向对应板卡,`ACTIVE_APP` 指向对应机器人应用。

## 6. 验证

```bash
cmake --preset <board>-<app>-Debug
cmake --build build/<board>-<app>-Debug
```

构建完成后应确认:

1. `.elf`、`.hex`、`.bin` 均生成于 `build/<board>-<app>-Debug/`。
2. 链接映射文件(`.map`)中 `Offline_Reg_Sec` 段非空(存在实际注册节点),`CAN_Reg_Sec`、`UART_Reg_Sec` 落段正确。

## 7. 注意事项

1. `hcan_t` 的 `typedef` 位置必须位于头文件顶部。
2. 使用 `hcan_t` 的头文件必须显式包含 `BSP_CAN.h`。
3. 链接脚本必须包含 `.custom_registry` 段及 `KEEP`。
4. 工具链描述文件中链接脚本路径的大小写须与磁盘一致,避免在区分大小写的文件系统(如 Linux CI)下失败。
5. 设备数据实例与 `Robot_Config.c` 接线属于板卡相关代码,不得置于 `Common/` 下。
