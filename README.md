# Horizon Control Framework

面向 RoboMaster 机器人竞技的嵌入式控制框架,基于 STM32 HAL 与 FreeRTOS 构建。核心设计目标是以**单一代码库支持多块 MCU 平台**:上层逻辑(算法、外设驱动、系统运行时)跨板卡复用,板级差异被收敛到每块板的独立实现中。

## 特性

- **跨平台复用**:上层逻辑与具体 MCU 解耦,同一套代码在 STM32F4 / STM32H7 等平台间复用。
- **严格分层**:设备数据不以全局变量暴露,而是由板级「数据所有者」私有持有,上层仅通过发布/订阅访问。
- **声明式硬件配置**:以链接段描述外设接线与离线检测,替代手写初始化流程。
- **模块化通信**:基于主题的发布/订阅与同步事件总线,解耦任务间数据流。
- **实时调度**:FreeRTOS 多任务与中断驱动,配合 DWT 周期计数器提供高精度计时。
- **标准化构建**:CMake 预设,板卡与机器人型号独立选择,新增板卡无需改动顶层构建脚本。

## 架构

### 分层

框架分为共享上层与板级实现两层,由 BSP 中性接口与发布/订阅在中间衔接:

- **Common/**:与硬件无关的上层逻辑,包含算法、外设驱动、系统运行时与公共工具。此层只保留类型定义与纯函数,不出现任何芯片相关符号(具体外设句柄、芯片 HAL 头)。
- **Boards/\<board\>/**:每块板自包含,包含 CubeMX 生成的 HAL/FreeRTOS/启动/链接脚本、硬件抽象层(BSP)与机器人应用(App)。


## 目录结构

```
Horizon_Control_Framework/
├── CMakeLists.txt              # 选板/选 App + 板卡校验,委托给板级 CMakeLists
├── CMakePresets.json           # 板卡 + App 组合预设
├── Common/                     # 共享上层(与硬件无关)
│   ├── Algorithm/              # 控制、滤波、运动学、功率控制
│   ├── Device/                 # 外设驱动(类型 + 纯函数,不含数据实例)
│   ├── System/                 # 运行时(消息中心、事件总线、系统状态)
│   └── Utils/                  # 公共定义、数学库、离线检测、hal.h
└── Boards/
    └── <board>/
        ├── CMakeLists.txt      # 板级构建配置(CPU 参数、源收集、链接)
        ├── BSP/                # 硬件抽象层(CAN/SPI/TIM/UART/DWT/Indicator)
        ├── App/                # 机器人应用(All_Task/Robot_Config/Robot_Cmd 等)
        ├── Core/               # CubeMX 生成(HAL/FreeRTOS/启动/链接脚本)
        ├── Drivers/            # ST HAL 与 CMSIS
        ├── Middlewares/        # FreeRTOS、CMSIS-DSP、USB 库
        └── cmake/              # 工具链描述 + stm32cubemx 规则
```

## 快速开始

### 环境要求

- CMake ≥ 3.22
- Ninja
- ARM GNU Toolchain(`arm-none-eabi-gcc`,路径需加入 `PATH`)

### 构建

```bash
cmake --preset H723-Debug
cmake --build build/H723-Debug
```

编译产物位于 `build/H723-Debug/`,包括 `.elf`、`.hex`、`.bin`。

## 板卡与机器人选择

- `ACTIVE_BOARD`:目标板卡,对应 `Boards/` 下的目录名(如 `F407`、`H723`)。
- `ACTIVE_APP`:机器人应用,对应板卡 `App/` 下的目录名。

预设提供常用组合(`F407-Debug`、`H723-Debug` 等),也可在配置阶段覆盖单个变量:

```bash
cmake --preset H723-Debug -DACTIVE_APP=2_Engineer
```

## 文档

- [新增板卡指南](Boards/README.md)
