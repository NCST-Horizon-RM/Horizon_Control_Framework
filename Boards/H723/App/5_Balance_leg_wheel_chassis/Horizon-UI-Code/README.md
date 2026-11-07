# 步兵3UI · 嵌入式接入

协议版本：2027 V1.0.0。

## 三步接入

1. 将 horizon_ui.c/.h、ui_layout.c/.h 和 ui_bindings.c/.h 加入 MCU 工程，C 标准选择 C99 或更高。
2. 首次接入时，将 UI_Task.c.example / UI_Task.h.example 复制并改名为 UI_Task.c / UI_Task.h。实现 referee_send，确认机器人 ID，并将 UI_Task.c 加入工程。
3. 上电调用一次 App_UI_Init()；循环调用 App_UI_Task(now_ms)，建议周期 1 ms。在任务中填写 ui_data 字段，随后 UI_Update(&ui_data) 和 UI_Tick(now_ms) 完成更新与发送。

ui_bindings.h 中的数据字段来自设计器「数据绑定」，ui_bindings.c 自动完成数据到图形属性的换算。默认值在 UI_DataDefault 中；未绑定属性保留设计值。范围映射会限制到输入区间，输出端点可以引用设计属性，布局调整后重新生成即可。

协议层 horizon_ui.c 负责打包、CRC、发送调度；ui_layout.c 只描述图元；业务层只调用命名 API。无需移植宏、C++、动态内存或第三方库。运行状态为静态分配。

## 文件分层与更新

| 文件 | 职责 | 更新方式 |
| --- | --- | --- |
| horizon_ui.c / .h | 状态、协议编码、组包、30 Hz 调度和重绘 | 通用运行库；升级运行库时成对替换 |
| ui_layout.c / .h | 图元编号、页面和初始布局 | 设计器生成，可覆盖 |
| ui_bindings.c / .h | UI_Data、只读默认值和 UI_Update 映射 | 设计器生成，可覆盖 |
| UI_Task.c / .h | 串口适配、运行数据和业务更新 | 用户维护；模板以 .example 导出，不覆盖已有 .c/.h |

首次使用“完整代码”。已有工程调整设计时使用“仅更新布局”，只替换 ui_layout.c/.h、ui_bindings.c/.h；保持现有 UI_Task 和运行库。修改变量名或类型后，同步调整 UI_Task 中引用的字段。旧版工程首次升级需要完整更新 horizon_ui.c/.h，因为发送回调已从 void 改为 uint8_t。

UI_DataDefault 和 UI_Initial 均保持 const。UI_Task 中使用独立的 static UI_Data ui_data，初始化时执行 ui_data = UI_DataDefault；之后只修改 ui_data。文字字段为 const char *，UI_Update 会将内容复制到图元；调用更新时提供有效字符串即可。普通更新、属性 setter、UI_Refresh、切页和 UI_Tick 放在同一 UI 任务中执行；跨任务或中断的重绘事件使用 App_UI_RequestRedraw / UI_RequestRedraw。

旧工程若曾将 UI_DataDefault 改为可写全局变量：先在 UI_Task 中将该变量改名为 ui_data，并增加初始化赋值；再使用生成的 ui_bindings.c/.h 恢复只读 UI_DataDefault，避免同名定义冲突。旧 example.c 如已用于业务接入，可继续保留或迁入 UI_Task；工程只编译一套 App_UI_Init / App_UI_Task 实现。

## 发送适配

referee_send 返回 1 表示已接受本包，返回 0 表示当前不能发送。返回 0 时，调度器保留待发送状态，并在后续 UI_Tick 中重新组合最新值后重试；不会忙等。

使用已有发送队列时，将完整数据复制入队后返回 1，队列满时返回 0。直接使用 DMA 时，先检查串口空闲，再复制到专用缓冲区并启动 DMA，以 HAL_OK 作为接受成功；DMA 完成前不能覆盖缓冲区。缓冲区必须位于芯片 DMA 可访问的内存；H723 可沿用工程的 .RAM_D2 区域。不要直接把调度器的 bytes 指针交给异步 DMA 长期持有。模板默认返回 0，接入实际发送函数后才会发包。

返回 1 仅表示本地发送函数接受了本包，不表示选手端已经收到。上电漏收、客户端重连或操作手请求时，调用一次 App_UI_RequestRedraw()：后续任务清屏并重新增加当前可见图元，保留最新属性和业务数据。也可在 Live Watch 中将 ui_redraw_request 写为 1；任务处理后归零。重绘请求由事件触发，不要在每次循环中重复调用。

## 设计器与客户端

- 分辨率 1920 × 1080；原点在左下角。圆和椭圆坐标为圆心；矩形坐标为左下角；文字坐标为左上锚点。
- 图形名由设计器数字 ID 写成三个小端字节；请在整个工程中保持唯一。
- 字符区最多 30 字节，协议未明确指定编码。本生成器暂使用可打印 ASCII；中文可用于名称和注释。
- 浮点数以 int32_t 定点数传输，显示值 = 原始值 / 1000；细节参数 B 无作用。
- 圆弧 0° 在正上方，顺时针；字号、线宽和坐标按协议字段发送，不额外缩放。
- 接收者自动设为 robot_id + 0x100。机器人 ID 可用 1、3–6、101、103–106；2 和 102 为保留位。1/101 为重装机器人，6/106 为空中系统。

## 动态修改与页面

UI_Initial 是只读初始布局，UI_Init 将它复制为运行时状态。所有图形都支持动态属性，不需要再生成静态和动态两套实现。业务层在自己的更新函数中调用以下接口；参数变化时接口自动标记图形，重复设置相同值不会重复标记。

| 接口 | 修改内容 |
| --- | --- |
| UI_SetPosition(id, x, y) | 移动任意图形，直线整体平移 |
| UI_SetLine(id, x, y, x2, y2) | 直线两个端点 |
| UI_SetLineEnd(id, x2, y2) | 直线终点，可用于改变长度 |
| UI_SetSize(id, width, height) | 矩形宽高 |
| UI_SetRadius(id, radius) | 圆半径 |
| UI_SetRadii(id, rx, ry) | 椭圆或圆弧的半轴 |
| UI_SetAngles(id, start_angle, end_angle) | 圆弧起止角度 |
| UI_SetColor(id, color) | 任意图形的颜色，使用 UI_GREEN 等枚举 |
| UI_SetLineWidth(id, line_width) | 线宽 |
| UI_SetFontSize(id, font_size) | 文字或数值的字号 |
| UI_SetNumber(id, value) | 整数或浮点数内容 |
| UI_SetText(id, text) | 文本内容 |
| UI_SetVisible(id, visible) | 显示或隐藏 |

设计器中的绑定会自动生成数据到长度、角度、半径等属性的换算，业务代码填写 UI_Data 字段并调用 UI_Update 即可。也可在需要时直接调用属性接口。更新只修改状态，不立即发送；同一图形在发送前的多次修改会合并成最终状态。数字按协议精度比较：整数为 1，浮点数为 0.001。没有绑定且未手动修改的图形保持初始状态。建议在同一个 UI 任务中更新状态。

姿态尺的俯仰、横滚数据单位为度；默认俯仰抬头为负、低头为正，可在设计器中反转方向。生成代码调用标准数学库，GCC 工程需链接 m（-lm）。


- UI_SetNumber(UI_ID_XXX, value)：整型范围 −2147483648～2147483647；浮点范围 −2147483.648～2147483.647。输入须在有效范围内。
- UI_SetText(UI_ID_XXX, "READY")：仅对文字使用，最多 30 个 ASCII 字符。
- UI_SetPosition(UI_ID_XXX, x, y)：移动图元；直线同时移动终点。
- UI_SetVisible(UI_ID_XXX, 0/1)：发送删除或重新增加。
- UI_Get(UI_ID_XXX) 返回图元结构，修改尺寸、角度或线宽后调用 UI_MarkDirty(UI_ID_XXX)。坐标、颜色、图层和尺寸须满足协议位宽。
- UI_ShowFrame(UI_FRAME_XXX)：请求清空客户端 UI，随后逐批增加该界面的图元。
- UI_ClearLayer(layer) / UI_Clear()：请求删除图层 / 全部 UI。UI_ShowFrame 可恢复页面。
- UI_Refresh()：在 UI 任务内重新发送当前可见图元的增加包，不清屏。
- UI_RequestRedraw() / App_UI_RequestRedraw()：请求清屏并重建当前可见界面，可由其他任务或中断触发一次。
- 相同值的 setter 不会反复排队；短时间多次更新同一图元会合并为最新值。静态与动态图元共用清晰的调用接口。


## 状态指示

数值选择对应提示、颜色和线宽；不匹配的值使用“其他值”。有效标志为 0 时也使用“其他值”。有效标志由业务层根据接收状态和超时更新。

| 数据字段 | 数值与提示 | 有效标志 |
| --- | --- | --- |
| base_r_state | 0 → OWN；1 → TEMP；2 → NOT OWN | field_data_valid |
| outpost_r_state | 0 → OWN；1 → TEMP；2 → NOT OWN | field_data_valid |
| outpost_b_state | 0 → OWN；1 → TEMP；2 → NOT OWN | field_data_valid |
| base_b_state | 0 → OWN；1 → TEMP；2 → NOT OWN | field_data_valid |
| small_rune_state | 0 → OFF；1 → ACTIVE；2 → ACTIVATING | field_data_valid |
| large_rune_state | 0 → OFF；1 → ACTIVE；2 → ACTIVATING | field_data_valid |
| chassis_output | 0 → OFF；1 → ON | robot_data_valid |
| shooter_output | 0 → OFF；1 → ON | robot_data_valid |
| rfid_contact | 0 → OFF；1 → CONTACT | robot_data_valid |
| combat_state | 0 → ；1 → CHASSIS POWER OFF；2 → SHOOTER POWER OFF；3 → HEAT LIMIT；4 → BUFFER LOW；5 → AMMO LOW；6 → VULNERABLE；7 → BASE GUARD LOST；8 → REFEREE LOST；9 → RUNE ACTIVE；10 → GIMBAL POWER OFF | robot_data_valid |
| gimbal_output | 0 → OFF；1 → ON | robot_data_valid |

## 发送调度

每次 UI_Tick 最多接受一包，全部 UI 包共用 30 Hz 上限，按 34 / 33 / 33 ms 间隔轮转；任务迟到时不补发突发包。未变化的图元不占用刷新包，同一图元的连续修改合并为最新状态。

连续变化的普通图元尽量每包装入 7 个，按最久未发送的顺序轮转。文字必须单独发送，调度器按平均刷新次数分配普通图元与文字的发送机会。静态加载、重绘按最少包数、再最少字节选择 1/2/5/7 组合，不足的槽位使用协议空操作。例如静态 10 个图元使用 5+5，4 个使用 5 槽包，6 个使用 7 槽包。

在数据持续变化、1 ms 调用任务且串口持续接受的模拟测试中：1～7 个普通图元每个 30 Hz；10 个每个 21 Hz；14 个每个 15 Hz；8 个普通图元加 1 个文字，每项约 14 Hz。实际内容变化还受业务数据更新频率、任务周期和链路影响。

整个 0x0301 命令共享 30 Hz 上行限制。如果工程还发机器人交互数据，应将 send 接入统一发送队列，在队列中共同限频。UI_ShowFrame、UI_Clear 和 UI_RequestRedraw 清空的是客户端全部 UI，工程内其他模块绘制的图元也会被删除。

2027 另将机器人每 1000 ms 的接收额度设为 round(x × λ)：重装、步兵、空中及飞镖系统的基准 x 为 3720 字节，雷达和哨兵为 5120 字节；λ = 1 − 己方基地已损失血量 / 20000。该额度限制与上述 30 Hz 上行包频率不同，其他机器人交互数据也需按接收方的实际额度调度。

send 的接受结果只在成功时提交：成功后才推进序号、发送节拍及图元状态。失败的增加、修改、删除和清屏均保留重试。重新增加尚未发出的图元时，连续属性更新不会把增加操作覆盖成修改。

## 联调

先使用设计器的串口模拟器接收 MCU 发送数据，检查图元与 CRC；再接裁判系统常规链路和选手端验证。此工具无法替代真实客户端、链路和现场规则验证。无连接硬件时不会声称串口或上机验证通过。
