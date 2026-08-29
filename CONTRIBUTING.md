# Horizon Control Framework 协作流程

本文档用于规范本仓库的 Issue、分支、PR、Review 和合并流程。避免框架在多人使用后变成不可维护的代码库。

## 1. 基本原则

1. 主分支必须保持可编译。本仓库当前使用 `master`,如果后续切换为 `main`,以下流程中的主分支名称同步替换。
2. 所有问题和需求先开 Issue,不要只口头描述。
3. 所有代码改动通过 PR 合并,不要直接推主分支。
4. `Common/`、`Boards/*/BSP/`、`CMakeLists.txt`、`CMakePresets.json` 属于框架核心,必须经过框架维护者 Review。
5. 普通机器人功能开发优先放在 `Boards/<board>/App/<robot>/` 下,不要随意改公共层。
6. 没有硬件测试条件时必须如实写明,不要把“编译通过”当成“实车可用”。
7. 不要为了本地切换 App 修改 `CMakeLists.txt`。官方 App 配置由 `scripts/generate_cmake_presets.py` 生成。

## 2. Issue 是什么

Issue 是任务单,用于记录 bug、需求、重构、文档和硬件测试项。任何需要别人处理或跟踪的事情都应该开 Issue。

### 2.1 Issue 类型

- `bug`: 已知错误或异常现象。
- `feature`: 新功能。
- `refactor`: 重构,不应该改变外部行为。
- `docs`: 文档补充或修正。
- `test`: 测试、验证、实车实验。
- `hardware-test`: 需要实体硬件验证。

### 2.2 Bug Issue 模板

```md
## 问题

简要描述出现了什么异常。

## 环境

- 板卡:
- App:
- 分支/commit:
- 是否实车:
- 相关外设:

## 复现步骤

1. 
2. 
3. 

## 实际现象

串口输出、CAN 现象、LED/蜂鸣器状态、调试截图或日志。

## 期望现象

应该发生什么。

## 最近改动

最近修改过哪些文件或配置。
```

### 2.3 功能 Issue 模板

```md
## 目标

说明要新增什么能力。

## 范围

- 涉及板卡:
- 涉及 App:
- 涉及模块:

## 验收标准

- [ ] 编译通过
- [ ] 接口不破坏已有 App
- [ ] 已完成必要硬件测试或标记为未测试
- [ ] 文档已更新
```

## 3. 分支规则

不要直接在主分支上改代码。每个任务从主分支拉一个新分支:

```bash
git checkout master
git pull
git checkout -b fix/h723-can-offline
```

分支命名建议:

- `fix/<short-name>`: 修 bug。
- `feat/<short-name>`: 新功能。
- `refactor/<short-name>`: 重构。
- `docs/<short-name>`: 文档。
- `test/<short-name>`: 测试或验证。

## 4. CMake Profile 规则

CLion 中通过 CMake Profile 选择板卡和 App。官方 Profile 来自 `CMakePresets.json`,命名格式为:

```text
<board>-<app>-<Debug|Release>
```

例如:

```text
H723-1_Catapult_Hero-Debug
F407-3_Mecanum_Infanty_Chassis-Debug
F407-3_Mecanum_Infanty_Gimbal-Release
```

新增、删除或重命名 App 后,运行:

```bash
python scripts/generate_cmake_presets.py
```

然后把更新后的 `CMakePresets.json` 一起提交。不要手动维护 App 列表。

个人本地配置写入 `CMakeUserPresets.json`,不要提交到仓库。

## 5. PR 是什么

PR 是 Pull Request,即“请求把当前分支的代码合并进主分支”。PR 用来展示改动、跑 CI、接受 Review,最后再合并。

### 5.1 提交 PR 前

提交 PR 前至少确认:

- 改动范围和 Issue 对得上。
- 没有顺手改无关文件。
- 本地能编译的配置已经编译过。
- 没有把临时调试代码、无用注释、大段废代码提交进去。
- 如果改了 `Common/` 或 `BSP/`,确认不会破坏其他板卡/App。

### 5.2 PR 描述模板

```md
## 改动内容

- 
- 

## 关联 Issue

Closes #

## 测试情况

- [ ] F407 Debug 编译通过
- [ ] F407 Release 编译通过
- [ ] H723 Debug 编译通过
- [ ] H723 Release 编译通过
- [ ] 已完成硬件测试
- [ ] 暂无硬件测试条件

## 风险说明

说明可能影响哪些模块、板卡或机器人。
```

`Closes #12` 表示这个 PR 合并后自动关闭第 12 号 Issue。

## 6. Review 是什么

Review 是代码合并前的审查，确认这次改动不会破坏框架、不会引入明显 bug，并且测试和风险说明足够清楚。

### 6.1 Review 主要看什么

Reviewer 需要检查:

1. 改动是否解决了对应 Issue。
2. 是否改了不该改的范围。
3. 是否破坏 `Common/` 和 `BSP/` 的接口契约。
4. F4/H7 差异是否被正确处理。
5. 中断、DMA、RTOS task、CAN/UART 回调里是否有明显时序风险。
6. 是否存在空指针、数组越界、长度错误、单位错误、符号方向错误。
7. 是否补充了必要测试或明确写明未测试原因。
8. 文档是否和当前实现一致。

### 6.2 PR 作者需要做什么

作者需要:

1. 回答 Review 评论。
2. 根据意见继续提交修复 commit。
3. 不要自行解决评论后强行合并。
4. 如果不同意 Review 意见,说明技术理由。
5. 改完后重新确认 CI 和测试情况。

### 6.3 Review 结论

常见结论:

- `Approve`: 可以合并。
- `Request changes`: 必须修改后再合并。
- `Comment`: 有建议或问题,不一定阻塞合并。

## 7. 合并标准

满足以下条件才能合并:

1. CI 通过。
2. 至少一名维护者 Review 通过。
3. 改动 `Common/`、`BSP/`、构建系统时,必须由框架维护者 Review。
4. PR 描述中写清楚测试情况。
5. 有硬件风险但未测试时,必须打上 `hardware-test` 或在 Issue 中继续跟踪。

不允许合并的情况:

- CI 失败。
- PR 描述为空。
- 大量无关格式化或无关重构混在一起。
- 改了公共接口但没有说明影响范围。
- 实车未测却写成已验证。

## 8. 推荐标签

建议在 GitHub Labels 中建立:

- `bug`
- `feature`
- `docs`
- `refactor`
- `test`
- `app`
- `bsp`
- `common`
- `build`
- `hardware-test`
- `good-first-issue`
- `blocked`

`good-first-issue` 用来分配给新人,例如补文档、修注释、补简单 App 示例、修明确的小 bug。

## 9. 目录权限建议

建议按目录划分维护权限:

- `Boards/*/App/*`: 普通电控成员可改,但仍需 PR。
- `Common/Device/*`: 熟悉对应设备的人可改,维护者 Review。
- `Common/Algorithm/*`: 修改前说明数学含义、单位和测试方式。
- `Common/System/*`: 框架核心,必须维护者 Review。
- `Boards/*/BSP/*`: 板级核心,必须维护者 Review。
- `CMakeLists.txt`、`CMakePresets.json`、`.github/workflows/*`: 构建核心,必须维护者 Review。

## 10. 硬件测试记录

涉及实车或外设的 PR,建议在 Issue 或 PR 中记录:

```md
## 硬件测试

- 测试人:
- 日期:
- 板卡:
- App:
- 外设:
- 测试项目:
- 结果:
- 异常:
```

没有硬件测试条件时写:

```md
暂无硬件测试条件,仅完成编译/静态检查。需要后续实车验证。
```

## 11. 紧急修复

比赛或调车现场可以先走紧急修复,但事后必须补 Issue 和 PR 记录。

紧急修复也要遵守:

1. 不直接改主分支。
2. 最小改动。
3. 记录修复原因。
4. 赛后补 Review。

## 12. 新人上手任务

新人不要一开始改框架核心。推荐任务:

1. 阅读 `README.md` 和 `Boards/README.md`。
2. 编译一个现有 App。
3. 在 App 层新增一个只读调试变量或 VOFA 输出。
4. 新增一个电机注册项并说明 CAN ID。
5. 根据模板开一个 Issue。
6. 根据 Issue 提一个小 PR。

能独立完成以上流程后,再接触 `Common/Device` 或 `BSP`。
