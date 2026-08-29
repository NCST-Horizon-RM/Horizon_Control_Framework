cmake_minimum_required(VERSION 3.22)

# 项目基本配置
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_PROJECT_NAME Horizon_Control_Framework)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ================= 板卡 / 机器人选择 =================
# ACTIVE_BOARD: 由工具链文件声明（Boards/<board>/cmake/*.cmake 里的 TOOLCHAIN_BOARD）。
#               CMakePresets.json 通过 toolchainFile 选择工具链，即选定板卡。
# ACTIVE_APP:   对应板卡 App/ 下的机器人目录名,由预设或命令行参数注入
set(ACTIVE_BOARD "${TOOLCHAIN_BOARD}")
if(NOT DEFINED ACTIVE_APP OR ACTIVE_APP STREQUAL "")
    message(FATAL_ERROR
        "ACTIVE_APP 未指定。请在 CLion 的 CMake Profile 中选择具体 App 配置，"
        "或运行 scripts/generate_cmake_presets.py 生成官方配置。")
endif()

# ---- 校验 ACTIVE_BOARD:板卡目录不存在直接报错 ----
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Boards/${ACTIVE_BOARD}/cmake/stm32cubemx")
    file(GLOB _entries LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/Boards/*")
    set(_names "")
    foreach(_e IN LISTS _entries)
        if(IS_DIRECTORY "${_e}")
            get_filename_component(_n "${_e}" NAME)
            list(APPEND _names "${_n}")
        endif()
    endforeach()
    list(JOIN _names ", " _list)
    message(FATAL_ERROR "ACTIVE_BOARD='${ACTIVE_BOARD}' 无效:找不到 Boards/${ACTIVE_BOARD}/cmake/stm32cubemx。可用板卡: ${_list}")
endif()

# Common 共享层（跨板卡复用代码，独立成库；板级 flags 由 Boards/<board> 注入）
add_subdirectory(Common)

# 板卡的 CPU 参数、源文件收集、编译/链接配置均在 Boards/<board>/CMakeLists.txt 中定义
add_subdirectory(Boards/${ACTIVE_BOARD})
