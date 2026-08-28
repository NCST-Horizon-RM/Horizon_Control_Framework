# ================= H723 板级构建配置 =================
# 校验工具链与板卡匹配：H723 需要 cortex-m7 工具链
if(NOT TOOLCHAIN_MCU STREQUAL "cortex-m7")
    message(FATAL_ERROR
        "[H723] 工具链与板卡不匹配：当前工具链 TOOLCHAIN_MCU='${TOOLCHAIN_MCU}'，"
        "H723 需要 'cortex-m7'。请在 CLion 切换到 H723 的 CMake Profile。")
endif()

# 硬件架构参数
set(CPU_FLAGS      -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
set(BOARD_CHIP_DEF STM32H723xx)
set(BOARD_MATH_DEF ARM_MATH_CM7)

# ---- 校验 ACTIVE_APP:App 目录不存在直接报错 ----
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/App/${ACTIVE_APP}")
    file(GLOB _entries LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/App/*")
    set(_names "")
    foreach(_e IN LISTS _entries)
        if(IS_DIRECTORY "${_e}")
            get_filename_component(_n "${_e}" NAME)
            list(APPEND _names "${_n}")
        endif()
    endforeach()
    list(JOIN _names ", " _list)
    message(FATAL_ERROR "ACTIVE_APP='${ACTIVE_APP}' 无效:找不到 App/${ACTIVE_APP}。可用 App: ${_list}")
endif()

# ================= 源文件收集 =================
# 共享上层(跨板卡复用)由 Common/CMakeLists.txt 独立管理，此处只收集板级 BSP
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "BSP/*.c" "BSP/*.cpp" "BSP/*.s")

# CMSIS-DSP(板级 Middlewares 内,按芯片宏选择 ARM_MATH_CMx)
file(GLOB_RECURSE CMSIS_DSP_SOURCES "Middlewares/Third_Party/CMSIS-DSP/Source/*/*.c")
set(CMSIS_DSP_MAIN_SOURCES "")
foreach(SOURCE_FILE ${CMSIS_DSP_SOURCES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME)
    get_filename_component(DIR_NAME ${SOURCE_FILE} DIRECTORY)
    get_filename_component(DIR_NAME ${DIR_NAME} NAME)
    if(FILE_NAME STREQUAL "${DIR_NAME}.c")
        list(APPEND CMSIS_DSP_MAIN_SOURCES ${SOURCE_FILE})
    endif()
endforeach()
# 只编译聚合主文件（如 MatrixFunctions.c 内部 #include 各实现），避免 NEON 模板文件被单独编译
set(CMSIS_DSP_SOURCES ${CMSIS_DSP_MAIN_SOURCES})

add_executable(${CMAKE_PROJECT_NAME} ${BSP_SOURCES} ${CMSIS_DSP_SOURCES})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# ================= 头文件目录收集 =================
file(GLOB_RECURSE BSP_H_FILES CONFIGURE_DEPENDS "BSP/*.h" "BSP/*.hpp")
set(ALL_INCLUDE_DIRS "")
foreach(FILE_PATH ${BSP_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND ALL_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${ALL_INCLUDE_DIRS}
        Core/Inc
        Middlewares/Third_Party/CMSIS-DSP/Include
        Middlewares/Third_Party/CMSIS-DSP/PrivateInclude
)

# ================= 编译选项与宏定义 =================
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        ${BOARD_CHIP_DEF} USE_HAL_DRIVER ${BOARD_MATH_DEF} __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1 ARM_MATH_MATRIX_CHECK=0 ARM_MATH_ROUNDING=0
        $<$<CONFIG:Debug>:DEBUG_MODE> $<$<CONFIG:Release>:RELEASE_MODE>
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -g3 -funroll-loops>
)

# 链接配置
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        $<$<CONFIG:Release>:-flto=auto>
)

# ================= 子目录 =================
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(App/${ACTIVE_APP})

# ================= Common 库：注入板级依赖 =================
# Common 源码 include 了 BSP 头（BSP_CAN.h/BSP_UART.h），BSP 又依赖 HAL/Core，
# 因此由板级把 BSP/Core/CMSIS 头目录、芯片宏、CPU 参数注入 Common 库。
target_include_directories(Common PRIVATE
        ${ALL_INCLUDE_DIRS}
        Core/Inc
        Middlewares/Third_Party/CMSIS-DSP/Include
        Middlewares/Third_Party/CMSIS-DSP/PrivateInclude
)
target_compile_definitions(Common PRIVATE
        ${BOARD_CHIP_DEF} USE_HAL_DRIVER ${BOARD_MATH_DEF} __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1 ARM_MATH_MATRIX_CHECK=0 ARM_MATH_ROUNDING=0
        $<$<CONFIG:Debug>:DEBUG_MODE> $<$<CONFIG:Release>:RELEASE_MODE>
)
target_compile_options(Common PRIVATE
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -g3 -funroll-loops>
)
target_link_libraries(Common PRIVATE stm32cubemx)   # HAL 驱动头文件

target_link_libraries(${CMAKE_PROJECT_NAME}
        Common
        stm32cubemx
        m
)

# ================= 后处理 =================
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating Output Files..."
)
