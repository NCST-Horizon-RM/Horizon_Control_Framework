# ================= Common 共享层构建配置 =================
# 跨板卡复用的共享代码，独立成库，由各板卡链接。
# 注意：Common 源码 include 了板级 BSP 头（BSP_CAN.h/BSP_UART.h），
#       因此 CPU 参数 / 芯片宏 / BSP / HAL 头目录由 Boards/<board>/CMakeLists.txt 注入本目标。

# 源文件收集
file(GLOB_RECURSE COMMON_SOURCES CONFIGURE_DEPENDS "*.c" "*.cpp" "*.s")

# 头文件目录收集（PUBLIC 暴露给链接方，App 模块靠链接本库拿到 Common 头目录）
file(GLOB_RECURSE COMMON_H_FILES CONFIGURE_DEPENDS "*.h" "*.hpp")
set(COMMON_INCLUDE_DIRS "")
foreach(FILE_PATH ${COMMON_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND COMMON_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES COMMON_INCLUDE_DIRS)

add_library(Common STATIC ${COMMON_SOURCES})
target_include_directories(Common PUBLIC ${COMMON_INCLUDE_DIRS})
