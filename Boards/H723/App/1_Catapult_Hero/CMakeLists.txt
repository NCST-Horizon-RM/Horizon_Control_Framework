cmake_minimum_required(VERSION 3.22)

# 搜集当前 App 目录下的所有源文件
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "*.c" "*.cpp" "*.s")
add_library(App_Module STATIC ${APP_SOURCES})

# 自动递归搜集并包含当前 App 目录下的所有头文件路径
file(GLOB_RECURSE APP_H_FILES CONFIGURE_DEPENDS "*.h" "*.hpp")
set(APP_INCLUDE_DIRS "")
foreach(FILE_PATH ${APP_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND APP_INCLUDE_DIRS ${DIR_PATH})
endforeach()
if(APP_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES APP_INCLUDE_DIRS)
endif()

target_include_directories(App_Module PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
)

if(CPU_FLAGS)
    target_compile_options(App_Module PRIVATE ${CPU_FLAGS})
endif()

target_compile_options(App_Module PRIVATE
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -g3 -funroll-loops>
)

# 显式注入生成配置宏
target_compile_definitions(App_Module PRIVATE
        STM32H723xx
        USE_HAL_DRIVER
        ARM_MATH_CM7
        __FPU_PRESENT=1U
        $<$<CONFIG:Debug>:DEBUG_MODE>
        $<$<CONFIG:Release>:RELEASE_MODE>
)

# 链接底层的 stm32cubemx 驱动库
target_link_libraries(App_Module PRIVATE stm32cubemx)

# 获取并包含主项目（Core、Drivers等）的头文件路径
get_target_property(FW_INCLUDES ${CMAKE_PROJECT_NAME} INCLUDE_DIRECTORIES)
if(FW_INCLUDES)
    target_include_directories(App_Module PRIVATE ${FW_INCLUDES})
endif()

# 将编译好的子模块库静态链接回主项目的可执行文件中
target_link_libraries(${CMAKE_PROJECT_NAME}
        -Wl,--whole-archive
        App_Module
        -Wl,--no-whole-archive
)