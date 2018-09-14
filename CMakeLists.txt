set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

## Set for your ARM toolchain paths (or leave as is if you have them set in PATH variables)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(AS arm-none-eabi-as)
set(AR arm-none-eabi-ar)
set(OBJCOPY arm-none-eabi-objcopy)
set(OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

## Specific HAL settings =============
set(TUP_CONFIG tup.config)
execute_process(COMMAND python "${CMAKE_SOURCE_DIR}/CMake_Helpers/CMake_Read_TUP_Config.py" "${CMAKE_SOURCE_DIR}/Firmware/${TUP_CONFIG}" "${CMAKE_SOURCE_DIR}/Firmware/HAL_Config.lua" "${CMAKE_SOURCE_DIR}/CMake_Helpers")# OUTPUT_VARIABLE HAL_SETTINGS)

include(${CMAKE_SOURCE_DIR}/CMake_Helpers/HAL_Generated.cmake)


set(ARM_MATH_LIB ${CMAKE_SOURCE_DIR}/${BOARD_DIR}/Drivers/CMSIS/Lib/libarm_cortexM4lf_math.a)


## ====================================

set(OPT_FLAGS "-Og -ffast-math -fno-finite-math-only")

set(COMMON_FLAGS "${COMMON_FLAGS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wfloat-conversion -fdata-sections -ffunction-sections -g -gdwarf-2 ")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${OPT_FLAGS} -std=c++14")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}  ${OPT_FLAGS} -std=c99")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} ${OPT_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS  "${LINKER_FLAGS} -lc -lm -lnosys -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections -Wl,--undefined=uxTopUsedPriority ${OPT_FLAGS}")
link_libraries(${ARM_MATH_LIB})

PROJECT(ODrive ASM CXX C)

add_definitions(-D__weak=__attribute__\(\(weak\)\))
add_definitions(-D__packed=__attribute__\(\(__packed__\)\))
add_definitions(-DUSE_HAL_DRIVER)


# Remove strings matching given regular expression from a list.
# @param(in,out) aItems Reference of a list variable to filter.
# @param aRegEx Value of regular expression to match.
function (filter_items aItems aRegEx)
    # For each item in our list
    foreach (item ${${aItems}})
        # Check if our items matches our regular expression
        if ("${item}" MATCHES ${aRegEx})
            # Remove current item from our list
            list (REMOVE_ITEM ${aItems} ${item})
        endif ("${item}" MATCHES ${aRegEx})
    endforeach(item)
    # Provide output parameter
    set(${aItems} ${${aItems}} PARENT_SCOPE)
endfunction (filter_items)

## Find the main sources
FILE(GLOB_RECURSE SOURCES_NO_HAL "${CMAKE_SOURCE_DIR}/Firmware/*.cpp" "${CMAKE_SOURCE_DIR}/Firmware/*.hpp" "${CMAKE_SOURCE_DIR}/Firmware/*.h" "${CMAKE_SOURCE_DIR}/Firmware/*.c")
filter_items(SOURCES_NO_HAL "${CMAKE_SOURCE_DIR}/Firmware/Board/*")
filter_items(SOURCES_NO_HAL "${CMAKE_SOURCE_DIR}/Firmware/fibre/*")

## Remove posix based files.
FILE(GLOB_RECURSE SOURCES_fibre "${CMAKE_SOURCE_DIR}/Firmware/fibre/cpp/*.c*" "${CMAKE_SOURCE_DIR}/Firmware/fibre/cpp/*.h*")
filter_items(SOURCES_fibre "posix")
LIST(APPEND SOURCES_NO_HAL ${SOURCES_fibre})

# Find the specific HAL sources
FILE(GLOB_RECURSE SOURCES_HAL "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/*.hpp" "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/*.c" "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/*.h" "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/*.s")
filter_items(SOURCES_HAL "${CMAKE_SOURCE_DIR}/${BOARD_DIR}/Src/prev_board_ver/*")
set(SOURCES ${SOURCES_NO_HAL} ${SOURCES_HAL} )

# From the sources, work out the folders that need to be include dirs.
execute_process(COMMAND python "${CMAKE_SOURCE_DIR}/CMake_Helpers/CMake_Find_Include_Dirs.py" -SourcesAndHeaders "${SOURCES}" OUTPUT_VARIABLE INCLUDE_DIRS)
include_directories(${INCLUDE_DIRS})
include_directories(Firmware)


add_executable(${PROJECT_NAME}.elf ${SOURCES})

set(CMAKE_EXE_LINKER_FLAGS " ${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(ASM_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.asm)

# Runs BEFORE building starts (workaround for mingw) to create version.h (imitates TUP)
add_custom_target(PreBuildHelper
        COMMAND python "${CMAKE_SOURCE_DIR}/CMake_Helpers/CMake_Create_Build_Folder.py" -Path "${CMAKE_SOURCE_DIR}/Firmware/build"
        COMMAND python "${CMAKE_SOURCE_DIR}/tools/odrive/version.py" --output ${CMAKE_SOURCE_DIR}/Firmware/build/version.h
        )
add_dependencies(${PROJECT_NAME}.elf PreBuildHelper) # Workaround for mingw_arm

# Runs AFTER building finishes to delete version.h (so TUP can generate its own if it likes)
add_custom_command(TARGET ${PROJECT_NAME}.elf
        POST_BUILD
        COMMAND ${OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${OBJDUMP} $<TARGET_FILE:${PROJECT_NAME}.elf> -dSC> ${ASM_FILE}
        COMMAND ${}
        COMMAND python "${CMAKE_SOURCE_DIR}/CMake_Helpers/CMake_Remove_Build_Folder.py" --Path "${CMAKE_SOURCE_DIR}/Firmware/build"
        COMMAND ${SIZE} --format=SysV ${PROJECT_NAME}.elf
        )


# Remote upload
add_custom_target(UPLOAD
        arm-none-eabi-gdb -iex "target remote tcp:127.0.0.1:3333"
        -iex "monitor program $<TARGET_FILE:${PROJECT_NAME}.elf>"
        -iex "monitor reset init"
        -iex "disconnect" -iex "quit")





## Something up with .rodata .text .debugstr .debugline

















## OLD ARCHIVED STUFF

#include_directories(Firmware/Board/v3/Drivers/CMSIS)
#include_directories(Firmware/Board/v3/Drivers/CMSIS/Device)
#include_directories(Firmware/Board/v3/Drivers/CMSIS/Device/ST)
#include_directories(Firmware/Board/v3/Drivers/CMSIS/Device/ST/STM32F4xx)
#include_directories(Firmware/Board/v3/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
#include_directories(Firmware/Board/v3/Drivers/CMSIS/Include)
#include_directories(Firmware/Board/v3/Drivers/STM32F4xx_HAL_Driver/Inc)
#include_directories(Firmware/Board/v3/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)
#include_directories(Firmware/Board/v3/Inc)
#include_directories(Firmware/Board/v3/Inc/prev_board_ver)
#include_directories(Firmware/Board/v3/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)
#include_directories(Firmware/Board/v3/Middlewares/ST/STM32_USB_Device_Library/Core/Inc)
#include_directories(Firmware/Board/v3/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS)
#include_directories(Firmware/Board/v3/Middlewares/Third_Party/FreeRTOS/Source/include)
#include_directories(Firmware/Board/v3/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)
#include_directories(Firmware/build)
#include_directories(Firmware/communication)
#
#include_directories(Firmware/Drivers/DRV8301)
#include_directories(Firmware/fibre/cpp/include)
#include_directories(Firmware/fibre/cpp/include/fibre)
#include_directories(Firmware/MotorControl)