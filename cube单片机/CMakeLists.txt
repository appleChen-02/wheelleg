cmake_minimum_required(VERSION 3.22)

# Also usable with a standalone Arm GNU Toolchain on PATH.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake")
endif()

project(standard_robot LANGUAGES C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/target.cmake)
include(cmake/sources.cmake)
include(cmake/dsp.cmake)

add_executable(standard_robot ${ROBOT_SOURCES})
set_target_properties(standard_robot PROPERTIES SUFFIX ".elf")
target_include_directories(standard_robot PRIVATE ${ROBOT_INCLUDE_DIRS})
target_compile_definitions(standard_robot PRIVATE
    USE_HAL_DRIVER STM32F407xx ARM_MATH_CM4 __FPU_PRESENT=1U
    ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING
)
target_compile_options(standard_robot PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    "$<$<COMPILE_LANGUAGE:C>:-ffunction-sections;-fdata-sections;-fno-common;-fno-strict-aliasing;-fshort-enums;-Wall;-Wextra>"
    "$<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:Debug>>:-Og;-g3>"
    "$<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:Release>>:-O2;-g3>"
    "$<$<COMPILE_LANGUAGE:ASM>:-x;assembler-with-cpp;-g3>"
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407IGHX_FLASH.ld")
target_link_options(standard_robot PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    "-T${LINKER_SCRIPT}"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/standard_robot.map"
    -Wl,-u,_printf_float
    # AHRS has an ARMCC 16-bit wchar_t tag, but its API contains no wchar_t.
    # Keep Newlib's default wchar_t ABI; this archive is used only for floats.
    -Wl,--no-wchar-size-warning
)
set_property(TARGET standard_robot APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_libraries(standard_robot PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/AHRS.lib"
    m
)

add_custom_command(TARGET standard_robot POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:standard_robot>" "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:standard_robot>" "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:standard_robot>"
    VERBATIM
)
