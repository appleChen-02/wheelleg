set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# The STM32Cube terminal supplies these tools through its bundle environment.
find_program(CMAKE_C_COMPILER arm-none-eabi-gcc REQUIRED)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
find_program(CMAKE_AR arm-none-eabi-ar REQUIRED)
find_program(CMAKE_RANLIB arm-none-eabi-ranlib REQUIRED)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size REQUIRED)

set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS_INIT "${CMAKE_C_FLAGS_INIT}")
# Optimization is controlled by the application target for both configurations.
set(CMAKE_C_FLAGS_DEBUG "" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG" CACHE STRING "" FORCE)
