cmake_minimum_required(VERSION 3.15)
project(LQRSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "D:/mujoco-3.13.0-windows-x86_64" CACHE PATH "Path to MuJoCo root")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_LIB "${MUJOCO_ROOT}/lib/mujoco.lib")
set(MUJOCO_DLL "${MUJOCO_ROOT}/bin/mujoco.dll")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
    message(FATAL_ERROR "MuJoCo headers not found under ${MUJOCO_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${MUJOCO_LIB}")
    message(FATAL_ERROR "MuJoCo import library not found: ${MUJOCO_LIB}")
endif()
if(NOT EXISTS "${MUJOCO_DLL}")
    message(FATAL_ERROR "MuJoCo DLL not found: ${MUJOCO_DLL}")
endif()

set(GLFW_SOURCE_DIR "D:/mujoco-3.13.0-windows-x86_64/glfw-3.5.1" CACHE PATH "Path to local GLFW source")
if(NOT EXISTS "${GLFW_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "GLFW source tree not found: ${GLFW_SOURCE_DIR}")
endif()

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory("${GLFW_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/glfw-build" EXCLUDE_FROM_ALL)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/balance_controller.cpp
    src/state_reader.cpp
    src/system_identification.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${GLFW_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    "${MUJOCO_LIB}"
    glfw
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE opengl32 shell32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MUJOCO_DLL}"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )
endif()

message(STATUS "MuJoCo root: ${MUJOCO_ROOT}")
message(STATUS "GLFW source: ${GLFW_SOURCE_DIR}")

option(LQR_BUILD_SYSID_TESTS "Build system identification integration checks" OFF)
if(LQR_BUILD_SYSID_TESTS)
    enable_testing()
    add_executable(SysIdIntegration
        tests/sysid_integration.cpp
        src/balance_controller.cpp
        src/state_reader.cpp
        src/system_identification.cpp
    )
    target_include_directories(SysIdIntegration PRIVATE "${MUJOCO_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(SysIdIntegration PRIVATE "${MUJOCO_LIB}")
    add_test(NAME sysid_integration COMMAND SysIdIntegration
        "${CMAKE_CURRENT_SOURCE_DIR}/models/scene.xml" "${CMAKE_BINARY_DIR}/sysid_tests")
endif()
