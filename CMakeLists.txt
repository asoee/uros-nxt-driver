# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(agent-reconnect C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

link_directories(libmicroros)

# Add executable. Default name is the project name, version 0.1

add_executable(agent-reconnect 
    agent-reconnect.cpp 
    pico_uart_transport.c
    )

pico_set_program_name(agent-reconnect "agent-reconnect")
pico_set_program_version(agent-reconnect "0.1")


# Modify the below lines to enable/disable output over UART/USB
# USB version (current)
pico_enable_stdio_uart(agent-reconnect 0)
pico_enable_stdio_usb(agent-reconnect 1)

# Uncomment these for UART version instead:
# pico_enable_stdio_uart(agent-reconnect 1)  
# pico_enable_stdio_usb(agent-reconnect 0)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_pwm
    hardware_clocks
	hardware_irq
    microros
)

# Add the standard include files to the build
target_include_directories(agent-reconnect PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

# Add any user requested libraries
target_link_libraries(agent-reconnect 

        )

pico_add_extra_outputs(agent-reconnect)

# Configure UART transport for micro-ROS
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Set minimum log severity for micro-ROS logging
add_compile_definitions(RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_SEVERITY_INFO)

