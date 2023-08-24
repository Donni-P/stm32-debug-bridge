cmake_minimum_required(VERSION 3.13)

if(NOT CMAKE_TOOLCHAIN_FILE)
    if (NOT CROSS_COMPILE)
        set(CROSS_COMPILE arm-none-eabi-)
    endif(NOT CROSS_COMPILE)
    set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)
endif(NOT CMAKE_TOOLCHAIN_FILE)


project (debug-bridge ASM C CXX)

if(NOT PLATFORM)
	set (PLATFORM stlink-v2-dupboot)
endif(NOT PLATFORM)

set (PLATFORM_DIR ${PROJECT_SOURCE_DIR}/platform/${PLATFORM})

find_file(
	CONFIG_CMAKE
	config.cmake
	PATHS ${PLATFORM_DIR}
	REQUIRED
	NO_DEFAULT_PATH
	)
find_file(
	CONFIG_HEADER
	config.h
	PATHS ${PLATFORM_DIR}
	REQUIRED
	NO_DEFAULT_PATH
	)

include(${CONFIG_CMAKE})
include(default.cmake)

configure_file(src/linker.ld.in linker.ld)
add_compile_definitions(STM32F103xB HSE_VALUE=8000000U)
add_compile_options(-mthumb -fno-pic -ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)

if (MCPU)
    add_compile_options(-mcpu=${MCPU})
    add_link_options(-mcpu=${MCPU})
endif(MCPU)
add_compile_options(-Wall -Wextra)
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -gdwarf-2")
set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
set (CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -Os")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -gdwarf-2")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -fno-use-cxa-atexit")

add_link_options(-mthumb -Tlinker.ld --specs=nosys.specs -Xlinker -Map=firmware.map)

add_executable(firmware.elf
    ${STARTUP}
    src/main.cpp
    src/usb_descriptors.cpp
    src/usb.cpp
    src/uart.cpp
    src/jtag.cpp
    src/global_resources.cpp
)

add_library(gpio STATIC
    src/gpio.cpp
)

add_library(flash STATIC
    src/flash.cpp
)

target_link_libraries(firmware.elf PRIVATE gpio flash)

add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary firmware.elf firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -Oihex firmware.elf firmware.hex
)

add_custom_target(upload 
    COMMAND dfu-util -D firmware.bin
    DEPENDS firmware.elf)

include(ExternalProject)

ExternalProject_Add(tests
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/tests
    CMAKE_ARGS -DREPORT_PATH=${CMAKE_BINARY_DIR}
    TEST_BEFORE_INSTALL true TEST_COMMAND ctest --output-on-failure
    INSTALL_COMMAND cmake --build . --target coverage
    BUILD_ALWAYS true
)
