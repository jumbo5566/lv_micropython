cmake_minimum_required(VERSION 3.12)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DISP_BOARD)
    set(DISP_BOARD "PICO_DM_QD3503728")
endif()
set(MICROPY_PY_DISP_PORT_LVGL 1)

# Set build type to reduce firmware size
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel)
endif()

# Set main target and component locations
set(MICROPY_TARGET firmware)
get_filename_component(MICROPY_DIR "../.." ABSOLUTE)
if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    set(PICO_SDK_PATH ../../lib/pico-sdk)
endif()

# Use the local tinyusb instead of the one in pico-sdk
set(PICO_TINYUSB_PATH ${MICROPY_DIR}/lib/tinyusb)

# Set the location of this port's directory.
set(MICROPY_PORT_DIR ${CMAKE_SOURCE_DIR})

# Set the board if it's not already set.
if(NOT MICROPY_BOARD)
    set(MICROPY_BOARD PICO)
endif()

# Set the board directory and check that it exists.
if(NOT MICROPY_BOARD_DIR)
    set(MICROPY_BOARD_DIR ${MICROPY_PORT_DIR}/boards/${MICROPY_BOARD})
endif()
if(NOT EXISTS ${MICROPY_BOARD_DIR}/mpconfigboard.cmake)
    message(FATAL_ERROR "Invalid MICROPY_BOARD specified: ${MICROPY_BOARD}")
endif()

set(MICROPY_USER_FROZEN_MANIFEST ${MICROPY_FROZEN_MANIFEST})

# Include board config, it may override MICROPY_FROZEN_MANIFEST
include(${MICROPY_BOARD_DIR}/mpconfigboard.cmake) 

# Set the PICO_BOARD if it's not already set (allow a board to override it).
if(NOT PICO_BOARD)
    string(TOLOWER ${MICROPY_BOARD} PICO_BOARD)
endif()

# Set the amount of C heap, if it's not already set.
# If a board uses malloc then it must set this to at least 4096.
if(NOT MICROPY_C_HEAP_SIZE)
    set(MICROPY_C_HEAP_SIZE 0)
endif()

# Enable extmod components that will be configured by extmod.cmake.
# A board may also have enabled additional components.
set(MICROPY_SSL_MBEDTLS ON)

# Include component cmake fragments
include(${MICROPY_DIR}/py/py.cmake)
include(${MICROPY_DIR}/extmod/extmod.cmake)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Define the top-level project
project(${MICROPY_TARGET})

pico_sdk_init()

include(${MICROPY_DIR}/py/usermod.cmake)

add_executable(${MICROPY_TARGET})

set(MICROPY_QSTRDEFS_PORT
    ${PROJECT_SOURCE_DIR}/qstrdefsport.h
)

set(MICROPY_SOURCE_LIB
    ${MICROPY_DIR}/lib/littlefs/lfs1.c
    ${MICROPY_DIR}/lib/littlefs/lfs1_util.c
    ${MICROPY_DIR}/lib/littlefs/lfs2.c
    ${MICROPY_DIR}/lib/littlefs/lfs2_util.c
    ${MICROPY_DIR}/lib/mbedtls_errors/mp_mbedtls_errors.c
    ${MICROPY_DIR}/lib/oofatfs/ff.c
    ${MICROPY_DIR}/lib/oofatfs/ffunicode.c
    ${MICROPY_DIR}/shared/netutils/netutils.c
    ${MICROPY_DIR}/shared/netutils/trace.c
    ${MICROPY_DIR}/shared/readline/readline.c
    ${MICROPY_DIR}/shared/runtime/gchelper_m0.s
    ${MICROPY_DIR}/shared/runtime/gchelper_native.c
    ${MICROPY_DIR}/shared/runtime/interrupt_char.c
    ${MICROPY_DIR}/shared/runtime/mpirq.c
    ${MICROPY_DIR}/shared/runtime/pyexec.c
    ${MICROPY_DIR}/shared/runtime/stdout_helpers.c
    ${MICROPY_DIR}/shared/runtime/sys_stdio_mphal.c
    ${MICROPY_DIR}/shared/timeutils/timeutils.c
)

set(MICROPY_SOURCE_DRIVERS
    ${MICROPY_DIR}/drivers/bus/softspi.c
    ${MICROPY_DIR}/drivers/dht/dht.c
)

set(MICROPY_SOURCE_PORT
    fatfs_port.c
    machine_adc.c
    machine_bitstream.c
    machine_i2c.c
    machine_i2s.c
    machine_pin.c
    machine_rtc.c
    machine_spi.c
    machine_timer.c
    machine_uart.c
    machine_wdt.c
    main.c
    modmachine.c
    modrp2.c
    modutime.c
    mphalport.c
    mpnetworkport.c
    mpthreadport.c
    rp2_flash.c
    rp2_pio.c
    tusb_port.c
    uart.c
    msc_disk.c
    mbedtls/mbedtls_port.c
    led.c
)

set(MICROPY_SOURCE_QSTR
    ${MICROPY_SOURCE_PY}
    ${MICROPY_DIR}/shared/runtime/mpirq.c
    ${MICROPY_DIR}/shared/runtime/sys_stdio_mphal.c
    ${PROJECT_SOURCE_DIR}/machine_adc.c
    ${PROJECT_SOURCE_DIR}/machine_i2c.c
    ${PROJECT_SOURCE_DIR}/machine_i2s.c
    ${PROJECT_SOURCE_DIR}/machine_pin.c
    ${PROJECT_SOURCE_DIR}/machine_rtc.c
    ${PROJECT_SOURCE_DIR}/machine_spi.c
    ${PROJECT_SOURCE_DIR}/machine_timer.c
    ${PROJECT_SOURCE_DIR}/machine_uart.c
    ${PROJECT_SOURCE_DIR}/machine_wdt.c
    ${PROJECT_SOURCE_DIR}/modmachine.c
    ${PROJECT_SOURCE_DIR}/modrp2.c
    ${PROJECT_SOURCE_DIR}/moduos.c
    ${PROJECT_SOURCE_DIR}/modutime.c
    ${PROJECT_SOURCE_DIR}/rp2_flash.c
    ${PROJECT_SOURCE_DIR}/rp2_pio.c
    ${PROJECT_SOURCE_DIR}/led.c
)

set(PICO_SDK_COMPONENTS
    hardware_adc
    hardware_base
    hardware_clocks
    hardware_dma
    hardware_flash
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_pio
    hardware_pwm
    hardware_regs
    hardware_rtc
    hardware_spi
    hardware_structs
    hardware_sync
    hardware_timer
    hardware_uart
    hardware_watchdog
    pico_base_headers
    pico_binary_info
    pico_bootrom
    pico_multicore
    pico_platform
    pico_stdio
    pico_stdlib
    pico_sync
    pico_time
    pico_unique_id
    pico_util
    tinyusb_common
    tinyusb_device
)

if (MICROPY_PY_LWIP)
    target_link_libraries(${MICROPY_TARGET} micropy_lib_lwip)

    target_include_directories(${MICROPY_TARGET} PRIVATE
        lwip_inc
    )
    target_compile_definitions(${MICROPY_TARGET} PRIVATE
        MICROPY_PY_LWIP=1
    )

    list(APPEND MICROPY_SOURCE_EXTMOD
        ${MICROPY_EXTMOD_DIR}/modlwip.c
    )

    string(CONCAT GIT_SUBMODULES "${GIT_SUBMODULES} " lib/lwip)
endif()

if(MICROPY_PY_BLUETOOTH)
    list(APPEND MICROPY_SOURCE_PORT mpbthciport.c)
    target_compile_definitions(${MICROPY_TARGET} PRIVATE
        MICROPY_PY_BLUETOOTH=1
        MICROPY_PY_BLUETOOTH_USE_SYNC_EVENTS=1
        MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE=1
        MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING=1
        MICROPY_PY_BLUETOOTH_ENABLE_L2CAP_CHANNELS=1
    )
endif()

if(MICROPY_BLUETOOTH_NIMBLE)
    list(APPEND MICROPY_SOURCE_PORT mpnimbleport.c)
    target_compile_definitions(${MICROPY_TARGET} PRIVATE
        MICROPY_BLUETOOTH_NIMBLE=1
        MICROPY_BLUETOOTH_NIMBLE_BINDINGS_ONLY=0
    )
    target_compile_options(${MICROPY_TARGET} PRIVATE
    # TODO: This flag is currently needed to make nimble build.
    -Wno-unused-but-set-variable
    )
    include(${MICROPY_DIR}/extmod/nimble/nimble.cmake)
    target_link_libraries(${MICROPY_TARGET} micropy_extmod_nimble)
    get_target_property(NIMBLE_INCLUDE micropy_extmod_nimble INTERFACE_INCLUDE_DIRECTORIES)
    list(APPEND MICROPY_INC_CORE ${NIMBLE_INCLUDE})
endif()

if (MICROPY_PY_NETWORK_NINAW10)
    target_compile_definitions(${MICROPY_TARGET} PRIVATE
        MICROPY_PY_NETWORK_NINAW10=1
    )

    target_include_directories(${MICROPY_TARGET} PRIVATE
        ${MICROPY_DIR}/drivers/ninaw10/
    )

    # Enable NINA-W10 WiFi and Bluetooth drivers.
    list(APPEND MICROPY_SOURCE_DRIVERS
        ${MICROPY_DIR}/drivers/ninaw10/nina_bt_hci.c
        ${MICROPY_DIR}/drivers/ninaw10/nina_wifi_drv.c
        ${MICROPY_DIR}/drivers/ninaw10/nina_wifi_bsp.c
    )

    list(APPEND MICROPY_SOURCE_EXTMOD
        ${MICROPY_DIR}/extmod/network_ninaw10.c
    )
endif()

if (MICROPY_PY_NETWORK_WIZNET5K)
    target_compile_definitions(${MICROPY_TARGET} PRIVATE
        MICROPY_PY_NETWORK_WIZNET5K=1
        WIZCHIP_PREFIXED_EXPORTS=1
        _WIZCHIP_=${MICROPY_PY_NETWORK_WIZNET5K}
        WIZCHIP_YIELD=mpy_wiznet_yield
    )

    if (MICROPY_PY_LWIP)
        target_compile_definitions(${MICROPY_TARGET} PRIVATE
            # When using MACRAW mode (with lwIP), maximum buffer space must be used for the raw socket
            WIZCHIP_USE_MAX_BUFFER=1
        )
    endif()

    target_include_directories(${MICROPY_TARGET} PRIVATE
        ${MICROPY_DIR}/lib/wiznet5k/
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/
    )

    list(APPEND MICROPY_SOURCE_LIB
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/W5100/w5100.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/W5100S/w5100s.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/W5200/w5200.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/W5300/w5300.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/W5500/w5500.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/socket.c
        ${MICROPY_DIR}/lib/wiznet5k/Ethernet/wizchip_conf.c
        ${MICROPY_DIR}/lib/wiznet5k/Internet/DNS/dns.c
        ${MICROPY_DIR}/lib/wiznet5k/Internet/DHCP/dhcp.c
    )

    list(APPEND MICROPY_SOURCE_EXTMOD
        ${MICROPY_DIR}/extmod/network_wiznet5k.c
    )

    string(CONCAT GIT_SUBMODULES "${GIT_SUBMODULES} " lib/wiznet5k)
endif()

# Include our board porting files here
if (MICROPY_PY_DISP_PORT_LVGL)
    set(LVGL_BINDING_DIR ${LVGL_DIR}/..)
    set(LVGL_RP2_DRV_DIR ${LVGL_BINDING_DIR}/driver/rp2)
    set(BOARD_DIR ${LVGL_RP2_DRV_DIR}/${DISP_BOARD})
    include(${BOARD_DIR}/board.cmake)

    file(GLOB_RECURSE BOARD_FILES ${BOARD_DIR}/*.c)
    list(APPEND MICROPY_SOURCE_EXTMOD
        ${BOARD_FILES}
    )
endif()


# Add qstr sources for extmod and usermod, in case they are modified by components above.
list(APPEND MICROPY_SOURCE_QSTR
    ${MICROPY_SOURCE_EXTMOD}
    ${MICROPY_SOURCE_USERMOD}
)

# Define mpy-cross flags
set(MICROPY_CROSS_FLAGS -march=armv7m)

# Set the frozen manifest file
if (MICROPY_USER_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${MICROPY_USER_FROZEN_MANIFEST})
elseif (NOT MICROPY_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${PROJECT_SOURCE_DIR}/boards/manifest.py)
endif()

target_sources(${MICROPY_TARGET} PRIVATE
    ${MICROPY_SOURCE_PY}
    ${MICROPY_SOURCE_EXTMOD}
    ${MICROPY_SOURCE_LIB}
    ${MICROPY_SOURCE_DRIVERS}
    ${MICROPY_SOURCE_PORT}
)

target_link_libraries(${MICROPY_TARGET} micropy_lib_mbedtls)

# Filter out library/error.c as we're using mp_mbedtls_errors.c instead.
set_source_files_properties(${MICROPY_LIB_MBEDTLS_DIR}/library/error.c
    TARGET_DIRECTORY micropy_lib_mbedtls
    PROPERTIES HEADER_FILE_ONLY ON
)

target_link_libraries(${MICROPY_TARGET} usermod)

target_include_directories(${MICROPY_TARGET} PRIVATE
    ${MICROPY_INC_CORE}
    ${MICROPY_INC_USERMOD}
    ${MICROPY_BOARD_DIR}
    "${PROJECT_SOURCE_DIR}"
    "${CMAKE_BINARY_DIR}"
)

target_compile_options(${MICROPY_TARGET} PRIVATE
    -Wall
    -Werror
)

target_link_options(${MICROPY_TARGET} PRIVATE
    -Wl,--print-memory-usage,--defsym=__micropy_c_heap_size__=${MICROPY_C_HEAP_SIZE}
)

set_source_files_properties(
    ${PICO_SDK_PATH}/src/rp2_common/pico_double/double_math.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_float/float_math.c
    PROPERTIES
    COMPILE_OPTIONS "-Wno-error=uninitialized"
)

set_source_files_properties(
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040/rp2040_usb.c
    PROPERTIES
    COMPILE_OPTIONS "-Wno-error=array-bounds;-Wno-error=unused-but-set-variable"
)

target_compile_definitions(${MICROPY_TARGET} PRIVATE
    FFCONF_H=\"${MICROPY_OOFATFS_DIR}/ffconf.h\"
    LFS1_NO_MALLOC LFS1_NO_DEBUG LFS1_NO_WARN LFS1_NO_ERROR LFS1_NO_ASSERT
    LFS2_NO_MALLOC LFS2_NO_DEBUG LFS2_NO_WARN LFS2_NO_ERROR LFS2_NO_ASSERT
    PICO_FLOAT_PROPAGATE_NANS=1
    PICO_STACK_SIZE=0x2000
    PICO_CORE1_STACK_SIZE=0
    PICO_PROGRAM_NAME="MicroPython"
    PICO_NO_PROGRAM_VERSION_STRING=1 # do it ourselves in main.c
    MICROPY_BUILD_TYPE="${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION} ${CMAKE_BUILD_TYPE}"
    PICO_NO_BI_STDIO_UART=1 # we call it UART REPL
    PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
)

target_link_libraries(${MICROPY_TARGET}
    ${PICO_SDK_COMPONENTS}
)

if (MICROPY_HW_ENABLE_DOUBLE_TAP)
# Enable double tap reset into bootrom.
target_link_libraries(${MICROPY_TARGET}
    pico_bootsel_via_double_reset
)
endif()

# todo this is a bit brittle, but we want to move a few source files into RAM (which requires
#  a linker script modification) until we explicitly add  macro calls around the function
#  defs to move them into RAM.
if (PICO_ON_DEVICE AND NOT PICO_NO_FLASH AND NOT PICO_COPY_TO_RAM)
    pico_set_linker_script(${MICROPY_TARGET} ${CMAKE_CURRENT_LIST_DIR}/memmap_mp.ld)
endif()

pico_add_extra_outputs(${MICROPY_TARGET})

add_custom_command(TARGET ${MICROPY_TARGET}
    POST_BUILD
    COMMAND arm-none-eabi-size --format=berkeley ${PROJECT_BINARY_DIR}/${MICROPY_TARGET}.elf
    VERBATIM
)

# Collect all the include directories and compile definitions for the pico-sdk components.
foreach(comp ${PICO_SDK_COMPONENTS})
    micropy_gather_target_properties(${comp})
    micropy_gather_target_properties(${comp}_headers)
endforeach()

# Include the main MicroPython cmake rules.
include(${MICROPY_DIR}/py/mkrules.cmake)
