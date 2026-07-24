cmake_minimum_required(VERSION 3.20)
project(wiznet_w5500_transport_tests C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(IOLIBRARY_ROOT "${CMAKE_SOURCE_DIR}/../.." CACHE PATH "Root W5500 ioLibrary directory")

function(add_transport_test TEST_NAME TEST_SRC)
    add_executable(${TEST_NAME}
        ${TEST_SRC}
        ${IOLIBRARY_ROOT}/Ethernet/W5500/w5500.c
        ${IOLIBRARY_ROOT}/Ethernet/wizchip_conf.c
        ${IOLIBRARY_ROOT}/Ethernet/socket.c
        ${CMAKE_SOURCE_DIR}/fakes/fake_pico_sdk.c
        ${ARGN}
    )
    target_include_directories(${TEST_NAME} PRIVATE
        ${IOLIBRARY_ROOT}/Ethernet
        ${IOLIBRARY_ROOT}/Ethernet/W5500
        ${IOLIBRARY_ROOT}/tests/support
        ${CMAKE_SOURCE_DIR}/fakes
        ${CMAKE_SOURCE_DIR}/../port/ioLibrary_Driver/inc
        ${CMAKE_SOURCE_DIR}/../port
    )
    target_compile_definitions(${TEST_NAME} PRIVATE
        _WIZCHIP_=5500
        _WIZCHIP_IO_MODE_=0x0203
    )
    target_compile_options(${TEST_NAME} PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

add_transport_test(
    test_wizchip_qspi_pio
    test_wizchip_qspi_pio.c
    ${CMAKE_SOURCE_DIR}/../port/ioLibrary_Driver/src/wizchip_qspi_pio.c)
add_transport_test(
    test_wizchip_spi
    test_wizchip_spi.c
    ${CMAKE_SOURCE_DIR}/../port/ioLibrary_Driver/src/wizchip_spi.c
    ${CMAKE_SOURCE_DIR}/../port/ioLibrary_Driver/src/wizchip_qspi_pio.c)
target_compile_definitions(test_wizchip_spi PRIVATE DEVICE_BOARD_NAME=3)
add_transport_test(
    test_wizchip_gpio_irq
    test_wizchip_gpio_irq.c
    ${IOLIBRARY_ROOT}/tests/support/w5500_spi_model.c
    ${CMAKE_SOURCE_DIR}/../port/ioLibrary_Driver/src/wizchip_gpio_irq.c)
