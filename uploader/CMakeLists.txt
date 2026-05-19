cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sp1-uploader)

set(BSP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../subsys/system/ResetBreadcrumbs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../subsys/system/Watchdog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../subsys/power/PowerManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/charger/charger_bq24232.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/audio/cs42l42_codec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/audio/tas2505_codec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../storagethingies-2026-05-09/EmmcDriver.cpp
)

set(APP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/Leds.cpp
)

target_sources(app PRIVATE ${BSP_SOURCES} ${APP_SOURCES})
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../app
    ${CMAKE_CURRENT_SOURCE_DIR}/../subsys
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../storagethingies-2026-05-09
    ${ZEPHYR_HAL_NORDIC_MODULE_DIR}/nrfx/bsp/stable/mdk
)

target_compile_features(app PRIVATE cxx_std_17)
target_compile_options(app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)
