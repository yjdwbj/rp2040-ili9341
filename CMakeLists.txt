cmake_minimum_required(VERSION 3.12)

set(PROJECT pico_74hc595_ili9341_lcd)
add_executable(${PROJECT})
#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(${PROJECT} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
set(CMAKE_VERBOSE_MAKEFILE ON)

target_include_directories(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

if(USE_BIT_BANGING AND (USE_PIO OR USE_SPI) OR
 (USE_PIO AND (USE_BIT_BANGING OR USE_SPI)) OR
 (USE_SPI AND (USE_BIT_BANGING OR USE_PIO)) )
    message(FATAL_ERROR "You must select only of these USE_BIT_BANGING USE_PIO USE_SPI, ${CMAKE_CURRENT_BINARY_DIR}")
endif()

if(NOT USE_BIT_BANGING AND NOT USE_PIO AND NOT USE_SPI)
    message(FATAL_ERROR "You must select only of these USE_BIT_BANGING USE_PIO USE_SPI, ${CMAKE_CURRENT_BINARY_DIR}")
endif()

if(USE_BIT_BANGING)
	add_definitions(-DUSE_BIT_BANGING=1)
    target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_interp)
endif()

if(USE_SPI)
    # Otherwise use PIOs
    add_definitions(-DUSE_SPI=1)
    pico_generate_pio_header(${PROJECT}  ${CMAKE_CURRENT_LIST_DIR}/ili9341_lcd.pio)
    target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_interp hardware_spi)
endif()


if(USE_PIO)
    # Otherwise use PIOs
    add_definitions(-DUSE_PIO=1)
    pico_generate_pio_header(${PROJECT}  ${CMAKE_CURRENT_LIST_DIR}/ili9341_lcd.pio)
    target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_pio hardware_interp)
endif()

if (USE_LVGL)
	add_definitions(-DUSE_LVGL=1)
    target_link_libraries(${PROJECT} PRIVATE
            pico_stdlib
            hardware_pio
            hardware_interp
            hardware_spi
            lvgl
            lvgl_examples
    )
    target_sources(${PROJECT} PRIVATE
            main.c
            ili9341.c
            xpt2046.c
            spi_rp2040.c
    )
    add_subdirectory(lvgl)
else()
    target_sources(${PROJECT} PRIVATE main.c ili9341.c)
endif()

if (USE_SPI)
    target_sources(${PROJECT} PRIVATE spi_rp2040.c xpt2046.c)  # append by condition.
endif()

pico_add_extra_outputs(${PROJECT})

# add url via pico_set_program_url
# example_auto_set_url(${PROJECT})
