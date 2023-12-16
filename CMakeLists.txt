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

if(${USE_BIT_BANGING} EQUAL 1 OR ${USE_BIT_BANGING} EQUAL ON )
	add_definitions(-DUSE_BIT_BANGING=1)
    target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_interp)
else()
    # Otherwise use PIOs
    add_definitions(-DUSE_BIT_BANGING=0)
    pico_generate_pio_header(${PROJECT}  ${CMAKE_CURRENT_LIST_DIR}/ili9341_lcd.pio)
    target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_pio hardware_interp)
endif()


# set(CMAKE_VERBOSE_MAKEFILE ON)


target_sources(${PROJECT} PRIVATE ili9341_lcd.c)

pico_add_extra_outputs(${PROJECT})

# add url via pico_set_program_url
# example_auto_set_url(${PROJECT})
