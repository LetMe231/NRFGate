cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_data_handler)

get_filename_component(SRC_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src ABSOLUTE)
get_filename_component(INC_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc ABSOLUTE)
get_filename_component(COMMON_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../common ABSOLUTE)

target_sources(app PRIVATE
    src/main.c
    ${SRC_DIR}/data_handler.c
)

target_include_directories(app PRIVATE
    ${COMMON_DIR}
    ${SRC_DIR}
    ${INC_DIR}
)

target_compile_options(app PRIVATE
    -include ${COMMON_DIR}/test_stubs.h
    -Wno-format-truncation
)