cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_semantic_handler)
get_filename_component(SRC_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src ABSOLUTE)
get_filename_component(INC_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc ABSOLUTE)
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common ABSOLUTE)
target_sources(app PRIVATE
    src/main.c
    ${SRC_DIR}/core/semantic_handler.c
    ${SRC_DIR}/core/gw_store.c
)
target_include_directories(app PRIVATE
    ${COMMON_DIR} ${INC_DIR} ${INC_DIR}/core ${INC_DIR}/adapters ${INC_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(app PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/src/test_semantic_stubs.h
    -Wno-format-truncation)
