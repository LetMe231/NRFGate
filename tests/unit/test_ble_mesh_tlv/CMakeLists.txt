cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_ble_mesh_tlv)

get_filename_component(INC_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/../../../inc ABSOLUTE)
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common ABSOLUTE)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ${COMMON_DIR} ${INC_DIR} ${INC_DIR}/core)
target_compile_options(app PRIVATE -Wno-format-truncation)
