cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(NRFGateway)

target_sources(app PRIVATE 
    src/main.c
    src/model_handler.c
    src/thread_handler.c
    src/ble_nus.c
    src/lora_handler.c
    src/data_handler.c
    src/semantic_handler.c
    src/mesh_ctrl.c
    src/rule_engine.c
    src/mesh_parser.c
    src/wire_format.c)

target_include_directories(app PRIVATE inc)