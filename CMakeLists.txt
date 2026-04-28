cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(NRFGateway)

target_sources(app PRIVATE
    src/main.c
    src/core/gw_store.c
    src/core/event_ingest.c
    src/core/semantic_handler.c
    src/core/command_router.c
    src/core/reliability_manager.c
    src/core/rule_engine.c
    src/core/nus_handler.c
    src/core/transport_port.c
    src/core/scheduler.c
    src/core/gw_addr.c
    src/transports/ble_mesh/ble_mesh_adapter.c
    src/transports/ble_mesh/ble_mesh_prov.c
    src/transports/ble_mesh/ble_mesh_codec.c
    src/transports/thread/thread_adapter.c
    src/transports/thread/thread_codec.c
    src/transports/lorawan/lorawan_adapter.c
    src/transports/lorawan/lora_wire_format.c
    src/drivers/ble_nus.c
    src/drivers/mesh_ctrl.c)

target_include_directories(app PRIVATE
    inc
    inc/core
    inc/transports/ble_mesh
    inc/transports/thread
    inc/transports/lorawan
    inc/drivers)