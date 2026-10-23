add_library(tof_core STATIC
    src/version.c
    src/crc32.c
    src/params.c
    src/geometry.c
    src/quality.c
    src/cell_fsm.c
    src/sensor_health.c
    src/ground.c
    src/phase.c
    src/mavlink_rx.c
    src/mavlink_tx.c
    src/fusion.c
    src/tof_core.c)

target_include_directories(tof_core PUBLIC include)
# Generated MAVLink headers (third_party, unchanged, SW-04); part of the mavlink_rx/tx interface.
target_link_libraries(tof_core PUBLIC mavlink_v2)
target_compile_definitions(tof_core PRIVATE TOF_GIT_HASH="${TOF_GIT_HASH}")
tof_apply_warnings(tof_core)

if(NOT CMAKE_CROSSCOMPILING)
    # Single-precision libm functions (fmodf etc.) on the host.
    target_link_libraries(tof_core PUBLIC m)
endif()
