set(TOF_UNITY_DIR "${PROJECT_SOURCE_DIR}/third_party/unity")
if(NOT EXISTS "${TOF_UNITY_DIR}/src/unity.c")
    message(FATAL_ERROR "Unity submodule missing: run 'git submodule update --init --recursive'")
endif()

# Unity is third-party: built without the project's -Werror policy.
add_library(unity STATIC "${TOF_UNITY_DIR}/src/unity.c")
target_include_directories(unity SYSTEM PUBLIC "${TOF_UNITY_DIR}/src")

# One executable per core module; the CTest name matches the executable.
function(tof_add_host_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE tof_core unity)
    tof_apply_test_warnings(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

tof_add_host_test(test_version test_version.c)
tof_add_host_test(test_geometry test_geometry.c)
tof_add_host_test(test_params test_params.c mock_flash.c)
tof_add_host_test(test_quality test_quality.c)
tof_add_host_test(test_cell_fsm test_cell_fsm.c)
tof_add_host_test(test_sensor_health test_sensor_health.c)
tof_add_host_test(test_fusion test_fusion.c)
tof_add_host_test(test_tof_core test_tof_core.c)
tof_add_host_test(test_ground test_ground.c)
tof_add_host_test(test_phase test_phase.c)

# MAP-07a: params built with GROUND_POLICY = DROP allowed (a separate copy, not the tof_core library).
add_executable(test_params_drop test_params_drop.c
    "${PROJECT_SOURCE_DIR}/core/src/params.c"
    "${PROJECT_SOURCE_DIR}/core/src/crc32.c")
target_include_directories(test_params_drop PRIVATE "${PROJECT_SOURCE_DIR}/core/include")
target_compile_definitions(test_params_drop PRIVATE TOF_ALLOW_GROUND_DROP=1u)
target_link_libraries(test_params_drop PRIVATE unity)
tof_apply_test_warnings(test_params_drop)
add_test(NAME test_params_drop COMMAND test_params_drop)

# MAP-07a: export policy with DROP allowed (same build flag, separate copy of phase.c).
add_executable(test_phase_drop test_phase_drop.c
    "${PROJECT_SOURCE_DIR}/core/src/phase.c"
    "${PROJECT_SOURCE_DIR}/core/src/params.c"
    "${PROJECT_SOURCE_DIR}/core/src/crc32.c")
target_include_directories(test_phase_drop PRIVATE "${PROJECT_SOURCE_DIR}/core/include")
target_compile_definitions(test_phase_drop PRIVATE TOF_ALLOW_GROUND_DROP=1u)
target_link_libraries(test_phase_drop PRIVATE unity)
tof_apply_test_warnings(test_phase_drop)
add_test(NAME test_phase_drop COMMAND test_phase_drop)

# SW-07a: docs/params.md is generated from the parameter table and must stay in sync.
add_executable(params_md "${PROJECT_SOURCE_DIR}/tools/params_md/params_md.c")
target_link_libraries(params_md PRIVATE tof_core)
tof_apply_test_warnings(params_md)
add_test(NAME test_SW07a_params_md_up_to_date
    COMMAND params_md --check "${PROJECT_SOURCE_DIR}/docs/params.md")
