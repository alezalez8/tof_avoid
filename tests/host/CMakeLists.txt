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
