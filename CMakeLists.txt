cmake_minimum_required(VERSION 3.22)

# Single source of truth for the version is core/include/core/version.h (SW-10).
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/core/include/core/version.h" _tof_ver_lines
     REGEX "^#define TOF_VERSION_(MAJOR|MINOR|PATCH) ")
foreach(_part MAJOR MINOR PATCH)
    string(REGEX MATCH "TOF_VERSION_${_part} +([0-9]+)" _ "${_tof_ver_lines}")
    set(_tof_ver_${_part} "${CMAKE_MATCH_1}")
endforeach()

project(tof_avoid
    VERSION ${_tof_ver_MAJOR}.${_tof_ver_MINOR}.${_tof_ver_PATCH}
    LANGUAGES C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(TofCompilerFlags)

option(TOF_COVERAGE "Instrument host build for gcov/gcovr" OFF)
if(CMAKE_CROSSCOMPILING)
    set(_tof_tests_default OFF)
else()
    set(_tof_tests_default ON)
endif()
option(TOF_BUILD_TESTS "Build host unit tests" ${_tof_tests_default})

# Git hash is captured at configure time; reconfigure to refresh it.
set(TOF_GIT_HASH "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=8 HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE _tof_hash
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _tof_hash_rc
        ERROR_QUIET)
    if(_tof_hash_rc EQUAL 0 AND _tof_hash)
        set(TOF_GIT_HASH "${_tof_hash}")
    endif()
endif()
message(STATUS "tof_avoid ${PROJECT_VERSION} (${TOF_GIT_HASH})")

if(TOF_COVERAGE)
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "TOF_COVERAGE is host-only")
    endif()
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

# MAVLink headers (ardupilotmega dialect), unchanged third-party code (SW-04).
set(TOF_MAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink/c_library_v2")
if(NOT EXISTS "${TOF_MAVLINK_DIR}/ardupilotmega/mavlink.h")
    message(FATAL_ERROR "MAVLink submodule missing: run 'git submodule update --init --recursive'")
endif()
add_library(mavlink_v2 INTERFACE)
target_include_directories(mavlink_v2 SYSTEM INTERFACE "${TOF_MAVLINK_DIR}")

add_subdirectory(core)

if(TOF_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/host)
endif()

# Software-in-the-loop module for SITL (E3): Linux host only.
if(NOT CMAKE_CROSSCOMPILING AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    add_subdirectory(tools/emulator/tof_sil)
endif()

if(CMAKE_CROSSCOMPILING)
    get_filename_component(_tof_tool_dir "${CMAKE_C_COMPILER}" DIRECTORY)
    find_program(TOF_ARM_SIZE arm-none-eabi-size HINTS "${_tof_tool_dir}" REQUIRED)
    # Always out of date: prints size and checks FPU code on every build.
    add_custom_target(core_size
        COMMAND "${CMAKE_COMMAND}"
            -DLIB=$<TARGET_FILE:tof_core>
            -DSIZE=${TOF_ARM_SIZE}
            -DOBJDUMP=${CMAKE_OBJDUMP}
            -DNM=${CMAKE_NM}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_target_lib.cmake"
        DEPENDS tof_core
        VERBATIM)
endif()
