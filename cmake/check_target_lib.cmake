# Script mode: cmake -DLIB=... -DSIZE=... -DOBJDUMP=... -DNM=... -P check_target_lib.cmake
# Prints the size of the core library and checks that float code uses the FPU
# and that no double-precision soft-float helpers are referenced (SW-02).

foreach(_var LIB SIZE OBJDUMP NM)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "${_var} is not set")
    endif()
endforeach()

execute_process(COMMAND "${SIZE}" -t "${LIB}" RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "arm-none-eabi-size failed")
endif()

execute_process(COMMAND "${OBJDUMP}" -d "${LIB}" OUTPUT_VARIABLE _disasm RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "objdump failed")
endif()
string(REGEX MATCHALL "[\t ]v[a-z]+(\\.[a-z0-9]+)?\\.f32[\t ]" _fpu_insns "${_disasm}")
list(LENGTH _fpu_insns _fpu_count)
if(_fpu_count EQUAL 0)
    message(FATAL_ERROR "FPU check: no single-precision VFP instructions in ${LIB}")
endif()

execute_process(COMMAND "${NM}" -u "${LIB}" OUTPUT_VARIABLE _undef RESULT_VARIABLE _rc)
string(REGEX MATCHALL "__aeabi_(d[a-z0-9]+|[a-z0-9]+2d)" _dbl_helpers "${_undef}")
if(_dbl_helpers)
    list(REMOVE_DUPLICATES _dbl_helpers)
    message(FATAL_ERROR "FPU check: double-precision helpers referenced: ${_dbl_helpers}")
endif()

message("FPU check: ${_fpu_count} single-precision VFP instructions, no double-precision helpers")
