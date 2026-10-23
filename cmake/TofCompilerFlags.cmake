# Warning policy for own code (SW-09). third_party/ is compiled without these.

set(TOF_COMMON_WARNINGS
    -Wall
    -Wextra
    -Werror
    -Wshadow
    -Wvla
    -Wdouble-promotion
    -Wfloat-conversion
    -Wstrict-prototypes)

# Core and firmware code: every external function must have a prototype.
function(tof_apply_warnings target)
    target_compile_options(${target} PRIVATE ${TOF_COMMON_WARNINGS} -Wmissing-prototypes)
    set_target_properties(${target} PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF)
endfunction()

# Unity test files define test_* functions without prototypes.
function(tof_apply_test_warnings target)
    target_compile_options(${target} PRIVATE ${TOF_COMMON_WARNINGS})
    set_target_properties(${target} PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF)
endfunction()
