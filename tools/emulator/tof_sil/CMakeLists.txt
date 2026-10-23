# tof_sil: core/ + simulated board for SITL (E3). Host (Linux) only.
add_executable(tof_sil main.c bsp_sim.c)
target_link_libraries(tof_sil PRIVATE tof_core)
target_compile_definitions(tof_sil PRIVATE _DEFAULT_SOURCE _POSIX_C_SOURCE=200809L)
tof_apply_warnings(tof_sil)
