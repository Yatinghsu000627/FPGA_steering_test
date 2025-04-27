add_executable(steering_test steering_test.cpp
                            NiFpga_FPGA_CANBus_4module_v3_steering.c
                            NiFpga.c                       
)
target_link_libraries(steering_test ${CMAKE_DL_LIBS} )

add_executable(pid_test PID_testing.cpp
                            NiFpga_FPGA_CANBus_4module_v3_steering.c
                            NiFpga.c                       
)
target_link_libraries(pid_test ${CMAKE_DL_LIBS} )
