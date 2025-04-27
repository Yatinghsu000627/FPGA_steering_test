#ifndef __STEERINGTEST_H
#define __STEERINGTEST_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "NiFpga.h"
#include "NiFpga_FPGA_CANBus_4module_v3_steering.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool enable_btn_;
NiFpga_Status status_;
NiFpga_Session session_;
NiFpga_IrqContext irqContext_;
NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool hall;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool w_pb_digital_;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool w_pb_signal_;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool w_pb_power_;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool w_vicon_trigger;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16 voltage;
NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool dir;
NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32 encoder;
uint16_t steer_vol = 0;
int32_t position = 0;
int32_t r_hall = 0;
int32_t l_hall = 0;
int32_t zero_offset = 0;
NiFpga_Bool complete = 0;
void write_steering_(uint32_t value);

#endif
