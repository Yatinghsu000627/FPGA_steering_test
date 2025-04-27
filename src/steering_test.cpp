#include "steering_test.hpp"

void write_steering_(uint32_t value, NiFpga_Bool direction, uint16_t speed)
{
    std::cout << green << "Send steering cmd!" << reset << std::endl;
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, direction));
    NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
    steer_vol = speed; 
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, true));
    while(position != (value + zero_offset)){
        NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
        NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
    }
    steer_vol = 0;
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, false));
    std::cout << green << "Finished!" << reset << std::endl;
}


int main(int argc, char* argv[])
{   

    std::string input;
    while (true) {
        std::cout << red << "Send cmd or exit: " << reset;
        std::cin >> input;
        if (input == "0"){
            status_ = NiFpga_Initialize();
            enable_btn_ =  NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_EN;
            hall = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Hall_effect;
            w_pb_digital_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Digital;
            w_pb_signal_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Signal;
            w_pb_power_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Power;
            w_vicon_trigger = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Conn9_2w;
            voltage = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16_input_voltage;
            dir = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_DIR;
            encoder = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_EncoderPosition;
            std::cout << "Set up the system" << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile,NiFpga_FPGA_CANBus_4module_v3_steering_Signature, "RIO0", 0, &session_));
            std::cout << "[FPGA Handler] Session opened"<< std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
            std::cout << "[FPGA Handler] IRQ reserved"<< std::endl;
        }
        else if (input == "1") 
        {
            std::cout << "Digital on" << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, true));
            sleep(1);
            std::cout << "Signal on" << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, true));
            sleep(1);
            std::cout << "Power on" << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, true));
            sleep(1);
        }
        else if (input == "2")
        {
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, true));
            std::cout << "steering is Enabled" << std::endl;
        }
        else if (input =="3"){
            std::cout << "Read Hall effect" << std::endl;
            for (size_t i = 0; i < 100; i++)
            {
                NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
                std::cout << std::to_string(complete) << std::endl;
            }
        }
        else if (input == "4"){
            std::cout << "Encoder test" << std::endl;
            for (size_t i = 0; i < 3000; i++)
            {                               
                steer_vol = 4000; 
                NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
                NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
                std::cout << green << std::to_string(position) << reset ;
                NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
                std::cout << yellow << std::to_string(complete) << reset << std::endl;
            }
        }
        else if (input == "5")
        {
            steer_vol = 3000; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            sleep(1);
            steer_vol = 0; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
        }
        else if (input =="dir1"){
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, true));
        }
        else if (input =="dir0"){
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, false));
        }
        else if (input =="trigger1"){
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_vicon_trigger, true));
        }
        else if (input =="trigger0"){
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_vicon_trigger, false));
        }
        else if (input == "cali"){
            std::cout << " Start calibration" << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, true));

            std::cout << yellow << "Right turn" << reset << std::endl;
            // dir = 0 (right turn)
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, false));
            NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
            steer_vol = 4000; 
            while(complete == true){
                NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
                NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
                NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
            }
            steer_vol = 0; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            // store r_hall
            std::cout << green << "store r_hall" << reset << std::endl;
            r_hall = position;

            std::cout << yellow << "Left turn" << reset << std::endl;
            // dir = 1 (left turn)
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, true));
            steer_vol = 4000; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            usleep(1000*500);
            NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
            while(complete == true){
                NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
                NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &complete));
                NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
            }
            steer_vol = 0; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            // store l_hall
            std::cout << green << "store l_hall" << reset << std::endl;
            l_hall = position;

            // turn back to set zero and initialplace
            std::cout << yellow << "Set Zero" << reset << std::endl;
            zero_offset = (r_hall+l_hall)/2;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, false));
            while(position!= zero_offset){
                NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
                steer_vol = 4000; 
                NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            }
            
            steer_vol = 0; 
            NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, steer_vol));
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, false));
            std::cout << green << "Done!" << reset << std::endl;
        }
        else if(input == "cmd"){
            // position (calibrated), direction, speed
            write_steering_(0, 1, 3000);
        }
        else if (input =="test"){
            std::cout << green << "green" << reset << std::endl;
            std::cout << yellow << "yellow" << reset << std::endl;
            std::cout << red << "red" << reset << std::endl;
        }
        else if (input == "exit") {
            std::cout << "Exiting..." << std::endl;

            std::cout << yellow << "Digital off" << reset << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, false));
            sleep(1);
            std::cout << yellow << "Signal off" << reset << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, false));
            sleep(1);
            std::cout << yellow << "Power off" << reset << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, false));
            sleep(1);

            /* unreserve IRQ status to prevent memory leaks */
            NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));
            /* Close the session */
            NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
            std::cout << "[FPGA Handler] Session Closed"<< std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_Finalize());
            std::cout << "[FPGA Handler] Fpga Finalized"<< std::endl;
            break;
        }
        else
        {
            std::cout << "Invalid input. Try again." << std::endl;
        }
    }

    return 0;
}
