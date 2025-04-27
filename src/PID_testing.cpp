// steering_test.cpp
#include "steering_test.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// PID 參數（請依實際系統調整）
static constexpr double Kp = 0.8;
static constexpr double Ki = 0.02;
static constexpr double Kd = 0.1;

// 控制迴圈時間間隔 (秒)
static constexpr double DT = 0.01;

// 位置允許誤差 (encoder counts)
static constexpr int POSITION_TOLERANCE = 5;

/**
 * @brief 用 PID 閉環控制讓舵機轉到目標位置
 * @param target_value 目標 encoder 讀值（+ zero_offset）
 */
void write_steering_PID(uint32_t target_value)
{
    // 啟用馬達驅動
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, true));

    double integral   = 0.0;
    double last_error = 0.0;
    int32_t goal      = static_cast<int32_t>(target_value + zero_offset);

    while (true) {
        // 1) 讀取目前位置
        NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));

        // 2) 計算誤差
        double error = static_cast<double>(goal - position);
        if (std::abs(error) <= POSITION_TOLERANCE) {
            // 誤差夠小，結束迴圈
            break;
        }

        // 3) 積分（可加入上／下限防飽和）
        integral += error * DT;
        // integral = std::clamp(integral, -10000.0, 10000.0);

        // 4) 微分
        double derivative = (error - last_error) / DT;
        last_error = error;

        // 5) PID 輸出
        double u = Kp * error + Ki * integral + Kd * derivative;

        // 6) 方向與 PWM 限幅
        NiFpga_Bool dir_flag = (u >= 0.0) ? true : false;
        uint16_t pwm = static_cast<uint16_t>(std::min(std::abs(u), 4095.0));

        // 7) 寫入 FPGA
        NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, dir_flag));
        NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, pwm));

        // 8) 固定迴圈速率
        std::this_thread::sleep_for(std::chrono::duration<double>(DT));
    }

    // 停止並關閉馬達
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, 0));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, false));
    std::cout << green << "Steering reached position: " << position << reset << std::endl;
}

int main(int argc, char* argv[])
{
    std::string input;
    while (true) {
        std::cout << red << "Send cmd or exit: " << reset;
        std::cin >> input;

        if (input == "0") {
            // FPGA 初始化
            status_        = NiFpga_Initialize();
            enable_btn_    = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_EN;
            hall           = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Hall_effect;
            w_pb_digital_  = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Digital;
            w_pb_signal_   = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Signal;
            w_pb_power_    = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Power;
            w_vicon_trigger= NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Conn9_2w;
            voltage        = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16_input_voltage;
            dir            = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_DIR;
            encoder        = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_EncoderPosition;

            std::cout << "Setting up FPGA session..." << std::endl;
            NiFpga_MergeStatus(&status_, NiFpga_Open(
                NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile,
                NiFpga_FPGA_CANBus_4module_v3_steering_Signature,
                "RIO0", 0, &session_));
            NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
            std::cout << "[FPGA] Initialized and IRQ reserved" << std::endl;
        }
        else if (input == "1") {
            // ... (保留你原本的數位/信號/電源開啟程式碼)
        }
        else if (input == "2") {
            // ... (保留原本使能程式碼)
        }
        else if (input == "3") {
            // ... (保留 Hall effect 測試程式碼)
        }
        else if (input == "4") {
            // ... (保留原本的 Encoder 測試程式碼)
        }
        else if (input == "5") {
            // ... (保留原本的單段輸出測試)
        }
        else if (input == "cali") {
            // ... (保留原本的校正流程)
        }
        else if (input == "cmd") {
            // 呼叫 PID 控制版本
            write_steering_PID(0 /* 這裡填入你要的 target encoder value */);
        }
        else if (input == "exit") {
            // 關閉流程
            std::cout << yellow << "Shutting down..." << reset << std::endl;
            // 關閉數位/信號/電源
            // ...
            NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));
            NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
            NiFpga_MergeStatus(&status_, NiFpga_Finalize());
            break;
        }
        else {
            std::cout << "Invalid input. Try again." << std::endl;
        }
    }
    return 0;
}
