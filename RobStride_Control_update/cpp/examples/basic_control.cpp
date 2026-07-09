/*
 * RobStride C++ Basic Control Example
 * Demonstrates basic motor control operations
 */

#include "../include/can_interface.h"
#include "../include/protocol.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char* argv[]) {
    int motor_id = 11;
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }

    std::cout << "🎯 RobStride C++ Basic Control Example" << std::endl;
    std::cout << "Using Motor ID: " << motor_id << std::endl;

    CanInterface can;

    // Initialize CAN
    if (!can.init("can0")) {
        std::cerr << "❌ Failed to initialize CAN interface" << std::endl;
        return 1;
    }

    std::cout << "✅ CAN interface initialized" << std::endl;

    try {
        // Enable motor
        std::cout << "⚡ Enabling motor..." << std::endl;
        enable_motor(can.socket(), motor_id);
        usleep(500000);

        // Set MIT mode
        std::cout << "⚙️ Setting MIT mode..." << std::endl;
        set_mode_raw(can.socket(), motor_id, ControlMode::MIT_MODE);
        usleep(200000);

        // Set limits
        std::cout << "📊 Setting limits..." << std::endl;
        write_limit(can.socket(), motor_id, ParamID::VELOCITY_LIMIT, 20.0);
        write_limit(can.socket(), motor_id, ParamID::TORQUE_LIMIT, 20.0);

        // Test movements
        std::cout << "🎮 Starting test movements..." << std::endl;

        double movements[] = {0.0, M_PI/2, -M_PI/2, 0.0};  // 0°, 90°, -90°, 0°
        const char* descriptions[] = {"Home", "90°", "-90°", "Home"};

        for (int i = 0; i < 4; i++) {
            std::cout << "📍 Moving to " << descriptions[i] << std::endl;
            write_operation_frame(can.socket(), motor_id, movements[i], 30.0, 0.5);
            sleep(2);  // Wait for movement
        }

        std::cout << "✅ Test completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}