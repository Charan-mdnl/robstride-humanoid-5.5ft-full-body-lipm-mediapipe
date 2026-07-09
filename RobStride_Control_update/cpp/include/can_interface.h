/*
 * CAN Interface for RobStride Motor Control
 * Low-level SocketCAN interface wrapper
 */

#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <cstdint>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>

class CanInterface {
public:
    CanInterface();
    ~CanInterface();

    // Initialize CAN interface
    bool init(const std::string& interface = "can0");

    // Close CAN interface
    void close();

    // Send CAN frame
    bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);

    // Receive CAN frame with timeout
    bool read_frame(struct can_frame* frame, int timeout_ms = 100);

    // Check if interface is ready
    bool is_ready() const { return socket_fd_ >= 0; }

private:
    int socket_fd_;
    std::string interface_name_;
};

#endif // CAN_INTERFACE_H