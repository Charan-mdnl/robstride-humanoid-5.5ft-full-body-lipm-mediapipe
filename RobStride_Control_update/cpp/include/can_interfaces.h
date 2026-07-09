#ifndef __CAN_INTERFACES_H__
#define __CAN_INTERFACES_H__

#include <cstdint>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include<linux/can/raw.h>

class CanInterface
{
// create an init function for can
public:
    CanInterface();
    ~CanInterface();

    bool init(const std::string &interface = "can0");
    void write_frame(uint32_t can_id,const uint8_t *data,uint8_t dlc);
    void write_std_frame(uint32_t can_id,const uint8_t *data,uint8_t dlc);
    bool read_frame(struct can_frame *frame,int timeout_ms=100);
    bool is_ready() const { return socket_fd_ >= 0; }
private:
    int socket_fd_;
    std::string interface_name_;
};
#endif // __CAN_INTERFACES_H__