#include "can_interfaces.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <string>

/*
 * can_interfaces.cpp — implement the CanInterface class yourself.
 *
 * WHAT YOU NEED TO KNOW
 * =====================
 *
 * 1. FILE DESCRIPTORS
 *    Linux represents open resources (files, sockets, devices) as integers
 *    called file descriptors (fd). -1 means "not open". Valid fds are >= 0.
 *
 * 2. OPENING A CAN SOCKET
 *    socket(PF_CAN, SOCK_RAW, CAN_RAW)   → ask kernel for a raw CAN socket
 *                                           returns fd on success, -1 on error
 *
 * 3. FINDING THE INTERFACE INDEX
 *    The kernel identifies network interfaces by integer index, not name.
 *    To get the index for "can0":
 *      struct ifreq ifr;
 *      strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
 *      ioctl(fd, SIOCGIFINDEX, &ifr);   → fills ifr.ifr_ifindex
 *
 * 4. BINDING THE SOCKET TO THE INTERFACE
 *    struct sockaddr_can addr;
 *    addr.can_family  = AF_CAN;
 *    addr.can_ifindex = ifr.ifr_ifindex;
 *    bind(fd, (struct sockaddr*)&addr, sizeof(addr));
 *
 * 5. SENDING A FRAME
 *    Fill a struct can_frame:
 *      frame.can_id  = your_id | CAN_EFF_FLAG;  // CAN_EFF_FLAG = 29-bit mode
 *      frame.can_dlc = number of data bytes;
 *      memcpy(frame.data, data, dlc);
 *    Then: write(fd, &frame, sizeof(struct can_frame))
 *
 * 6. RECEIVING WITH TIMEOUT (select)
 *    select() sleeps until data arrives OR timeout expires:
 *      fd_set read_fds;
 *      FD_ZERO(&read_fds);
 *      FD_SET(fd, &read_fds);
 *      struct timeval timeout;
 *      timeout.tv_sec  = ms / 1000;
 *      timeout.tv_usec = (ms % 1000) * 1000;
 *      int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
 *    If ret > 0: data arrived → read(fd, frame, sizeof(struct can_frame))
 *    If ret == 0: timeout, nothing arrived
 *    If ret < 0: error
 *
 * 7. ERROR HANDLING
 *    perror("tag") prints the system error message to stderr.
 *    Always close(fd) and set fd = -1 before returning false on error.
 */

// ---- TASK 1 ------------------------------------------------
// Constructor: initialise socket_fd_ to -1 (not open yet)
// Destructor:  if socket_fd_ >= 0, close it
//              (use ::close to call the global close, not a member)

CanInterface::CanInterface() : socket_fd_(-1) {}

CanInterface::~CanInterface()
{
    // YOUR CODE HEREi
    if(socket_fd_>=0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

// ---- TASK 2 ------------------------------------------------
bool CanInterface::init(const std::string& interface)
{
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_fd_ < 0)
    {
        perror("socket");
        return false;
    }
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ-1);
    if(ioctl(socket_fd_, SIOCGIFINDEX, &ifr)<0)
    {
        perror("ioctl");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr))<0)
    {
        perror("bind");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    interface_name_ = interface;
    return true;
}
// Steps:
//   a. Call socket(PF_CAN, SOCK_RAW, CAN_RAW), store in socket_fd_
//      If < 0: perror, return false
//   b. Declare struct ifreq ifr, copy interface name in with strncpy
//   c. Call ioctl(socket_fd_, SIOCGIFINDEX, &ifr)
//      If < 0: perror, close socket, set socket_fd_=-1, return false
//   d. Declare struct sockaddr_can addr, set can_family and can_ifindex
//   e. Call bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr))
//      If < 0: perror, close socket, set socket_fd_=-1, return false
//   f. Store interface_name_ = interface, return true



// ---- TASK 3 ------------------------------------------------
// void write_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
// Steps:
//   a. Declare struct can_frame frame
//   b. Set frame.can_id  = can_id | CAN_EFF_FLAG
//   c. Set frame.can_dlc = dlc
//   d. If data != nullptr and dlc > 0: memcpy(frame.data, data, dlc)
//   e. Call ::write(socket_fd_, &frame, sizeof(struct can_frame))
//      If < 0: perror("write")
// static int counter = 0;

void CanInterface::write_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    if(data != nullptr && dlc > 0)
    {
        memcpy(frame.data, data,dlc);

    }
    if(::write(socket_fd_, &frame, sizeof(struct can_frame)) < 0)
    {
        perror("write");
    }

   // std::cout << "Writing frame: " << frame.can_id << std::endl;
}

void CanInterface::write_std_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id & CAN_SFF_MASK;  // standard 11-bit, no EFF flag
    frame.can_dlc = dlc;
    if (data != nullptr && dlc > 0)
        memcpy(frame.data, data, dlc);
    if (::write(socket_fd_, &frame, sizeof(struct can_frame)) < 0)
        perror("write_std_frame");
}

// ---- TASK 4 ------------------------------------------------
// bool read_frame(struct can_frame* frame, int timeout_ms)
// Steps:
//   a. Declare fd_set read_fds, call FD_ZERO and FD_SET(socket_fd_)
//   b. Declare struct timeval timeout
//      timeout.tv_sec  = timeout_ms / 1000
//      timeout.tv_usec = (timeout_ms % 1000) * 1000
//   c. Call select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout)
//   d. If ret > 0 and FD_ISSET(socket_fd_, &read_fds):
//        call ::read(socket_fd_, frame, sizeof(struct can_frame))
//        if read < 0: perror, return false
//        return true
//   e. return false  (timeout or error)

bool CanInterface::read_frame(struct can_frame* frame, int timeout_ms)
{
    // YOUR CODE HERE
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    int ret = select(socket_fd_ + 1, &read_fds,nullptr,nullptr, &timeout);
    if(ret >0 && FD_ISSET(socket_fd_, &read_fds))
    {
        if(::read(socket_fd_, frame, sizeof(struct can_frame)) < 0)
        {
            perror("read");
            return false;
        }
        return true;
    }
    return false;
}
