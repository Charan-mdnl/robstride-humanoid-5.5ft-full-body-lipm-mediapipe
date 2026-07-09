#include <full_body_hardware/can_interfaces.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <string>
/*WHAT YOU NEED TO KNOW
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
// int main()
// {
//     int socket_fd;
//     int nbytes;

//     struct sockaddr_can can;
//     struct ifreq ifr;
//     struct can_frame frame;
//     socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (socket_fd < 0)
//     {
//         perror("Error opening socket");
//         return -1;
//     }
//     strcpy(ifr.ifr_name, "can0");
//     ioctl(socket_fd, SIOCGIFINDEX, &ifr);
//     can.can_family = AF_CAN;
//     can.can_ifindex = ifr.ifr_ifindex;
//     if(bind(socket_fd,(struct sockaddr*)&can,sizeof(can)<0))
//     {
//         perror("error in socket bind");
//         return -2;
//     }
//     while (1)
//     {
//         nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
//         if (nbytes < 0)
//         {
//             perror("read");
//         }
//         printf("Received can fram - id 0x%X Data", frame.can_id);
//         for(int i = 0; i<frame.can_dlc;i++)
//         {
//             printf("%02X", frame.data[i]);
//         }
//         printf("\n");
//         /* code */
//     }
    
// }

CanInterface::CanInterface() : socket_fd_(-1) {}

CanInterface::~CanInterface()
{
    // YOUR CODE HEREi
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanInterface::init(const std::string &interface = "can0");
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_fd <0)
    {
        perror("Failed to open the socket can ");
        return false;
    }
    struct ifreq ifr;
    struct can_frame frame;
    strncpy(ifr.ifr_name, "can0",IFNAMSIZ-1);
    ioctl(socket_fd_, SIOCGIFINDEX, &ifr);
    struct sockaddr_can can_addr;
    can_addr.can_family = AF_CAN;
    *can_addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(socket_fd, (struct sockaddr *)can_addr, sizeof(can_addr)<0))
    {
        
            perror("bind");
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        
    }
    interface_name_ = interface;
    return true;
}

void CanInterface::write_frame(uint32_t can_id,uint8_t *data,uint8_t dlc)
{

    struct can_frame frame;
    frame.can_id = can_id|CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    if( data != nullptr && dlc>0)
    {
    memcpy(frame.data, data, dlc);
    if(write(socket_fd,&frame,sizeof(frame)<0))
    {
        perror("write");
    }
}
}

bool CanInterface::read_frame(struct can_frame *frame, uint32_t timeout)
{

    fd_set read_fds;
    FD_ZERO(&read_fds);
    struct timeval timeout_i;
    timeout_i.tv_sec = timeout / 1000;
    timeout_i.tv_usec = (timeout % 1000) * 1000;
    int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout_i);
    if (ret > 0 && FD_ISSET(socket_fd_, &read_fds))
    {
        if(read(socket_fd,&frame,size_of(frame)<0))
        {   perror("failed to read")
            return false;
        }
        return true;
    }
    return false;
}

void close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}