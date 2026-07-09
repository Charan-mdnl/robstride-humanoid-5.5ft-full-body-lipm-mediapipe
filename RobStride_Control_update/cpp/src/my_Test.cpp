#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <time.h>
#include <climits>
#include <cmath>
#include <algorithm>
#include <sys/socket.h>
#include <sys/time.h>
#include <cstdint>
int main(int argc, char* argv[])
{
    int can_fd = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    struct sockaddr_can addr;
    struct ifreq ifr;
    addr.can_family= AF_CAN;
    strcpy(ifr.ifr_name,"can0");
    ioctl(can_fd,SIOCGIFINDEX,&ifr);
    addr.can_ifindex= ifr.ifr_ifindex;
    bind(can_fd,(struct sockaddr *)&addr,sizeof(addr));
    

    
     
    // frame.data[0]=0x11;
    // frame.data[1]=0x22;
    // frame.data[2]=0x33;
    // frame.data[3]=0x44;
    // frame.data[4]=0x55;
    // frame.data[5]=0x66;
    // frame.data[6]=0x77;
    // frame.data[7]=0x88;
    int N = 1000;
    timespec t1,t2;
    long max = 0, min = LONG_MAX, sum = 0;
    struct can_frame tx{}, rx{},tx_send{},tx_enable{};           // zero-init all
     
    tx.can_id  = (0x12u<<24) | (0x00<<8) | 0x0A | CAN_EFF_FLAG;   // type 18: write run_mode
    tx.can_dlc = 8;
    tx.data[0] = 0x05; tx.data[1] = 0x70;                        // index 0x7005 (little-endian)
    tx.data[4] = 0x00;                                           // value 0 = operation mode
    tx_enable.can_id  = (0x03u<<24) | (0x00<<8) | 0x0A | CAN_EFF_FLAG;  // type 3: enable
    tx_enable.can_dlc = 8;
    double angle = 0.5; // rad
    uint16_t angle_u = (angle + 4*M_PI)/(4*M_PI + 4*M_PI)*65535;
    double torque = 0.0;
    uint16_t torque_u = (torque + 120.0)/240.0*65535;
    double vel = 0.0;
    uint16_t vel_u = (vel + 15.0)/30.0*65535;
    double Kp = 10.0;
    uint16_t Kp_u = (Kp/5000.0)*65535;
    double Kd = 0.1;
    uint16_t Kd_u = (Kd/100.0)*65535;
    tx_send.can_id = (0x01u<<24) | (torque_u<<8) | 0x0A | CAN_EFF_FLAG;   // type 1: control
    tx_send.can_dlc = 8;
    tx_send.data[0]= (angle_u>>8)& 0xFF;
    tx_send.data[1]= (angle_u)& 0XFF;
    tx_send.data[2] = (vel_u>>8);
    tx_send.data[3] = vel_u;
    tx_send.data[4] = (Kp_u>>8);
    tx_send.data[5] = Kp_u;
    tx_send.data[6] = (Kd_u>>8);
    tx_send.data[7] = Kd_u;
    write(can_fd, &tx, sizeof(tx));               // set run_mode = operation
    read(can_fd, &rx, sizeof(rx));
    write(can_fd, &tx_enable, sizeof(tx_enable)); // enable
    read(can_fd, &rx, sizeof(rx));
    for (int i = 0; i < N; i++) {
    clock_gettime(CLOCK_MONOTONIC, &t1);
    write(can_fd, &tx_send, sizeof(tx_send));
    read(can_fd, &rx, sizeof(rx));      // reply lands in rx, tx untouched
    clock_gettime(CLOCK_MONOTONIC, &t2);
        long ns = (t2.tv_sec-t1.tv_sec)*1e9 +(t2.tv_nsec - t1.tv_nsec);
        max = std::max(max, ns);
        min = std::min(min, ns);
        sum += ns;
    }
    std::cout << "Average round-trip time: " << (sum / N)/1e6 << " ms\n";
    std::cout << "Min round-trip time: " << min/1e6 << " ms\n";
    std::cout << "Max round-trip time: " << max/1e6 << " ms\n";
    std::cout << "jitter time : " << (max - min)/1e6 << "ms\n";
    if(can_fd < 0)
    {
        perror("socket");
        return 1;
    }



    return 0;
}