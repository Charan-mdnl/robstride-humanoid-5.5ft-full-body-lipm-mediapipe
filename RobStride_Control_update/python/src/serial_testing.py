#!/usr/bin/env python3
import time
from robstride_dynamics import RobstrideBus, Motor

channel = "slcan0"

motors = {
    "motor5": Motor(id=5, model="rs-02"),
}

bus = RobstrideBus(channel=channel, motors=motors, bitrate=1000000)

while True:
    bus.write_operation_frame(
        motor="motor5",
        position=0.0,
        kp=30.0,
        kd=0.5,
        velocity=0.0,
        torque=0.0,
    )
    time.sleep(0.01)
