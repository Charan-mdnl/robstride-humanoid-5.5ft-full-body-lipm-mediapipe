#!/usr/bin/env python3
import sys
import os

sdk_path = "/home/me/RobStride_Control/python"
if sdk_path not in sys.path:
    sys.path.insert(0, sdk_path)

from robstride_dynamics import RobstrideBus

bus = RobstrideBus("slcan0")
print(bus.scan_channel())
