#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import threading
import signal
from typing import Optional

# Import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor
except ImportError:
    from bus import RobstrideBus, Motor


class MultiMotorMITSafe:
    def __init__(self, motor_ids, motor_models=None, channel='can0'):
        self.motor_ids = motor_ids
        self.motor_names = [f"motor_{i}" for i in motor_ids]
        self.channel = channel

        # ✅ Handle models
        if motor_models is None:
            motor_models = ["rs-02"] * len(motor_ids)

        self.motor_models = motor_models

        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()

        self.running = True
        self.motion_enabled = False
        self.user_input_active = False

        self.target_positions = {name: 0.0 for name in self.motor_names}

        self.kp = 4.5
        self.kd = 1.8

        # Anti-spam system
        self.last_print_time = 0
        self.last_line = ""
        self.print_interval = 0.2

    # 🔧 Visual angle bar (center = 0°)
    def angle_bar(self, angle_deg, width=20):
        angle_deg = max(-180, min(180, angle_deg))
        center = width // 2
        scale = (width / 2) / 180
        pos = int(center + angle_deg * scale)

        bar = ["-"] * width
        if 0 <= pos < width:
            bar[pos] = "|"

        return "[" + "".join(bar) + "]"

    def connect(self):
        print(f"🔌 Connecting to {self.channel}...")

        # ✅ Create motors with models
        motors = {
            name: Motor(id=mid, model=model)
            for name, mid, model in zip(
                self.motor_names, self.motor_ids, self.motor_models
            )
        }

        # Print configuration
        for name, model in zip(self.motor_names, self.motor_models):
            print(f"⚙️ {name} → model: {model}")

        calibration = {
            name: {"direction": 1, "homing_offset": 0.0}
            for name in self.motor_names
        }

        self.bus = RobstrideBus(self.channel, motors, calibration)
        self.bus.connect(handshake=True)

        with self.lock:
            for name in self.motor_names:
                print(f"⚡ Enabling {name}")
                self.bus.enable(name)
                time.sleep(0.2)

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

        print("\n🛑 SAFE MODE ACTIVE (no motion)")
        print("👉 Press ENTER to enable motion\n")

    def loop(self):
        while self.running:
            try:
                states = {}

                with self.lock:
                    for name in self.motor_names:
                        try:
                            if not self.motion_enabled:
                                # Safe mode → no torque
                                self.bus.write_operation_frame(
                                    name, 0.0, 0.0, 0.0, 0.0, 0.0
                                )
                            else:
                                self.bus.write_operation_frame(
                                    name,
                                    self.target_positions[name],
                                    self.kp,
                                    self.kd,
                                    0.0,
                                    0.0
                                )

                            state = self.bus.read_operation_frame(name)

                            if state:
                                states[name] = state

                        except Exception:
                            states[name] = None

                # Clean display
                if not self.user_input_active:
                    now = time.time()

                    if now - self.last_print_time >= self.print_interval:
                        line_parts = []

                        for name in self.motor_names:
                            state = states.get(name)

                            if state:
                                deg = math.degrees(state[0])
                                bar = self.angle_bar(deg)
                                line_parts.append(f"{name}:{deg:6.1f}° {bar}")
                            else:
                                line_parts.append(f"{name}:  ---   [NO DATA]")

                        line = " | ".join(line_parts)

                        if line != self.last_line:
                            print("\r" + line, end="", flush=True)
                            self.last_line = line

                        self.last_print_time = now

                time.sleep(0.02)

            except Exception as e:
                print(f"\n⚠️ Loop error: {e}")
                time.sleep(0.5)

    def enable_motion(self):
        self.motion_enabled = True
        print("\n✅ MOTION ENABLED")

    def set_angle(self, motor_id, angle_deg):
        name = f"motor_{motor_id}"
        if name in self.target_positions:
            self.target_positions[name] = math.radians(angle_deg)
            print(f"\n🎯 {name} → {angle_deg}°")

    def stop(self):
        print("\n🛑 Stopping...")
        self.running = False

        if hasattr(self, "thread"):
            self.thread.join(timeout=1)

        if self.bus:
            for name in self.motor_names:
                try:
                    self.bus.disable(name)
                except:
                    pass

            self.bus.disconnect()

        print("👋 Exit complete")


def main():
    args = sys.argv[1:]

    if len(args) < 2 or len(args) % 2 != 0:
        print("Usage: python3 multi_motor_safe.py <id model> <id model> ...")
        print("Example: python3 multi_motor_safe.py 10 rs-02 127 rs-03")
        sys.exit(1)

    motor_ids = []
    motor_models = []

    for i in range(0, len(args), 2):
        motor_ids.append(int(args[i]))
        motor_models.append(args[i + 1])

    controller = MultiMotorMITSafe(motor_ids, motor_models)

    signal.signal(signal.SIGINT, lambda s, f: controller.stop())
    signal.signal(signal.SIGTERM, lambda s, f: controller.stop())

    controller.connect()

    # Wait before enabling motion
    input()
    controller.enable_motion()

    # Multi-motor control input
    while True:
        try:
            controller.user_input_active = True

            cmd = input("\nEnter: <id angle pairs> (e.g. 10 30 127 -45) or q: ")

            controller.user_input_active = False

            if cmd.lower() in ['q', 'quit']:
                break

            parts = cmd.split()

            if len(parts) % 2 != 0:
                print("❌ Enter pairs like: 10 30 127 -45")
                continue

            for i in range(0, len(parts), 2):
                mid = int(parts[i])
                ang = float(parts[i + 1])
                controller.set_angle(mid, ang)

        except KeyboardInterrupt:
            break

    controller.stop()


if __name__ == "__main__":
    main()