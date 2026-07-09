#!/usr/bin/env python3
import threading
import cv2
import mediapipe as mp
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- CONFIGURATION ---
SMOOTH_WINDOW = 5
history = {
    "L_sh": deque(maxlen=SMOOTH_WINDOW), "R_sh": deque(maxlen=SMOOTH_WINDOW),
    "L_el": deque(maxlen=SMOOTH_WINDOW), "R_el": deque(maxlen=SMOOTH_WINDOW)
}

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

def calculate_angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    if angle > 180.0: angle = 360 - angle
    return angle

def get_smooth_val(key, new_val):
    history[key].append(new_val)
    return int(sum(history[key]) / len(history[key]))

class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_tracker')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.last_state = "DEFAULT"

    def publish_state(self, state):
        if state != self.last_state:
            self.last_state = state
            msg = String()
            msg.data = state
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published gesture: {state}")

def main(args=None):
    rclpy.init(args=args)
    gesture_node = GestureNode()

    # Spin in a background thread to handle ROS callbacks cleanly
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gesture_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while cap.isOpened() and rclpy.ok():
            ret, frame = cap.read()
            if not ret: break
            
            # Convert frame to RGB for MediaPipe
            results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            current_state = "DEFAULT"

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                
                # 1. CALCULATE SMOOTHED PITCHES
                # Shoulders: Hip(23/24), Shoulder(11/12), Elbow(13/14)
                ls = get_smooth_val("L_sh", calculate_angle([lm[23].x, lm[23].y], [lm[11].x, lm[11].y], [lm[13].x, lm[13].y]))
                rs = get_smooth_val("R_sh", calculate_angle([lm[24].x, lm[24].y], [lm[12].x, lm[12].y], [lm[14].x, lm[14].y]))
                
                # Elbows: Shoulder(11/12), Elbow(13/14), Wrist(15/16)
                le = get_smooth_val("L_el", calculate_angle([lm[11].x, lm[11].y], [lm[13].x, lm[13].y], [lm[15].x, lm[15].y]))
                re = get_smooth_val("R_el", calculate_angle([lm[12].x, lm[12].y], [lm[14].x, lm[14].y], [lm[16].x, lm[16].y]))

                # 2. GESTURE LOGIC (Priority Order)
                
                # --- HI DETECTION ---
                l_hi = lm[15].y < lm[11].y
                r_hi = lm[16].y < lm[12].y

                if l_hi and r_hi:
                    current_state = "HI_BOTH"
                elif l_hi:
                    current_state = "HI_L"
                elif r_hi:
                    current_state = "HI_R"
                    
                else:
                    # --- NAMASTE DETECTION ---
                    lw, rw = np.array([lm[15].x, lm[15].y]), np.array([lm[16].x, lm[16].y])
                    dist = np.linalg.norm(lw - rw)
                    # Hands close + between shoulder and hip height
                    if dist < 0.05 and lm[11].y < lw[1] < lm[23].y:
                        current_state = "NAMASTE"
                    
                    # --- HANDSHAKE DETECTION ---
                    else:
                        # Logic: Wrist z-depth forward + extended elbow + at chest level
                        l_hs = (lm[15].z < lm[11].z - 0.45 and le > 110 and lm[11].y < lm[15].y < lm[23].y)
                        r_hs = (lm[16].z < lm[12].z - 0.45 and re > 110 and lm[12].y < lm[16].y < lm[24].y)

                        if l_hs and r_hs:
                            current_state = "HANDSHAKE_BOTH"
                        elif l_hs:
                            current_state = "HANDSHAKE_L"
                        elif r_hs:
                            current_state = "HANDSHAKE_R"

                # 3. OUTPUT & VISUALS
                print(f"[{current_state:^14}] | SHLDR L:{ls:3} R:{rs:3} | ELBOW L:{le:3} R:{re:3}")
                
                cv2.putText(frame, current_state, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Publish to ROS
            gesture_node.publish_state(current_state)

            cv2.imshow('ROS2 Gesture Tracker', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
