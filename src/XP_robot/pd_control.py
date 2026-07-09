import mujoco
import mujoco.viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path("XP_robot_with_actuators.xml")

data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

# ---- GAINS ----
kp = 100
kd = 0.5

# ---- TARGET POSTURE ----
target = np.zeros(model.nu)

for i in range(model.nu):
    name = model.actuator(i).name

    if "knee" in name:
        target[i] = 0.1
    elif "hip_pitch" in name:
        target[i] = 0.03
    elif "ankle_pitch" in name:
        target[i] = 0.02
    else:
        target[i] = 0.0



# ---- SENSOR IDs ----
gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
acc_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_acc")



# ---- CONTROL LOOP ----
while viewer.is_running():

    # Joint states
    qpos = data.qpos[7:]
    qvel = data.qvel[6:]

    # ---- IMU DATA ----
    gyro = data.sensordata[gyro_id:gyro_id+3]
    acc = data.sensordata[acc_id:acc_id+3]

    # ---- COMPUTE TILT ----
    # Forward/back tilt from accelerometer
    tilt = acc[0]

    # Angular velocity (fall speed)
    tilt_rate = gyro[1]

    # ---- BALANCE CONTROLLER ----
    quat = data.qpos[3:7]

    # forward tilt (approx)
    tilt = 2 * (quat[0] * quat[2] - quat[3] * quat[1])
    tilt_rate = data.qvel[4]

    balance = -80 * tilt - 20 * tilt_rate

    # ---- PD JOINT CONTROL ----
    torque = kp * (target - qpos[:model.nu]) - kd * qvel[:model.nu]
    torque -= 0.15 * qvel[:model.nu]

    # ---- APPLY BALANCE ----
    for i in range(model.nu):
        name = model.actuator(i).name

        # Ankles = primary balance
        if "ankle_pitch" in name:
            torque[i] += 0.6 * balance

        # Hips = secondary balance
        if "hip_pitch" in name:
            torque[i] += 1.0 * balance

    # ---- TORQUE LIMIT (VERY IMPORTANT) ----
    torque = np.clip(torque, -120, 120)

    # Apply control
    data.ctrl[:] = torque

    mujoco.mj_step(model, data)
    viewer.sync()