## Foot Force Array Publisher

This ROS 2 node publishes labeled foot force and position data for the Angad full-body MuJoCo robot.

### Custom Messages
- `FootSensor.msg`: Contains `label`, `force`, `x`, `y`, `z` for each sensor.
- `FootSensorArray.msg`: Contains an array of `FootSensor` messages.

### Topics
- `/foot_force_left`  (`FootSensorArray`): Left foot sensors (heel1, heel2, toe1, toe2)
- `/foot_force_right` (`FootSensorArray`): Right foot sensors (heel1, heel2, toe1, toe2)

### Usage
1. Build the package:
   ```bash
   colcon build --packages-select full_body_mujoco
   source install/setup.bash
   ```
2. Run the node:
   ```bash
   ros2 run full_body_mujoco foot_force_array_publisher.py
   ```
3. Subscribe to the topics:
   ```bash
   ros2 topic echo /foot_force_left
   ros2 topic echo /foot_force_right
   ```

### Message Example
Each message contains an array of labeled sensors:
```
sensors:
- label: foot_l_heel_1
  force: ...
  x: ...
  y: ...
  z: ...
- label: foot_l_heel_2
  ...
...
```