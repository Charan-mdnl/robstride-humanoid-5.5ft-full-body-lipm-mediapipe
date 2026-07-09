Upper Body mediapipe mimic namaste

1. cd humanoid_ws && source install/setup.bash
2. ros2 launch full_body_hardware full_body_hardware.launch.py 
3. ros2 launch full_body_moveit_hardware move_group.launch.py 
4. ros2 launch full_body_moveit_scripts mediapipe.launch.py 
5. source venv/bin/activate
6. python3 src/full_body_moveit_scripts/src/namaste.py 

7. ros2 launch full_body_moveit_hardware moveit_rviz.launch.py 


Full-Body Walking Demo (nr_ik_test) — FAKE hardware, RViz only

Safe dry-run: no real motors, no CAN bus needed. Use this to check the walking
logic/visualization before ever running it on the real legs.

1. ros2 launch full_body_hardware full_body_hardware.launch.py use_real_hardware:=false
   # HARDWARE (fake): starts robot_state_publisher + controller_manager with FakeSystem
   # for both arms and legs, spawns joint_state_broadcaster/lower_body_controller/
   # torso_controller/left_arm_controller/right_arm_controller, and opens RViz
   # automatically — pre-configured with RobotModel + TF + MarkerArray, Fixed Frame
   # base_link. No manual rviz2 step needed. Leave this running in its own terminal.

2. ros2 run full_body_mujoco nr_ik_test
   # Full default walk (n_steps=6, step_length=0.06, step_height=0.04) — safe here since
   # nothing is real. Watch it crouch and walk live in the RViz window from step 1.


Full-Body Walking Demo (nr_ik_test) — REAL hardware, legs suspended, RViz

⚠️ FIRST-TIME REAL WALKING TEST. Robot MUST be mechanically supported/suspended
   (harness, stand, crane) so the legs carry NO body weight before this demo starts.
   Keep hands/cables clear of the legs while they move. Start gentle (steps 2-3) —
   do not jump straight to a full walk.

   Package used here is full_body_hardware_leg (REAL legs on can4/can5 + fake arms) —
   this is DIFFERENT from full_body_hardware above (fake everything, or real arms only).
   nr_ik_test itself lives in full_body_mujoco and is already built.

1. ros2 launch full_body_hardware_leg full_body_hardware.launch.py
   # HARDWARE (real legs): starts robot_state_publisher + controller_manager with REAL
   # legs (RobStrideSystem on can4/can5) and fake arms, spawns joint_state_broadcaster/
   # lower_body_controller/torso_controller/left_arm_controller/right_arm_controller,
   # and opens RViz automatically — pre-configured with RobotModel + TF + MarkerArray,
   # Fixed Frame base_link. No manual rviz2 step needed.
   # TF here comes from REAL motor encoder feedback, so this is how you catch a
   # control/hardware problem live: if a leg is commanded to move but doesn't animate
   # (or lags/stalls) in RViz, that joint isn't actually tracking on the real hardware.
   # Leave this running in its own terminal.

2. ros2 run full_body_mujoco nr_ik_test --ros-args -p n_steps:=0
   # FIRST RUN: crouch and HOLD only (n_steps=0 skips all stepping).
   # Confirms both legs can hold a bent-knee crouch before ever attempting to lift a leg.
   # Watch RViz: both legs should crouch and stay put, matching the commanded pose/markers.

3. ros2 run full_body_mujoco nr_ik_test --ros-args -p n_steps:=1 -p step_length:=0.02 -p step_height:=0.02
   # SECOND RUN, only after step 2 looks solid: a single small step (2cm forward, 2cm lift).
   # Re-run with n_steps / step_length / step_height increased gradually toward the
   # defaults (n_steps=6, step_length=0.06, step_height=0.04) once each smaller step
   # tracks cleanly in RViz.

Notes
- RViz now opens automatically with both launches above (previously needed a manual
  `rviz2` step — fixed by wiring up each package's launch file to its own
  config/display.rviz). Pass `rviz:=false` to either launch command to skip it.
- RViz alone (e.g. you closed the window but hardware is still running): re-run
  `rviz2 -d install/<package>/share/<package>/config/display.rviz` in a new terminal —
  use full_body_hardware or full_body_hardware_leg as <package>, matching whichever
  launch is currently running.





