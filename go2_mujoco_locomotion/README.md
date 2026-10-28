# go2_mujoco_locomotion

Go2 terrain demo for the workshop repository: MuJoCo physics + exported RL
policy + Nav2 + RViz, with a low-step maze world.

This package is intentionally separate from `workshop_navigation` (Go1 flat
factory). Both demos publish the same ROS 2 contract:

```text
subscribe   /cmd_vel              geometry_msgs/Twist
publish     /odom                 nav_msgs/Odometry
publish     /scan                 sensor_msgs/LaserScan
publish     /joint_states         sensor_msgs/JointState
publish     /tf                   odom -> base_link
```

## Build

ROS 2 Jazzy is the target environment. The node additionally needs
`mujoco` and `torch` in the Python environment used by ROS:

```bash
cd <repo>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select go2_mujoco_locomotion
source install/setup.bash
```

## Run

```bash
ros2 launch go2_mujoco_locomotion go2_terrain.launch.py
```

Optional:

```bash
ros2 launch go2_mujoco_locomotion go2_terrain.launch.py mujoco_viewer:=false rviz:=false
```

Then send a Nav2 goal from RViz (`2D Goal Pose`) or:

```bash
ros2 run workshop_navigation goal.py 2.0 0.4 --yaw 0.0 --timeout 120
```

The packaged map and scene are the same low-step maze: two 6 cm/14 cm steps and
a low ramp in the starting corridor.

## What is in the box

```text
config/go2_locomotion.yaml     policy/actuator/observation constants
config/go2_terrain_nav2.yaml   Nav2/DWB parameters used by the terrain demo
config/go2_terrain.rviz        RViz layout
maps/maze_steps.{pgm,yaml}     occupancy map (steps are traversable)
models/maze_steps.xml          MuJoCo world, includes models/go2.xml
models/go2.xml + models/go2/   Unitree Go2 MuJoCo model + meshes
models/policy.pt               exported 45-D observation / 10-frame history policy
```

## Known limits / integration notes

- The exported policy uses a **50 Hz control loop** (one policy action per four
  5 ms physics steps). Calling the policy at 200 Hz makes the robot stall at
  steps; this was a real bug in the first ROS bridge and is fixed here.
- The policy has no height scan. The low steps are traversable in MuJoCo, but
  the map must not mark them as static obstacles.
- `max_vx` is capped at 1.5 m/s in this demo. On the flat floor the policy is
  stable faster, but Nav2's DWB needs the stronger deceleration/inflation
  values in `go2_terrain_nav2.yaml` to avoid wall overshoot.
- The Go2 model/meshes are derived from the Unitree MuJoCo model used in the
  training repository; see the main repo's third-party notes.
