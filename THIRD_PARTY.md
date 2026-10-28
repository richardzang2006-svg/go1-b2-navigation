# Reused components and provenance

- Go1 MuJoCo locomotion project: supplied locally at `go1_mujoco_locomotion/`. It includes `go1_corridor.mjb`, policy weights, and observation normalization data. `go1_maze.mjb` and `go1_factory.mjb` are derived from `go1_corridor.mjb` by the scripts in `go1_mujoco_locomotion/scripts/`; the factory dressing is original primitive geometry. Its README describes the policy as PPO-trained; the original training run and weight provenance have not been independently verified.
- Go1 robot model: the supplied project attributes its model to [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (BSD-3-Clause).
- Navigation2 and AMCL: installed ROS 2 Jazzy modules, configured for the Go1 MuJoCo factory/maze scenes and the policy speed limits.
- Historical, inactive source retained for reference: [CHAMP](https://github.com/chvmp/champ), and [Unitree Go2 ROS 2](https://github.com/khaledgabr77/unitree_go2_ros2). The Go2 repository lacks a root license declaration; its description and mesh terms should be reviewed before redistribution. These source trees contain `COLCON_IGNORE` files and are not used by the Go1 project.

Project-specific integration reuses the Go1 MuJoCo policy interface, generates a static Nav2 map from the model's collision geometry, and connects Nav2 commands through the watchdog and collision monitor to `/cmd_vel_safe`.

- Go2 terrain extension (`go2_mujoco_locomotion/`): the Go2 MuJoCo model and meshes are derived from the Unitree Go2 model used by the local training repository; the TorchScript policy `models/policy.pt` was trained in that repository and exported for CPU deployment. The model/scene files are included to make the demo reproducible, not as an independently licensed redistributable asset. Review the upstream Unitree model terms before redistribution.
