#!/usr/bin/env python3
"""ROS 2 MuJoCo runner for the exported Go2 policy.

The node owns the MuJoCo simulation, consumes Nav2's /cmd_vel, and publishes
odom, base_link TF and a synthetic 360-degree LaserScan for the Nav2 costmaps.
"""
from __future__ import annotations

import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def gravity_orientation(quat: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quat
    return np.array(
        [
            2.0 * (-qz * qx + qw * qy),
            -2.0 * (qz * qy + qw * qx),
            1.0 - 2.0 * (qw * qw + qz * qz),
        ],
        dtype=np.float32,
    )


def quat_rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    qw = float(quat[0])
    qxyz = np.asarray(quat[1:], dtype=np.float32)
    vec = np.asarray(vec, dtype=np.float32)
    return vec * (2.0 * qw * qw - 1.0) - np.cross(qxyz, vec) * qw * 2.0 + qxyz * np.dot(qxyz, vec) * 2.0


def joint_indices(model, names):
    qpos, qvel, acts = [], [], []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"joint {name!r} not found")
        qpos.append(model.jnt_qposadr[jid])
        qvel.append(model.jnt_dofadr[jid])
        match = np.flatnonzero(model.actuator_trnid[:, 0] == jid)
        if match.size != 1:
            raise ValueError(f"expected one actuator for {name!r}")
        acts.append(int(match[0]))
    return np.asarray(qpos), np.asarray(qvel), np.asarray(acts)


def package_share_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("go2_mujoco_locomotion")
    except Exception:
        # source-tree fallback: .../go2_mujoco_locomotion/go2_mujoco_locomotion
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class MujocoPolicyNode(Node):
    def __init__(self):
        super().__init__("go2_locomotion")
        share = package_share_dir()
        self.declare_parameter("config_path", os.path.join(share, "config", "go2_locomotion.yaml"))
        self.declare_parameter("xml_path", os.path.join(share, "models", "maze_steps.xml"))
        self.declare_parameter("policy_path", os.path.join(share, "models", "policy.pt"))
        self.declare_parameter("cmd_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("cmd_timeout", 0.6)
        self.declare_parameter("max_vx", 1.50)
        self.declare_parameter("max_vy", 0.18)
        self.declare_parameter("max_wz", 0.50)
        self.declare_parameter("smoothing", 0.30)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("enable_viewer", False)
        self.declare_parameter("hold_when_zero", True)
        self.declare_parameter("hold_delay", 0.8)
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_stop_radius", 0.35)

        self.cfg = load_yaml(Path(self.get_parameter("config_path").value))
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.alpha = float(self.get_parameter("smoothing").value)
        self.publish_scan = bool(self.get_parameter("publish_scan").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_joint_states = bool(self.get_parameter("publish_joint_states").value)
        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.hold_when_zero = bool(self.get_parameter("hold_when_zero").value)
        self.hold_delay = float(self.get_parameter("hold_delay").value)
        self.zero_since = None
        self.goal_xy = None
        self.goal_hold = False

        self.model = mujoco.MjModel.from_xml_path(str(self.get_parameter("xml_path").value))
        self.model.opt.timestep = 0.005
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        self.mujoco_names = list(self.cfg["mujoco_joint_names"])
        self.policy_names = list(self.cfg["policy_joint_names"])
        self.mujoco_from_policy = np.asarray([self.policy_names.index(n) for n in self.mujoco_names])
        self.qpos_ids, self.qvel_ids, self.act_ids = joint_indices(self.model, self.mujoco_names)
        self.default_policy = np.asarray(self.cfg["default_angles"], dtype=np.float32)
        self.kp_policy = np.asarray(self.cfg["kps"], dtype=np.float32)
        self.kd_policy = np.asarray(self.cfg["kds"], dtype=np.float32)
        self.default_mujoco = self.default_policy[self.mujoco_from_policy]
        self.kp = self.kp_policy[self.mujoco_from_policy]
        self.kd = self.kd_policy[self.mujoco_from_policy]
        self.action_scale = float(self.cfg["action_scale"])
        self.action_clip = float(self.cfg["action_clip"])
        self.ang_scale = float(self.cfg["ang_vel_scale"])
        self.dof_pos_scale = float(self.cfg["dof_pos_scale"])
        self.dof_vel_scale = float(self.cfg["dof_vel_scale"])
        self.command_scale = np.asarray(self.cfg["command_scale"], dtype=np.float32)
        self.control_decimation = int(self.cfg["control_decimation"])
        self.control_dt = float(self.model.opt.timestep) * self.control_decimation
        self.motor_cfg = self.cfg["motor"]

        self.data.qpos[self.qpos_ids] = self.default_mujoco
        self.data.qpos[0:3] = np.array([0.0, 0.0, 0.445])
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        mujoco.mj_forward(self.model, self.data)
        self.start_xy = self.data.qpos[0:2].copy()

        self.policy = torch.jit.load(str(self.get_parameter("policy_path").value), map_location="cpu")
        self.policy.eval()
        if hasattr(self.policy, "reset"):
            self.policy.reset()

        self.num_actions = len(self.mujoco_names)
        self.obs = np.zeros(3 + 3 + 3 + 3 * self.num_actions, dtype=np.float32)
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.command_raw = np.zeros(3, dtype=np.float32)
        self.command = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = -1.0
        self.scan_count = 0
        self.base_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")

        self.create_subscription(Twist, str(self.get_parameter("cmd_topic").value), self.on_cmd, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self.on_goal, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf = TransformBroadcaster(self)
        self.viewer = None
        if self.enable_viewer:
            try:
                from mujoco import viewer as mj_viewer
            except ImportError:
                import mujoco.viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.model, self.data)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.base_body
            self.viewer.cam.distance = 2.0
            self.viewer.cam.elevation = -20.0
            self.get_logger().info("MuJoCo viewer enabled")

        self.create_timer(self.control_dt, self.step)

        self.get_logger().info(
            f"MuJoCo policy up: xml={self.get_parameter('xml_path').value} "
            f"cmd={self.get_parameter('cmd_topic').value} dt={self.control_dt:.3f}s"
        )

    def on_goal(self, msg: PoseStamped):
        self.goal_xy = np.array([msg.pose.position.x, msg.pose.position.y], dtype=np.float32)
        self.goal_hold = False
        self.get_logger().info(f"goal target ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})")

    def on_cmd(self, msg: Twist):
        self.command_raw[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_cmd_time = time.monotonic()

    def _clip_command(self) -> np.ndarray:
        if self.last_cmd_time < 0 or time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            target = np.zeros(3, dtype=np.float32)
        else:
            target = self.command_raw.copy()
        target[0] = np.clip(target[0], 0.0, self.max_vx)
        target[1] = np.clip(target[1], -self.max_vy, self.max_vy)
        target[2] = np.clip(target[2], -self.max_wz, self.max_wz)
        self.command += self.alpha * (target - self.command)
        return self.command

    def _control(self):
        cmd = self._clip_command()
        if self.goal_xy is not None and not self.goal_hold:
            qxy = self.data.qpos[0:2] - self.start_xy
            if float(np.linalg.norm(qxy - self.goal_xy)) < float(self.get_parameter("goal_stop_radius").value):
                self.goal_hold = True
                self.get_logger().info("goal reached; holding default stance")
        if self.goal_hold:
            self.action[:] = 0.0
            target_mujoco = self.default_mujoco
            tau = (target_mujoco - self.data.qpos[self.qpos_ids]) * self.kp - self.data.qvel[self.qvel_ids] * self.kd
            self.data.ctrl[self.act_ids] = tau
            return
        if self.hold_when_zero and np.linalg.norm(cmd[:2]) < 0.02 and abs(cmd[2]) < 0.05:
            if self.zero_since is None:
                self.zero_since = time.monotonic()
            if time.monotonic() - self.zero_since >= self.hold_delay:
                # Explicit zero-command stance: do not let the policy drift while idle.
                self.action[:] = 0.0
                target_mujoco = self.default_mujoco
                tau = (target_mujoco - self.data.qpos[self.qpos_ids]) * self.kp - self.data.qvel[self.qvel_ids] * self.kd
                self.data.ctrl[self.act_ids] = tau
                return
        else:
            self.zero_since = None
        self.goal_xy = None
        self.goal_hold = False
        q = self.data.qpos[self.qpos_ids][self.mujoco_from_policy]
        qd = self.data.qvel[self.qvel_ids][self.mujoco_from_policy]
        self.obs[0:3] = self.data.qvel[3:6] * self.ang_scale
        self.obs[3:6] = gravity_orientation(self.data.qpos[3:7])
        self.obs[6:9] = cmd * self.command_scale
        self.obs[9:21] = (q - self.default_policy) * self.dof_pos_scale
        self.obs[21:33] = qd * self.dof_vel_scale
        self.obs[33:45] = self.action
        with torch.inference_mode():
            out = self.policy(torch.from_numpy(self.obs[None, :]))
        action = out.detach().cpu().numpy().reshape(-1)
        if action.shape != (self.num_actions,):
            raise RuntimeError(f"policy action shape {action.shape}")
        self.action = np.clip(action, -self.action_clip, self.action_clip).astype(np.float32)
        target_policy = self.default_policy + self.action * self.action_scale
        target_mujoco = target_policy[self.mujoco_from_policy]
        tau = (target_mujoco - self.data.qpos[self.qpos_ids]) * self.kp - self.data.qvel[self.qvel_ids] * self.kd
        # Apply the same torque-speed envelope as the training actuator.
        if self.motor_cfg["use_torque_speed_curve"]:
            v = self.data.qvel[self.qvel_ids]
            x1, x2 = float(self.motor_cfg["X1"]), float(self.motor_cfg["X2"])
            y1, y2 = float(self.motor_cfg["Y1"]), float(self.motor_cfg["Y2"])
            same = v * tau > 0.0
            vmax = np.where(same, y1, y2)
            vs = vmax * (1.0 - (np.abs(v) - x1) / (x2 - x1))
            vmax = np.where(np.abs(v) < x1, vmax, np.maximum(vs, 0.0))
            tau = np.clip(tau, -vmax, vmax)
        self.data.ctrl[self.act_ids] = tau

    def _publish(self):
        now = self.get_clock().now().to_msg()
        q = self.data.qpos
        yaw = math.atan2(
            2.0 * (q[3] * q[6] + q[4] * q[5]),
            1.0 - 2.0 * (q[5] * q[5] + q[6] * q[6]),
        )
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(q[0] - self.start_xy[0])
        odom.pose.pose.position.y = float(q[1] - self.start_xy[1])
        odom.pose.pose.position.z = float(q[2])
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        local = quat_rotate_inverse(q[3:7], self.data.qvel[:3])
        odom.twist.twist.linear.x = float(local[0])
        odom.twist.twist.linear.y = float(local[1])
        odom.twist.twist.angular.z = float(self.data.qvel[5])
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(q[0] - self.start_xy[0])
        t.transform.translation.y = float(q[1] - self.start_xy[1])
        t.transform.translation.z = float(q[2])
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        if self.publish_tf:
            self.tf.sendTransform(t)

        if self.publish_scan:
            n = 180
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = self.base_frame
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / n
            scan.range_min = 0.15
            scan.range_max = 6.0
            pnt = np.array([q[0], q[1], q[2] + 0.18], dtype=np.float64)
            group = np.ones(6, dtype=np.uint8)
            ranges = []
            for i in range(n):
                a = yaw - math.pi + i * scan.angle_increment
                vec = np.array([math.cos(a), math.sin(a), 0.0], dtype=np.float64)
                dist = mujoco.mj_ray(self.model, self.data, pnt, vec, group, True, self.base_body, None)
                ranges.append(float(min(dist, scan.range_max)) if dist >= 0 else float(scan.range_max))
            scan.ranges = ranges
            self.scan_pub.publish(scan)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = now
            js.name = list(self.mujoco_names)
            js.position = [float(v) for v in q[self.qpos_ids]]
            js.velocity = [float(v) for v in self.data.qvel[self.qvel_ids]]
            self.joint_pub.publish(js)

    def step(self):
        self._control()
        for _ in range(self.control_decimation):
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception:
                self.viewer = None
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
