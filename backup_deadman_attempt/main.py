import os
import time
import numpy as np
import pyarrow as pa
import dora

import rclpy
from rclpy.node import Node as RosNode
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState


RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]


def _float_env(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except Exception:
        return default


def _bool_env(name: str, default: bool = False) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.lower() in ("1", "true", "yes", "on")


class OpenArmV1RosBridge(RosNode):
    def __init__(self):
        super().__init__("openarm_v1_dora_ros2_bridge")

        self.dry_run = _bool_env("DRY_RUN", True)
        self.enable_gripper = _bool_env("ENABLE_GRIPPER", False)
        self.send_hz = _float_env("SEND_HZ", 5.0)
        self.traj_time = _float_env("TRAJ_TIME", 0.5)
        self.gripper_max = _float_env("GRIPPER_MAX", 0.044)
        self.gripper_reverse = _bool_env("GRIPPER_REVERSE", False)

        # Safety options
        self.enable_deadman = _bool_env("ENABLE_DEADMAN", True)
        self.deadman_pressed = False
        self.max_jump_rad = _float_env("MAX_JUMP_RAD", 0.35)

        self.min_period = 1.0 / max(self.send_hz, 0.1)
        self.last_send = {"right": 0.0, "left": 0.0}
        self.last_q = {"right": None, "left": None}
        self.current_q = {"right": None, "left": None}

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.pub_arm = {
            "right": self.create_publisher(
                JointTrajectory,
                "/right_joint_trajectory_controller/joint_trajectory",
                10,
            ),
            "left": self.create_publisher(
                JointTrajectory,
                "/left_joint_trajectory_controller/joint_trajectory",
                10,
            ),
        }

        self.pub_gripper = {
            "right": self.create_publisher(
                GripperCommand,
                "/right_gripper_controller/gripper_cmd",
                10,
            ),
            "left": self.create_publisher(
                GripperCommand,
                "/left_gripper_controller/gripper_cmd",
                10,
            ),
        }

        self.get_logger().info(
            f"OpenArm v1 Dora ROS2 bridge started. "
            f"DRY_RUN={self.dry_run}, ENABLE_GRIPPER={self.enable_gripper}, "
            f"SEND_HZ={self.send_hz}, TRAJ_TIME={self.traj_time}, " f"ENABLE_DEADMAN={self.enable_deadman}, MAX_JUMP_RAD={self.max_jump_rad}"
        )


    def _on_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        for side, joints in {
            "right": RIGHT_JOINTS,
            "left": LEFT_JOINTS,
        }.items():
            if all(j in name_to_pos for j in joints):
                self.current_q[side] = np.array(
                    [name_to_pos[j] for j in joints],
                    dtype=np.float32,
                )

    def set_deadman(self, pressed: bool):
        self.deadman_pressed = pressed
        print(f"[DEADMAN] {'ON' if pressed else 'OFF'}", flush=True)

    def publish_arm(self, side: str, values: np.ndarray):
        print(f"[PUBLISH_ARM ENTER] side={side} dry_run={self.dry_run} deadman={self.deadman_pressed}", flush=True)
        if values.shape[0] < 7:
            return

        now = time.time()
        if now - self.last_send[side] < self.min_period:
            return
        self.last_send[side] = now

        q_np = np.array([float(x) for x in values[:7]], dtype=np.float32)
        q = [float(x) for x in q_np]

        if self.enable_deadman and not self.deadman_pressed:
            print(f"[PUBLISH_ARM SKIP] {side} deadman OFF", flush=True)
            return

        ref = self.last_q[side]
        if ref is None:
            ref = self.current_q[side]

        if ref is not None:
            jump = float(np.max(np.abs(q_np - ref)))
            if jump > self.max_jump_rad:
                print(
                    f"[SAFETY SKIP] {side} jump={jump:.3f} rad > {self.max_jump_rad:.3f}. "
                    f"Move Quest closer to current robot pose.",
                    flush=True,
                )
                return

        if self.dry_run:
            print(f"[DRY_RUN ARM] {side} q={np.round(q_np, 4)}", flush=True)
            self.last_q[side] = q_np
            return

        msg = JointTrajectory()
        msg.joint_names = RIGHT_JOINTS if side == "right" else LEFT_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start.sec = int(self.traj_time)
        pt.time_from_start.nanosec = int((self.traj_time - int(self.traj_time)) * 1e9)

        msg.points.append(pt)
        print(f"[PUBLISH_ARM ROS2 SEND] {side} topic publish", flush=True)
        self.pub_arm[side].publish(msg)
        self.last_q[side] = q_np
        print(f"[ROS2 ARM] {side} q={np.round(q_np, 4)}", flush=True)

    def publish_gripper(self, side: str, values: np.ndarray):
        if not self.enable_gripper:
            return
        if values.shape[0] < 8:
            return

        raw = float(values[7])

        # Dora IK gripper output is about +/-0.785 rad.
        norm = min(abs(raw) / 0.785, 1.0)

        if self.gripper_reverse:
            pos = self.gripper_max * (1.0 - norm)
        else:
            pos = self.gripper_max * norm

        pos = float(np.clip(pos, 0.0, self.gripper_max))

        if self.dry_run:
            print(f"[DRY_RUN GRIPPER] {side} raw={raw:.4f} pos={pos:.5f}", flush=True)
            return

        msg = GripperCommand()
        msg.position = pos
        msg.max_effort = 20.0
        self.pub_gripper[side].publish(msg)


def main():
    rclpy.init()
    ros = OpenArmV1RosBridge()
    dora_node = dora.Node()

    print("[dora-ros2-v1] Event loop started.", flush=True)

    try:
        for event in dora_node:
            if event["type"] != "INPUT":
                rclpy.spin_once(ros, timeout_sec=0.0)
                continue

            eid = event["id"]
            print(f"[BRIDGE INPUT] id={eid}", flush=True)

            if eid == "button_a":
                print(f"[BUTTON_A RAW] {event['value']}", flush=True)
                try:
                    pressed = bool(event["value"].to_pylist()[0])
                except Exception as e:
                    print(f"[BUTTON_A ERROR] {e}", flush=True)
                    pressed = False
                print(f"[BUTTON_A] pressed={pressed}", flush=True)
                ros.set_deadman(pressed)
                rclpy.spin_once(ros, timeout_sec=0.0)
                continue

            values = event["value"].to_numpy().astype(np.float32)
            print(f"[BRIDGE VALUE] id={eid} shape={values.shape} values={values}", flush=True)

            if eid == "position_right":
                ros.publish_arm("right", values)
                ros.publish_gripper("right", values)

            elif eid == "position_left":
                ros.publish_arm("left", values)
                ros.publish_gripper("left", values)

            rclpy.spin_once(ros, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
