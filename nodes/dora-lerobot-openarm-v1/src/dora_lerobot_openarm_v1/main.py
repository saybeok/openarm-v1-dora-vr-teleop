import argparse
import time

import dora
import numpy as np
import pyarrow as pa

from lerobot.robots.openarm_follower import OpenArmFollower, OpenArmFollowerConfig


JOINT_KEYS = [
    "joint_1.pos",
    "joint_2.pos",
    "joint_3.pos",
    "joint_4.pos",
    "joint_5.pos",
    "joint_6.pos",
    "joint_7.pos",
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", required=True, choices=["right", "left"])
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", default=None)

    parser.add_argument("--send-hz", type=float, default=50.0)
    parser.add_argument("--obs-hz", type=float, default=50.0)

    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--max-delta", type=float, default=60.0)
    parser.add_argument("--max-step-deg", type=float, default=1.0)

    # Gripper:
    # trigger 0.0 -> open
    # trigger 1.0 -> close
    parser.add_argument("--gripper-open-deg", type=float, default=35.0)
    parser.add_argument("--gripper-close-deg", type=float, default=0.0)
    parser.add_argument("--gripper-max-step-deg", type=float, default=2.0)
    parser.add_argument("--gripper-invert", action="store_true")

    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def make_openarm_config(args):
    return OpenArmFollowerConfig(
        port=args.port,
        side=args.side,
        can_interface="socketcan",
        use_can_fd=True,
        can_bitrate=1000000,
        can_data_bitrate=5000000,
        id=args.id or f"openarm_{args.side}",
    )


class LeRobotOpenArmNode:
    def __init__(self, args):
        self.args = args
        self.side = args.side

        self.min_send_period = 1.0 / max(args.send_hz, 1.0)
        self.min_obs_period = 1.0 / max(args.obs_hz, 1.0)

        self.last_send = 0.0
        self.last_obs = 0.0

        self.robot = None

        self.ik_ref_deg = None
        self.motor_ref_deg = None
        self.last_action_deg = None

        self.gripper_trigger = 0.0
        self.gripper_active = False
        self.gripper_deadzone = 0.05
        self.last_gripper_deg = float(args.gripper_open_deg)

        # Joint direction mapping.
        # If a joint moves opposite, change that index to -1.
        self.sign = np.array([1, 1, 1, 1, 1, -1, 1], dtype=np.float32)

        if not args.dry_run:
            cfg = make_openarm_config(args)
            print(f"[OpenArmFollowerConfig] {cfg}", flush=True)
            self.robot = OpenArmFollower(cfg)
            self.robot.connect()

            obs = self.robot.get_observation()
            if "gripper.pos" in obs:
                self.last_gripper_deg = float(obs["gripper.pos"])

        print(
            f"[LeRobot OpenArm] side={args.side} port={args.port} "
            f"send_hz={args.send_hz} obs_hz={args.obs_hz} "
            f"scale={args.scale} max_delta={args.max_delta} "
            f"max_step_deg={args.max_step_deg} "
            f"gripper_open={args.gripper_open_deg} "
            f"gripper_close={args.gripper_close_deg} "
            f"gripper_max_step={args.gripper_max_step_deg} "
            f"dry_run={args.dry_run}",
            flush=True,
        )

    def close(self):
        if self.robot is not None:
            self.robot.disconnect()

    def get_motor_deg(self):
        if self.robot is None:
            return np.zeros(7, dtype=np.float32)

        obs = self.robot.get_observation()

        vals = []
        for key in JOINT_KEYS:
            if key not in obs:
                print(f"[OBS MISSING] {self.side} key={key} keys={list(obs.keys())}", flush=True)
                return None
            vals.append(float(obs[key]))

        return np.array(vals, dtype=np.float32)

    def get_position8_rad(self):
        motor_deg = self.get_motor_deg()
        if motor_deg is None:
            return None

        rad = np.deg2rad(motor_deg).astype(np.float32)

        # 8th value is gripper dummy for IK position[16] format.
        return np.concatenate([rad, np.array([0.0], dtype=np.float32)]).astype(np.float32)

    def set_gripper_trigger(self, values):
        arr = np.asarray(values, dtype=np.float32).reshape(-1)
        if arr.size == 0:
            return

        t = float(arr[0])
        t = max(0.0, min(1.0, t))

        if self.args.gripper_invert:
            t = 1.0 - t

        self.gripper_trigger = t
        if t > self.gripper_deadzone:
            self.gripper_active = True
        print(f"[{self.side}] gripper trigger={self.gripper_trigger:.3f}", flush=True)

    def compute_gripper_deg(self):
        # Startup hold:
        # Until the trigger is actually pressed, do not command gripper at all.
        # This prevents pushing the gripper against its mechanical open limit
        # when the robot powers on already open.
        if not self.gripper_active:
            return None

        # Analog trigger:
        # trigger 0.0 -> open
        # trigger 1.0 -> close
        target = (
            float(self.args.gripper_open_deg) * (1.0 - self.gripper_trigger)
            + float(self.args.gripper_close_deg) * self.gripper_trigger
        )

        max_step = float(self.args.gripper_max_step_deg)
        step = target - self.last_gripper_deg
        step = max(-max_step, min(max_step, step))

        self.last_gripper_deg = self.last_gripper_deg + step
        return self.last_gripper_deg

    def send_position(self, values):
        if values.shape[0] < 7:
            return

        now = time.time()
        if now - self.last_send < self.min_send_period:
            return

        self.last_send = now

        # Dora IK output: radians -> degrees
        ik_deg = np.rad2deg(values[:7].astype(np.float32))

        # First command: capture relative reference.
        if self.ik_ref_deg is None:
            self.ik_ref_deg = ik_deg.copy()

            motor_deg = self.get_motor_deg()
            if motor_deg is None:
                print(f"[{self.side}] cannot read motor observation, skip", flush=True)
                return

            self.motor_ref_deg = motor_deg.copy()
            self.last_action_deg = motor_deg.copy()

            print(f"[{self.side}] RELATIVE REF SET", flush=True)
            print(f"  ik_ref_deg    = {np.round(self.ik_ref_deg, 3)}", flush=True)
            print(f"  motor_ref_deg = {np.round(self.motor_ref_deg, 3)}", flush=True)

            # Hold current arm pose first.
            action_deg = motor_deg.copy()

        else:
            delta = (ik_deg - self.ik_ref_deg) * self.sign * float(self.args.scale)
            delta = np.clip(delta, -float(self.args.max_delta), float(self.args.max_delta))
            action_deg = self.motor_ref_deg + delta

            # Limit per-frame joint movement based on the previous command.
            # At 200Hz, max_step_deg=1.0 means up to about 200 deg/sec.
            max_step_deg = float(self.args.max_step_deg)
            step = action_deg - self.last_action_deg
            step = np.clip(step, -max_step_deg, max_step_deg)
            action_deg = self.last_action_deg + step

        gripper_deg = self.compute_gripper_deg()

        action = {
            "joint_1.pos": float(action_deg[0]),
            "joint_2.pos": float(action_deg[1]),
            "joint_3.pos": float(action_deg[2]),
            "joint_4.pos": float(action_deg[3]),
            "joint_5.pos": float(action_deg[4]),
            "joint_6.pos": float(action_deg[5]),
            "joint_7.pos": float(action_deg[6]),
        }

        if gripper_deg is not None:
            action["gripper.pos"] = float(gripper_deg)

        self.last_action_deg = action_deg.copy()

        if self.args.dry_run:
            print(f"[DRY_RUN {self.side}] {action}", flush=True)
            return

        self.robot.send_action(action)


def main():
    args = parse_args()

    arm = LeRobotOpenArmNode(args)
    node = dora.Node()

    node.send_output("status", pa.array([f"{args.side}_ready"]))

    try:
        for event in node:
            now = time.time()

            # Feedback to IK: current physical joint position.
            if not args.dry_run and now - arm.last_obs >= arm.min_obs_period:
                arm.last_obs = now
                pos8 = arm.get_position8_rad()
                if pos8 is not None:
                    node.send_output("position", pa.array(pos8, type=pa.float32()))

            if event["type"] != "INPUT":
                continue

            eid = event["id"]

            if eid == "gripper":
                values = event["value"].to_numpy().astype(np.float32)
                arm.set_gripper_trigger(values)
                continue

            if eid != "move_position":
                continue

            values = event["value"].to_numpy().astype(np.float32)
            arm.send_position(values)

    except KeyboardInterrupt:
        pass

    finally:
        arm.close()


if __name__ == "__main__":
    main()
