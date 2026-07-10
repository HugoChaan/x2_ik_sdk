from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Iterable

from .config import ArmSide, X2IKConfig
from .solver import X2ArmIKSolver
from .trajectory import interpolate_arm_pos


def _require_ros2_imports():
    try:
        import rclpy
        from aimdk_msgs.msg import UpperBodyCommandArray
        from aimdk_msgs.srv import GetAllJointState, SetMcAction
    except ImportError as exc:
        raise SystemExit(
            "ROS2 adapter dependencies are not importable.\n"
            "Confirm that rclpy, aimdk_msgs, and the new UpperBodyCommandArray message "
            "are available in the X2 SDK environment.\n"
            f"Original error: {exc}"
        ) from exc
    return rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction


def joint_states_to_arm_pos(joint_states: Iterable[object], fallback: list[float]) -> list[float]:
    by_name = {state.name: state.position for state in joint_states if hasattr(state, "name")}
    from .config import ARM_POS_ORDER

    values = []
    for i, name in enumerate(ARM_POS_ORDER):
        values.append(float(by_name.get(name, fallback[i])))
    return values


class X2UpperBodyIKNode:
    def __init__(self, rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction, args):
        self.rclpy = rclpy
        self.UpperBodyCommandArray = UpperBodyCommandArray
        self.GetAllJointState = GetAllJointState
        self.SetMcAction = SetMcAction

        from rclpy.node import Node

        class _Node(Node):
            pass

        self.node = _Node("x2_upper_body_ik_node")
        cfg = X2IKConfig.default_omnipicker()
        if args.urdf:
            cfg = X2IKConfig(urdf_path=args.urdf)
        self.solver = X2ArmIKSolver(cfg)
        self.publisher = self.node.create_publisher(UpperBodyCommandArray, args.command_topic, 10)
        self.joint_client = self.node.create_client(GetAllJointState, args.joint_state_service)
        self.action_client = self.node.create_client(SetMcAction, args.action_service)
        self.source = args.source
        self.frame_id = args.frame_id
        self.sequence = 0

    def switch_action(self, action_desc: str) -> None:
        if not self.action_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warning("SetMcAction service not available")
            return
        req = self.SetMcAction.Request()
        req.source = self.source
        req.command.action_desc = action_desc
        future = self.action_client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.node.get_logger().info(f"SetMcAction requested: {action_desc}")

    def read_current_arm_pos(self) -> list[float]:
        fallback = self.solver.ready_arm_pos()
        if not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warning("GetAllJointState not available; using ready arm seed")
            return fallback
        req = self.GetAllJointState.Request()
        future = self.joint_client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if future.result() is None:
            self.node.get_logger().warning("GetAllJointState failed; using ready arm seed")
            return fallback
        return joint_states_to_arm_pos(future.result().arm_joints, fallback)

    def make_msg(self, arm_pos: list[float], hand_open: tuple[float, float]):
        msg = self.UpperBodyCommandArray()
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.header.sequence = self.sequence
        msg.source = self.source
        msg.hand_sub_mode = 1
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = [float(v) for v in arm_pos]
        msg.hand_pos = [float(hand_open[0]), float(hand_open[1])]
        self.sequence += 1
        return msg

    def publish_trajectory(self, start: list[float], goal: list[float], duration: float) -> None:
        waypoints = interpolate_arm_pos(start, goal, duration=duration, rate_hz=50.0)
        for wp in waypoints:
            self.publisher.publish(self.make_msg(wp.arm_pos, (1.0, 1.0)))
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            # Use wall-clock sleep here; create_rate() can stall in this
            # single-threaded publish loop before additional waypoints are sent.
            time.sleep(max(0.0, wp.duration))

    def run_once(
        self,
        side: ArmSide,
        target_xyz: list[float],
        dry_run: bool,
        duration: float,
        target_rpy: list[float] | None,
        keep_current_rpy: bool,
        orientation_weight: float,
        orientation_eps: float,
    ) -> None:
        current = self.read_current_arm_pos()
        if target_rpy is not None and keep_current_rpy:
            raise ValueError("Use either target_rpy or keep_current_rpy, not both")
        if keep_current_rpy:
            target_rpy = self.solver.fk_rpy(side, current)

        if target_rpy is None:
            result = self.solver.solve_position(
                side=side,
                target_xyz=target_xyz,
                current_arm_pos=current,
            )
        else:
            result = self.solver.solve_pose(
                side=side,
                target_xyz=target_xyz,
                target_rpy=target_rpy,
                current_arm_pos=current,
                orientation_weight=orientation_weight,
                orientation_eps=orientation_eps,
            )
        self.node.get_logger().info(
            f"IK success={result.success} error={result.error_norm:.6f} arm_pos={result.arm_pos}"
        )
        if dry_run:
            return
        if not result.success:
            raise RuntimeError(f"IK failed: {result.message}, err={result.error_norm}")
        self.publish_trajectory(current, result.arm_pos, duration)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X2 upper-body IK ROS2 adapter")
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--side", choices=[side.value for side in ArmSide], default="right")
    parser.add_argument("--target", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--target-rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--keep-current-rpy", action="store_true")
    parser.add_argument("--orientation-weight", type=float, default=1.0)
    parser.add_argument("--orientation-eps", type=float, default=1e-3)
    parser.add_argument("--dry-run", action="store_true", help="Compute IK but do not publish motion")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--command-topic", default="/mc/upper_body_command")
    parser.add_argument("--joint-state-service", default="/aimdk_5Fmsgs/srv/GetAllJointState")
    parser.add_argument("--action-service", default="/aimdk_5Fmsgs/srv/SetMcAction")
    parser.add_argument("--source", default="x2_ik_sdk")
    parser.add_argument("--frame-id", default="mc_upper_body")
    parser.add_argument("--skip-mode-switch", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction = _require_ros2_imports()
    rclpy.init()
    node = X2UpperBodyIKNode(rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction, args)
    try:
        if not args.skip_mode_switch and not args.dry_run:
            node.switch_action("STAND_DEFAULT")
            node.switch_action("UPPERBODY_REMOTE_SPLIT")
        node.run_once(
            ArmSide(args.side),
            [float(v) for v in args.target],
            args.dry_run,
            args.duration,
            args.target_rpy,
            args.keep_current_rpy,
            args.orientation_weight,
            args.orientation_eps,
        )
    finally:
        node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
