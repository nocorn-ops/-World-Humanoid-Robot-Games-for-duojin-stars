#!/usr/bin/env python3
"""``arm`` — a small terminal interface for the public arm Actions.

Every ``move``, ``shift``, and ``joint`` invocation sends a goal to the
server.  Omitting ``--execute`` asks the server for a real preview: it checks
feedback, TF, limits, ownership, and reachability without publishing motion.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
import sys
import time
from typing import Callable, Optional, Sequence, Tuple


os.environ.setdefault("RCUTILS_LOG_LEVEL", "ERROR")


@dataclass(frozen=True)
class _ArmApi:
    """Functions needed by this CLI, kept injectable for ROS-free tests."""

    get_pose: Callable
    move_by: Callable
    move_joints: Callable
    move_to: Callable


def _load_api() -> _ArmApi:
    from duojin_robot_interface.arm_client_functions import (
        get_pose,
        move_by,
        move_joints,
        move_to,
    )

    return _ArmApi(get_pose, move_by, move_joints, move_to)


def _color(label: str, code: str, stream) -> str:
    if stream.isatty() and not os.environ.get("NO_COLOR"):
        return f"\033[{code}m{label}\033[0m"
    return label


def _ok(message: str) -> None:
    print(f"{_color('OK', '32', sys.stdout)}: {message}", flush=True)


def _preview(message: str) -> None:
    label = _color("PREVIEW", "36", sys.stdout)
    print(f"{label}: {message}; no motion target was published", flush=True)


def _fail(message: str) -> int:
    print(f"{_color('FAIL', '31', sys.stderr)}: {message}", file=sys.stderr, flush=True)
    return 1


def _fmt_xyz(x: float, y: float, z: float) -> str:
    return f"({x:.4f}, {y:.4f}, {z:.4f}) m"


def _fmt_joints(positions: Sequence[float]) -> str:
    return "(" + ", ".join(f"{value:+.4f}" for value in positions) + ") rad"


def _elapsed(started_s: float) -> str:
    return f"{time.monotonic() - started_s:.1f}s"


def _finite_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def _nonnegative_float(value: str) -> float:
    parsed = _finite_float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be zero or positive")
    return parsed


def _positive_float(value: str) -> float:
    parsed = _finite_float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _speed_scale(value: str) -> float:
    parsed = _positive_float(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError("must be in (0, 1]")
    return parsed


def _frame_name(value: str) -> str:
    frame = value.strip()
    if not frame:
        raise argparse.ArgumentTypeError("must not be empty")
    return frame


def _result_failure(result) -> str:
    detail = f"{result.message} (outcome={result.outcome}, reason={result.reason})"
    if not result.execution_state_known:
        return f"{detail} Do not send another goal; inspect the robot and use E-stop if needed."
    return detail


def _finish_pose(result, *, execute: bool, started_s: float, verb: str) -> int:
    if not result.succeeded:
        return _fail(_result_failure(result))
    if not execute:
        if result.executed is not False:
            return _fail("preview request returned an unexpected executed state")
        _preview(result.message)
        return 0
    if result.executed is not True or not result.final_state_valid:
        return _fail("server returned success without confirmed final pose feedback")
    position = result.final_position_xyz
    if position is None:
        return _fail("server returned final pose marked valid but without coordinates")
    error = result.position_error_m
    suffix = f", position_error={error:.4f} m" if error is not None else ""
    _ok(f"{verb}: final_xyz={_fmt_xyz(*position)}{suffix}, elapsed={_elapsed(started_s)}")
    return 0


def _finish_joint(result, *, execute: bool, started_s: float) -> int:
    if not result.succeeded:
        return _fail(_result_failure(result))
    if not execute:
        if result.executed is not False:
            return _fail("preview request returned an unexpected executed state")
        _preview(result.message)
        return 0
    if result.executed is not True or not result.final_state_valid:
        return _fail("server returned success without confirmed final joint feedback")
    error = result.max_position_error_rad
    suffix = f"max_joint_error={error:.4f} rad" if error is not None else "joint feedback confirmed"
    _ok(f"joint goal completed: {suffix}, elapsed={_elapsed(started_s)}")
    return 0


def _read_joint_positions(arm: str, timeout_sec: float) -> Optional[Tuple[float, ...]]:
    """Return one joint sample when available; a pose query must still succeed alone."""

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from sensor_msgs.msg import JointState

    rclpy.init(args=[])
    node = Node(f"_arm_status_{arm}", start_parameter_services=False)
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    latest = []

    def _on_joints(message: JointState) -> None:
        latest[:] = list(message.position[:6])

    node.create_subscription(JointState, f"/hdas/feedback_arm_{arm}", _on_joints, qos)
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and len(latest) < 6:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return tuple(float(value) for value in latest) if len(latest) == 6 else None


def _cmd_status(args, api: _ArmApi) -> int:
    reading = api.get_pose(arm=args.arm, timeout_sec=args.timeout)
    joints = _read_joint_positions(args.arm, min(args.timeout, 1.0))
    message = (
        f"{args.arm}: xyz={_fmt_xyz(*reading.position_xyz)} "
        f"frame={reading.frame_id or '?'} stamp={reading.stamp_sec:.3f}"
    )
    if joints is not None:
        message += f" | joints={_fmt_joints(joints)}"
    else:
        message += " | joints=unavailable"
    print(message, flush=True)
    return 0


def _cmd_move(args, api: _ArmApi) -> int:
    started_s = time.monotonic()
    result = api.move_to(
        args.x, args.y, args.z, arm=args.arm, frame_id=args.frame,
        timeout_sec=args.timeout, execute=args.execute,
    )
    return _finish_pose(result, execute=args.execute, started_s=started_s, verb="move completed")


def _cmd_shift(args, api: _ArmApi) -> int:
    started_s = time.monotonic()
    result = api.move_by(
        args.dx, args.dy, args.dz, arm=args.arm, frame_id=args.frame,
        timeout_sec=args.timeout, execute=args.execute,
    )
    return _finish_pose(result, execute=args.execute, started_s=started_s, verb="shift completed")


def _cmd_joint(args, api: _ArmApi) -> int:
    positions = (args.j1, args.j2, args.j3, args.j4, args.j5, args.j6)
    started_s = time.monotonic()
    result = api.move_joints(
        positions, arm=args.arm, speed_scale=args.speed,
        timeout_sec=args.timeout, execute=args.execute,
    )
    return _finish_joint(result, execute=args.execute, started_s=started_s)


def _add_execute_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout", type=_nonnegative_float, default=0.0,
                        metavar="SEC", help="timeout in seconds (0 = server default)")
    parser.add_argument("--execute", action="store_true",
                        help="request physical execution; server startup must also allow it")


def _add_pose_command(subparsers, name: str, help_text: str, axes: Tuple[str, ...]) -> None:
    parser = subparsers.add_parser(name, help=help_text)
    parser.add_argument("arm", choices=["left", "right"], help="arm to command")
    for axis in axes:
        parser.add_argument(axis, type=_finite_float, required=True, metavar="M")
    parser.add_argument("--frame", type=_frame_name, default="base_link", metavar="FRAME")
    _add_execute_args(parser)


def _add_joint_command(subparsers) -> None:
    parser = subparsers.add_parser("joint", help="move to absolute joint positions")
    parser.add_argument("arm", choices=["left", "right"], help="arm to command")
    for number in range(1, 7):
        parser.add_argument(f"--j{number}", type=_finite_float, required=True, metavar="RAD")
    parser.add_argument("--speed", type=_speed_scale, default=0.2, metavar="SCALE",
                        help="speed scale in (0, 1] (default: 0.2)")
    _add_execute_args(parser)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        "arm", description="R1 Lite arm control with service-side previews",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    status = subparsers.add_parser("status", help="show current end-effector pose and joints")
    status.add_argument("arm", choices=["left", "right"], help="arm to query")
    status.add_argument("--timeout", type=_positive_float, default=3.0, metavar="SEC")
    _add_pose_command(subparsers, "move", "move to an absolute Cartesian position", ("--x", "--y", "--z"))
    _add_pose_command(subparsers, "shift", "move by a Cartesian offset", ("--dx", "--dy", "--dz"))
    _add_joint_command(subparsers)
    return parser


def main(argv: Optional[Sequence[str]] = None, api: Optional[_ArmApi] = None) -> int:
    """Run the console entry point and return a normal process exit code."""

    args = _build_parser().parse_args(argv)
    api = api or _load_api()
    handlers = {
        "status": _cmd_status,
        "move": _cmd_move,
        "shift": _cmd_shift,
        "joint": _cmd_joint,
    }
    try:
        return handlers[args.command](args, api)
    except KeyboardInterrupt:
        return _fail("cancelled by user; no new goal will be sent")
    except Exception as exc:
        return _fail(f"request failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
