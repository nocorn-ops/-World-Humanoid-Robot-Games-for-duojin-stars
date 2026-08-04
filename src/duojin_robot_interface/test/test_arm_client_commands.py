"""ROS-free contract tests for public Python goal construction."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COMMANDS_MODULE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_client_commands.py"


def _message():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=""),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        ),
        vector=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )


class _Action:
    class Goal:
        pass


def _load_commands_module():
    interfaces = ModuleType("duojin_interfaces.action")
    interfaces.MoveArmJoints = _Action
    interfaces.MoveArmPose = _Action
    interfaces.MoveArmRelative = _Action
    geometry = ModuleType("geometry_msgs.msg")
    geometry.PoseStamped = _message
    geometry.Vector3Stamped = _message
    results = ModuleType("duojin_robot_interface.arm_client_result")
    results.ArmPoseReading = object
    results.ArmResult = object
    results.map_joint_result = object()
    results.map_pose_result = object()
    results.map_pose_reading = lambda message: message
    stubs = {
        "duojin_interfaces.action": interfaces,
        "geometry_msgs.msg": geometry,
        "duojin_robot_interface.arm_client_result": results,
    }
    name = "duojin_robot_interface.arm_client_commands_under_test"
    spec = importlib.util.spec_from_file_location(name, COMMANDS_MODULE)
    module = importlib.util.module_from_spec(spec)
    previous = {key: sys.modules.get(key) for key in (*stubs, name)}
    try:
        sys.modules.update(stubs)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    finally:
        for key, old in previous.items():
            if old is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = old
    return module


COMMANDS = _load_commands_module()


def test_move_by_builds_a_base_link_relative_goal_and_uses_relative_action() -> None:
    captured = {}
    client = SimpleNamespace(
        _runtime=SimpleNamespace(
            relative_client="relative-action",
            relative_cancel_client="relative-cancel",
        )
    )

    def run_goal(*args):
        captured["args"] = args
        return "result"

    client._run_goal = run_goal
    result = COMMANDS.ArmClientCommands.move_by(client, 0.01, -0.02, 0.03)

    action_client, cancel_client, goal, _, mapper, callback = captured["args"]
    assert result == "result"
    assert action_client == "relative-action"
    assert cancel_client == "relative-cancel"
    assert goal.delta.header.frame_id == "base_link"
    assert (goal.delta.vector.x, goal.delta.vector.y, goal.delta.vector.z) == (
        0.01,
        -0.02,
        0.03,
    )
    assert goal.keep_current_orientation is True
    assert goal.execute is False
    assert mapper is COMMANDS.map_pose_result
    assert callback is None
