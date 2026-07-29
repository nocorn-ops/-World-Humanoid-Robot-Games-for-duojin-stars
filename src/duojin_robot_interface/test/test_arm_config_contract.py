"""ROS-free checks that runtime parameter fallbacks match the shipped YAML."""

import ast
from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CONFIG_SOURCE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_server_config.py"
POSE_ACTION_SOURCE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_pose_action.py"
SDK_ADAPTER_SOURCE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_sdk_adapter.py"
CONFIG_FILE = PACKAGE_ROOT / "config" / "arm_motion.yaml"
START_SOURCE = WORKSPACE_ROOT / "start.sh"
CONTROL_CHECK_SOURCE = WORKSPACE_ROOT / "scripts" / "check_robot_control_chains.sh"


def _safe_literal(node):
    try:
        return ast.literal_eval(node)
    except (TypeError, ValueError):
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mult):
            return _safe_literal(node.left) * _safe_literal(node.right)
        raise AssertionError(
            "server fallback defaults must remain statically inspectable literals"
        )


def _server_parameter_defaults():
    tree = ast.parse(CONFIG_SOURCE.read_text(encoding="utf-8"))
    assignment = next(
        node
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "PARAMETER_DEFAULTS"
            for target in node.targets
        )
    )
    assert isinstance(assignment.value, ast.Dict)
    return {
        _safe_literal(key): _safe_literal(value)
        for key, value in zip(assignment.value.keys, assignment.value.values)
    }


def _yaml_parameter_defaults():
    defaults = {}
    pattern = re.compile(r"^ {4}([A-Za-z][A-Za-z0-9_]*):\s*(.*?)\s*$")
    for line in CONFIG_FILE.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            defaults[match.group(1)] = _yaml_scalar(match.group(2))
    return defaults


def _yaml_scalar(text):
    if text in ("true", "false"):
        return text == "true"
    if text.startswith("[") and text.endswith("]"):
        contents = text[1:-1].strip()
        return [] if not contents else [_yaml_scalar(item.strip()) for item in contents.split(",")]
    try:
        return ast.literal_eval(text)
    except (SyntaxError, ValueError):
        return text


def test_shipped_yaml_matches_every_server_fallback_default() -> None:
    assert _yaml_parameter_defaults() == _server_parameter_defaults()


def test_both_default_sources_keep_physical_execution_disabled() -> None:
    assert _yaml_parameter_defaults()["execute"] is False
    assert _server_parameter_defaults()["execute"] is False


def test_pose_feedback_topic_matches_robot_observation() -> None:
    source = SDK_ADAPTER_SOURCE.read_text(encoding="utf-8")
    assert 'f"/relaxed_ik/motion_control/pose_ee_arm_{arm}"' in source
    assert 'f"/motion_control/pose_ee_arm_{arm}"' not in source


def test_arm_execution_profile_reports_cameras_without_requiring_them() -> None:
    start_source = START_SOURCE.read_text(encoding="utf-8")
    check_source = CONTROL_CHECK_SOURCE.read_text(encoding="utf-8")
    assert '"${DUOJIN_CONTROL_CHECK}" --arm-motion' in start_source
    assert 'DUOJIN_CAMERA_REQUIRED' not in check_source
    assert "duojin_camera_required=false" in check_source
    assert "not required by this profile" in check_source


def test_pose_execution_keeps_preview_server_gate_before_publication() -> None:
    tree = ast.parse(POSE_ACTION_SOURCE.read_text(encoding="utf-8"))
    functions = {
        node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    gate = functions["_preview_or_require_execution"]
    reasons = {
        call.args[1].attr
        for call in ast.walk(gate)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "MotionFailure"
        and len(call.args) >= 2
        and isinstance(call.args[1], ast.Attribute)
    }
    assert "EXECUTION_DISABLED" in reasons
    assert "POSE_EXECUTION_UNSAFE" not in reasons
    assert all(
        not (isinstance(call.func, ast.Name) and call.func.id == "_publish_target")
        for call in ast.walk(gate)
        if isinstance(call, ast.Call)
    )

    execute_pose = functions["_execute_pose"]
    call_lines = {
        call.func.id: call.lineno
        for call in ast.walk(execute_pose)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id in ("_preview_or_require_execution", "_publish_target")
    }
    assert call_lines["_preview_or_require_execution"] < call_lines["_publish_target"]
