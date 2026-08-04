"""ROS-free checks for the installed ``arm`` command behaviour."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CLI_MODULE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_cli.py"


def _load_cli_module():
    name = "arm_cli_under_test"
    spec = importlib.util.spec_from_file_location(name, CLI_MODULE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous = sys.modules.get(name)
    try:
        sys.modules[name] = module
        spec.loader.exec_module(module)
    finally:
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous
    return module


CLI = _load_cli_module()


def _result(*, succeeded=True, executed=False, valid=True, message="preview complete"):
    return SimpleNamespace(
        succeeded=succeeded,
        outcome=0 if succeeded else 1,
        reason=0 if succeeded else 11,
        message=message,
        executed=executed,
        execution_state_known=True,
        final_state_valid=valid,
        final_position_xyz=(0.1, 0.2, 0.3) if valid else None,
        position_error_m=0.001 if valid else None,
        max_position_error_rad=0.002 if valid else None,
    )


def _api(**overrides):
    calls = []

    def unavailable(*_args, **_kwargs):
        raise AssertionError("unexpected CLI backend call")

    def move_to(*args, **kwargs):
        calls.append(("move_to", args, kwargs))
        return overrides.get("move_to", _result())

    def move_by(*args, **kwargs):
        calls.append(("move_by", args, kwargs))
        return overrides.get("move_by", _result())

    def move_joints(*args, **kwargs):
        calls.append(("move_joints", args, kwargs))
        return overrides.get("move_joints", _result())

    backend = CLI._ArmApi(unavailable, move_by, move_joints, move_to)
    return backend, calls


def test_preview_is_a_real_server_request_not_a_local_fake(capsys) -> None:
    api, calls = _api()

    code = CLI.main(
        ["shift", "left", "--dx", "0", "--dy", "0", "--dz", "0.01"],
        api=api,
    )

    assert code == 0
    assert calls == [
        (
            "move_by",
            (0.0, 0.0, 0.01),
            {"arm": "left", "frame_id": "base_link", "timeout_sec": 0.0, "execute": False},
        )
    ]
    assert "PREVIEW: preview complete; no motion target was published" in capsys.readouterr().out


def test_execute_forwards_the_second_execution_gate(capsys) -> None:
    api, calls = _api(move_to=_result(executed=True))

    code = CLI.main(
        ["move", "right", "--x", "0.1", "--y", "0.2", "--z", "0.3", "--execute"],
        api=api,
    )

    assert code == 0
    assert calls[0][0] == "move_to"
    assert calls[0][2]["execute"] is True
    assert "OK: move completed" in capsys.readouterr().out


def test_server_failure_is_cleanly_reported_without_a_traceback(capsys) -> None:
    api, calls = _api(move_by=_result(succeeded=False, valid=False, message="TF unavailable"))

    code = CLI.main(
        ["shift", "left", "--dx", "0", "--dy", "0", "--dz", "0.01"],
        api=api,
    )

    assert code == 1
    assert calls[0][2]["execute"] is False
    output = capsys.readouterr()
    assert "FAIL: TF unavailable" in output.err
    assert "Traceback" not in output.err


def test_status_uses_public_pose_api_and_keeps_joint_feedback_optional(monkeypatch, capsys) -> None:
    reading = SimpleNamespace(
        position_xyz=(0.1, 0.2, 0.3), frame_id="base_link", stamp_sec=12.5,
    )
    api = CLI._ArmApi(lambda **kwargs: reading, None, None, None)
    monkeypatch.setattr(CLI, "_read_joint_positions", lambda *_args: None)

    code = CLI.main(["status", "left"], api=api)

    assert code == 0
    assert "left: xyz=(0.1000, 0.2000, 0.3000) m frame=base_link" in capsys.readouterr().out


def test_parser_rejects_nan_and_invalid_speed() -> None:
    with pytest.raises(SystemExit, match="2"):
        CLI.main(["move", "left", "--x", "nan", "--y", "0", "--z", "0"])
    with pytest.raises(SystemExit, match="2"):
        CLI.main([
            "joint", "left", "--j1", "0", "--j2", "0", "--j3", "0",
            "--j4", "0", "--j5", "0", "--j6", "0", "--speed", "1.1",
        ])
