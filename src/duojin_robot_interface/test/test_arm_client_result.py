"""ROS-free contract tests for the public Python arm result mapping."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RESULT_MODULE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_client_result.py"


class GoalStatusStub:
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class ArmMotionStatusStub:
    SUCCESS = 0
    RETRYABLE_FAILURE = 1
    FATAL_FAILURE = 2
    CANCELLED = 3
    TIMEOUT = 4

    NONE = 0
    SERVER_UNAVAILABLE = 5
    EXECUTION_STATE_UNKNOWN = 14


def _load_result_module():
    """Load only arm_client_result with generated ROS modules replaced by stubs."""

    action_msgs = ModuleType("action_msgs")
    action_msgs_msg = ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = GoalStatusStub
    action_msgs.msg = action_msgs_msg
    interfaces = ModuleType("duojin_interfaces")
    interfaces_msg = ModuleType("duojin_interfaces.msg")
    interfaces_msg.ArmMotionStatus = ArmMotionStatusStub
    interfaces.msg = interfaces_msg
    stubs = {
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "duojin_interfaces": interfaces,
        "duojin_interfaces.msg": interfaces_msg,
    }
    module_name = "_duojin_arm_client_result_under_test"
    spec = importlib.util.spec_from_file_location(module_name, RESULT_MODULE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous = {name: sys.modules.get(name) for name in (*stubs, module_name)}
    try:
        sys.modules.update(stubs)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    finally:
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module
    return module


RESULTS = _load_result_module()


def _status(outcome, *, reason=ArmMotionStatusStub.NONE, executed=False):
    return SimpleNamespace(
        outcome=outcome,
        reason=reason,
        message=f"outcome={outcome}",
        executed=executed,
    )


def _pose(frame_id="base_link"):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id, stamp=SimpleNamespace(sec=12, nanosec=500_000_000)
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.11, y=-0.22, z=0.33),
            orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9),
        ),
    )


def _pose_result(transport, outcome, *, valid=True, executed=False):
    payload = SimpleNamespace(
        status=_status(outcome, executed=executed),
        final_pose=_pose("shelf_frame"),
        final_pose_valid=valid,
        position_error_m=0.004,
        orientation_error_rad=0.03,
    )
    return SimpleNamespace(status=transport, result=payload)


def _joint_result(transport, outcome, *, valid=True, executed=False):
    payload = SimpleNamespace(
        status=_status(outcome, executed=executed),
        final_positions_rad=[0.1, 0.2, -0.3, 0.4, -0.5, 0.6],
        final_positions_valid=valid,
        max_position_error_rad=0.012,
    )
    return SimpleNamespace(status=transport, result=payload)


@pytest.mark.parametrize(
    ("outcome", "transport", "succeeded"),
    [
        (ArmMotionStatusStub.SUCCESS, GoalStatusStub.STATUS_SUCCEEDED, True),
        (ArmMotionStatusStub.CANCELLED, GoalStatusStub.STATUS_CANCELED, False),
        (ArmMotionStatusStub.RETRYABLE_FAILURE, GoalStatusStub.STATUS_ABORTED, False),
        (ArmMotionStatusStub.FATAL_FAILURE, GoalStatusStub.STATUS_ABORTED, False),
        (ArmMotionStatusStub.TIMEOUT, GoalStatusStub.STATUS_ABORTED, False),
    ],
)
def test_transport_terminal_state_must_match_payload_outcome(
    outcome, transport, succeeded
) -> None:
    mapped = RESULTS.map_joint_result(
        _joint_result(
            transport,
            outcome,
            valid=outcome == ArmMotionStatusStub.SUCCESS,
            executed=True,
        )
    )

    assert mapped.succeeded is succeeded
    assert mapped.outcome == outcome
    assert mapped.executed is True
    assert mapped.execution_state_known is True


@pytest.mark.parametrize("mapper", ["pose", "joint"])
def test_success_without_valid_final_state_is_execution_unknown(mapper) -> None:
    wrapped = (
        _pose_result(
            GoalStatusStub.STATUS_SUCCEEDED,
            ArmMotionStatusStub.SUCCESS,
            valid=False,
        )
        if mapper == "pose"
        else _joint_result(
            GoalStatusStub.STATUS_SUCCEEDED,
            ArmMotionStatusStub.SUCCESS,
            valid=False,
        )
    )
    mapped = getattr(RESULTS, f"map_{mapper}_result")(wrapped)

    assert mapped.succeeded is False
    assert mapped.execution_state_known is False
    assert mapped.executed is None
    assert mapped.reason == ArmMotionStatusStub.EXECUTION_STATE_UNKNOWN


@pytest.mark.parametrize(
    ("transport", "outcome"),
    [
        (GoalStatusStub.STATUS_ABORTED, ArmMotionStatusStub.SUCCESS),
        (GoalStatusStub.STATUS_SUCCEEDED, ArmMotionStatusStub.RETRYABLE_FAILURE),
        (GoalStatusStub.STATUS_ABORTED, ArmMotionStatusStub.CANCELLED),
        (GoalStatusStub.STATUS_ACCEPTED, ArmMotionStatusStub.SUCCESS),
        (GoalStatusStub.STATUS_ABORTED, 99),
    ],
)
def test_transport_payload_mismatch_is_execution_unknown(transport, outcome) -> None:
    mapped = RESULTS.map_pose_result(_pose_result(transport, outcome))

    assert mapped.succeeded is False
    assert mapped.outcome == ArmMotionStatusStub.FATAL_FAILURE
    assert mapped.reason == ArmMotionStatusStub.EXECUTION_STATE_UNKNOWN
    assert mapped.executed is None
    assert mapped.execution_state_known is False
    assert mapped.final_state_valid is False
    assert "UNKNOWN" in mapped.message
    assert "do not send another goal" in mapped.message


def test_pose_valid_result_preserves_frame_position_and_orientation() -> None:
    mapped = RESULTS.map_pose_result(
        _pose_result(
            GoalStatusStub.STATUS_SUCCEEDED,
            ArmMotionStatusStub.SUCCESS,
            executed=True,
        )
    )

    assert mapped.final_state_valid is True
    assert mapped.final_frame_id == "shelf_frame"
    assert mapped.final_position_xyz == pytest.approx((0.11, -0.22, 0.33))
    assert mapped.final_orientation_xyzw == pytest.approx((0.1, 0.2, 0.3, 0.9))
    assert mapped.position_error_m == pytest.approx(0.004)
    assert mapped.orientation_error_rad == pytest.approx(0.03)


def test_public_pose_reading_preserves_frame_pose_and_stamp() -> None:
    mapped = RESULTS.map_pose_reading(_pose("base_link"))

    assert mapped.frame_id == "base_link"
    assert mapped.position_xyz == pytest.approx((0.11, -0.22, 0.33))
    assert mapped.orientation_xyzw == pytest.approx((0.1, 0.2, 0.3, 0.9))
    assert mapped.stamp_sec == pytest.approx(12.5)


def test_pose_invalid_result_hides_all_placeholder_final_fields() -> None:
    mapped = RESULTS.map_pose_result(
        _pose_result(
            GoalStatusStub.STATUS_ABORTED,
            ArmMotionStatusStub.TIMEOUT,
            valid=False,
            executed=True,
        )
    )

    assert mapped.final_state_valid is False
    assert mapped.final_frame_id is None
    assert mapped.final_position_xyz is None
    assert mapped.final_orientation_xyzw is None
    assert mapped.position_error_m is None
    assert mapped.orientation_error_rad is None


def test_joint_validity_controls_final_vector_and_error_mapping() -> None:
    valid = RESULTS.map_joint_result(
        _joint_result(GoalStatusStub.STATUS_SUCCEEDED, ArmMotionStatusStub.SUCCESS)
    )
    invalid = RESULTS.map_joint_result(
        _joint_result(
            GoalStatusStub.STATUS_ABORTED,
            ArmMotionStatusStub.RETRYABLE_FAILURE,
            valid=False,
        )
    )

    assert valid.final_state_valid is True
    assert valid.final_positions_rad == pytest.approx((0.1, 0.2, -0.3, 0.4, -0.5, 0.6))
    assert valid.max_position_error_rad == pytest.approx(0.012)
    assert invalid.final_state_valid is False
    assert invalid.final_positions_rad is None
    assert invalid.max_position_error_rad is None


def test_unknown_execution_never_claims_target_was_not_executed() -> None:
    mapped = RESULTS.unknown_execution("result wait timed out")

    assert mapped.executed is None
    assert mapped.execution_state_known is False
    assert mapped.outcome == ArmMotionStatusStub.FATAL_FAILURE
    assert mapped.reason == ArmMotionStatusStub.EXECUTION_STATE_UNKNOWN
    assert "result wait timed out" in mapped.message
