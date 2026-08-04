"""Duojin's production boundary for vendor robot interfaces.

ROS-dependent client symbols are loaded lazily so the pure domain module can
still be tested on the documented non-ROS development machine.
"""

__all__ = [
    "ArmClient",
    "ArmPoseReading",
    "ArmResult",
    "get_pose",
    "move_by",
    "move_joints",
    "move_to",
]


def __getattr__(name):
    if name in __all__:
        from importlib import import_module

        module_name = (
            ".arm_client_result" if name == "ArmPoseReading" else ".arm_client"
        )
        module = import_module(module_name, __name__)
        return getattr(module, name)
    raise AttributeError(name)
