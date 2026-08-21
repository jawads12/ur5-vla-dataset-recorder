import math

import pytest

from ur5_vla_dataset_recorder.core import (
    UR5_JOINT_ORDER,
    ValidationError,
    normalized_gripper_value,
    reorder_command,
    reorder_joint_positions,
)


def test_joint_state_is_reordered_by_name():
    names = [
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "shoulder_pan_joint",
    ]
    positions = [-1.0, -2.0, -3.0, -4.0, -5.0, 0.5]
    assert reorder_joint_positions(names, positions) == [0.5, -1.0, -2.0, -3.0, -4.0, -5.0]


def test_missing_joint_is_rejected():
    with pytest.raises(ValidationError, match="missing joints"):
        reorder_joint_positions(["shoulder_pan_joint"], [0.0])


def test_non_finite_command_is_rejected():
    values = [0.0] * 5 + [math.nan]
    with pytest.raises(ValidationError, match="NaN"):
        reorder_command(values, UR5_JOINT_ORDER)


def test_normalized_gripper_value_accepts_binary_states():
    assert normalized_gripper_value(0.0, "gripper") == 0.0
    assert normalized_gripper_value(1.0, "gripper") == 1.0


def test_normalized_gripper_value_rejects_out_of_range_state():
    with pytest.raises(ValidationError, match="range"):
        normalized_gripper_value(1.5, "gripper")
