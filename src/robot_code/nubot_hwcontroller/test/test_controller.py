import math

from nubot_hwcontroller.controller import (
    directed_angle_error,
    MotionController,
    MotionTarget,
    NO_ACTION,
)
import pytest


def test_world_target_is_converted_to_robot_frame():
    """目标在世界 x 正方向、机器人朝向 pi/2 时，应转换成本体 y 负方向速度。"""
    controller = MotionController()
    target = MotionTarget(
        target_x=100.0,
        robot_orientation=math.pi / 2.0,
        max_velocity=200.0,
        max_angular_velocity=3.0,
        move_action=16,
        rotate_action=NO_ACTION,
    )

    velocity = controller.calculate(target)

    assert velocity.vx == pytest.approx(0.0, abs=1e-5)
    assert velocity.vy == pytest.approx(-200.0)
    assert velocity.w == 0.0


def test_no_action_stops_only_its_own_axis_group():
    """NO_ACTION 只关闭对应轴组：平移停止时旋转控制仍应生效。"""
    controller = MotionController()
    target = MotionTarget(
        target_x=100.0,
        target_orientation=1.0,
        max_velocity=200.0,
        max_angular_velocity=3.0,
        move_action=NO_ACTION,
        rotate_action=16,
    )

    velocity = controller.calculate(target)

    assert velocity.vx == 0.0
    assert velocity.vy == 0.0
    assert velocity.w == pytest.approx(2.0)


def test_rotate_mode_preserves_requested_direction():
    """rotate_mode 为 1/-1 时保持指定旋转方向，为 0 时取最短角度误差。"""
    assert directed_angle_error(-0.1, 0.1, 1) > 0.0
    assert directed_angle_error(0.1, -0.1, -1) < 0.0
    assert directed_angle_error(-3.0, 3.0, 0) == pytest.approx(
        2.0 * math.pi - 6.0
    )
