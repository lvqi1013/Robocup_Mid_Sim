import math

from nubot_control.geometry import normalize_angle, Point
import pytest


def test_point_towards_matches_legacy_pointofline():
    assert Point(0.0, 0.0).point_towards(Point(10.0, 0.0), 3.0) == Point(
        3.0, 0.0
    )


def test_normalize_angle_crosses_pi_boundary():
    assert normalize_angle(2.0 * math.pi + 0.2) == pytest.approx(0.2)
    assert normalize_angle(-math.pi) == pytest.approx(math.pi)
