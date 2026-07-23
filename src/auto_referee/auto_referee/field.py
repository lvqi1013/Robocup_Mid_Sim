"""Pure-Python RoboCup MSL field geometry used by the referee."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point:
    """Two-dimensional point in centimetres."""

    x: float = 0.0
    y: float = 0.0

    def distance(self, other: 'Point') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


class Field:
    """Field dimensions and containment tests from FieldInformation."""

    length = 2200.0
    width = 1400.0
    half_length = length / 2.0
    half_width = width / 2.0

    ball_radius = 11.0
    goal_height = 101.0
    goal_depth = 75.0
    goal_width = 240.0
    goal_opening_half_width = 120.0

    penalty_inner_x = 875.0
    penalty_half_width = 345.0
    goal_area_inner_x = 1025.0
    goal_area_half_width = 195.0

    corner_x = half_length - 30.0
    corner_y = half_width - 30.0
    restart_x = half_length - 360.0
    restart_y = width / 4.0

    def is_out_left(self, point: Point) -> bool:
        return point.x < -self.half_length

    def is_out_right(self, point: Point) -> bool:
        return point.x > self.half_length

    def is_out_up(self, point: Point) -> bool:
        return point.y > self.half_width

    def is_out_down(self, point: Point) -> bool:
        return point.y < -self.half_width

    def is_goal(self, point: Point, z: float) -> bool:
        in_mouth = abs(point.y) < (
            self.goal_opening_half_width - self.ball_radius
        )
        below_crossbar = abs(z) < self.goal_height - self.ball_radius
        in_goal_depth = (
            -self.half_length - self.goal_depth
            < point.x
            < self.half_length + self.goal_depth
        )
        return in_mouth and below_crossbar and in_goal_depth and (
            self.is_out_left(point) or self.is_out_right(point)
        )

    def is_our_penalty(self, point: Point) -> bool:
        return (
            -self.half_length < point.x < -self.penalty_inner_x
            and abs(point.y) < self.penalty_half_width
        )

    def is_opp_penalty(self, point: Point) -> bool:
        return (
            self.penalty_inner_x < point.x < self.half_length
            and abs(point.y) < self.penalty_half_width
        )

    def is_our_goal_area(self, point: Point) -> bool:
        return (
            -self.half_length < point.x < -self.goal_area_inner_x
            and abs(point.y) < self.goal_area_half_width
        )

    def is_opp_goal_area(self, point: Point) -> bool:
        return (
            self.goal_area_inner_x < point.x < self.half_length
            and abs(point.y) < self.goal_area_half_width
        )

    def is_goal_pole_area(self, point: Point) -> bool:
        in_y = abs(point.y) < self.goal_width / 2.0
        left = (
            -self.half_length - self.goal_depth
            < point.x
            < -self.half_length
        )
        right = (
            self.half_length
            < point.x
            < self.half_length + self.goal_depth
        )
        return in_y and (left or right)

    def restart_outside_penalty(self, point: Point) -> Point:
        if self.is_opp_penalty(point):
            y = self.restart_y if point.y >= 0.0 else -self.restart_y
            return Point(self.restart_x, y)
        if self.is_our_penalty(point):
            y = self.restart_y if point.y >= 0.0 else -self.restart_y
            return Point(-self.restart_x, y)
        return point

    def nearest_corner(self, right: bool, y: float) -> Point:
        return Point(
            self.corner_x if right else -self.corner_x,
            self.corner_y if y >= 0.0 else -self.corner_y,
        )

    def nearest_goal_kick_restart(self, right: bool, y: float) -> Point:
        return Point(
            self.restart_x if right else -self.restart_x,
            self.restart_y if y >= 0.0 else -self.restart_y,
        )
