"""与当前 RoboCup 中型组仿真场地一致的关键位置."""

from .geometry import Point


class Field:
    """只保存当前已实现策略实际使用的场地信息."""

    own_goal = Point(-1100.0, 0.0)
    opponent_goal = Point(1100.0, 0.0)
    opponent_goal_mid_upper = Point(1100.0, 60.0)
    opponent_goal_mid_lower = Point(1100.0, -60.0)

    @staticmethod
    def is_own_penalty(point: Point) -> bool:
        return -1100.0 < point.x < -875.0 and abs(point.y) < 345.0
