"""控制策略使用的轻量二维几何工具."""

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    """把角度归一化到 ``(-pi, pi]``."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass(frozen=True)
class Point:
    """以厘米为单位的世界坐标二维点."""

    x: float = 0.0
    y: float = 0.0

    def distance(self, other: 'Point') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def point_towards(self, other: 'Point', distance: float) -> 'Point':
        """返回从当前点向 ``other`` 前进指定距离后的点."""
        direction = math.atan2(other.y - self.y, other.x - self.x)
        return Point(
            self.x + math.cos(direction) * distance,
            self.y + math.sin(direction) * distance,
        )

    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y)
