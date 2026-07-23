"""ROS1 ``PassiveRole`` 的同名 Python 角色."""

from .constants import Role
from .geometry import Point
from .models import ActionDecision, WorldState


class PassiveRole:
    """防守角色占位实现；保留原类成员和公开入口."""

    def __init__(self) -> None:
        self.world_model_ = None
        self.plan_ = None
        self.isInOurfeild_ = False
        self.IsOurDribble_ = False
        self.defence_pos_ = Point()
        self.defence_ori_ = 0.0

    def process(self, world: WorldState) -> ActionDecision:
        """返回安全停止决策，等待参赛者补写防守站位策略."""
        robot = world.self_robot
        return ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            role=int(Role.PASSIVE),
        )

    def passiveCalculate(self) -> None:
        """保留 ROS1 尚未实现的防守位置计算入口."""

    def findPointOut(self, ang_goal2ball) -> None:
        """保留 ROS1 尚未实现的禁区外目标计算入口."""

    def awayFromActive(self) -> None:
        """保留 ROS1 尚未实现的主攻避让入口."""
