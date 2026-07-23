"""ROS1 ``MidfieldRole`` 的同名 Python 角色."""

from .constants import Role
from .geometry import Point
from .models import ActionDecision, WorldState


class MidfieldRole:
    """中场角色占位实现；保留原类成员和公开入口."""

    def __init__(self) -> None:
        self.world_model_ = None
        self.plan_ = None
        self.midfield_pos_ = Point()
        self.midfield_ori_ = 0.0

    def process(self, world: WorldState) -> ActionDecision:
        """返回安全停止决策，等待参赛者补写中场策略."""
        robot = world.self_robot
        return ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            role=int(Role.MIDFIELD),
        )

    def asAssist(self) -> None:
        """保留 ROS1 尚未实现的助攻模式入口."""

    def asPassiver(self) -> None:
        """保留 ROS1 尚未实现的防守模式入口."""

    def findOppRobot(self) -> bool:
        """ROS1 未实现；当前没有选中的盯防机器人."""
        return False

    def isNullInTrap(self, robot_pos, obs_pts) -> bool:
        """在没有障碍物时认为周边为空."""
        return not obs_pts

    def isBestPass(self, world_pt) -> bool:
        """ROS1 未实现；当前不宣称任何点为最佳接球点."""
        return False

    def isObsOpp(self, candidate_point) -> bool:
        """ROS1 未实现；当前不进行对方射门线判定."""
        return False

    def isObsActiver(self, candidate_point) -> bool:
        """ROS1 未实现；当前不进行主攻射门线判定."""
        return False
