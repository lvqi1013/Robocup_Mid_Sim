"""ROS1 ``AssistRole`` 的同名 Python 角色."""

from .constants import Role
from .models import ActionDecision, WorldState


class AssistRole:
    """助攻角色占位实现；ROS1 中仅存在构造函数和接口声明."""

    def __init__(self) -> None:
        self.world_model_ = None
        self.plan_ = None
        self.isInOurfeild_ = False
        self.IsOurDribble_ = False

    def process(self, world: WorldState) -> ActionDecision:
        """返回安全停止决策，等待参赛者补写助攻站位策略."""
        robot = world.self_robot
        return ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            role=int(Role.ASSISTANT),
        )

    def assistCalculate(self) -> None:
        """保留 ROS1 尚未实现的同名计算入口."""

    def isBestPass(self, world_pt) -> bool:
        """ROS1 未实现；当前不宣称任何点为最佳接球点."""
        return False

    def isObsActiver(self, candidate_point) -> bool:
        """ROS1 未实现；当前不进行主攻射门线判定."""
        return False

    def isNullInTrap(self, robot_pos, obs_pts) -> bool:
        """在没有障碍物时认为周边为空."""
        return not obs_pts
