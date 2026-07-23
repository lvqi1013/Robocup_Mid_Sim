"""ROS1 ``goaliestrategy`` 中同名类的 Python 迁移."""

from enum import IntEnum

from .constants import Role
from .geometry import Point
from .models import ActionDecision, BallState, RobotState, WorldState


class ParabolaFitter3D:
    """保留 ROS1 尚未接入主循环的三维球轨迹拟合器状态."""

    def __init__(self) -> None:
        self.model_param_ = [0.0] * 6
        self.bounding_point_ = Point()
        self.bounding_time_ = 0.0
        self.crossing_point_ = Point()
        self.crossing_time_ = 0.0
        self.n_ = 0
        self.fly_flag_ = 0
        self.data_pointer_ = -1

    def flyCheckAndAddData(self, z_now, pos_now, timestamp) -> bool:
        """ROS1 只有声明；当前不进行三维飞行球判断."""
        return False

    def clearDataBuffer(self) -> None:
        """清空轨迹拟合状态."""
        self.n_ = 0
        self.fly_flag_ = 0
        self.data_pointer_ = -1

    def fitting(self, fitting_error=None) -> None:
        """保留 ROS1 尚未实现的拟合入口."""

    def saveFileTXT(self, filename) -> None:
        """保留 ROS1 尚未实现的数据导出入口."""

    def getStartTime(self) -> float:
        """没有轨迹数据时返回零."""
        return 0.0

    def getEndTime(self) -> float:
        """没有轨迹数据时返回零."""
        return 0.0


class GoalieStrategy:
    """守门员角色；当前保留 ROS1 初始化和安全停止行为."""

    class GoalieState(IntEnum):
        """与 ROS1 守门员有限状态机编号一致."""

        StandBy = 0
        Move2Ball = 1
        Move2Origin = 2
        Turn2Ball = 3

    def __init__(self) -> None:
        self.robot_info_ = RobotState(
            agent_id=1,
            position=Point(-850.0, 0.0),
            heading=0.0,
        )
        self.ball_info_2d_ = BallState(known=False)
        self.ball_info1_3d_ = None
        self.ball_info2_3d_ = None
        self.ball_info_3d_ = None
        self.state_ = self.GoalieState.Move2Origin
        self.dest_point_ = Point(-850.0, 0.0)
        self.dest_angle_ = 0.0
        self.thresh_vel_ = 0.0
        self.thresh_omiga_ = 0.0
        self.debug_str_ = ''
        self.parabola_fitter_ = ParabolaFitter3D()
        self.predicted_3d_ = False
        self.predicted_2d_ = False
        self.predictec_omi_ = False

    def process(self, world: WorldState) -> ActionDecision:
        """返回安全停止决策；原 ROS1 守门策略没有可运行实现."""
        robot = world.self_robot
        return ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            role=int(Role.GOALIE),
        )

    def setBallInfo3dRel(self, robot_info_3d_rel) -> None:
        """保存相对三维球信息，保留 ROS1 同名入口."""
        self.ball_info_3d_ = robot_info_3d_rel

    def ballTrack(
        self, THRESH_GROUND_VEL=200, use_parabola_fitter_=True
    ) -> bool:
        """ROS1 只有声明；当前不报告飞行球入门预测."""
        return False

    def strategy(self, world: WorldState) -> ActionDecision:
        """兼容 ROS1 同名入口，使用当前安全守门决策."""
        return self.process(world)
