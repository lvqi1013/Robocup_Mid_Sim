"""ROS1 ``ActiveRole`` 的 Python 迁移实现."""

from .constants import (
    Action,
    DEG_TO_RAD,
    MAX_ANGULAR_VELOCITY,
    MAX_VELOCITY,
    Role,
)
from .field import Field
from .geometry import normalize_angle, Point
from .models import ActionDecision, WorldState


class ActiveRole:
    """主攻角色：执行原 ``nubot_control.cpp`` 中实际生效的主动行为."""

    def __init__(self) -> None:
        # 保留 ROS1 activerole.cpp 构造函数中的状态。
        self.stuckflg_ = False
        self.pass_lock_ = 0
        self.stucktime_ = 0
        self.NeedEvaluat = False
        self.dynamic_shoot_state_ = False
        self.dynamic_shoot_count_ = 0
        self.quick_shoot_state_ = False
        self.quick_shoot_count_ = 0
        self.currentstate_ = int(Action.CANNOT_SEE_BALL)
        self.catch_in_ourfeild_ = False
        self.kick_enable_ = False
        self.dribble_enable_ = False
        self.kick_force_ = 0.0
        self.kick_target_ = Point()
        self.target_shoot_ = Point()
        self.ldetAng_shoot_ = 0.0
        self.rdetAng_shoot_ = 0.0

        self.field = Field()
        self.shoot_cooldown_ticks = 0

    def process(
        self, world: WorldState, ball_holding: bool
    ) -> ActionDecision:
        """执行追球、带球和射门；射门后短暂输出停止动作."""
        robot = world.self_robot
        if self.shoot_cooldown_ticks > 0:
            self.shoot_cooldown_ticks -= 1
            return ActionDecision(
                target=robot.position,
                target_orientation=robot.heading,
                role=int(Role.ACTIVE),
                active=True,
            )

        is_dribbling = ball_holding or robot.dribbling
        self.dribble_enable_ = is_dribbling
        if not is_dribbling:
            return self.activeCatchBall(world)
        return self._dribble_and_shoot(world)

    def clearActiveState(self) -> None:
        """清除一次性射门状态，保留 ROS1 同名接口."""
        self.dynamic_shoot_state_ = False
        self.dynamic_shoot_count_ = 0
        self.quick_shoot_state_ = False
        self.quick_shoot_count_ = 0
        self.kick_enable_ = False
        self.kick_force_ = 0.0
        self.shoot_cooldown_ticks = 0

    def checkPass(self) -> bool:
        """ROS1 只有声明；当前不触发传球."""
        return False

    def isNullInTrap(
        self,
        obs_info,
        robot_pos,
        direction=0.0,
        back_width=50.0,
        front_width=75.0,
        back_len=-25.0,
        front_len=100.0,
    ) -> bool:
        """在没有障碍物时认为检测区域为空."""
        return not obs_info

    def pnpoly(self, pts, obs_pts) -> bool:
        """ROS1 只有声明；当前不执行多边形障碍物判定."""
        return False

    def activeDecisionMaking(self) -> None:
        """保留 ROS1 尚未实现的主动决策入口."""

    def selectCurrentState(self) -> None:
        """保留 ROS1 尚未实现的状态选择入口."""

    def selectCurrentAction(self, state) -> None:
        """保留 ROS1 尚未实现的动作选择入口."""

    def evaluateKick(self, kick_target, leftdelta, rightdelta) -> bool:
        """ROS1 只有声明；当前不报告复杂射门评估成功."""
        return False

    def caculatePassEnergy(self, energy=0.0, label=0) -> tuple[float, int]:
        """返回中性传球效能，保留 ROS1 原拼写."""
        return 0.0, 0

    def caculateDribblingEnergy(
        self, avoid_enegy=0.0, isNullFrontRobot=False
    ) -> float:
        """返回中性带球效能，保留 ROS1 原拼写."""
        return 0.0

    def selectDribblingOrPassing(self, isNullFrontRobot) -> None:
        """保留 ROS1 尚未实现的带球/传球选择入口."""

    def findBall(self) -> None:
        """保留 ROS1 尚未实现的找球入口."""

    def turn4Shoot(self, kicktarget) -> None:
        """保留 ROS1 尚未实现的复杂射门转向入口."""

    def NewAvoidObs(self) -> None:
        """保留 ROS1 尚未实现的避障入口."""

    def NewAvoidObsForPass(self) -> None:
        """保留 ROS1 尚未实现的传球避障入口."""

    def stuckProcess(self) -> None:
        """保留 ROS1 尚未实现的堵转处理入口."""

    def kickball4Coop(self, target) -> None:
        """保留 ROS1 尚未实现的协作传球入口."""

    def IsLocationInOppGoalArea(self, location: Point) -> bool:
        """判断位置是否处于对方球门口的简化矩形区域."""
        return location.x >= 1050.0 and abs(location.y) <= 100.0

    def IsLocationInField(self, location: Point) -> bool:
        """判断位置是否位于当前 2200 cm × 1400 cm 场地内."""
        return abs(location.x) <= 1100.0 and abs(location.y) <= 700.0

    def IsLocationInOppPenalty(self, location: Point) -> bool:
        """判断位置是否位于对方禁区."""
        return 875.0 < location.x < 1100.0 and abs(location.y) < 345.0

    def IsLocationInOurPenalty(self, location: Point) -> bool:
        """判断位置是否位于己方禁区."""
        return self.field.is_own_penalty(location)

    def IsLocationInOppField(self, location: Point) -> bool:
        """判断位置是否位于对方半场."""
        return self.IsLocationInField(location) and location.x > 0.0

    def IsLocationInOurField(self, location: Point) -> bool:
        """判断位置是否位于己方半场."""
        return self.IsLocationInField(location) and location.x <= 0.0

    def activeCatchBall(self, world: WorldState) -> ActionDecision:
        """朝向足球后接近足球，对应 ROS1 当前简单接球行为."""
        robot = world.self_robot
        ball = world.ball.position
        decision = ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            move_action=int(Action.CATCH_BALL),
            rotate_action=int(Action.CATCH_BALL),
            handle_enable=1,
            role=int(Role.ACTIVE),
            active=True,
        )
        aligned = self._set_orientation(
            decision, robot.heading, (ball - robot.position).angle()
        )
        if aligned:
            self._set_translation(decision, robot.position, ball)
        return decision

    def triggerShoot(
        self, decision: ActionDecision, strength: float
    ) -> None:
        """产生一次射门请求，并进入与 ROS1 相同的 21 周期冷却."""
        decision.shoot_position = 1
        decision.strength = max(3.0, strength)
        decision.handle_enable = 0
        self.kick_enable_ = True
        self.kick_force_ = decision.strength
        self.shoot_cooldown_ticks = 21

    def _dribble_and_shoot(self, world: WorldState) -> ActionDecision:
        robot = world.self_robot
        dribble_target = Point(200.0, 300.0)
        goal_vector = self.field.opponent_goal - robot.position
        decision = ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
            handle_enable=1,
            role=int(Role.ACTIVE),
            active=True,
        )

        if robot.position.distance(dribble_target) > 30.0:
            decision.move_action = int(Action.MOVE_WITH_BALL)
            decision.rotate_action = int(Action.MOVE_WITH_BALL)
            aligned = self._set_orientation(
                decision,
                robot.heading,
                (dribble_target - robot.position).angle(),
            )
            if aligned:
                self._set_translation(
                    decision, robot.position, dribble_target
                )
            return decision

        decision.move_action = int(Action.TURN_FOR_SHOOT)
        decision.rotate_action = int(Action.TURN_FOR_SHOOT)
        self._set_translation(decision, robot.position, dribble_target)
        self._set_orientation(
            decision, robot.heading, goal_vector.angle()
        )
        if self._heading_inside_goal(robot.position, robot.heading):
            self.triggerShoot(
                decision, goal_vector.distance(Point()) / 100.0
            )
        return decision

    def _heading_inside_goal(self, robot: Point, heading: float) -> bool:
        lower = normalize_angle(
            (self.field.opponent_goal_mid_lower - robot).angle() - heading
        )
        upper = normalize_angle(
            (self.field.opponent_goal_mid_upper - robot).angle() - heading
        )
        return lower <= 0.0 <= upper

    @staticmethod
    def _set_translation(
        decision: ActionDecision,
        current: Point,
        target: Point,
        threshold: float = 20.0,
    ) -> bool:
        distance = current.distance(target)
        decision.target = target
        decision.max_velocity = min(distance, MAX_VELOCITY)
        return distance <= threshold

    @staticmethod
    def _set_orientation(
        decision: ActionDecision,
        current: float,
        target: float,
        threshold: float = 8.0 * DEG_TO_RAD,
    ) -> bool:
        error = normalize_angle(target - current)
        decision.target_orientation = target
        decision.max_angular_velocity = min(
            abs(error) * 2.0, MAX_ANGULAR_VELOCITY
        )
        return abs(error) <= threshold
