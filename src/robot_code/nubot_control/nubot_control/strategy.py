"""
从 ROS 1 ``nubot_control.cpp`` 迁移的现有简单比赛策略.

本文件没有补写原工程中空缺的复杂角色策略。当前行为仍是：非守门员中距离球
最近者追球，持球后移动到固定点并朝球门射门；其余机器人停止。
"""

import math
import time

from .activerole import ActiveRole
from .assistrole import AssistRole
from .constants import (
    Action,
    DEG_TO_RAD,
    MatchMode,
    MAX_ANGULAR_VELOCITY,
    MAX_VELOCITY,
    OPP_RESTARTS,
    OUR_RESTARTS,
    Role,
)
from .field import Field
from .geometry import normalize_angle, Point
from .goaliestrategy import GoalieStrategy
from .midfieldrole import MidfieldRole
from .models import ActionDecision, WorldState
from .passiverole import PassiveRole


class SimpleMatchStrategy:
    """对应 ROS 1 主循环中真正生效的不完整策略."""

    def __init__(self) -> None:
        self.field = Field()
        self.active_role = ActiveRole()
        self.assist_role = AssistRole()
        self.passive_role = PassiveRole()
        self.midfield_role = MidfieldRole()
        self.goalie_role = GoalieStrategy()
        self._last_role = int(Role.NO_ROLE)
        self._role_started = time.monotonic()

    @property
    def role_time(self) -> float:
        return time.monotonic() - self._role_started

    def step(self, world: WorldState, ball_holding: bool) -> ActionDecision:
        """根据比赛模式和世界模型生成本周期动作."""
        robot = world.self_robot
        decision = ActionDecision(
            target=robot.position,
            target_orientation=robot.heading,
        )

        mode = self._match_mode(world.match_mode)
        if mode == MatchMode.STOP:
            decision.role = self._role_for_idle(world.agent_id)
        elif mode in OUR_RESTARTS:
            decision = self._our_default_ready(world)
        elif mode in OPP_RESTARTS:
            decision = self._opponent_default_ready(world)
        elif mode == MatchMode.PARKING:
            decision = self._parking(world)
        else:
            decision = self._normal_game(world, ball_holding)

        self._update_role_timer(decision.role)
        return decision

    @staticmethod
    def _match_mode(value: int) -> MatchMode:
        try:
            return MatchMode(value)
        except ValueError:
            return MatchMode.START

    @staticmethod
    def _role_for_idle(agent_id: int) -> int:
        return int(Role.GOALIE if agent_id == 1 else Role.NO_ROLE)

    def _update_role_timer(self, role: int) -> None:
        if role != self._last_role:
            self._last_role = role
            self._role_started = time.monotonic()

    def _our_default_ready(self, world: WorldState) -> ActionDecision:
        robot = world.self_robot
        ball = world.ball.position
        fixed_targets = {
            1: Point(-1050.0, 0.0),
            4: Point(-550.0, 200.0),
            5: Point(-550.0, -200.0),
        }
        if world.agent_id == 2:
            target = ball.point_towards(robot.position, 100.0)
        elif world.agent_id == 3:
            target = ball.point_towards(robot.position, 200.0)
        else:
            target = fixed_targets.get(world.agent_id, robot.position)
        return self._positioning_decision(world, target)

    def _opponent_default_ready(self, world: WorldState) -> ActionDecision:
        fixed_targets = {
            1: Point(-1050.0, 0.0),
            2: Point(-200.0, 100.0),
            3: Point(-200.0, -100.0),
            4: Point(-550.0, 200.0),
            5: Point(-550.0, -200.0),
        }
        target = fixed_targets.get(world.agent_id, world.self_robot.position)
        if (
            target.distance(world.ball.position) < 300.0
            and not self.field.is_own_penalty(target)
        ):
            target = world.ball.position.point_towards(target, 320.0)
        return self._positioning_decision(world, target)

    def _positioning_decision(
        self, world: WorldState, target: Point
    ) -> ActionDecision:
        robot = world.self_robot
        decision = ActionDecision(
            target=target,
            target_orientation=robot.heading,
            move_action=int(Action.POSITIONED_STATIC),
            rotate_action=int(Action.POSITIONED_STATIC),
            role=self._role_for_idle(world.agent_id),
        )
        close = self._set_translation(decision, robot.position, target)
        if close:
            self._set_orientation(
                decision,
                robot.heading,
                (world.ball.position - robot.position).angle(),
            )
        return decision

    def _parking(self, world: WorldState) -> ActionDecision:
        target = Point(-1100.0 + 150.0 * world.agent_id, -680.0)
        robot = world.self_robot
        decision = ActionDecision(
            target=target,
            target_orientation=robot.heading,
            move_action=int(Action.POSITIONED_STATIC),
            rotate_action=int(Action.POSITIONED_STATIC),
            role=self._role_for_idle(world.agent_id),
        )
        if self._set_translation(decision, robot.position, target):
            self._set_orientation(
                decision, robot.heading, math.pi / 2.0
            )
        return decision

    def _normal_game(
        self, world: WorldState, ball_holding: bool
    ) -> ActionDecision:
        robot = world.self_robot
        if world.agent_id == 1:
            return self.goalie_role.process(world)

        active = world.agent_id != 1 and self._is_nearest_field_robot(world)
        if not active:
            return ActionDecision(
                target=robot.position,
                target_orientation=robot.heading,
                role=self._role_for_idle(world.agent_id),
            )
        return self.active_role.process(world, ball_holding)

    def _is_nearest_field_robot(self, world: WorldState) -> bool:
        candidates = [
            robot
            for agent_id, robot in world.robots.items()
            if agent_id != 1 and robot.valid
        ]
        if not candidates:
            return False
        nearest = min(
            candidates,
            key=lambda robot: (
                robot.position.distance(world.ball.position),
                robot.agent_id,
            ),
        )
        return nearest.agent_id == world.agent_id

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
