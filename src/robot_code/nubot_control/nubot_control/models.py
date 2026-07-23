"""ROS 消息之外的控制领域数据模型."""

from dataclasses import dataclass, field
from typing import Dict

from .constants import Action, MatchMode, Role
from .geometry import Point


@dataclass
class RobotState:
    """单台机器人在世界坐标系中的状态."""

    agent_id: int
    position: Point = Point()
    heading: float = 0.0
    velocity: Point = Point()
    angular_velocity: float = 0.0
    valid: bool = False
    stuck: bool = False
    dribbling: bool = False
    current_role: int = int(Role.NO_ROLE)


@dataclass
class BallState:
    """本机器人视角下的足球状态."""

    position: Point = Point()
    velocity: Point = Point()
    known: bool = False


@dataclass
class WorldState:
    """策略每个周期读取的世界模型快照."""

    agent_id: int
    robots: Dict[int, RobotState] = field(default_factory=dict)
    ball: BallState = field(default_factory=BallState)
    match_mode: int = int(MatchMode.STOP)
    previous_match_mode: int = int(MatchMode.STOP)
    world_received: bool = False

    @property
    def self_robot(self) -> RobotState:
        return self.robots.get(self.agent_id, RobotState(self.agent_id))


@dataclass
class ActionDecision:
    """策略输出，稍后由 ROS 节点转换成 ActionCmd."""

    target: Point = Point()
    target_orientation: float = 0.0
    target_velocity: Point = Point()
    target_angular_velocity: float = 0.0
    max_velocity: float = 0.0
    max_angular_velocity: float = 0.0
    move_action: int = int(Action.NO_ACTION)
    rotate_action: int = int(Action.NO_ACTION)
    rotate_mode: int = 0
    handle_enable: int = 0
    strength: float = 0.0
    shoot_position: int = 0
    role: int = int(Role.NO_ROLE)
    active: bool = False
