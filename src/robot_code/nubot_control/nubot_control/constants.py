"""
比赛模式、角色和动作编号.

数值保持与 ROS 1 ``core.hpp`` 一致，因为 ActionCmd 的下游仍按照这些编号解释
机器人当前动作。长度单位沿用旧仿真系统的厘米，角速度单位为 rad/s。
"""

from enum import IntEnum
import math


TEAM_SIZE = 5
FIELD_LENGTH = 2200.0
FIELD_WIDTH = 1400.0
MAX_VELOCITY = 400.0
MAX_ANGULAR_VELOCITY = 10.0
DEG_TO_RAD = math.pi / 180.0


class MatchMode(IntEnum):
    """裁判盒比赛模式."""

    STOP = 0
    OUR_KICKOFF = 1
    OPP_KICKOFF = 2
    OUR_THROWIN = 3
    OPP_THROWIN = 4
    OUR_PENALTY = 5
    OPP_PENALTY = 6
    OUR_GOALKICK = 7
    OPP_GOALKICK = 8
    OUR_CORNERKICK = 9
    OPP_CORNERKICK = 10
    OUR_FREEKICK = 11
    OPP_FREEKICK = 12
    DROPBALL = 13
    START = 15
    PARKING = 25
    TEST = 27


class Action(IntEnum):
    """旧控制链路中的动作编号."""

    STUCKED = 0
    PENALTY = 1
    CANNOT_SEE_BALL = 2
    SEE_NOT_DRIBBLE_BALL = 3
    TURN_FOR_SHOOT = 4
    TURN_FOR_SHOOT_ROBOT = 5
    AT_SHOOT_SITUATION = 6
    TURN_TO_PASS = 7
    TURN_TO_PASS_ROBOT = 8
    STATIC_PASS = 9
    AVOID_OBS = 10
    CATCH_POSITIONED = 11
    POSITIONED = 12
    POSITIONED_STATIC = 13
    KICK_COOP = 14
    KICK_COOP_TURN = 15
    CATCH_BALL = 16
    CATCH_BALL_SLOW = 17
    CIRCLE_TEST = 18
    MOVE_WITH_BALL = 19
    TELEOP_JOY = 20
    NO_ACTION = 21


class Role(IntEnum):
    """机器人角色编号，当前只实际选择守门员和主攻."""

    GOALIE = 0
    ACTIVE = 1
    PASSIVE = 2
    MIDFIELD = 3
    ASSISTANT = 4
    ACID_PASSIVE = 5
    GAZER = 6
    BLOCK = 7
    NO_ROLE = 8
    CATCH_OF_PASS = 9
    PASS_OF_PASS = 10


OUR_RESTARTS = {
    MatchMode.OUR_KICKOFF,
    MatchMode.OUR_THROWIN,
    MatchMode.OUR_PENALTY,
    MatchMode.OUR_GOALKICK,
    MatchMode.OUR_CORNERKICK,
    MatchMode.OUR_FREEKICK,
    MatchMode.DROPBALL,
}

OPP_RESTARTS = {
    MatchMode.OPP_KICKOFF,
    MatchMode.OPP_THROWIN,
    MatchMode.OPP_PENALTY,
    MatchMode.OPP_GOALKICK,
    MatchMode.OPP_CORNERKICK,
    MatchMode.OPP_FREEKICK,
}
