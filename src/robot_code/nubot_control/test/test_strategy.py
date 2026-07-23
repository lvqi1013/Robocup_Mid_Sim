from nubot_control.constants import Action, MatchMode, Role
from nubot_control.geometry import Point
from nubot_control.models import BallState, RobotState, WorldState
from nubot_control.strategy import SimpleMatchStrategy
import pytest


def make_world(agent_id=2, mode=MatchMode.START):
    robots = {
        1: RobotState(1, Point(-1050.0, 0.0), valid=True),
        2: RobotState(2, Point(0.0, 0.0), valid=True),
        3: RobotState(3, Point(-500.0, 0.0), valid=True),
    }
    return WorldState(
        agent_id=agent_id,
        robots=robots,
        ball=BallState(Point(100.0, 0.0), known=True),
        match_mode=int(mode),
        world_received=True,
    )


def test_stop_mode_produces_no_action():
    decision = SimpleMatchStrategy().step(
        make_world(mode=MatchMode.STOP), False
    )
    assert decision.move_action == Action.NO_ACTION
    assert decision.rotate_action == Action.NO_ACTION
    assert decision.handle_enable == 0


def test_our_restart_keeps_robot_two_behind_ball():
    decision = SimpleMatchStrategy().step(
        make_world(mode=MatchMode.OUR_KICKOFF), False
    )
    assert decision.target.x == pytest.approx(0.0)
    assert decision.target.y == pytest.approx(0.0)
    assert decision.move_action == Action.POSITIONED_STATIC


def test_nearest_field_robot_chases_ball():
    decision = SimpleMatchStrategy().step(make_world(), False)
    assert decision.role == Role.ACTIVE
    assert decision.active
    assert decision.move_action == Action.CATCH_BALL
    assert decision.handle_enable == 1
    assert decision.target == Point(100.0, 0.0)


def test_non_nearest_robot_stays_idle():
    world = make_world(agent_id=3)
    decision = SimpleMatchStrategy().step(world, False)
    assert not decision.active
    assert decision.move_action == Action.NO_ACTION


def test_dribbling_robot_uses_existing_fixed_target():
    decision = SimpleMatchStrategy().step(make_world(), True)
    assert decision.move_action == Action.MOVE_WITH_BALL
    assert decision.target_orientation != 0.0
    assert decision.handle_enable == 1
