"""Tests for the ROS1-compatible role classes."""

from nubot_control import (
    ActiveRole,
    AssistRole,
    GoalieStrategy,
    MidfieldRole,
    ParabolaFitter3D,
    PassiveRole,
)
from nubot_control.constants import Action, MatchMode, Role
from nubot_control.geometry import Point
from nubot_control.models import BallState, RobotState, WorldState


def make_world(agent_id=2):
    """Create a minimal valid strategy world."""
    return WorldState(
        agent_id=agent_id,
        robots={
            1: RobotState(1, Point(-850.0, 0.0), valid=True),
            2: RobotState(2, Point(0.0, 0.0), valid=True),
        },
        ball=BallState(Point(100.0, 0.0), known=True),
        match_mode=int(MatchMode.START),
        world_received=True,
    )


def test_active_role_keeps_existing_catch_behavior():
    """The active role must contain the migrated runnable behavior."""
    decision = ActiveRole().process(make_world(), False)
    assert decision.role == Role.ACTIVE
    assert decision.move_action == Action.CATCH_BALL
    assert decision.handle_enable == 1


def test_active_role_preserves_declared_ros1_entry_points():
    """The active role must expose the original public API names."""
    role = ActiveRole()
    method_names = (
        'checkPass',
        'clearActiveState',
        'activeDecisionMaking',
        'selectCurrentState',
        'selectCurrentAction',
        'evaluateKick',
        'caculatePassEnergy',
        'caculateDribblingEnergy',
        'selectDribblingOrPassing',
        'findBall',
        'turn4Shoot',
        'NewAvoidObs',
        'NewAvoidObsForPass',
        'activeCatchBall',
        'triggerShoot',
        'stuckProcess',
        'kickball4Coop',
        'IsLocationInField',
    )
    assert all(callable(getattr(role, name, None)) for name in method_names)


def test_empty_roles_return_explicit_safe_decisions():
    """Incomplete ROS1 roles must not accidentally reuse an old command."""
    world = make_world()
    role_cases = (
        (AssistRole(), Role.ASSISTANT),
        (PassiveRole(), Role.PASSIVE),
        (MidfieldRole(), Role.MIDFIELD),
    )
    for role, expected in role_cases:
        decision = role.process(world)
        assert decision.role == expected
        assert decision.move_action == Action.NO_ACTION
        assert decision.target == world.self_robot.position


def test_goalie_names_and_initial_state_match_ros1():
    """The goalie classes must preserve their ROS1 names."""
    goalie = GoalieStrategy()
    fitter = ParabolaFitter3D()
    decision = goalie.process(make_world(agent_id=1))

    assert goalie.state_ == GoalieStrategy.GoalieState.Move2Origin
    assert goalie.robot_info_.position == Point(-850.0, 0.0)
    assert decision.role == Role.GOALIE
    assert fitter.n_ == 0
    assert fitter.data_pointer_ == -1
