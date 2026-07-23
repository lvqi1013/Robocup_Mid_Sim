"""ROS 2 / Gazebo Harmonic automatic referee implementation."""

from dataclasses import dataclass
from functools import partial
import math
import re
import time
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseArray
from nubot_interfaces.msg import BallIsHolding, CoachInfo, SendingOff
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Int16, Int8

from .field import Field, Point


CYAN_TEAM = -1
MAGENTA_TEAM = 1
NONE_TEAM = 0

STOPROBOT = 0
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
STARTROBOT = 15
PARKINGROBOT = 25

OPPOSITE_COMMAND = {
    OUR_KICKOFF: OPP_KICKOFF,
    OPP_KICKOFF: OUR_KICKOFF,
    OUR_THROWIN: OPP_THROWIN,
    OPP_THROWIN: OUR_THROWIN,
    OUR_PENALTY: OPP_PENALTY,
    OPP_PENALTY: OUR_PENALTY,
    OUR_GOALKICK: OPP_GOALKICK,
    OPP_GOALKICK: OUR_GOALKICK,
    OUR_CORNERKICK: OPP_CORNERKICK,
    OPP_CORNERKICK: OUR_CORNERKICK,
    OUR_FREEKICK: OPP_FREEKICK,
    OPP_FREEKICK: OUR_FREEKICK,
}


@dataclass
class ModelState:
    """Model state in the legacy referee's centimetre coordinate system."""

    name: str
    robot_id: int
    position: Point
    z: float = 0.0
    orientation: float = 0.0


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw without depending on copied Gazebo quaternion sources."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class AutoRefereeNode(Node):
    """Apply the legacy match rules using modern ROS/Gazebo interfaces."""

    def __init__(self) -> None:
        super().__init__('auto_referee')
        self.declare_parameter('cyan_prefix', 'nubot')
        self.declare_parameter('magenta_prefix', 'rival')
        self.declare_parameter('team_size', 5)
        self.declare_parameter('ball_name', 'football')
        self.declare_parameter('world_name', 'RoboCup15MSL')
        self.declare_parameter('start_team', CYAN_TEAM)
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('stop_wait_seconds', 2.0)
        self.declare_parameter('pregame_wait_seconds', 4.0)
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('enforce_area_rules', True)

        self.cyan_prefix = str(self.get_parameter('cyan_prefix').value)
        self.magenta_prefix = str(self.get_parameter('magenta_prefix').value)
        self.team_size = int(self.get_parameter('team_size').value)
        self.ball_name = str(self.get_parameter('ball_name').value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.stop_wait = float(self.get_parameter('stop_wait_seconds').value)
        self.pregame_wait = float(
            self.get_parameter('pregame_wait_seconds').value
        )
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.enforce_area_rules = bool(
            self.get_parameter('enforce_area_rules').value
        )

        self.field = Field()
        self.ball_state: Optional[ModelState] = None
        self.cyan_states: Dict[int, ModelState] = {}
        self.magenta_states: Dict[int, ModelState] = {}
        self.last_pose_time = 0.0
        self.last_touch_team = NONE_TEAM
        self.dribbler: Optional[Tuple[int, int]] = None
        self.last_dribbler: Optional[Tuple[int, int]] = None
        self.dribble_start = Point()
        self.ball_reset = Point()
        self.cyan_score = 0
        self.magenta_score = 0
        self.current_command = STOPROBOT
        start_team = int(self.get_parameter('start_team').value)
        self.next_command = (
            OUR_KICKOFF if start_team == CYAN_TEAM else OPP_KICKOFF
        )
        self.command_started = time.monotonic()
        self._last_reset_request = 0.0

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        command_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cyan_coach_pub = self.create_publisher(
            CoachInfo, f'/{self.cyan_prefix}/receive_from_coach', command_qos
        )
        self.magenta_coach_pub = self.create_publisher(
            CoachInfo,
            f'/{self.magenta_prefix}/receive_from_coach',
            command_qos,
        )
        self.cyan_sending_off_pub = self.create_publisher(
            SendingOff, f'/{self.cyan_prefix}/redcard/chatter', 10
        )
        self.magenta_sending_off_pub = self.create_publisher(
            SendingOff, f'/{self.magenta_prefix}/redcard/chatter', 10
        )
        self.create_subscription(
            Contacts, '/football/contacts', self._contact_callback, 10
        )
        self.create_subscription(
            Int16, '/dribble_id', self._dribble_id_callback, 10
        )
        self.create_subscription(
            Int8,
            '/auto_referee/set_game_command',
            lambda msg: self.send_game_command(int(msg.data)),
            10,
        )
        self.create_subscription(
            SendingOff,
            '/auto_referee/sending_off',
            self._relay_sending_off,
            10,
        )
        self._pose_subscriptions = []
        self._holding_subscriptions = []
        self._subscribe_model(self.ball_name, NONE_TEAM, 0)
        for robot_id in range(1, self.team_size + 1):
            self._subscribe_model(
                f'{self.cyan_prefix}{robot_id}', CYAN_TEAM, robot_id
            )
            self._subscribe_model(
                f'{self.magenta_prefix}{robot_id}', MAGENTA_TEAM, robot_id
            )
            self._subscribe_holding(
                f'{self.cyan_prefix}{robot_id}', CYAN_TEAM, robot_id
            )
            self._subscribe_holding(
                f'{self.magenta_prefix}{robot_id}', MAGENTA_TEAM, robot_id
            )

        service_name = f'/world/{self.world_name}/set_pose'
        self.set_pose_client = self.create_client(SetEntityPose, service_name)
        period = max(
            0.005, float(self.get_parameter('control_period').value)
        )
        self.create_timer(period, self._control_loop)

        self.send_game_command(STOPROBOT)
        self.get_logger().info(
            'Auto referee started with Gazebo Harmonic ros_gz interfaces: '
            f'contacts=/football/contacts, set_pose={service_name}'
        )

    def _subscribe_model(self, name: str, team: int, robot_id: int) -> None:
        callback = partial(
            self._pose_callback,
            name=name,
            team=team,
            robot_id=robot_id,
        )
        self._pose_subscriptions.append(
            self.create_subscription(PoseArray, f'/{name}/pose', callback, 10)
        )

    def _subscribe_holding(self, name: str, team: int, robot_id: int) -> None:
        callback = partial(
            self._holding_callback,
            team=team,
            robot_id=robot_id,
        )
        self._holding_subscriptions.append(
            self.create_subscription(
                BallIsHolding,
                f'/{name}/ballisholding/BallIsHolding',
                callback,
                10,
            )
        )

    def _pose_callback(
        self, message: PoseArray, name: str, team: int, robot_id: int
    ) -> None:
        if not message.poses:
            return
        pose = message.poses[0]
        orientation = pose.orientation
        state = ModelState(
            name=name,
            robot_id=robot_id,
            position=Point(
                pose.position.x * 100.0, pose.position.y * 100.0
            ),
            z=pose.position.z * 100.0,
            orientation=quaternion_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        if team == CYAN_TEAM:
            self.cyan_states[robot_id] = state
        elif team == MAGENTA_TEAM:
            # Rival model frames are flipped in the original simulation.
            state.orientation = (
                state.orientation - math.pi
                if state.orientation > 0.0
                else state.orientation + math.pi
            )
            self.magenta_states[robot_id] = state
        else:
            self.ball_state = state
            self.last_pose_time = time.monotonic()

    def _holding_callback(
        self, message: BallIsHolding, team: int, robot_id: int
    ) -> None:
        holder = (team, robot_id)
        if bool(message.ball_is_holding):
            self.dribbler = holder
            self.last_touch_team = team
        elif self.dribbler == holder:
            self.dribbler = None

    def _dribble_id_callback(self, message: Int16) -> None:
        # Compatibility with the ROS 1 global numbering convention. The
        # per-robot BallIsHolding topics above disambiguate modern rival ids.
        dribble_id = int(message.data)
        if dribble_id == -1:
            self.dribbler = None
            return
        if self.dribbler is not None:
            return
        if dribble_id <= self.team_size:
            self.dribbler = (CYAN_TEAM, dribble_id)
        else:
            self.dribbler = (MAGENTA_TEAM, dribble_id - self.team_size)
        self.last_touch_team = self.dribbler[0]

    def _contact_callback(self, message: Contacts) -> None:
        touched_team = NONE_TEAM
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            for name in names:
                if self._matches_robot(name, self.cyan_prefix):
                    touched_team = CYAN_TEAM
                elif self._matches_robot(name, self.magenta_prefix):
                    touched_team = MAGENTA_TEAM
        if self.dribbler is None and touched_team != NONE_TEAM:
            self.last_touch_team = touched_team

    @staticmethod
    def _matches_robot(collision_name: str, prefix: str) -> bool:
        return re.search(rf'(^|::){re.escape(prefix)}\d+(::|$)', collision_name) is not None

    def _control_loop(self) -> None:
        now = time.monotonic()
        if self.ball_state is None or now - self.last_pose_time > self.pose_timeout:
            return

        elapsed = now - self.command_started
        if self.current_command == STOPROBOT:
            if elapsed >= self.stop_wait:
                self.set_ball_position(self.ball_reset)
                self.send_game_command(self.next_command)
        elif self.current_command == STARTROBOT:
            if self._check_dribble_distance():
                return
            if self._check_ball_out_or_goal():
                return
            if self.enforce_area_rules:
                self._check_goal_and_penalty_areas()
        elif self.current_command != PARKINGROBOT:
            if elapsed >= self.pregame_wait:
                self.send_game_command(STARTROBOT)
            elif self.ball_state.position.distance(self.ball_reset) > 10.0:
                self.set_ball_position(self.ball_reset)

    def _check_dribble_distance(self) -> bool:
        if self.dribbler is None:
            self.last_dribbler = None
            return False
        team, robot_id = self.dribbler
        states = self.cyan_states if team == CYAN_TEAM else self.magenta_states
        state = states.get(robot_id)
        if state is None:
            return False
        if self.last_dribbler != self.dribbler:
            self.dribble_start = state.position
        self.last_dribbler = self.dribbler
        if self.dribble_start.distance(state.position) <= 300.0:
            return False

        self.ball_reset = self.field.restart_outside_penalty(
            self.ball_state.position
        )
        self.send_game_command(STOPROBOT)
        self.next_command = (
            OPP_FREEKICK if team == CYAN_TEAM else OUR_FREEKICK
        )
        self.get_logger().warning(
            f'{state.name} dribbled more than 300 cm'
        )
        return True

    def _check_ball_out_or_goal(self) -> bool:
        ball = self.ball_state
        point = ball.position
        if self.field.is_goal(point, ball.z):
            if self.field.is_out_right(point):
                self.cyan_score += 1
                self.next_command = OPP_KICKOFF
                scorer = 'Cyan/black'
            else:
                self.magenta_score += 1
                self.next_command = OUR_KICKOFF
                scorer = 'Magenta/red'
            self.ball_reset = Point()
            self.send_game_command(STOPROBOT)
            self.get_logger().info(
                f'{scorer} goal: {self.cyan_score}:{self.magenta_score}'
            )
            return True

        if self.field.is_out_left(point):
            if self.last_touch_team == CYAN_TEAM:
                self.next_command = OPP_CORNERKICK
                self.ball_reset = self.field.nearest_corner(False, point.y)
            elif self.last_touch_team == MAGENTA_TEAM:
                self.next_command = OUR_GOALKICK
                self.ball_reset = self.field.nearest_goal_kick_restart(
                    False, point.y
                )
            else:
                self.next_command = DROPBALL
                self.ball_reset = Point()
        elif self.field.is_out_right(point):
            if self.last_touch_team == CYAN_TEAM:
                self.next_command = OPP_GOALKICK
                self.ball_reset = self.field.nearest_goal_kick_restart(
                    True, point.y
                )
            elif self.last_touch_team == MAGENTA_TEAM:
                self.next_command = OUR_CORNERKICK
                self.ball_reset = self.field.nearest_corner(True, point.y)
            else:
                self.next_command = DROPBALL
                self.ball_reset = Point()
        elif self.field.is_out_up(point) or self.field.is_out_down(point):
            self.ball_reset = Point(
                point.x,
                self.field.half_width - 30.0
                if point.y > 0.0
                else -self.field.half_width + 30.0,
            )
            if self.last_touch_team == CYAN_TEAM:
                self.next_command = OPP_THROWIN
            elif self.last_touch_team == MAGENTA_TEAM:
                self.next_command = OUR_THROWIN
            else:
                self.next_command = DROPBALL
        else:
            return False

        self.send_game_command(STOPROBOT)
        self.get_logger().info(
            f'Ball out at ({point.x:.1f}, {point.y:.1f}) cm; '
            f'next command={self.next_command}'
        )
        return True

    def _check_goal_and_penalty_areas(self) -> bool:
        for team, states in (
            (CYAN_TEAM, self.cyan_states),
            (MAGENTA_TEAM, self.magenta_states),
        ):
            our_penalty_count = 0
            opp_penalty_count = 0
            for robot_id, state in states.items():
                if robot_id == 1:
                    continue
                point = state.position
                if (
                    self.field.is_our_goal_area(point)
                    or self.field.is_opp_goal_area(point)
                    or self.field.is_goal_pole_area(point)
                ):
                    return self._area_fault(team, f'{state.name} in goal area')
                our_penalty_count += int(self.field.is_our_penalty(point))
                opp_penalty_count += int(self.field.is_opp_penalty(point))
            if our_penalty_count >= 2 or opp_penalty_count >= 2:
                return self._area_fault(
                    team, 'two or more robots in a penalty area'
                )
        return False

    def _area_fault(self, team: int, reason: str) -> bool:
        self.ball_reset = self.field.restart_outside_penalty(
            self.ball_state.position
        )
        self.send_game_command(STOPROBOT)
        self.next_command = (
            OPP_FREEKICK if team == CYAN_TEAM else OUR_FREEKICK
        )
        self.get_logger().warning(reason)
        return True

    def send_game_command(self, command: int) -> None:
        if command not in {
            STOPROBOT,
            OUR_KICKOFF,
            OPP_KICKOFF,
            OUR_THROWIN,
            OPP_THROWIN,
            OUR_PENALTY,
            OPP_PENALTY,
            OUR_GOALKICK,
            OPP_GOALKICK,
            OUR_CORNERKICK,
            OPP_CORNERKICK,
            OUR_FREEKICK,
            OPP_FREEKICK,
            DROPBALL,
            STARTROBOT,
            PARKINGROBOT,
        }:
            command = STOPROBOT

        cyan_message = CoachInfo()
        magenta_message = CoachInfo()
        stamp = self.get_clock().now().to_msg()
        cyan_message.header.stamp = stamp
        magenta_message.header.stamp = stamp
        cyan_message.match_mode = command
        magenta_message.match_mode = OPPOSITE_COMMAND.get(command, command)
        cyan_message.match_type = self.current_command
        magenta_message.match_type = OPPOSITE_COMMAND.get(
            self.current_command, self.current_command
        )
        cyan_message.angle_a = 450
        cyan_message.angle_b = 10
        magenta_message.angle_a = 450
        magenta_message.angle_b = 10
        self.cyan_coach_pub.publish(cyan_message)
        self.magenta_coach_pub.publish(magenta_message)
        self.current_command = command
        self.command_started = time.monotonic()

    def set_ball_position(self, point: Point) -> None:
        now = time.monotonic()
        if now - self._last_reset_request < 0.1:
            return
        self._last_reset_request = now
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warning(
                'Gazebo SetEntityPose service is not ready; ball was not reset',
                throttle_duration_sec=5.0,
            )
            return
        request = SetEntityPose.Request()
        request.entity.name = self.ball_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = point.x / 100.0
        request.pose.position.y = point.y / 100.0
        request.pose.position.z = 0.12
        request.pose.orientation.w = 1.0
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(self._set_pose_done)

    def _set_pose_done(self, future) -> None:
        try:
            if not future.result().success:
                self.get_logger().error('Gazebo rejected the ball pose request')
        except Exception as exc:  # rclpy service exceptions vary by middleware
            self.get_logger().error(f'Ball pose request failed: {exc}')

    def _relay_sending_off(self, message: SendingOff) -> None:
        if bool(message.team_info):
            self.magenta_sending_off_pub.publish(message)
        else:
            self.cyan_sending_off_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AutoRefereeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
