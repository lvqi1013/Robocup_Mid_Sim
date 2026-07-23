"""底盘控制计算模块。

本文件不依赖 ROS 2，专门放置 ActionCmd 到 VelCmd 的数学计算逻辑，
便于 ROS 节点复用并通过单元测试直接验证。
"""

from dataclasses import dataclass
import math


# 旧工程动作枚举中的“无动作”编号。平移或旋转动作等于该值时，对应速度置零。
NO_ACTION = 21


def clamp(value: float, lower: float, upper: float) -> float:
    """把 value 限制在闭区间 [lower, upper] 内。"""
    return max(lower, min(value, upper))


def normalize_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]，避免跨越正负 pi 时产生大角度误差。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def directed_angle_error(target: float, current: float, mode: int) -> float:
    """按旧工程 rotate_mode 语义计算目标角和当前角之间的误差。

    mode 为 0 时选择最短旋转方向；mode 为 1 时保持正方向旋转；
    mode 为 -1 时保持负方向旋转。返回值单位为 rad。
    """
    error = target - current
    if mode == -1:
        while error > 0.0:
            error -= 2.0 * math.pi
    elif mode == 1:
        while error < 0.0:
            error += 2.0 * math.pi
    else:
        error = normalize_angle(error)
    return error


@dataclass
class MotionTarget:
    """控制器计算所需的目标和当前状态。

    字段主要来自 ActionCmd：target_* 表示世界坐标系下的目标点/目标朝向，
    robot_* 表示世界坐标系下的机器人当前位姿，max_* 表示速度上限，
    move_action/rotate_action 控制是否启用平移或旋转，rotate_mode 控制旋转方向。
    """

    # 目标点世界坐标，通常使用厘米。
    target_x: float = 0.0
    target_y: float = 0.0
    # 目标点自身速度或上层期望叠加速度，沿机器人本体坐标叠加到输出速度。
    target_vx: float = 0.0
    target_vy: float = 0.0
    # 目标朝向，单位 rad。
    target_orientation: float = 0.0
    # 当前机器人世界坐标和朝向，位置通常使用厘米，朝向单位 rad。
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_orientation: float = 0.0
    # 本条 ActionCmd 允许的最大平移/角速度。
    max_velocity: float = 0.0
    max_angular_velocity: float = 0.0
    # 旧工程动作编号；等于 NO_ACTION 时分别禁止平移或旋转。
    move_action: int = NO_ACTION
    rotate_action: int = NO_ACTION
    # 旋转方向模式：0 最短方向，1 正方向，-1 负方向。
    rotate_mode: int = 0


@dataclass
class Velocity:
    """机器人本体坐标系速度。

    vx/vy 通常为 cm/s，w 为绕 z 轴角速度 rad/s，直接对应 VelCmd 消息字段。
    """

    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0


class MotionController:
    """兼容旧工程语义的位置/朝向 PD 控制器。

    控制器输入世界坐标目标和当前机器人位姿，输出机器人本体坐标系速度。
    平移和旋转分别使用 PD 控制，并根据动作编号决定是否输出对应轴组速度。
    """

    def __init__(
        self,
        move_kp: float = 5.0,
        move_kd: float = 0.0,
        rotation_kp: float = 2.0,
        rotation_kd: float = 0.0,
        linear_speed_limit: float = 500.0,
        angular_speed_limit: float = 6.0,
    ) -> None:
        """初始化控制增益、速度上限和 D 项历史误差。

        move_kp/move_kd: 平移距离误差的 P/D 增益。
        rotation_kp/rotation_kd: 朝向误差的 P/D 增益。
        linear_speed_limit: 节点级平移速度绝对上限，单位通常为 cm/s。
        angular_speed_limit: 节点级角速度绝对上限，单位 rad/s。
        """
        self.move_kp = move_kp
        self.move_kd = move_kd
        self.rotation_kp = rotation_kp
        self.rotation_kd = rotation_kd
        self.linear_speed_limit = abs(linear_speed_limit)
        self.angular_speed_limit = abs(angular_speed_limit)
        self._previous_distance_error = 0.0
        self._previous_angle_error = 0.0

    def reset(self) -> None:
        """清空 D 项历史误差；禁用、恢复或命令超时时调用。"""
        self._previous_distance_error = 0.0
        self._previous_angle_error = 0.0

    @staticmethod
    def _pd(
        kp: float,
        kd: float,
        error: float,
        previous_error: float,
        limit: float,
    ) -> float:
        """执行单轴 PD 计算，并按给定速度上限限幅。"""
        # 旧 ROS 1 实现直接使用相邻控制周期误差差值作为 D 项，没有除以 dt。
        # 这里保留该行为，避免迁移到 ROS 2 后控制响应变化。
        value = kp * error + kd * (error - previous_error)
        return clamp(value, -limit, limit)

    def calculate(self, target: MotionTarget) -> Velocity:
        """根据世界坐标目标计算机器人本体坐标系 VelCmd。

        计算流程：
        1. 使用目标点与机器人当前位置得到距离误差和目标方位；
        2. 对距离误差做 PD，得到期望平移速度；
        3. 将世界方位转换到机器人本体坐标系并叠加 target_vx/target_vy；
        4. 按 rotate_mode 计算角度误差并做 PD；
        5. 根据 move_action/rotate_action 分别决定是否输出平移/旋转速度。
        """
        max_velocity = min(
            abs(target.max_velocity), self.linear_speed_limit
        )
        max_angular_velocity = min(
            abs(target.max_angular_velocity), self.angular_speed_limit
        )

        dx = target.target_x - target.robot_x
        dy = target.target_y - target.robot_y
        distance_error = math.hypot(dx, dy)
        target_bearing = math.atan2(dy, dx)
        speed = self._pd(
            self.move_kp,
            self.move_kd,
            distance_error,
            self._previous_distance_error,
            max_velocity,
        )

        relative_bearing = target_bearing - target.robot_orientation
        vx = speed * math.cos(relative_bearing) + target.target_vx
        vy = speed * math.sin(relative_bearing) + target.target_vy
        linear_speed = math.hypot(vx, vy)
        if linear_speed > max_velocity and linear_speed > 0.0:
            scale = max_velocity / linear_speed
            vx *= scale
            vy *= scale

        angle_error = directed_angle_error(
            target.target_orientation,
            target.robot_orientation,
            target.rotate_mode,
        )
        w = self._pd(
            self.rotation_kp,
            self.rotation_kd,
            angle_error,
            self._previous_angle_error,
            max_angular_velocity,
        )

        self._previous_distance_error = distance_error
        self._previous_angle_error = angle_error

        if target.move_action == NO_ACTION:
            vx = 0.0
            vy = 0.0
        if target.rotate_action == NO_ACTION:
            w = 0.0
        return Velocity(vx=vx, vy=vy, w=w)
