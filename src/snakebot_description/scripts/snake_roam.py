#!/data/ros2_ws/.venv/bin/python3
"""Known-map wandering and visual orbiting; planar velocity + independent wave."""

import json
import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


PILLARS = ((1.5, 0.0), (-1.0, 2.0), (2.5, -2.0))
TREES = ((-4.0, -3.0), (-4.0, 0.0), (-4.0, 3.0), (-2.8, 4.2),
         (-0.5, 4.8), (2.0, 4.5), (4.0, 3.4), (5.0, 1.2),
         (5.0, -1.8), (3.8, -4.0), (1.0, -4.5), (-1.8, -4.2))
OBSTACLES = tuple((x, y, 0.25) for x, y in PILLARS) + tuple(
    (x, y, 0.16) for x, y in TREES)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def ease_velocity(value, derivative, target, dt, transition_time=0.5):
    """Exact critically damped response: ~96% settled after 0.5 seconds.

    Retaining the derivative preserves acceleration on continuously changing
    targets, unlike restarting a smoothstep at every control tick.
    """
    omega = 5.0 / transition_time
    offset = value - target
    temp = (derivative + omega * offset) * dt
    decay = math.exp(-omega * dt)
    return target + (offset + temp) * decay, (derivative - omega * temp) * decay


def segment_centers(joints):
    """Body collision envelopes and raised-head center in the base frame."""
    points = [(0.0, 0.0, 0.105)]
    head_yaw = joints.get('joint_1', 0.0)
    reach = 0.11 * math.cos(0.52)
    points.append((0.095 + reach * math.cos(head_yaw),
                   reach * math.sin(head_yaw), 0.13))
    x, y, angle = -0.095, 0.0, 0.0
    for index in range(2, 9):
        angle += joints.get(f'joint_{index}', 0.0)
        points.append((x - 0.075 * math.cos(angle),
                       y - 0.075 * math.sin(angle), 0.105))
        x -= 0.17 * math.cos(angle)
        y -= 0.17 * math.sin(angle)
    return points


class SnakeRoam(Node):
    def __init__(self):
        super().__init__('snake_roam')
        self.linear_min = float(self.declare_parameter('linear_min', 0.1).value)
        self.linear_max = float(self.declare_parameter('linear_max', 0.2).value)
        self.angular_min = float(self.declare_parameter('angular_min', -0.3).value)
        self.angular_max = float(self.declare_parameter('angular_max', 0.3).value)
        self.avoid_distance = float(self.declare_parameter('avoid_distance', 1.0).value)
        self.emergency_distance = float(self.declare_parameter('emergency_distance', 0.4).value)
        self.circle_radius = float(self.declare_parameter('circle_radius', 0.9).value)
        self.transition_time = float(self.declare_parameter('transition_time', 0.5).value)
        seed = int(self.declare_parameter('random_seed', 7).value)
        if not (0 < self.linear_min <= self.linear_max <= 0.2):
            raise ValueError('Require 0 < linear_min <= linear_max <= 0.2')
        if not (0 < self.transition_time <= 2.0):
            raise ValueError('transition_time must be in (0, 2] seconds')
        if not (0.25 < self.emergency_distance < self.avoid_distance):
            raise ValueError('Require pillar radius < emergency_distance < avoid_distance')
        self.rng = random.Random(seed)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/snakebot/roam_state', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.pose = None
        self.joints = {}
        self.odom_time = None
        self.joint_time = None
        self.last_time = None
        self.last_status = -1.0
        self.state = 'ROAM'
        self.until = None
        self.cooldown = 0.0
        self.pillar = PILLARS[0]
        self.circle_sign = 1
        self.dodge_sign = 1
        self.wander_v = 0.16
        self.wander_w = 0.0
        self.v = self.w = 0.0
        self.v_derivative = self.w_derivative = 0.0
        self.history = deque()
        self.command_distance = 0.0
        self.kinematic_pose = None
        self.recovering = False
        self.recovery_count = 0
        self.recovery_reason = ''
        self.last_motion_distance = 0.0
        self.guard_blocked_since = None
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('Known-map avoidance enabled; waiting for /odom and eight joints')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.kinematic_pose is None:
            self.kinematic_pose = self.pose
        self.odom_time = self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.joint_time = self.get_clock().now().nanoseconds * 1e-9

    def world_points(self, pose=None):
        x, y, yaw = self.pose if pose is None else pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * px - s * py, y + s * px + c * py, radius)
                for px, py, radius in segment_centers(self.joints)]

    def clearance(self, pose=None):
        return min(math.hypot(x - px, y - py) - radius - obstacle_radius
                   for x, y, radius in self.world_points(pose)
                   for px, py, obstacle_radius in OBSTACLES)

    def record_motion(self, now, dt):
        self.command_distance += abs(self.v) * dt
        if self.kinematic_pose is None:
            self.kinematic_pose = self.pose
        x, y, yaw = self.kinematic_pose
        self.kinematic_pose = (x + self.v * math.cos(yaw) * dt,
                               y + self.v * math.sin(yaw) * dt, wrap(yaw + self.w * dt))
        self.history.append((now, self.pose[0], self.pose[1], self.command_distance))
        while len(self.history) > 1 and self.history[1][0] <= now - 3.0:
            self.history.popleft()

    def is_stuck(self):
        if len(self.history) < 2 or self.history[-1][0] - self.history[0][0] < 2.99:
            return False
        expected = self.history[-1][3] - self.history[0][3]
        # Compare actual odometry path length to integrated published commands.
        # Command-only dead reckoning cannot detect a physically blocked robot.
        self.last_motion_distance = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                                        for a, b in zip(self.history, list(self.history)[1:]))
        return expected > 0.05 * 3.0 and self.last_motion_distance < 0.1

    def start_recovery(self, now, reason):
        self.change_state('DODGE', now, reason)
        self.recovering = True
        self.recovery_reason = reason
        self.recovery_count += 1
        self.until = now + 2.0
        self.history.clear()
        self.guard_blocked_since = None
        self.get_logger().warning(f'RECOVERY #{self.recovery_count}: reverse -0.1 m/s for 2 s; {reason}')

    def predicted_clearance(self, v, w):
        x, y, yaw = self.pose
        result = self.clearance()
        # Include the whole current body envelope, not only the head.
        for t in (0.2, 0.4, 0.6):
            if abs(w) < 1e-6:
                pose = (x + v * t * math.cos(yaw), y + v * t * math.sin(yaw), yaw)
            else:
                pose = (x + v / w * (math.sin(yaw + w * t) - math.sin(yaw)),
                        y - v / w * (math.cos(yaw + w * t) - math.cos(yaw)), yaw + w * t)
            result = min(result, self.clearance(pose))
        return result

    def change_state(self, state, now, reason):
        if self.state != state:
            self.get_logger().info(f'{self.state} -> {state}: {reason}')
        self.state = state
        if state == 'DODGE':
            self.dodge_sign = self.rng.choice((-1, 1))
            if self.predicted_clearance(0.0, self.dodge_sign) < 0.06:
                self.dodge_sign = max((-1, 1), key=lambda sign: self.predicted_clearance(0.0, sign))
            self.until = now + 1.5
        elif state == 'CIRCLE':
            self.until = now + self.rng.uniform(3.0, 5.0)
            self.circle_sign = self.rng.choice((-1, 1))
        elif state == 'ROAM':
            self.until = now + self.rng.uniform(3.0, 5.0)
            self.wander_v = self.rng.uniform(self.linear_min, self.linear_max)
            self.wander_w = self.rng.uniform(self.angular_min, self.angular_max)

    def choose_velocity(self, now):
        head_x, head_y, _ = self.world_points()[1]
        nearest = min(OBSTACLES, key=lambda p: math.hypot(head_x - p[0], head_y - p[1]))[:2]
        distance = math.hypot(head_x - nearest[0], head_y - nearest[1])
        if self.until is None:
            self.until = now + 3.5
        if self.recovering:
            if now < self.until:
                return -0.1, float(self.dodge_sign), distance
            self.recovering = False
            self.history.clear()
            self.cooldown = now + 1.0
            self.change_state('ROAM', now, '2-second recovery complete')
            return self.wander_v, self.wander_w, distance
        if distance <= self.emergency_distance and self.state != 'DODGE':
            self.pillar = nearest
            self.change_state('DODGE', now, 'head emergency distance')
        elif self.state in ('CIRCLE', 'DODGE') and now >= self.until:
            self.cooldown = now + 1.0
            self.change_state('ROAM', now, 'maneuver complete')
        elif self.state == 'ROAM':
            if distance < self.avoid_distance and now >= self.cooldown:
                self.pillar = nearest
                self.change_state('AVOID', now, 'head within avoidance distance')
                self.change_state('CIRCLE' if self.rng.random() < 0.5 else 'DODGE',
                                  now, '50/50 random choice')
            elif now >= self.until:
                self.change_state('ROAM', now, 'new random velocity')
        if self.state == 'DODGE':
            return 0.0, float(self.dodge_sign), distance
        if self.state == 'CIRCLE':
            x, y, yaw = self.pose
            dx, dy = x - self.pillar[0], y - self.pillar[1]
            radius = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / radius, dy / radius
            radial = 1.8 * (self.circle_radius - radius)
            vx = -self.circle_sign * uy + radial * ux
            vy = self.circle_sign * ux + radial * uy
            error = wrap(math.atan2(vy, vx) - yaw)
            v = 0.14 * max(0.0, math.cos(error))
            w = clamp(self.circle_sign * v / self.circle_radius + 1.2 * error,
                      self.angular_min, self.angular_max)
            return v, w, distance
        x, y, yaw = self.pose
        # Soft demonstration-area boundary keeps encounters on the map.
        if x < -2.5 or x > 4.0 or abs(y) > 3.2:
            error = wrap(math.atan2(-y, 0.5 - x) - yaw)
            return (self.linear_min * max(0.0, math.cos(error)),
                    clamp(error, self.angular_min, self.angular_max), distance)
        return self.wander_v, self.wander_w, distance

    def stop(self):
        self.v = self.w = 0.0
        self.v_derivative = self.w_derivative = 0.0
        self.publisher.publish(Twist())

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.pose is None or self.odom_time is None or self.joint_time is None
                or now - self.odom_time > 0.5 or now - self.joint_time > 0.5
                or len(self.joints) < 8):
            self.stop()
            self.history.clear()
            return
        if self.last_time is not None and now < self.last_time:
            self.state, self.until, self.cooldown = 'ROAM', None, 0.0
            self.history.clear()
            self.recovering = False
            self.guard_blocked_since = None
            self.kinematic_pose = self.pose
            self.stop()
        dt = min(0.1, max(0.0, now - self.last_time)) if self.last_time is not None else 0.05
        self.last_time = now
        self.record_motion(now, dt)
        if not self.recovering and self.is_stuck():
            self.start_recovery(now, 'commanded travel > 0.15 m, actual travel < 0.1 m in 3 s')
        target_v, target_w, distance = self.choose_velocity(now)
        clearance = self.clearance()
        blocked = False
        if not self.recovering and self.predicted_clearance(target_v, target_w) < 0.06:
            if self.state != 'DODGE':
                self.change_state('DODGE', now, 'whole-body swept envelope')
            target_v = 0.0
            target_w = float(self.dodge_sign)
            if self.predicted_clearance(0.0, target_w) < 0.02:
                target_w = 0.0
                blocked = True
        if blocked:
            if self.guard_blocked_since is None:
                self.guard_blocked_since = now
            elif now - self.guard_blocked_since >= 3.0:
                self.start_recovery(now, 'whole-body clearance guard deadlock for 3 s')
                target_v, target_w = -0.1, float(self.dodge_sign)
        else:
            self.guard_blocked_since = None
        # Exact ease-in-out filter, including transitions into/out of recovery.
        self.v, self.v_derivative = ease_velocity(
            self.v, self.v_derivative, target_v, dt, self.transition_time)
        self.w, self.w_derivative = ease_velocity(
            self.w, self.w_derivative, target_w, dt, self.transition_time)
        command = Twist()
        command.linear.x, command.angular.z = self.v, self.w
        self.publisher.publish(command)
        if now - self.last_status >= 0.2:
            message = String()
            message.data = json.dumps({'state': self.state, 'time': now,
                'x': self.pose[0], 'y': self.pose[1], 'yaw': self.pose[2],
                'head_distance': distance, 'body_clearance': clearance,
                'pillar': self.pillar, 'linear': self.v, 'angular': self.w,
                'recovering': self.recovering, 'recovery_count': self.recovery_count,
                'recovery_reason': self.recovery_reason,
                'actual_3s_distance': self.last_motion_distance,
                'kinematic_x': self.kinematic_pose[0], 'kinematic_y': self.kinematic_pose[1]})
            self.state_publisher.publish(message)
            self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = SnakeRoam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
