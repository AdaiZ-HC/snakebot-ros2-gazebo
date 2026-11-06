#!/data/ros2_ws/.venv/bin/python3
"""One owner of body, pitch, tongue, and planar navigation commands."""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from behavior_core import ALL_JOINTS, Biology, Navigation, clamp, ease, wrap


def make_trajectory(clock, positions, velocities, horizon):
    """Enforce the complete ten-joint controller contract at the wire boundary."""
    if len(positions) != len(ALL_JOINTS) or len(velocities) != len(ALL_JOINTS):
        raise ValueError('Every trajectory must contain all 10 joint positions and velocities')
    if not all(math.isfinite(value) for value in positions + velocities):
        raise ValueError('Joint trajectory contains a non-finite position or velocity')
    trajectory = JointTrajectory()
    trajectory.header.stamp = clock.to_msg()
    trajectory.joint_names = list(ALL_JOINTS)
    point = JointTrajectoryPoint()
    point.positions, point.velocities = positions, velocities
    point.time_from_start = Duration(seconds=horizon).to_msg()
    trajectory.points = [point]
    return trajectory


class SnakeBehavior(Node):
    def __init__(self):
        super().__init__('snake_behavior', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.seed = int(self.declare_parameter('random_seed', 42).value)
        self.frequency = float(self.declare_parameter('frequency', 0.7).value)
        self.transition = float(self.declare_parameter('transition_time', 0.3).value)
        self.navigation_parameters = {
            'linear_min': float(self.declare_parameter('linear_min', 0.1).value),
            'linear_max': float(self.declare_parameter('linear_max', 0.35).value),
            'angular_max': float(self.declare_parameter('angular_max', 0.6).value),
            'dodge_linear': float(self.declare_parameter('dodge_linear', -0.15).value),
            'dodge_angular': float(self.declare_parameter('dodge_angular', 1.5).value),
        }
        self.autonomous = bool(self.declare_parameter('autonomous', True).value)
        default_map = Path(get_package_share_directory('snakebot_description'))/'config/forest_map.yaml'
        map_file = self.declare_parameter('map_file', str(default_map)).value
        self.map = yaml.safe_load(Path(map_file).read_text())
        self.bio = Biology(self.seed, self.frequency, self.transition)
        self.nav = Navigation(self.map['obstacles'], self.seed, **self.navigation_parameters)
        self.nav.future_joints = self.future_joints
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snakebot/behavior_state', 10)
        self.compatibility_pub = self.create_publisher(String, '/snakebot/roam_state', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.odom_time = self.joint_time = None
        self.epoch = self.last_time = None
        self.phase, self.phase_time = 0.0, None
        self.last_navigation = self.last_status = -math.inf
        self.target_v = self.target_w = self.v = self.w = 0.0
        self.v_rate = self.w_rate = 0.0
        self.logged_state = self.logged_navigation = None
        self.logged_recovery = 0
        self.timer = self.create_timer(0.02, self.tick)
        self.context.on_shutdown(self.timer.cancel)
        self.get_logger().info('50 Hz / 10 joints; 0.7 Hz body; 0.3 s transitions; 1 s COILED/WRAP')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.nav.pose = (p.x, p.y, math.atan2(
            2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.odom_time = self.get_clock().now().nanoseconds*1e-9

    def on_joints(self, msg):
        self.nav.joints = dict(zip(msg.name, msg.position))
        self.joint_time = self.get_clock().now().nanoseconds*1e-9

    def publish_velocity(self):
        if self.autonomous:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.v, self.w
            self.velocity_pub.publish(msg)

    def phase_at(self, when):
        if self.phase_time is None or self.bio.blend is None:
            return self.phase
        dt = when-self.phase_time
        # Integrate frequency; multiplying time by a changing frequency would
        # jump the phase when WRAP changes 0.7 Hz to 1 Hz and back.
        frequencies = [self.bio.sample(self.phase_time+dt*f)[0]['frequency']
                       for f in (0.0, 0.5, 1.0)]
        return self.phase+2*math.pi*dt*(frequencies[0]+4*frequencies[1]+frequencies[2])/6

    def body_targets(self, when):
        values, rates = self.bio.sample(when)
        phase = self.phase_at(when)
        positions, velocities = [], []
        for index in range(8):
            a = phase-index*0.6
            positions.append(values['curvature']+values['amplitude']*math.sin(a))
            velocities.append(rates['curvature']+rates['amplitude']*math.sin(a)
                              +values['amplitude']*2*math.pi*values['frequency']*math.cos(a))
        if values['look']>0 and self.nav.pose is not None:
            x, y, yaw = self.nav.pose
            dx, dy = self.nav.pillar[0]-x, self.nav.pillar[1]-y
            angle = wrap(math.atan2(dy, dx)-yaw)
            aim = clamp(angle, -0.65, 0.65)
            aim_rate = ((dy*self.v*math.cos(yaw)-dx*self.v*math.sin(yaw))
                        /max(dx*dx+dy*dy, 1e-6)-self.w) if abs(angle)<0.65 else 0.0
            q, qdot = positions[0], velocities[0]
            positions[0] = (1-values['look'])*q+values['look']*aim
            velocities[0] = ((1-values['look'])*qdot+values['look']*aim_rate
                             +rates['look']*(aim-q))
        return positions, velocities, values, rates

    def future_joints(self, when):
        positions, _, values, _ = self.body_targets(when)
        return dict(zip(ALL_JOINTS, positions+[values['head'], 0.0]))

    def tick(self):
        clock = self.get_clock().now()
        now = clock.nanoseconds*1e-9
        if now<=0:
            return
        if self.last_time is not None and now<self.last_time:
            self.bio = Biology(self.seed, self.frequency, self.transition)
            pose, joints = self.nav.pose, self.nav.joints
            self.nav = Navigation(self.map['obstacles'], self.seed, **self.navigation_parameters)
            self.nav.pose, self.nav.joints = pose, joints
            self.nav.future_joints = self.future_joints
            self.last_navigation = self.last_status = -math.inf
            self.phase, self.phase_time = 0.0, None
            self.epoch = now
            self.v = self.w = self.v_rate = self.w_rate = 0.0
        if self.epoch is None:
            self.epoch = now
        dt = min(0.1, max(0.0, now-self.last_time)) if self.last_time is not None else 0.02
        self.last_time = now
        self.phase, self.phase_time = self.phase_at(now), now
        values, rates, tongue, tongue_rate, flick_phase = self.bio.update(
            now, hold_choices=self.nav.state.startswith('WRAP'))
        fresh = (self.nav.pose is not None and self.odom_time is not None
                 and self.joint_time is not None and now-self.odom_time<0.5
                 and now-self.joint_time<0.5 and all(j in self.nav.joints for j in ALL_JOINTS))
        if fresh and now-self.last_navigation>=0.05:
            nav_dt = min(0.1, now-self.last_navigation) if math.isfinite(self.last_navigation) else 0.05
            self.nav.command_v = self.v
            # Planned base motion is checked against future body shapes too.
            self.target_v, self.target_w = self.nav.step(
                now, nav_dt, self.bio.TARGETS[self.bio.state]['speed'])
            self.last_navigation = now
            if self.nav.recovery_count!=self.logged_recovery:
                self.get_logger().warning(f'RECOVERY #{self.nav.recovery_count}: 2 s reverse')
                self.logged_recovery = self.nav.recovery_count
            if self.nav.state=='WRAP' and self.bio.state!='WRAP':
                self.bio.enter('WRAP', now)
                self.nav.until = self.bio.until
            elif self.bio.state=='WRAP' and self.nav.state!='WRAP':
                self.bio.enter('ROAM', now)
                self.bio.next_choice = now+self.bio.rng.uniform(4.0, 8.0)
        elif not fresh:
            self.target_v = self.target_w = 0.0
        values, rates = self.bio.sample(now)
        duration = (1.0 if self.bio.state in Biology.SPECIAL
                    or now-self.bio.started<self.bio.transition_duration
                    and self.bio.transition_duration>=1.0 else self.transition)
        self.v, self.v_rate = ease(self.v, self.v_rate, self.target_v, dt, duration)
        self.w, self.w_rate = ease(self.w, self.w_rate, self.target_w, dt, duration)
        self.publish_velocity()
        if self.bio.state!=self.logged_state:
            self.get_logger().info(f'BIOLOGY -> {self.bio.state}')
            self.logged_state = self.bio.state
        if self.nav.state!=self.logged_navigation:
            self.get_logger().info(f'NAVIGATION -> {self.nav.state}; base radius={self.nav.base_tree_distance:.3f}')
            self.logged_navigation = self.nav.state
        # A single controller message owns all ten joints. Body/head use exact
        # timed parameter blends; the tongue deliberately bypasses them.
        horizon = 0.06
        positions, velocities, future, future_rates = self.body_targets(now+horizon)
        tongue_future, tongue_velocity = 0.0, 0.0
        if self.bio.state=='ALERT':
            tongue_future, tongue_velocity, _ = self.bio.flick.sample(now+horizon-self.bio.started)
        positions += [future['head'], tongue_future]
        velocities += [future_rates['head'], tongue_velocity]
        trajectory = make_trajectory(clock, positions, velocities, horizon)
        self.trajectory_pub.publish(trajectory)
        if now-self.last_status>=0.1:
            status = {'time': now, 'state': self.bio.state, 'state_elapsed': now-self.bio.started,
                      'navigation_state': self.nav.state, 'amplitude': values['amplitude'],
                      'frequency': values['frequency'], 'curvature': values['curvature'],
                      'wave_phase': self.phase, 'transition_duration': self.bio.transition_duration,
                      'head_pitch_target': values['head'], 'tongue_target': tongue,
                      'tongue_phase': flick_phase, 'alert_number': self.bio.alert_number,
                      'flick_count': self.bio.flick.count if self.bio.flick else 0,
                      'linear': self.v, 'angular': self.w, 'inputs_fresh': fresh,
                      'recovery_count': self.nav.recovery_count,
                      'actual_3s_distance': self.nav.actual_distance,
                      'wrap_count': self.nav.wrap_count, 'wrap_completed': self.nav.wrap_completed,
                      'wrap_aborted': self.nav.wrap_aborted, 'wrap_radius': self.nav.wrap_radius,
                      'base_tree_distance': self.nav.base_tree_distance,
                      'tree': list(self.nav.pillar),
                      'body_clearance': self.nav.minimum_clearance if fresh else None,
                      'x': self.nav.pose[0] if self.nav.pose else 0.0,
                      'y': self.nav.pose[1] if self.nav.pose else 0.0}
            message = String(data=json.dumps(status))
            self.status_pub.publish(message)
            self.compatibility_pub.publish(message)
            self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = SnakeBehavior()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
