#!/data/ros2_ws/.venv/bin/python3
"""Stream a travelling sine wave using the ROS simulation clock only."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SnakeWave(Node):
    def __init__(self):
        super().__init__(
            'snake_wave',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)],
        )
        self.frequency = float(self.declare_parameter('frequency', 0.4).value)
        self.amplitude = float(self.declare_parameter('amplitude', 0.45).value)
        self.phase_difference = float(self.declare_parameter('phase_difference', 0.6).value)
        if not all(math.isfinite(v) for v in (
                self.frequency, self.amplitude, self.phase_difference)):
            raise ValueError('Wave parameters must be finite')
        if self.frequency <= 0 or abs(self.amplitude) > 0.7:
            raise ValueError('frequency must be positive and |amplitude| <= 0.7 rad')
        self.publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.epoch_ns = None
        self.last_ns = None
        self.warned_no_clock = False
        self.timer = self.create_timer(0.02, self.publish_wave)
        # Before the first /clock, send one immediate point with zero duration.
        # The timer itself follows simulation time, including pauses.
        self.publish_wave()
        self.get_logger().info('Sine wave enabled: 50 Hz, use_sim_time=True')

    def publish_wave(self):
        now = self.get_clock().now()
        point = JointTrajectoryPoint()
        trajectory = JointTrajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.joint_names = [f'joint_{i}' for i in range(1, 9)]
        if now.nanoseconds <= 0:
            phase_time = 0.0
            point.time_from_start = Duration(nanoseconds=0).to_msg()
            if not self.warned_no_clock:
                self.get_logger().warning('No simulation clock yet: immediate trajectory point')
                self.warned_no_clock = True
        else:
            if self.last_ns is not None and now.nanoseconds < self.last_ns:
                self.epoch_ns = None
            if self.epoch_ns is None:
                self.epoch_ns = now.nanoseconds
            # time_from_start is a DURATION, not an absolute timestamp. Derive
            # it from two simulation-clock times; never use wall-clock time.
            target_time = now + Duration(seconds=0.04)
            point.time_from_start = (target_time - now).to_msg()
            phase_time = (target_time.nanoseconds - self.epoch_ns) * 1e-9
        self.last_ns = now.nanoseconds
        ramp = min(1.0, max(0.0, phase_time / 1.5))
        envelope = ramp * ramp * (3.0 - 2.0 * ramp)
        point.positions = [
            envelope * self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * phase_time
                - index * self.phase_difference)
            for index in range(8)
        ]
        trajectory.points = [point]
        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = SnakeWave()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
