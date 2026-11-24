#!/usr/bin/env python3
"""
Bridge: Nav2 geometry_msgs/Twist  ->  vehicle InternalDriveCommand (Ackermann).

Design notes
------------
* ONE publisher path. The Twist callback only stores the latest command; a
  fixed-rate timer turns it into a vehicle command. The old version published
  from both the callback and a watchdog timer, so brake=1 and brake=0 messages
  could interleave and the vehicle never got going.
* Steering is computed from the REQUESTED speed and yaw rate (curvature),
  not from the clamped speed. Clamping the speed first changes the curvature.
* An Ackermann vehicle cannot rotate in place, so "no linear speed" means stop.
* Small non-zero forward requests are raised to `min_drive_speed_mps` instead
  of being braked. Nav2 (especially MPPI) starts from ~0 m/s and ramps up; with
  a "brake below X" rule it can sit under X forever.
* Gear changes only happen after a short braked dwell.

UNTESTED on hardware. Verify with the drive wheels off the ground or with a
hardware e-stop in hand, at very low speed.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from vehicle_core.msg import InternalDriveCommand
from vehicle_core.srv import RequestInternalControl


class TwistToInternalDrive(Node):

    def __init__(self):
        super().__init__('twist_to_internal_drive')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/vehicle/internal_drive_cmd')
        self.declare_parameter('controller_name', 'nav2')

        self.declare_parameter('wheelbase_m', 0.8128)
        self.declare_parameter('max_speed_mps', 0.20)

        # Lowest speed sent to the vehicle while Nav2 is asking to move.
        self.declare_parameter('min_drive_speed_mps', 0.02)
        # |linear.x| below this is treated as "stop".
        self.declare_parameter('stop_speed_threshold_mps', 0.02)

        # Confirm the real mechanical limit first.
        self.declare_parameter('max_steer_rad', 0.25)

        self.declare_parameter('command_timeout_s', 0.30)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('gear_change_dwell_s', 0.5)
        self.declare_parameter('debug', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.controller_name = self.get_parameter('controller_name').value

        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.min_drive_speed = float(
            self.get_parameter('min_drive_speed_mps').value)
        self.stop_threshold = float(
            self.get_parameter('stop_speed_threshold_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout_s').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value)
        self.gear_dwell = float(
            self.get_parameter('gear_change_dwell_s').value)
        self.debug = bool(self.get_parameter('debug').value)

        if self.max_steer <= 0.0 or self.max_steer >= math.pi / 2:
            raise ValueError('max_steer_rad must be in (0, pi/2)')

        if self.min_drive_speed > self.max_speed:
            self.get_logger().warn(
                'min_drive_speed_mps > max_speed_mps; clamping it.')
            self.min_drive_speed = self.max_speed

        # Vehicle-specific gear values. No neutral gear exists.
        self.forward_gear = 2
        self.reverse_gear = 3
        self.last_gear = self.forward_gear
        self.gear_switch_deadline = None

        self.control_granted = False
        self.latest_twist = None
        self.last_cmd_time = None

        self.command_publisher = self.create_publisher(
            InternalDriveCommand, self.output_topic, 10)

        self.cmd_vel_subscription = self.create_subscription(
            Twist, self.input_topic, self.cmd_vel_callback, 10)

        self.control_client = self.create_client(
            RequestInternalControl,
            '/drive_arbiter/request_internal_control')

        self.control_timer = self.create_timer(1.0, self.request_control)
        self.loop_timer = self.create_timer(
            1.0 / self.publish_rate, self.control_loop)

        min_radius = self.wheelbase / math.tan(self.max_steer)

        self.get_logger().warn(
            'Safety bridge started. Keep the vehicle disabled '
            'until the command path is verified.')
        self.get_logger().info(f'Input topic:  {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.4f} m')
        self.get_logger().info(
            f'Speed range: {self.min_drive_speed:.3f} - '
            f'{self.max_speed:.3f} m/s')
        self.get_logger().info(f'Max steering: {self.max_steer:.3f} rad')
        self.get_logger().warn(
            f'Minimum turning radius with these limits = {min_radius:.2f} m. '
            f'Set Smac minimum_turning_radius and MPPI min_turning_r to at '
            f'least this value, and vx_max <= {self.max_speed:.2f}.')

    # ------------------------------------------------------------------
    # Control ownership
    # ------------------------------------------------------------------
    def request_control(self):
        if self.control_granted:
            return

        if not self.control_client.service_is_ready():
            self.get_logger().warn(
                'Waiting for /drive_arbiter/request_internal_control')
            return

        request = RequestInternalControl.Request()
        request.controller = self.controller_name

        future = self.control_client.call_async(request)
        future.add_done_callback(self.control_response)

    def control_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.control_granted = True
                self.get_logger().info(
                    f'Internal control granted: {response.message}')
            else:
                self.control_granted = False
                self.get_logger().warn(
                    f'Internal control rejected: {response.message}')

        except Exception as error:
            self.control_granted = False
            self.get_logger().error(f'Control request failed: {error}')

    # ------------------------------------------------------------------
    # Command construction
    # ------------------------------------------------------------------
    def make_stop_command(self):
        command = InternalDriveCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.controller = self.controller_name

        # No neutral gear exists, so keep the current gear and brake.
        command.brake = 100
        command.gear = self.last_gear
        command.speed_mps = 0.0
        command.steer_rad = 0.0
        return command

    def twist_to_command(self, twist, now):
        v_req = float(twist.linear.x)
        w_req = float(twist.angular.z)

        # Ackermann cannot rotate in place: no linear speed means stop.
        if abs(v_req) < self.stop_threshold:
            self.gear_switch_deadline = None
            return self.make_stop_command()

        # Gear selection with a braked dwell before changing direction.
        desired_gear = (
            self.forward_gear if v_req > 0.0 else self.reverse_gear)

        if desired_gear != self.last_gear:
            if self.gear_switch_deadline is None:
                self.gear_switch_deadline = now + Duration(
                    seconds=self.gear_dwell)

            if now < self.gear_switch_deadline:
                return self.make_stop_command()

            self.last_gear = desired_gear

        self.gear_switch_deadline = None

        # Speed magnitude, kept inside [min_drive_speed, max_speed].
        speed = min(max(abs(v_req), self.min_drive_speed), self.max_speed)

        # Bicycle model using the REQUESTED v and w (preserves curvature):
        #   steer = atan(L * w / v)   (v is signed, so reverse works too)
        steer_raw = math.atan(self.wheelbase * w_req / v_req)
        steer = max(-self.max_steer, min(self.max_steer, steer_raw))

        if self.debug:
            self.get_logger().info(
                f'cmd_vel v={v_req:.3f} w={w_req:.3f} -> '
                f'gear={desired_gear} speed={speed:.3f} '
                f'steer={steer:.3f} (raw {steer_raw:.3f})',
                throttle_duration_sec=1.0)

        if abs(steer_raw) > self.max_steer:
            self.get_logger().warn(
                f'Steering saturated: wanted {steer_raw:.3f} rad, '
                f'limit {self.max_steer:.3f}. Nav2 turning radius is '
                f'tighter than the vehicle can do.',
                throttle_duration_sec=2.0)

        command = InternalDriveCommand()
        command.stamp = now.to_msg()
        command.controller = self.controller_name
        command.brake = 0
        command.gear = self.last_gear
        command.speed_mps = float(speed)
        command.steer_rad = float(steer)
        return command

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        # Store only. Publishing happens in control_loop.
        self.latest_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()

        # Watchdog: no recent command, or ownership not granted -> brake.
        if self.latest_twist is None or self.last_cmd_time is None:
            self.command_publisher.publish(self.make_stop_command())
            return

        age = (now - self.last_cmd_time).nanoseconds / 1e9

        if age > self.command_timeout:
            self.command_publisher.publish(self.make_stop_command())
            return

        if not self.control_granted:
            self.command_publisher.publish(self.make_stop_command())
            return

        self.command_publisher.publish(
            self.twist_to_command(self.latest_twist, now))

    def publish_final_stop(self):
        try:
            for _ in range(3):
                self.command_publisher.publish(self.make_stop_command())
        except Exception:
            # Context may already be shutting down.
            pass


def main(args=None):
    rclpy.init(args=args)

    node = TwistToInternalDrive()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warn(
            'Stopping bridge and publishing safe-stop commands.')
        node.publish_final_stop()

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()