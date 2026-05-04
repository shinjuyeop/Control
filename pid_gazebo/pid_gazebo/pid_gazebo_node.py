#!/usr/bin/env python3
"""
Dual-channel PID controller for Gazebo Differential Drive simulation.

  Channel 1 – longitudinal (RPM)
    setpoint : /setpoint/rpm        [std_msgs/Float64]  – target RPM
    feedback : /odom linear.x       converted → RPM via wheel_radius
    output   : cmd_vel.linear.x     [m/s]

  Channel 2 – steering (yaw / heading)
    setpoint : /setpoint/steer_deg  [std_msgs/Float64]  – target heading [°]
    feedback : yaw extracted from /odom pose quaternion
    output   : cmd_vel.angular.z    [rad/s]

Anti-windup: integral clamp + back-calculation (see PidChannel).
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64

from .pid_channel import PidChannel


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _rpm_to_mps(rpm: float, radius_m: float) -> float:
    return rpm * 2.0 * math.pi * radius_m / 60.0


def _mps_to_rpm(mps: float, radius_m: float) -> float:
    if radius_m < 1e-9:
        return 0.0
    return mps * 60.0 / (2.0 * math.pi * radius_m)


class PidGazeboNode(Node):

    def __init__(self) -> None:
        super().__init__('pid_gazebo')

        # ── longitudinal PID parameters ──────────────────────────────
        self.declare_parameter('lon_kp', 0.5)
        self.declare_parameter('lon_ki', 0.05)
        self.declare_parameter('lon_kd', 0.02)
        self.declare_parameter('lon_out_max', 3.0)    # [m/s]
        self.declare_parameter('lon_i_limit', 50.0)   # integral windup limit [rpm·s]
        self.declare_parameter('wheel_radius', 0.1)   # [m]  – used for RPM ↔ m/s

        # ── steering PID parameters ───────────────────────────────────
        self.declare_parameter('str_kp', 1.0)
        self.declare_parameter('str_ki', 0.0)
        self.declare_parameter('str_kd', 0.1)
        self.declare_parameter('str_out_max', 1.5)    # [rad/s]
        self.declare_parameter('str_i_limit', 1.0)    # integral windup limit [rad·s]

        # ── general ───────────────────────────────────────────────────
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('target_timeout_s', 1.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('setpoint_rpm_topic', '/setpoint/rpm')
        self.declare_parameter('setpoint_steer_topic', '/setpoint/steer_deg')

        self._build_pids()

        # ── internal state ────────────────────────────────────────────
        self._sp_rpm: float = 0.0
        self._sp_steer_deg: float = 0.0
        self._odom_mps: float = 0.0
        self._odom_yaw: float = 0.0
        self._last_sp_t: float = 0.0       # monotonic time of last setpoint
        self._sp_seen: bool = False

        hz = self.get_parameter('control_hz').value
        self._dt: float = 1.0 / max(hz, 1.0)

        # ── subscribers ───────────────────────────────────────────────
        odom_t = self.get_parameter('odom_topic').value
        rpm_t = self.get_parameter('setpoint_rpm_topic').value
        str_t = self.get_parameter('setpoint_steer_topic').value

        self.create_subscription(Odometry, odom_t, self._odom_cb, 10)
        self.create_subscription(Float64, rpm_t, self._rpm_cb, 10)
        self.create_subscription(Float64, str_t, self._steer_cb, 10)

        # ── publisher ─────────────────────────────────────────────────
        cmd_t = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(Twist, cmd_t, 10)

        # ── control timer ─────────────────────────────────────────────
        self.create_timer(self._dt, self._tick)

        # ── live parameter update ─────────────────────────────────────
        self.add_on_set_parameters_callback(self._param_cb)

        self.get_logger().info(
            f'pid_gazebo started | hz={hz} dt={self._dt:.3f}s '
            f'| odom={odom_t} cmd={cmd_t}'
        )

    # ──────────────────────────────────────────────────────────────────
    # Construction helpers
    # ──────────────────────────────────────────────────────────────────

    def _build_pids(self) -> None:
        p = self.get_parameter

        lon_max = p('lon_out_max').value
        self._lon_pid = PidChannel(
            kp=p('lon_kp').value,
            ki=p('lon_ki').value,
            kd=p('lon_kd').value,
            out_min=-lon_max,
            out_max=lon_max,
            i_limit=p('lon_i_limit').value,
        )

        str_max = p('str_out_max').value
        self._str_pid = PidChannel(
            kp=p('str_kp').value,
            ki=p('str_ki').value,
            kd=p('str_kd').value,
            out_min=-str_max,
            out_max=str_max,
            i_limit=p('str_i_limit').value,
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_mps = msg.twist.twist.linear.x
        self._odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)

    def _rpm_cb(self, msg: Float64) -> None:
        self._sp_rpm = msg.data
        self._last_sp_t = time.monotonic()
        self._sp_seen = True

    def _steer_cb(self, msg: Float64) -> None:
        self._sp_steer_deg = msg.data
        self._last_sp_t = time.monotonic()
        self._sp_seen = True

    # ──────────────────────────────────────────────────────────────────
    # Control tick
    # ──────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        timeout = self.get_parameter('target_timeout_s').value
        fresh = self._sp_seen and (time.monotonic() - self._last_sp_t) < timeout

        if not fresh:
            # Stop and reset integrators on timeout
            self._cmd_pub.publish(Twist())
            if self._sp_seen:
                self._lon_pid.reset()
                self._str_pid.reset()
                self.get_logger().warn('setpoint timeout – stopping')
                self._sp_seen = False
            return

        r = self.get_parameter('wheel_radius').value

        # ── Channel 1: longitudinal ────────────────────────────────
        sp_mps = _rpm_to_mps(self._sp_rpm, r)
        fb_mps = self._odom_mps
        lon_err = sp_mps - fb_mps
        lon_out = self._lon_pid.compute(lon_err, self._dt)

        # ── Channel 2: steering (heading control) ──────────────────
        sp_yaw = math.radians(self._sp_steer_deg)
        str_err = _wrap(sp_yaw - self._odom_yaw)
        str_out = self._str_pid.compute(str_err, self._dt)

        # ── Publish ────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x = lon_out
        cmd.angular.z = str_out
        self._cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────────────
    # Live parameter update
    # ──────────────────────────────────────────────────────────────────

    def _param_cb(self, params) -> SetParametersResult:
        p = self.get_parameter

        for param in params:
            n = param.name
            v = param.value

            if n == 'lon_kp':
                self._lon_pid.update_gains(kp=v)
            elif n == 'lon_ki':
                self._lon_pid.update_gains(ki=v)
            elif n == 'lon_kd':
                self._lon_pid.update_gains(kd=v)
            elif n == 'lon_out_max':
                self._lon_pid.update_gains(out_min=-v, out_max=v)
            elif n == 'lon_i_limit':
                self._lon_pid.update_gains(i_limit=v)
            elif n == 'str_kp':
                self._str_pid.update_gains(kp=v)
            elif n == 'str_ki':
                self._str_pid.update_gains(ki=v)
            elif n == 'str_kd':
                self._str_pid.update_gains(kd=v)
            elif n == 'str_out_max':
                self._str_pid.update_gains(out_min=-v, out_max=v)
            elif n == 'str_i_limit':
                self._str_pid.update_gains(i_limit=v)

        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PidGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
