import math
from collections import deque

from geometry_msgs.msg import Point, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class GpsWaypointPublisher(Node):
    """Publish GPS, TF, and 3D scene markers for Foxglove demos."""

    def __init__(self):
        super().__init__('gps_waypoint_publisher')

        self.declare_parameter('latitude', 37.5424986)
        self.declare_parameter('longitude', 127.0764268)
        self.declare_parameter('altitude', 30.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('wheelbase_m', 1.45)
        self.declare_parameter('lane_width_m', 2.8)
        self.declare_parameter('gps_noise_m', 0.25)
        self.declare_parameter('lateral_noise_m', 0.15)
        self.declare_parameter('max_trail_points', 350)
        self.declare_parameter('follow_route', True)
        self.declare_parameter('route_loop', True)
        self.declare_parameter('route_end_pause_sec', 1.5)

        self.origin_latitude = float(self.get_parameter('latitude').value)
        self.origin_longitude = float(self.get_parameter('longitude').value)
        self.altitude = float(self.get_parameter('altitude').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.lane_width_m = float(self.get_parameter('lane_width_m').value)
        self.gps_noise_m = float(self.get_parameter('gps_noise_m').value)
        self.lateral_noise_m = float(self.get_parameter('lateral_noise_m').value)
        self.max_trail_points = int(self.get_parameter('max_trail_points').value)
        self.follow_route = bool(self.get_parameter('follow_route').value)
        self.route_loop = bool(self.get_parameter('route_loop').value)
        self.route_end_pause_sec = float(
            self.get_parameter('route_end_pause_sec').value
        )

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.waypoint_pub = self.create_publisher(
            MarkerArray,
            '/viz/waypoints',
            10,
        )
        self.scene_pub = self.create_publisher(MarkerArray, '/viz/scene', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32, '/current/speed', self.speed_cb, 10)
        self.create_subscription(Float32, '/viz/steering_deg', self.steering_cb, 10)

        self.lake_label_m = self.navsat_to_local(37.54078, 127.07620)
        self.route_navsat_points = [
            (37.5424986, 127.0764268),
            (37.5422055, 127.0760822),
            (37.5416455, 127.0758130),
            (37.5411188, 127.0756661),
            (37.5408832, 127.0755531),
            (37.5405660, 127.0752326),
            (37.5401462, 127.0748093),
            (37.5398785, 127.0747113),
            (37.5396096, 127.0748283),
            (37.5393652, 127.0751180),
            (37.5392605, 127.0767423),
            (37.5393058, 127.0769266),
            (37.5396020, 127.0770623),
            (37.5401360, 127.0771951),
            (37.5407162, 127.0775518),
            (37.5409566, 127.0778118),
            (37.5415309, 127.0773092),
            (37.5417958, 127.0773831),
            (37.5420764, 127.0774536),
            (37.5424365, 127.0774930),
            (37.5424654, 127.0771461),
        ]
        self.route_points_m = [
            self.navsat_to_local(latitude, longitude)
            for latitude, longitude in self.route_navsat_points
        ]
        self.route_segments = []
        self.route_length_m = 0.0
        self.build_route_segments()
        self.obstacle_distances_m = [
            min(120.0, self.route_length_m * 0.35),
            min(275.0, self.route_length_m * 0.7),
        ]

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.speed_kmh = 0.0
        self.steering_deg = 0.0
        self.route_progress_m = 0.0
        self.route_direction = 1.0
        self.route_hold_sec = 0.0
        self.start_time = self.get_clock().now()
        self.last_update = self.get_clock().now()
        self.trail = deque(maxlen=self.max_trail_points)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def make_point(x, y, z=0.0):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def quaternion_from_yaw(yaw_rad):
        half_yaw = yaw_rad * 0.5
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    @staticmethod
    def normalize_yaw(yaw_rad):
        return math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))

    def speed_cb(self, msg):
        self.speed_kmh = self.clamp(float(msg.data), -8.0, 30.0)

    def steering_cb(self, msg):
        self.steering_deg = self.clamp(float(msg.data), -30.0, 30.0)

    def navsat_to_local(self, latitude, longitude):
        earth_radius_m = 6378137.0
        origin_lat_rad = math.radians(self.origin_latitude)
        north_m = math.radians(latitude - self.origin_latitude) * earth_radius_m
        east_m = (
            math.radians(longitude - self.origin_longitude)
            * earth_radius_m
            * math.cos(origin_lat_rad)
        )
        return east_m, north_m

    def build_route_segments(self):
        total = 0.0
        route_points = list(self.route_points_m)
        if self.route_loop and len(route_points) > 2:
            route_points.append(route_points[0])

        for start, end in zip(route_points, route_points[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            yaw = math.atan2(dy, dx)
            self.route_segments.append(
                {
                    'start_distance': total,
                    'length': length,
                    'start': start,
                    'end': end,
                    'yaw': yaw,
                }
            )
            total += length

        self.route_length_m = total

    def pose_at_distance(self, distance_m):
        if not self.route_segments:
            return 0.0, 0.0, 0.0

        distance_m = self.clamp(distance_m, 0.0, self.route_length_m)
        for segment in self.route_segments:
            segment_start = segment['start_distance']
            segment_end = segment_start + segment['length']
            if distance_m <= segment_end:
                ratio = (distance_m - segment_start) / segment['length']
                start_x, start_y = segment['start']
                end_x, end_y = segment['end']
                x_m = start_x + (end_x - start_x) * ratio
                y_m = start_y + (end_y - start_y) * ratio
                return x_m, y_m, segment['yaw']

        last = self.route_segments[-1]
        return last['end'][0], last['end'][1], last['yaw']

    def route_point_with_offset(self, index, offset_m):
        point_x, point_y = self.route_points_m[index]
        if self.route_loop and len(self.route_points_m) > 2:
            prev_x, prev_y = self.route_points_m[index - 1]
            next_x, next_y = self.route_points_m[
                (index + 1) % len(self.route_points_m)
            ]
            yaw = math.atan2(next_y - prev_y, next_x - prev_x)
        elif index == 0:
            near_x, near_y = self.route_points_m[index + 1]
            yaw = math.atan2(near_y - point_y, near_x - point_x)
        elif index == len(self.route_points_m) - 1:
            near_x, near_y = self.route_points_m[index - 1]
            yaw = math.atan2(point_y - near_y, point_x - near_x)
        else:
            prev_x, prev_y = self.route_points_m[index - 1]
            next_x, next_y = self.route_points_m[index + 1]
            yaw = math.atan2(next_y - prev_y, next_x - prev_x)

        return (
            point_x - math.sin(yaw) * offset_m,
            point_y + math.cos(yaw) * offset_m,
        )

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        dt = self.clamp(dt, 0.001, 0.25)

        if self.follow_route and self.route_length_m > 0.0:
            if self.route_loop:
                speed_mps = abs(self.speed_kmh) / 3.6
                self.route_progress_m = (
                    self.route_progress_m + speed_mps * dt
                ) % self.route_length_m
            elif self.route_hold_sec > 0.0:
                self.route_hold_sec = max(0.0, self.route_hold_sec - dt)
            else:
                speed_mps = abs(self.speed_kmh) / 3.6
                self.route_progress_m += speed_mps * dt * self.route_direction

            if (
                not self.route_loop
                and self.route_progress_m >= self.route_length_m
                and self.route_direction > 0.0
            ):
                self.route_progress_m = self.route_length_m
                self.route_direction = -1.0
                self.route_hold_sec = self.route_end_pause_sec
                self.trail.clear()
            elif (
                not self.route_loop
                and self.route_progress_m <= 0.0
                and self.route_direction < 0.0
            ):
                self.route_progress_m = 0.0
                self.route_direction = 1.0
                self.route_hold_sec = self.route_end_pause_sec
                self.trail.clear()

            self.x_m, self.y_m, route_yaw = self.pose_at_distance(
                self.route_progress_m
            )
            if self.route_direction < 0.0:
                route_yaw = self.normalize_yaw(route_yaw + math.pi)

            lateral_offset = self.lateral_noise_m * math.sin(
                self.route_progress_m * 0.08
            )
            self.x_m -= math.sin(route_yaw) * lateral_offset
            self.y_m += math.cos(route_yaw) * lateral_offset
            self.yaw_rad = route_yaw
            self.trail.append((self.x_m, self.y_m))
            return

        speed_mps = self.speed_kmh / 3.6
        steering_rad = math.radians(self.steering_deg)
        yaw_rate = speed_mps / self.wheelbase_m * math.tan(steering_rad)
        self.yaw_rad += yaw_rate * dt
        self.x_m += speed_mps * math.cos(self.yaw_rad) * dt
        self.y_m += speed_mps * math.sin(self.yaw_rad) * dt

        if abs(self.x_m) > 100.0 or abs(self.y_m) > 60.0:
            self.x_m = 0.0
            self.y_m = 0.0
            self.yaw_rad = 0.0
            self.trail.clear()

        self.trail.append((self.x_m, self.y_m))

    def local_to_navsat(self, east_m, north_m):
        earth_radius_m = 6378137.0
        origin_lat_rad = math.radians(self.origin_latitude)
        latitude = self.origin_latitude + math.degrees(north_m / earth_radius_m)
        longitude = self.origin_longitude + math.degrees(
            east_m / (earth_radius_m * math.cos(origin_lat_rad))
        )
        return latitude, longitude

    def publish_tf(self, stamp):
        map_to_base = TransformStamped()
        map_to_base.header.stamp = stamp
        map_to_base.header.frame_id = 'map'
        map_to_base.child_frame_id = 'base_link'
        map_to_base.transform.translation.x = self.x_m
        map_to_base.transform.translation.y = self.y_m
        map_to_base.transform.translation.z = 0.0
        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw_rad)
        map_to_base.transform.rotation.x = qx
        map_to_base.transform.rotation.y = qy
        map_to_base.transform.rotation.z = qz
        map_to_base.transform.rotation.w = qw

        base_to_gps = TransformStamped()
        base_to_gps.header.stamp = stamp
        base_to_gps.header.frame_id = 'base_link'
        base_to_gps.child_frame_id = 'gps'
        base_to_gps.transform.translation.x = 0.25
        base_to_gps.transform.translation.y = 0.0
        base_to_gps.transform.translation.z = 0.65
        base_to_gps.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([map_to_base, base_to_gps])

    def make_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

    def route_markers(self, stamp):
        markers = []

        route = self.make_marker(stamp, 'route', 0, Marker.LINE_STRIP)
        route.scale.x = 0.18
        self.set_color(route, 0.08, 0.47, 0.95, 1.0)
        for east_m, north_m in self.route_points_m:
            route.points.append(self.make_point(east_m, north_m, 0.08))
        if self.route_loop and self.route_points_m:
            east_m, north_m = self.route_points_m[0]
            route.points.append(self.make_point(east_m, north_m, 0.08))
        markers.append(route)

        for offset_id, lateral_offset in enumerate(
            (-self.lane_width_m * 0.5, self.lane_width_m * 0.5),
            start=1,
        ):
            lane = self.make_marker(stamp, 'lane_boundary', offset_id, Marker.LINE_STRIP)
            lane.scale.x = 0.08
            self.set_color(lane, 1.0, 1.0, 1.0, 0.9)
            for index in range(len(self.route_points_m)):
                east_m, north_m = self.route_point_with_offset(index, lateral_offset)
                lane.points.append(self.make_point(east_m, north_m, 0.04))
            if self.route_loop and self.route_points_m:
                east_m, north_m = self.route_point_with_offset(0, lateral_offset)
                lane.points.append(self.make_point(east_m, north_m, 0.04))
            markers.append(lane)

        lake_label = self.make_marker(
            stamp,
            'route_labels',
            0,
            Marker.TEXT_VIEW_FACING,
        )
        lake_label.pose.position.x = self.lake_label_m[0]
        lake_label.pose.position.y = self.lake_label_m[1]
        lake_label.pose.position.z = 1.5
        lake_label.scale.z = 1.1
        lake_label.text = 'Lake Ilgam / 일감호'
        self.set_color(lake_label, 0.2, 0.8, 1.0, 1.0)
        markers.append(lake_label)

        for index, (east_m, north_m) in enumerate(self.route_points_m):
            marker = self.make_marker(stamp, 'virtual_waypoints', index, Marker.SPHERE)
            marker.pose.position.x = east_m
            marker.pose.position.y = north_m
            marker.pose.position.z = 0.35
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            self.set_color(marker, 1.0, 0.16, 0.12, 1.0)
            markers.append(marker)

            label = self.make_marker(
                stamp,
                'waypoint_labels',
                index,
                Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = east_m
            label.pose.position.y = north_m
            label.pose.position.z = 1.25
            label.scale.z = 0.75
            label.text = f'WP {index + 1}'
            self.set_color(label, 1.0, 1.0, 1.0, 1.0)
            markers.append(label)

        for index, distance_m in enumerate(self.obstacle_distances_m):
            east_m, north_m, yaw = self.pose_at_distance(distance_m)
            obstacle = self.make_marker(stamp, 'obstacles', index, Marker.CUBE)
            obstacle.pose.position.x = east_m
            obstacle.pose.position.y = north_m
            obstacle.pose.position.z = 0.45
            qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
            obstacle.pose.orientation.x = qx
            obstacle.pose.orientation.y = qy
            obstacle.pose.orientation.z = qz
            obstacle.pose.orientation.w = qw
            obstacle.scale.x = 1.0
            obstacle.scale.y = 1.0
            obstacle.scale.z = 0.9
            self.set_color(obstacle, 1.0, 0.55, 0.0, 0.85)
            markers.append(obstacle)

        return markers

    def vehicle_markers(self, stamp):
        markers = []

        vehicle = self.make_marker(stamp, 'vehicle', 0, Marker.CUBE)
        vehicle.pose.position.x = self.x_m
        vehicle.pose.position.y = self.y_m
        vehicle.pose.position.z = 0.32
        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw_rad)
        vehicle.pose.orientation.x = qx
        vehicle.pose.orientation.y = qy
        vehicle.pose.orientation.z = qz
        vehicle.pose.orientation.w = qw
        vehicle.scale.x = 1.9
        vehicle.scale.y = 0.95
        vehicle.scale.z = 0.45
        self.set_color(vehicle, 0.05, 0.85, 0.55, 1.0)
        markers.append(vehicle)

        heading = self.make_marker(stamp, 'vehicle', 1, Marker.ARROW)
        heading.pose.position.x = self.x_m
        heading.pose.position.y = self.y_m
        heading.pose.position.z = 0.75
        heading.pose.orientation.x = qx
        heading.pose.orientation.y = qy
        heading.pose.orientation.z = qz
        heading.pose.orientation.w = qw
        heading.scale.x = 2.0
        heading.scale.y = 0.15
        heading.scale.z = 0.15
        self.set_color(heading, 0.0, 0.75, 1.0, 1.0)
        markers.append(heading)

        antenna = self.make_marker(stamp, 'vehicle', 2, Marker.SPHERE)
        antenna.pose.position.x = self.x_m + 0.25 * math.cos(self.yaw_rad)
        antenna.pose.position.y = self.y_m + 0.25 * math.sin(self.yaw_rad)
        antenna.pose.position.z = 0.85
        antenna.scale.x = 0.2
        antenna.scale.y = 0.2
        antenna.scale.z = 0.2
        self.set_color(antenna, 0.0, 0.0, 0.0, 1.0)
        markers.append(antenna)

        trail = self.make_marker(stamp, 'vehicle_trail', 0, Marker.LINE_STRIP)
        trail.scale.x = 0.12
        self.set_color(trail, 1.0, 0.82, 0.08, 1.0)
        for east_m, north_m in self.trail:
            trail.points.append(self.make_point(east_m, north_m, 0.06))
        markers.append(trail)

        return markers

    def publish(self):
        self.update_pose()
        now = self.get_clock().now().to_msg()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        gps_noise_east = self.gps_noise_m * math.sin(elapsed * 0.7)
        gps_noise_north = self.gps_noise_m * math.cos(elapsed * 0.43)
        latitude, longitude = self.local_to_navsat(
            self.x_m + gps_noise_east,
            self.y_m + gps_noise_north,
        )

        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = 'gps'
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = self.altitude
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        gps_msg.position_covariance = [
            self.gps_noise_m * self.gps_noise_m,
            0.0,
            0.0,
            0.0,
            self.gps_noise_m * self.gps_noise_m,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.gps_pub.publish(gps_msg)
        self.publish_tf(now)

        waypoint_array = MarkerArray()
        waypoint_array.markers = self.route_markers(now)
        self.waypoint_pub.publish(waypoint_array)

        scene_array = MarkerArray()
        scene_array.markers = self.route_markers(now) + self.vehicle_markers(now)
        self.scene_pub.publish(scene_array)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
