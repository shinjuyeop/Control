"""Own the GNSS-only localization and waypoint-following processes for a Run."""

from pathlib import Path
import sys
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from .sysid_processes import OwnedProcess
from .sysid_profiles import ROUTE_FINISH_DISTANCE_M, ROUTE_STOP_SPEED_MPS


class RouteProcesses:
    def __init__(self, node, settings):
        self.children = []
        self.temporary = tempfile.TemporaryDirectory(prefix='auto_sysid_route_')
        try:
            decision = Path(get_package_share_directory('kaiev26_decision'))
            motion = Path(get_package_share_directory('kaiev26_motion_control'))
            localization = Path(get_package_share_directory('eskf_localization'))
            route_path = decision / 'waypoints' / settings.route_filename
            route = yaml.safe_load(route_path.read_text())
            gnss = yaml.safe_load((localization / 'config/kcity_gnss.yaml').read_text())[
                'eskf_localization']['ros__parameters']
            projection = yaml.safe_load((decision / 'config/decision_pipeline.yaml').read_text())[
                'kaiev26_route_zone_manager']['ros__parameters']
            controller = yaml.safe_load((motion / 'config/motion_control.yaml').read_text())[
                'kaiev26_motion_control']['ros__parameters']
            sim_time = node.get_parameter('use_sim_time').value
            gnss.update({
                'use_sim_time': sim_time,
                'map_origin': {'configured': True, **{
                    key: float(route['map_origin'][key])
                    for key in ('latitude_deg', 'longitude_deg', 'altitude_m')
                }},
            })
            gnss['topics'].update({
                'output_odom': '/sysid/odometry',
                'gnss_debug_odom': '/sysid/odometry_gnss',
            })
            projection.update({
                'use_sim_time': sim_time, 'route_config': str(route_path),
                'odometry_topic': '/sysid/odometry',
                'route_context_topic': '/sysid/route_context',
                'global_route_marker_topic': '/sysid/global_route_markers',
                'route_state_marker_topic': '/sysid/route_state_markers',
            })
            planner = {
                'use_sim_time': sim_time,
                'constant_speed_mps': settings.max_speed_kph / 3.6,
                'route_context_topic': '/sysid/route_context',
                'target_path_topic': '/sysid/target_path',
                'target_speed_topic': '/sysid/target_speed',
            }
            controller.update({
                'use_sim_time': sim_time,
                'target_path_topic': '/sysid/target_path',
                'target_speed_topic': '/sysid/target_speed',
                'command_topic': '/sysid/route_command',
                'planned_stop_brake_distance_m': ROUTE_FINISH_DISTANCE_M,
                'stop_speed_epsilon_mps': ROUTE_STOP_SPEED_MPS,
            })
            params = Path(self.temporary.name) / 'route.yaml'
            params.write_text(yaml.safe_dump({
                '/sysid/eskf_localization': {'ros__parameters': gnss},
                '/sysid/kaiev26_route_zone_manager': {'ros__parameters': projection},
                '/sysid/kaiev26_constant_route_plan': {'ros__parameters': planner},
                '/sysid/kaiev26_motion_control': {'ros__parameters': controller},
                '/sysid/route_input': {'ros__parameters': {'use_sim_time': sim_time}},
            }))
            args = ['--ros-args', '-r', '__ns:=/sysid', '--params-file', str(params),
                    '-r', '/tf:=/sysid/tf']
            executable = Path(get_package_prefix('eskf_localization')) / (
                'lib/eskf_localization/eskf_localization_node')
            self.children.append(OwnedProcess('GNSS-only', [str(executable), *args], node.log))
            self.children.append(OwnedProcess('경로 추종', [
                sys.executable, str(Path(__file__).with_name('sysid_route_worker.py')), *args,
            ], node.log))
        except Exception:
            self.close()
            raise

    def failure(self):
        for child in self.children:
            code = child.process.poll()
            if code is not None:
                return f'{child.name} 종료 (코드 {code}): {child.last_output}'
        return ''

    def close(self):
        errors = []
        for child in reversed(self.children):
            try:
                child.stop()
            except Exception as error:
                errors.append(str(error))
        self.temporary.cleanup()
        if errors:
            raise RuntimeError('; '.join(errors))
