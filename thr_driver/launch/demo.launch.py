"""Launch the simulator (thr_sim) together with the demo driver.

All thr_sim launch arguments are passed through, e.g.
    ros2 launch thr_driver demo.launch.py
    ros2 launch thr_driver demo.launch.py gui:=false shutdown_on_finish:=true
    ros2 launch thr_driver demo.launch.py path_shape:=circle num_runs:=3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    def include(pkg, name):
        path = os.path.join(get_package_share_directory(pkg), 'launch', name)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))

    return LaunchDescription([
        include('thr_sim', 'sim.launch.py'),
        include('thr_driver', 'driver.launch.py'),
    ])
