"""Launch the simulator: vehicle sim + sim controller (no driver).

Start a driver separately (e.g. `ros2 launch thr_driver driver.launch.py`)
or use `ros2 launch thr_driver demo.launch.py` to start both.

Examples:
    ros2 launch thr_sim sim.launch.py                      # with UI, press Start
    ros2 launch thr_sim sim.launch.py gui:=false           # headless, auto start
    ros2 launch thr_sim sim.launch.py path_shape:=circle num_runs:=2
    ros2 launch thr_sim sim.launch.py shutdown_on_finish:=true
    ros2 launch thr_sim sim.launch.py eval_mode:=true gui:=false shutdown_on_finish:=true \
        num_runs:=3 results_file:=~/eval.json    # auto evaluation
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('thr_sim'), 'config', 'sim.yaml')
    gui = LaunchConfiguration('gui')

    sim_ctrl = Node(
        package='thr_sim', executable='sim_ctrl', name='sim_ctrl', output='screen',
        parameters=[config, {
            'path_shape': LaunchConfiguration('path_shape'),
            'num_runs': LaunchConfiguration('num_runs'),
            'shutdown_on_finish': LaunchConfiguration('shutdown_on_finish'),
            'wait_for_driver': LaunchConfiguration('wait_for_driver'),
            'eval_mode': LaunchConfiguration('eval_mode'),
            'eval_shapes': LaunchConfiguration('eval_shapes'),
            'results_file': LaunchConfiguration('results_file'),
            'gui': gui,
            # with the UI the user starts runs; headless runs start by themselves
            'auto_start': PythonExpression(["'", gui, "' != 'true'"]),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('path_shape', default_value='wave',
                              description='straight | wave | lane_change | circle'),
        DeclareLaunchArgument('num_runs', default_value='1'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='show the sim control UI'),
        DeclareLaunchArgument('shutdown_on_finish', default_value='false',
                              description='stop all nodes when scoring is done'),
        DeclareLaunchArgument('eval_mode', default_value='false',
                              description='run every shape in eval_shapes num_runs times'),
        DeclareLaunchArgument('eval_shapes', default_value='straight,wave,lane_change,circle'),
        DeclareLaunchArgument('results_file', default_value='',
                              description='write the evaluation summary here (JSON)'),
        DeclareLaunchArgument('wait_for_driver', default_value='true',
                              description='auto runs / evaluation start only once a '
                                          'driver (running separately) subscribes to /path'),
        Node(package='thr_sim', executable='vehicle_sim', name='vehicle_sim',
             output='screen', parameters=[config]),
        sim_ctrl,
        RegisterEventHandler(OnProcessExit(
            target_action=sim_ctrl,
            on_exit=[EmitEvent(event=Shutdown(reason='sim_ctrl finished'))])),
    ])
