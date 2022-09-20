"""
    launch for yellow machine 202208
    xinnie@iflytek.com
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'true')

    ## ***** File paths ******
    pkg_share = FindPackageShare('base_driver').find('base_driver')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'iflytek_hh_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output = 'screen'
        )

    # 创建Actions.Node对象li_node，标明李四所在位置
    test_node = Node(
        package="base_driver",
        executable="sensor_app",
        output = "screen"
        )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        #robot_state_publisher_node,
        test_node
    ])
