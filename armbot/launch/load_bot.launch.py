import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_share_dir = get_package_share_directory('armbot')
    urdf_file_name = 'models/urdfs/panda.urdf.xacro'
    urdf = os.path.join(
        package_share_dir,
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    pkg_share = FindPackageShare(package='armbot').find('armbot')
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],# 'publish_frequency': 10.0}],
            arguments=[urdf]),
        Node(
           package='joint_state_publisher',
           executable='joint_state_publisher',
           name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=['-topic','/robot_description',
                      '-entity','armbot'],
            output='screen'
        ) 
    ])
