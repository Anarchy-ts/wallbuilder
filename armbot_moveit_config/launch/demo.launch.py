import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("armbot")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": True}
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("armbot_moveit_config") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )
    # static_perp = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "panda_link0", "perpendicular"],
    # )

    
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("armbot_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[moveit_config.robot_description, ros2_controllers_path],
    #     output="both",
    # )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    #######################################################################################################
    ##################                        GAZEBO                       ################################
    #######################################################################################################

    urdf_file_name = 'config/panda.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('armbot_moveit_config'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Publish TF
    robot_state_publisher = Node(
                            package="robot_state_publisher",
                            executable="robot_state_publisher",
                            name="robot_state_publisher",
                            output="both",
                            parameters=[{'use_sim_time': use_sim_time, 'robot_description' : robot_desc}],
                            arguments=[urdf],
                            )
    joint_state_publisher = Node(
                            package='joint_state_publisher',
                            executable='joint_state_publisher',
                            name='joint_state_publisher',
                            output='both',
                            parameters=[{'use_sim_time': use_sim_time, 'robot_description' : robot_desc}],
                            arguments=[urdf],
                            )

    spawnEntity = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'armbot'],
                output='screen',
                emulate_tty=True,
                )   

    

    return LaunchDescription(
        [   
            # ExecuteProcess(
            #     cmd=["gazebo","-s","libgazebo_ros_factory.so",],
            #     output="screen",
            # ),
            DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            rviz_node,
            static_tf,
            robot_state_publisher,
            # joint_state_publisher,
            run_move_group_node,
            # ros2_control_node,
            spawnEntity,
            # ExecuteProcess(
            #                 cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            #                     'joint_state_broadcaster'],
            #                 output='screen'
            #                     ),
 
            # ExecuteProcess(
            #                 cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            #                     'joint_trajectory_controller'],
            #                 output='screen'
            #                     ),
        ]
        + load_controllers
    )
