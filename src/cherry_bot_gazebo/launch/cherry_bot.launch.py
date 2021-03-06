import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = 'model.urdf'
    world_file_name = 'test.world'
    rviz_file_name = 'cherry_bot.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('cherry_bot_description'), 'urdf',
        urdf_file_name)

    rviz = os.path.join(
        get_package_share_directory('cherry_bot_gazebo'), 'rviz',
        rviz_file_name
    )

    world = os.path.join(
        get_package_share_directory('cherry_bot_gazebo'), 'worlds',
        world_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '--pause',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc
                         }],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz]
        ),

        # IncludeLaunchDescription(
        #  PythonLaunchDescriptionSource([os.path.join(
        #  get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "cherry_bot"]),

    ])
