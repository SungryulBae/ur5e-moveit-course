"""2~3교시: ros2_control(mock 하드웨어)로 관절을 제어한다 (MoveIt 없음)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('edu_arm_description')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'edu_arm.urdf.xacro']),
                 ' sim_mode:=mock']),
        value_type=str)
    controllers = PathJoinSubstitution([pkg, 'config', 'ros2_controllers.yaml'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers],
             remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        spawner('arm_controller'),
        spawner('gripper_controller'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(use_rviz),
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'edu_arm.rviz'])]),
    ])
