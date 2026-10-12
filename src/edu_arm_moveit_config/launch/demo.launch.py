"""4교시: edu_arm + MoveIt 2

  ros2 launch edu_arm_moveit_config demo.launch.py               # mock 하드웨어 + RViz
  ros2 launch edu_arm_moveit_config demo.launch.py sim:=gazebo    # Gazebo + RViz (RViz에서 계획 → Gazebo 로봇이 움직임)
  ros2 launch edu_arm_moveit_config demo.launch.py sim:=mujoco    # 6교시: MuJoCo + RViz (직접 만든 ROS 2 ↔ MuJoCo 브리지)

옵션: use_rviz:=false, gz_gui:=false (Gazebo 화면 없이 물리만), mj_viewer:=false (MuJoCo 창 없이 물리만)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    sim = context.launch_configurations['sim']
    use_rviz = context.launch_configurations['use_rviz'] == 'true'
    gz_gui = context.launch_configurations['gz_gui'] == 'true'
    if sim not in ('mock', 'gazebo', 'mujoco'):
        raise RuntimeError(f"sim:={sim} 는 지원하지 않습니다 (mock | gazebo | mujoco)")
    use_sim_time = sim == 'gazebo'

    description_share = get_package_share_directory('edu_arm_description')
    moveit_share = get_package_share_directory('edu_arm_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('edu_arm', package_name='edu_arm_moveit_config')
        .robot_description(
            file_path=os.path.join(description_share, 'urdf', 'edu_arm.urdf.xacro'),
            mappings={'sim_mode': 'gazebo' if sim == 'gazebo' else 'mock'})
        .robot_description_semantic(file_path='config/edu_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'],
                            default_planning_pipeline='ompl')
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': use_sim_time}
    controllers = os.path.join(description_share, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(moveit_share, 'config', 'moveit.rviz')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'],
                    parameters=[sim_time])

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[moveit_config.robot_description, sim_time]),
    ]

    if sim == 'mock':
        # ros2_control 을 직접 띄운다 (명령 = 상태인 가짜 하드웨어)
        actions.append(
            Node(package='controller_manager', executable='ros2_control_node',
                 parameters=[controllers],
                 remappings=[('~/robot_description', '/robot_description')]))
    elif sim == 'mujoco':
        # ros2_control 대신 직접 만든 브리지가 같은 이름의 액션(/arm_controller/..., /gripper_controller/...)과
        # /joint_states 를 제공한다. MuJoCo 가 설치된 가상환경의 파이썬으로 실행한다.
        python = os.path.expanduser(context.launch_configurations['mujoco_python'])
        viewer = context.launch_configurations['mj_viewer']
        actions.append(
            ExecuteProcess(cmd=[python, '-m', 'day1_examples.c6_mujoco_bridge',
                                '--ros-args', '-p', f'viewer:={viewer}'],
                           output='screen', name='mujoco_bridge'))
    else:
        # Gazebo 안의 gz_ros2_control 플러그인이 ros2_control 역할을 한다
        physics = context.launch_configurations['physics']
        gz_args = ('-r' if gz_gui else '-r -s') + f' --physics-engine gz-physics-{physics}-plugin empty.sdf'
        actions += [
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'edu_arm']),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        ]

    if sim != 'mujoco':
        actions += [spawner('joint_state_broadcaster'), spawner('arm_controller'), spawner('gripper_controller')]
    actions += [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[moveit_config.to_dict(), sim_time]),
    ]
    if use_rviz:
        actions.append(
            Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
                 parameters=[moveit_config.robot_description,
                             moveit_config.robot_description_semantic,
                             moveit_config.robot_description_kinematics,
                             moveit_config.planning_pipelines,
                             moveit_config.joint_limits,
                             sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='mock', description='mock | gazebo | mujoco'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_gui', default_value='true', description='Gazebo 화면 표시'),
        DeclareLaunchArgument('physics', default_value='bullet-featherstone',
                              description='Gazebo 물리 엔진: bullet-featherstone | dartsim'),
        DeclareLaunchArgument('mj_viewer', default_value='true', description='MuJoCo 뷰어 창 표시'),
        DeclareLaunchArgument('mujoco_python', default_value=os.environ.get('MUJOCO_PYTHON', 'python3'),
                              description='mujoco 패키지가 설치된 파이썬 (scripts/activate.sh 가 MUJOCO_PYTHON 으로 설정)'),
        OpaqueFunction(function=launch_setup),
    ])
