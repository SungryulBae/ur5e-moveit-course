from setuptools import find_packages, setup

package_name = 'day1_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bsr',
    maintainer_email='bsr@todo.todo',
    description='Day1 hands-on examples: ROS 2 joint control, DH FK/IK, MoveIt 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'c2_joint_state_listener = day1_examples.c2_joint_state_listener:main',
            'c2_joint_commander = day1_examples.c2_joint_commander:main',
            'c2_tf_listener = day1_examples.c2_tf_listener:main',
            'c3_dh_fk = day1_examples.c3_dh_fk:main',
            'c3_ik_numeric = day1_examples.c3_ik_numeric:main',
            'c4_pose_goal = day1_examples.c4_pose_goal:main',
            'c4_linear_gripper = day1_examples.c4_linear_gripper:main',
            'c4_collision = day1_examples.c4_collision:main',
            'c5_ur_status = day1_examples.c5_ur_status:main',
            'c5_ur_move = day1_examples.c5_ur_move:main',
            'c5_ur_io = day1_examples.c5_ur_io:main',
            'c6_gravity_check = day1_examples.c6_gravity_check:main',
        ],
    },
)
