from setuptools import find_packages, setup

package_name = 'day2_rl'

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
    maintainer_email='80299296+SungryulBae@users.noreply.github.com',
    description='Day 2: camera data, Q-learning, SAC, RLPD / HIL-SERL on MuJoCo UR5e',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
