from setuptools import setup

package_name = 'swarm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Centralized swarm control system for multi-robot exploration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_server = swarm_control.frontier_server:main',
            'goal_allocator = swarm_control.goal_allocator:main',
            'drone_executor = swarm_control.drone_executor:main',
            'mission_supervisor = swarm_control.mission_supervisor:main',
            'traffic_manager = swarm_control.traffic_manager:main',
            'swarm_dashboard = swarm_control.swarm_dashboard:main',
        ],
    },
)