from setuptools import setup, find_packages

package_name = 'tof_slam_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='user',
    author_email='user@todo.todo',
    description='Gazebo Harmonic ToF sensor SLAM simulation and tools.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_merger = tof_slam_sim.scan_merger:main',
            'auto_pilot  = tof_slam_sim.auto_pilot_node:main',
            'map_tf_fallback = tof_slam_sim.map_tf_fallback:main',
            'topic_monitor = tof_slam_sim.topic_monitor:main',
        ],
    },
)
