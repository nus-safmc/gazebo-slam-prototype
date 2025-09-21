from setuptools import setup

package_name = 'tof_slam_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch/config/etc if you already install them
        ('share/' + package_name + '/launch', ['launch/slam_test.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml', 'config/slam_toolbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rex',
    maintainer_email='rex@example.com',
    description='ToF SLAM sim',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add this line:
            'auto_pilot = tof_slam_sim.auto_pilot:main',
            # keep your existing entries too, e.g.:
            # 'tof8x8_to_scan = tof_slam_sim.tof8x8_to_scan:main',
            # 'scan_merger = tof_slam_sim.scan_merger:main',
            # 'test_controller = tof_slam_sim.test_controller:main',
        ],
    },
)

