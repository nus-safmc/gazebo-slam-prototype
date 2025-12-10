import os
from setuptools import setup

package_name = 'tof_slam_sim'


def package_files(source_dir, destination_subdir):
    """Collect data files so colcon installs the Gazebo assets."""
    data = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, source_dir)
        if rel_dir == '.':
            rel_dir = ''
        install_dir = os.path.join('share', package_name, destination_subdir, rel_dir)
        data.append((install_dir, [os.path.join(root, f) for f in files]))
    return data


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Install launch, config, scripts, worlds, and models so Gazebo can resolve model:// URIs.
data_files += package_files('launch', 'launch')
data_files += package_files('config', 'config')
data_files += package_files('scripts', 'scripts')
data_files += package_files('worlds', 'worlds')
data_files += package_files('models', 'models')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rex',
    maintainer_email='rex@example.com',
    description='ToF SLAM sim',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_pilot = tof_slam_sim.auto_pilot:main',
        ],
    },
)
