from setuptools import setup
import os
from glob import glob

package_name = 'pipe_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # This line installs ALL launch files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),      # This line installs ALL world files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),          # This line installs URDF files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Pipe inspection robot bringup package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'junction_detector = pipe_bringup.junction_detector:main',
            'gps_guided_navigator = pipe_bringup.gps_guided_navigator:main',
            'topological_mapper = pipe_bringup.topological_mapper:main',
            'center_between_walls = pipe_bringup.center_between_walls:main',
        ],
    },
)

