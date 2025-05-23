import os
from setuptools import setup
from glob import glob

package_name = 'robot_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jw_control = robot_control_pkg.jw_control:main',
            'jw_mapper = robot_control_pkg.jw_mapper:main', 
            'jw_planner = robot_control_pkg.jw_planner:main'
        ],
    },
)
