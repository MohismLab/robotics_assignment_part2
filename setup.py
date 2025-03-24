from setuptools import setup
import os
from glob import glob

package_name = 'robotics_assignment_part2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'robotics_assignment_part2.ros.potential_field_navigation',
        # Missing entry for rrt_navigatio
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('robotics_assignment_part2/ros/launch/*.py')),
        ('share/' + package_name + '/map', glob('robotics_assignment_part2/map/*')),
        ('lib/' + package_name + '/python', glob('robotics_assignment_part2/python/*.py')),  
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential_field_navigation = robotics_assignment_part2.ros.potential_field_navigation:main',
            # Missing entry point for rrt_navigation   

        ],
    },
)
