import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'florence2_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose.dominguez@iit.it',
    description='ROS2 Wrapper for Florence-2 VLM',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'florence2_node = florence2_ros2.florence2_node:main',
            'florence2_service_call_example = florence2_ros2.florence2_service_call_example:main',
            'florence2_action_example = florence2_ros2.florence2_action_example:main',
        ],
    },
)
