import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'qbot_rl_deploy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zinan',
    maintainer_email='zinan.cen@quanser.com',
    description='RL policy deployment nodes for QBot in ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_action_server = qbot_rl_deploy.navigate_action_server:main',
            'navigate_action_client = qbot_rl_deploy.navigate_action_client:main',
            'observation_builder = qbot_rl_deploy.observation_builder:main',
        ],
    },
)
