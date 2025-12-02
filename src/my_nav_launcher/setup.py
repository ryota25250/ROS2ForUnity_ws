import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_nav_launcher'

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
    maintainer='moriokalab',
    maintainer_email='moriokalab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_pose_relay = my_nav_launcher.amcl_pose_relay:main',
            'robot1_cmd_vel_pub = my_nav_launcher.robot1_cmd_vel_pub:main',
            'robot1_safety_stop = my_nav_launcher.robot1_safety_stop:main',
        ],
    },
)
