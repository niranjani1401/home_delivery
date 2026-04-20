from setuptools import setup
import os
from glob import glob

package_name = 'tb3_delivery'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,                     ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'),    glob('maps/*')),
        (os.path.join('share', package_name, 'config'),  glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niranjani',
    maintainer_email='niranjanihiremath@gmail.com',
    description='Autonomous Home Cleaning Robot using TurtleBot3',
    entry_points={
        'console_scripts': [
            'cleaning_node       = tb3_delivery.cleaning_node:main',
            'emergency_stop_node = tb3_delivery.emergency_stop_node:main',
            'robot_ui            = tb3_delivery.robot_ui:main',
        ],
    },
)