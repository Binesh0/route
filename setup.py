from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot_graph_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TurtleBot graph navigation package with JSON map structure',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_generator = turtlebot_graph_nav.graph_generator:main',
            'graph_navigator = turtlebot_graph_nav.graph_navigator:main',
            'goal_publisher = turtlebot_graph_nav.goal_publisher:main',
        ],
    },
)