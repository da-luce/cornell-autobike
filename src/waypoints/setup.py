"""Setup script for the waypoints package."""

from setuptools import setup

PACKAGE_NAME = 'waypoints'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=[PACKAGE_NAME],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'osmnx==1.9.4',
        'overpass==0.7',
        'requests==2.32.3',
        'pyroutelib3==1.7.2',
        'math',
        'statistics',
    ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Dalton Luce',
    maintainer_email='dcl252@cornell.edu',
    description='A ROS2 package for waypoints navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoints = waypoints.waypoint_generator:main',
            'once = waypoints.waypoint_generator:run_once',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
)
