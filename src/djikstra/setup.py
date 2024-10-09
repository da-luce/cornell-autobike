"""Setup script for the waypoints package."""

from setuptools import setup

PACKAGE_NAME = 'djikstra'

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
    maintainer='Falak Raheja',
    maintainer_email='fr273@cornell.edu',
    description='Path planning algorithm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'djikstra = djikstra.alg:main',
            'once = djikstra.alg:run_once',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
)
