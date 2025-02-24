"""Setup script for the waypoints package."""

from setuptools import setup

PACKAGE_NAME = 'purepursuit'

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
    maintainer='Kelly Zhang',
    maintainer_email='khz8@cornell.edu',
    description='Pure pursuit algorithm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'purepursuit = purepursuit.alg:main',
            'once = purepursuit.alg:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
)
