from setuptools import setup

package_name = 'mavros_takeoff'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nits',
    maintainer_email='duttaamandip1194@gmail'
    '.com',
    description='ROS 2 node for MAVROS takeoff',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'takeoff_node = mavros_takeoff.takeoff_node:main',
        ],
    },
)