from setuptools import setup
import os
from glob import glob

package_name = 'odom_rate_booster'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nits',
    maintainer_email='nits@example.com',
    description='Odometry rate booster for ROS 2 with MAVROS integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_rate_booster = odom_rate_booster.odom_rate_booster:main',
        ],
    },
)