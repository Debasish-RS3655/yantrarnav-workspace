from setuptools import setup

package_name = 'altitude_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nits',
    maintainer_email='duttaamandip1194@gmail.com',
    description='ROS 2 node for altitude estimation using static pressure',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'altitude_node = altitude_package.altitude_node:main',
        ],
    },
)
