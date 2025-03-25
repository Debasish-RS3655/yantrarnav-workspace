from setuptools import setup

package_name = 'drone_vio_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nits',
    maintainer_email='duttaamandip1194@gmail.com',
    description='ROS 2 package for drone VIO',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'Set_Home_Position = drone_vio_py.set_home_position:main',
        ],
    },
)
