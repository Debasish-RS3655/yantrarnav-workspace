from setuptools import find_packages, setup

package_name = 'my_mavros_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nits',
    maintainer_email='duttaamandip1194@gmail.com',
    description='A ROS 2 package to subscribe to /mavros/local_position/pose',
    license='Apache License 2.0',
    entry_points={
    'console_scripts': [
        'pose_subscriber = my_mavros_subscriber.pose_subscriber:main',
    ],
},
)
