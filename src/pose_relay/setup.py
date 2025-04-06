from setuptools import setup

package_name = 'pose_relay'

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
    description='A ROS 2 package to relay PoseStamped messages between topics.',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'pose_relay_node = pose_relay.pose_relay_node:main'
        ],
    },
)
