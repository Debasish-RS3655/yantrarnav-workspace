from setuptools import setup

package_name = 'vision_to_local_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='duttaamandip1194@gmail.com',
    description='Node to transform vision pose to local position using home position as origin',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vision_to_local_pose = vision_to_local_pose.vision_to_local_pose:main',
        ],
    },
)
