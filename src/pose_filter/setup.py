from setuptools import setup

package_name = 'pose_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='NITS',
    maintainer_email='duttaamandip1194@gmail.com',
    description='A package to filter /mavros/vision_pose/pose data using an EKF',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pose_filter_node = pose_filter.pose_filter_node:main'
        ],
    },
)
