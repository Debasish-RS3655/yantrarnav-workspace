from setuptools import setup

package_name = 'drone_takeoff'

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
    maintainer_email='vednarsekar@gmail.com',  
    description='Drone takeoff script',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'takeoff_node = drone_takeoff.takeoff_node:main',
        ],
    },
)
